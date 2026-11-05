#!/usr/bin/env bash
# sim/px4_sih_multi.sh: N PX4 SIH vehicles + one zenohd + one `control` per vehicle,
# each control in its own Zenoh namespace "cu<i>". Headless, no GUI, no arming.
#
# Usage:
#   sim/px4_sih_multi.sh --px4-build <PX4 build dir with bin/px4 and etc/> \
#       --zenohd <dir with zenohd and z_sub> --control <control binary> --out <dir> \
#       [--n 2] [--duration 90] [--zenoh-port 27447] [--lock /tmp/polyuas-px4-sitl.lock]
#
# Per vehicle i (0-based): PX4 SIH instance -i i (MAVLink sysid i+1, onboard link
# UDP 14540+i), control with MAVLINK_CONN=udpin:127.0.0.1:(14540+i) and a Zenoh client
# config with namespace "cu<i>", plus z_sub on "cu<i>/**". One extra z_sub listens on the
# bare "control/**" key: with working namespaces it must receive nothing.
# Writes <out>/summary.txt and exits 0 only if every check passes.
# Only processes started by this script are signalled (by PID); nothing is killed by name.
set -u -o pipefail

N=2
DURATION=90
PX4_BUILD=""
ZDIR=""
CONTROL=""
OUT=""
ZPORT=27447
LOCK=/tmp/polyuas-px4-sitl.lock
# Vehicle i home = (HOME_LAT, HOME_LON + i*SPACING_M east), HOME_ALT AMSL.
HOME_LAT=35.32844
HOME_LON=-120.75255
HOME_ALT=300.0
SPACING_M=20
ALIVE_CHECK_S=60     # "alive_at_60s" check time after the controls start
SAMPLE_PERIOD_S=5    # ps sampling period for CPU/RSS
MIN_GPI_SAMPLES=100  # per namespace, over the run (5 Hz requested -> ~450 in 90 s)
STOP_WAIT_S=10       # grace time after SIGINT before SIGTERM/SIGKILL

die() { echo "px4_sih_multi: $*" >&2; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --n) N=$2; shift 2 ;;
    --duration) DURATION=$2; shift 2 ;;
    --px4-build) PX4_BUILD=$2; shift 2 ;;
    --zenohd) ZDIR=$2; shift 2 ;;
    --control) CONTROL=$2; shift 2 ;;
    --out) OUT=$2; shift 2 ;;
    --zenoh-port) ZPORT=$2; shift 2 ;;
    --lock) LOCK=$2; shift 2 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ "$N" =~ ^[0-9]+$ ]] && [ "$N" -ge 1 ] && [ "$N" -le 10 ] || die "--n must be 1..10"
[[ "$DURATION" =~ ^[0-9]+$ ]] && [ "$DURATION" -gt "$ALIVE_CHECK_S" ] || die "--duration must be > $ALIVE_CHECK_S s"
[ -x "$PX4_BUILD/bin/px4" ] && [ -d "$PX4_BUILD/etc" ] || die "no px4 at $PX4_BUILD/bin/px4 (or no etc/)"
[ -x "$ZDIR/zenohd" ] && [ -x "$ZDIR/z_sub" ] || die "no zenohd/z_sub in $ZDIR"
[ -x "$CONTROL" ] || die "control binary not executable: $CONTROL"
[ -n "$OUT" ] || die "--out is required"
mkdir -p "$OUT" || die "cannot create $OUT"
OUT=$(cd "$OUT" && pwd)
rm -rf "$OUT"/px4-* "$OUT"/*.log "$OUT"/*.json5 "$OUT"/summary.txt "$OUT"/ps.txt

# ---- serialise all PX4 SITL runs on this machine
exec 9>"$LOCK" || die "cannot open lock $LOCK"
if ! flock -n 9; then
  echo "waiting for SITL lock $LOCK ..."
  flock -w 900 9 || die "SITL lock not acquired in 900 s"
fi

# ---- refuse to start if any port/socket of our instances is in use
busy=""
port_used() { ss -Hanu "sport = :$1" 2>/dev/null | grep -q . || ss -Hant "sport = :$1" 2>/dev/null | grep -q .; }
for i in $(seq 0 $((N-1))); do
  for p in $((14540+i)) $((14580+i)) $((18570+i)) $((13030+i)) $((14280+i)) $((14030+i)) $((13280+i)) $((19410+i)) $((19450+i)); do
    port_used "$p" && busy="$busy $p"
  done
  [ -e "/tmp/px4-sock-$i" ] && busy="$busy /tmp/px4-sock-$i"
done
port_used "$ZPORT" && busy="$busy tcp:$ZPORT"
[ -z "$busy" ] || die "in use, refusing to start:$busy"

ZEP="tcp/127.0.0.1:$ZPORT"
declare -a PX4_PIDS=() CTL_PIDS=() SUB_PIDS=()
ZPID=""
CTL_RC=()
CLEANED=0

stop_pid() {  # stop_pid <pid> <first signal>: signal children and the pid, then escalate
  local p=$1 sig=$2
  kill -0 "$p" 2>/dev/null || return 0
  pkill "-$sig" -P "$p" 2>/dev/null; kill "-$sig" "$p" 2>/dev/null
  for _ in $(seq 1 $((STOP_WAIT_S*10))); do kill -0 "$p" 2>/dev/null || return 0; sleep 0.1; done
  pkill -TERM -P "$p" 2>/dev/null; kill -TERM "$p" 2>/dev/null; sleep 2
  kill -0 "$p" 2>/dev/null && { echo "pid $p still alive -> KILL"; kill -KILL "$p" 2>/dev/null; }
  return 0
}

cleanup() {
  [ "$CLEANED" = 1 ] && return; CLEANED=1
  local i p
  for i in "${!CTL_PIDS[@]}"; do   # controls first, SIGINT = their Ctrl-C handler
    p=${CTL_PIDS[$i]}
    if kill -0 "$p" 2>/dev/null; then
      kill -INT "$p" 2>/dev/null
      for _ in $(seq 1 $((STOP_WAIT_S*10))); do kill -0 "$p" 2>/dev/null || break; sleep 0.1; done
      kill -0 "$p" 2>/dev/null && stop_pid "$p" TERM
    fi
    wait "$p" 2>/dev/null; CTL_RC[$i]=$?
  done
  for p in "${SUB_PIDS[@]}" "${PX4_PIDS[@]}"; do stop_pid "$p" TERM; done
  [ -n "$ZPID" ] && stop_pid "$ZPID" TERM
  wait 2>/dev/null
}
trap cleanup EXIT
trap 'echo "interrupted"; exit 130' INT TERM

echo "date=$(date -Is) n=$N duration=${DURATION}s zenoh=$ZEP out=$OUT"

# ---- router
"$ZDIR/zenohd" -l "$ZEP" --no-multicast-scouting > "$OUT/zenohd.log" 2>&1 &
ZPID=$!
for _ in $(seq 1 100); do ss -Hltn "sport = :$ZPORT" 2>/dev/null | grep -q . && break; sleep 0.1; done
ss -Hltn "sport = :$ZPORT" | grep -q . || die "zenohd did not listen on $ZEP (see zenohd.log)"

# ---- subscribers (no namespace: they see the full keys)
for i in $(seq 0 $((N-1))); do
  "$ZDIR/z_sub" -m client -e "$ZEP" --no-multicast-scouting -k "cu$i/**" > "$OUT/zsub-cu$i.log" 2>&1 &
  SUB_PIDS+=($!)
done
"$ZDIR/z_sub" -m client -e "$ZEP" --no-multicast-scouting -k "control/**" > "$OUT/zsub-bare.log" 2>&1 &
SUB_PIDS+=($!)

# ---- vehicles
for i in $(seq 0 $((N-1))); do
  read -r lat lon <<< "$(awk -v la="$HOME_LAT" -v lo="$HOME_LON" -v e="$((i*SPACING_M))" \
    'BEGIN { printf "%.7f %.7f", la, lo + e / (111320.0 * cos(la * 3.14159265358979 / 180.0)) }')"
  mkdir -p "$OUT/px4-$i"
  ( cd "$OUT/px4-$i" && PX4_HOME_LAT=$lat PX4_HOME_LON=$lon PX4_HOME_ALT=$HOME_ALT PX4_SIM_MODEL=sihsim_quadx \
      exec "$PX4_BUILD/bin/px4" -i "$i" -d "$PX4_BUILD/etc" > out.log 2> err.log ) &
  PX4_PIDS+=($!)
  cat > "$OUT/zenoh-cu$i.json5" <<EOF
{
  mode: "client",
  namespace: "cu$i",
  connect: { endpoints: ["$ZEP"] },
  scouting: { multicast: { enabled: false } },
}
EOF
  echo "vehicle $i: px4 pid ${PX4_PIDS[$i]} home $lat,$lon,$HOME_ALT udp $((14540+i))"
done
sleep 1

# ---- controls
T0=$(date +%s)
for i in $(seq 0 $((N-1))); do
  ZENOH_CONFIG="$OUT/zenoh-cu$i.json5" MAVLINK_CONN="udpin:127.0.0.1:$((14540+i))" \
    "$CONTROL" > "$OUT/control-$i.log" 2>&1 &
  CTL_PIDS+=($!)
  echo "control $i: pid ${CTL_PIDS[$i]} namespace cu$i"
done

# ---- run, sample CPU/RSS, alive check
declare -A PEAK_RSS=() ALIVE=()
CPU_AT_CHECK=""
all_pids() { echo "${PX4_PIDS[*]} ${CTL_PIDS[*]} $ZPID" | tr ' ' ','; }
checked=0
while :; do
  el=$(( $(date +%s) - T0 ))
  ps -o pid=,pcpu=,rss=,comm= -p "$(all_pids)" >> "$OUT/ps.txt" 2>/dev/null
  while read -r pid _cpu rss comm; do
    [ -z "$pid" ] && continue
    [ "${rss:-0}" -gt "${PEAK_RSS[$comm]:-0}" ] && PEAK_RSS[$comm]=$rss
  done < <(ps -o pid=,pcpu=,rss=,comm= -p "$(all_pids)" 2>/dev/null)
  if [ "$checked" = 0 ] && [ "$el" -ge "$ALIVE_CHECK_S" ]; then
    checked=1
    for i in "${!CTL_PIDS[@]}"; do kill -0 "${CTL_PIDS[$i]}" 2>/dev/null && ALIVE[$i]=yes || ALIVE[$i]=no; done
    CPU_AT_CHECK=$(ps -o comm=,pcpu= -p "$(all_pids)" 2>/dev/null | awk '{printf "%s:%s ", $1, $2}')
  fi
  [ "$el" -ge "$DURATION" ] && break
  sleep "$SAMPLE_PERIOD_S"
done

cleanup
leftover=""
for p in "${PX4_PIDS[@]}" "${CTL_PIDS[@]}" "${SUB_PIDS[@]}" $ZPID; do kill -0 "$p" 2>/dev/null && leftover="$leftover $p"; done

# ---- summary
pass=1
{
  echo "date=$(date -Is) n=$N duration_s=$DURATION zenoh=$ZEP zenohd=$("$ZDIR/zenohd" --version 2>&1 | head -1)"
  for i in $(seq 0 $((N-1))); do
    g=$(grep -o "cu$i/control/telem/gl_pos_int" "$OUT/zsub-cu$i.log" | wc -l)
    a=$(grep -o "cu$i/control/telem/attitude" "$OUT/zsub-cu$i.log" | wc -l)
    h=$(grep -o "cu$i/control/health/heartbeat" "$OUT/zsub-cu$i.log" | wc -l)
    sysid=$(grep -o 'FC sysid=[0-9]*' "$OUT/control-$i.log" | head -1 | cut -d= -f2)
    pan=$(grep -c panicked "$OUT/control-$i.log")
    rc=${CTL_RC[$i]:-none}
    echo "cu$i samples gl_pos_int=$g attitude=$a heartbeat=$h"
    echo "control$i fc_sysid=${sysid:-none} expected_sysid=$((i+1)) alive_at_60s=${ALIVE[$i]:-unknown} exit_rc=$rc panicked=$pan"
    [ "$g" -ge "$MIN_GPI_SAMPLES" ] || { pass=0; echo "FAIL cu$i gl_pos_int $g < $MIN_GPI_SAMPLES"; }
    [ "${sysid:-x}" = "$((i+1))" ] || { pass=0; echo "FAIL control$i sysid ${sysid:-none} != $((i+1))"; }
    [ "${ALIVE[$i]:-no}" = yes ] || { pass=0; echo "FAIL control$i not alive at ${ALIVE_CHECK_S}s"; }
    [ "$rc" = 0 ] || { pass=0; echo "FAIL control$i exit rc $rc (want 0 after SIGINT)"; }
    [ "$pan" = 0 ] || { pass=0; echo "FAIL control$i panicked"; }
  done
  bare=$(grep -c "Received" "$OUT/zsub-bare.log")
  echo "bare control/** samples=$bare"
  [ "$bare" = 0 ] || { pass=0; echo "FAIL bare control/** received $bare samples"; }
  pk=""; for c in "${!PEAK_RSS[@]}"; do pk="$pk $c:${PEAK_RSS[$c]}"; done
  echo "peak_rss_kb${pk}"
  echo "cpu_pct_at_${ALIVE_CHECK_S}s $CPU_AT_CHECK"
  echo "leftover_pids=${leftover:-none}"
  [ -z "$leftover" ] || pass=0
  [ "$pass" = 1 ] && echo "result=PASS" || echo "result=FAIL"
} > "$OUT/summary.txt"
cat "$OUT/summary.txt"
[ "$pass" = 1 ]
