# Simulation

## Multi-vehicle PX4 SIH smoke test (`px4_sih_multi.sh`)

Runs N PX4 SIH vehicles (software-in-the-hardware, no Gazebo), one `zenohd`, and one
`control` per vehicle. Each `control` uses its own Zenoh namespace `cu<i>`, so the keys
`control/telem/...` of vehicle i appear on the router as `cu<i>/control/telem/...`.
Nothing is armed and no setpoints are sent: this checks identity, isolation and lifecycle.

```sh
sim/px4_sih_multi.sh --n 2 --duration 90 \
  --px4-build <PX4 build dir, e.g. build/px4_sitl_sih> \
  --zenohd <dir containing zenohd and z_sub (zenoh 1.x)> \
  --control control/target/debug/control \
  --out /tmp/cu-sim/n2
cat /tmp/cu-sim/n2/summary.txt
```

Per vehicle `i` (0-based):

| Item | Value |
|---|---|
| PX4 | `px4 -i i`, `PX4_SIM_MODEL=sihsim_quadx`, MAVLink sysid `i+1`, home 20 m east per vehicle |
| control | `MAVLINK_CONN=udpin:127.0.0.1:(14540+i)`, `ZENOH_CONFIG=<out>/zenoh-cu<i>.json5` (client, `namespace: "cu<i>"`) |
| zenohd | one router on `tcp/127.0.0.1:27447` (`--zenoh-port`), multicast scouting off |
| subscribers | `z_sub` on `cu<i>/**`, plus one on the bare `control/**` (must stay empty) |

`summary.txt` and the exit code (0 = all checks pass) cover:
- at least 100 `gl_pos_int` samples per namespace;
- 0 samples on the bare `control/**` key;
- each `control` logs `FC sysid=<i+1>` (learned from the PX4 HEARTBEAT);
- each `control` is alive at 60 s (`alive_at_60s`), does not panic, and exits with rc 0 after SIGINT;
- no leftover processes; peak RSS and CPU per process type.

Safety of the harness: it takes `flock /tmp/polyuas-px4-sitl.lock` (`--lock`) for the
whole run so SITL runs on one machine do not overlap, refuses to start if any PX4 port of
its instances (14540+i, 14580+i, 18570+i, 13030+i, 14280+i, 14030+i, 13280+i, 19410+i,
19450+i, `/tmp/px4-sock-i`) or the Zenoh port is in use, and signals only the PIDs it started.
