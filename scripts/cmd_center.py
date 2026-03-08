#!/usr/bin/env python3
"""
cmd_center.py — BROS Flight Stack Command Center Dashboard

A zero-dependency (beyond stdlib + zenoh + guidance) terminal dashboard that
gives a live, at-a-glance view of the entire flight stack. Designed to look
and feel like a real GCS — confirms to a human operator that the system is
working whether connected to real hardware or the MuJoCo sim module.

Panels:
  1. Zenoh Topic Table  — topic, msg count, age, byte size, last payload
  2. Telemetry Gauges   — altitude, speed, battery, GPS, mode
  3. FSM State Panel    — mission state, supervisor signal, mode transition log
  4. Command/Ack Log    — scrolling log of commands sent + acks received
  5. Heartbeat Monitor  — per-component (FC, Camera, Gimbal, AirUnit) status
  6. FlatBuf Inspector  — raw hex + decoded fields for the last message on a
                          user-selected topic

Usage:
  python scripts/cmd_center.py                     # default 1 Hz refresh
  python scripts/cmd_center.py --refresh 0.5       # 2 Hz refresh
  python scripts/cmd_center.py --inspect vehicle/telemetry/fc

Keys (while running):
  q / Ctrl-C  — quit
  1-6         — cycle focused panel (for the FlatBuf inspector)
  i           — toggle FlatBuf inspector topic
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import struct
import sys
import termios
import threading
import time
import tty
from collections import defaultdict, deque
from datetime import datetime, timezone
from typing import Any, Callable

# ═══════════════════════════════════════════════════════════════════════════════
# ANSI helpers
# ═══════════════════════════════════════════════════════════════════════════════

ESC = "\033"
CSI = f"{ESC}["

# Cursor / screen
CLEAR_SCREEN = f"{CSI}2J"
HOME = f"{CSI}H"
HIDE_CURSOR = f"{CSI}?25l"
SHOW_CURSOR = f"{CSI}?25h"
SAVE_CURSOR = f"{ESC}7"
REST_CURSOR = f"{ESC}8"


def move_to(row: int, col: int) -> str:
    return f"{CSI}{row};{col}H"


# Colors (256-color)
def fg(code: int) -> str:
    return f"{CSI}38;5;{code}m"


def bg(code: int) -> str:
    return f"{CSI}48;5;{code}m"


RESET = f"{CSI}0m"
BOLD = f"{CSI}1m"
DIM = f"{CSI}2m"

# Semantic colors
C_TITLE = f"{BOLD}{fg(39)}"       # bright cyan
C_OK = fg(82)                      # green
C_WARN = fg(220)                   # yellow
C_ERR = fg(196)                    # red
C_DIM = fg(245)                    # gray
C_VAL = f"{BOLD}{fg(255)}"        # bright white
C_HEADER = f"{BOLD}{fg(33)}"      # blue
C_BOX = fg(240)                    # dark gray for box chars
C_HB_ALIVE = fg(82)               # heartbeat alive
C_HB_STALE = fg(220)              # heartbeat stale
C_HB_DEAD = fg(196)               # heartbeat dead
C_BAT_GOOD = fg(82)
C_BAT_MED = fg(220)
C_BAT_LOW = fg(196)


def bar(value: float, max_val: float, width: int, color_fn: Callable[[float], str]) -> str:
    """Render a horizontal bar gauge."""
    ratio = max(0.0, min(1.0, value / max_val if max_val else 0))
    filled = int(ratio * width)
    c = color_fn(ratio)
    return f"{c}{'█' * filled}{C_DIM}{'░' * (width - filled)}{RESET}"


def bat_color(ratio: float) -> str:
    if ratio > 0.5:
        return C_BAT_GOOD
    elif ratio > 0.2:
        return C_BAT_MED
    return C_BAT_LOW


# Box-drawing
BOX_H = "─"
BOX_V = "│"
BOX_TL = "┌"
BOX_TR = "┐"
BOX_BL = "└"
BOX_BR = "┘"
BOX_LT = "├"
BOX_RT = "┤"
BOX_TT = "┬"
BOX_BT = "┴"
BOX_CROSS = "┼"


def hline(width: int) -> str:
    return BOX_H * width


def box_top(width: int, title: str = "") -> str:
    if title:
        t = f" {title} "
        rem = width - 2 - len(t)
        return f"{C_BOX}{BOX_TL}{BOX_H}{C_TITLE}{t}{C_BOX}{hline(max(0, rem))}{BOX_TR}{RESET}"
    return f"{C_BOX}{BOX_TL}{hline(width - 2)}{BOX_TR}{RESET}"


def box_bot(width: int) -> str:
    return f"{C_BOX}{BOX_BL}{hline(width - 2)}{BOX_BR}{RESET}"


def box_row(content: str, width: int) -> str:
    """Pad content to width, wrapped with box-draw verticals."""
    # Strip ANSI for length calc
    visible = _strip_ansi(content)
    pad = max(0, width - 2 - len(visible))
    return f"{C_BOX}{BOX_V}{RESET}{content}{' ' * pad}{C_BOX}{BOX_V}{RESET}"


def _strip_ansi(s: str) -> str:
    """Remove ANSI escape codes for visible-length calculation."""
    import re
    return re.sub(r"\033\[[0-9;]*m", "", s)


# ═══════════════════════════════════════════════════════════════════════════════
# Zenoh topic map
# ═══════════════════════════════════════════════════════════════════════════════

TOPIC_MAP: dict[str, str] = {
    "vehicle/telemetry/fc":       "VehicleTelemetry",
    "vehicle/heartbeat/fc":       "FC heartbeat",
    "vehicle/heartbeat/camera":   "Camera heartbeat",
    "vehicle/heartbeat/gimbal":   "Gimbal heartbeat",
    "vehicle/heartbeat/air_unit": "AirUnit heartbeat",
    "guidance/mission/command":   "MissionCommand",
    "guidance/mission/ack":       "CommandAck",
    "guidance/supervisor/signal": "SupervisorSignal",
    "control/out/state":          "StateVector (FlatBuf)",
    "sim/mujoco/state":           "MuJoCo SimState",
}

HEARTBEAT_COMPONENTS = {
    "vehicle/heartbeat/fc":       "Flight Controller",
    "vehicle/heartbeat/camera":   "Camera",
    "vehicle/heartbeat/gimbal":   "Gimbal",
    "vehicle/heartbeat/air_unit": "Air Unit",
}

# ═══════════════════════════════════════════════════════════════════════════════
# Decoders
# ═══════════════════════════════════════════════════════════════════════════════


def _decode_telemetry_fb(raw: bytes) -> dict | None:
    """Attempt FlatBuffers VehicleTelemetry decode."""
    try:
        from guidance.generated.vehicle.VehicleTelemetry import (
            VehicleTelemetry as FBTel,
        )
        fb = FBTel.GetRootAs(raw, 0)
        mode = fb.FlightMode()
        if isinstance(mode, bytes):
            mode = mode.decode("utf-8")
        return {
            "lat_deg": fb.LatDeg(),
            "lon_deg": fb.LonDeg(),
            "alt_m": fb.AltM(),
            "vn_mps": fb.VnMps(),
            "ve_mps": fb.VeMps(),
            "vd_mps": fb.VdMps(),
            "battery_pct": fb.BatteryPct(),
            "flight_mode": mode,
            "link_quality": fb.LinkQuality(),
            "_format": "flatbuf",
        }
    except Exception:
        return None


def _decode_telemetry_json(raw: bytes) -> dict | None:
    """Decode JSON telemetry (sim/mujoco/state format or flat)."""
    try:
        obj = json.loads(raw.decode("utf-8"))
        pos = obj.get("position", {})
        vel = obj.get("velocity_ned", {})
        return {
            "lat_deg": pos.get("lat_deg", obj.get("lat_deg", 0)),
            "lon_deg": pos.get("lon_deg", obj.get("lon_deg", 0)),
            "alt_m": pos.get("alt_m", obj.get("alt_m", 0)),
            "vn_mps": vel.get("vn_mps", obj.get("vn_mps", 0)),
            "ve_mps": vel.get("ve_mps", obj.get("ve_mps", 0)),
            "vd_mps": vel.get("vd_mps", obj.get("vd_mps", 0)),
            "battery_pct": obj.get("battery_pct", 0),
            "flight_mode": obj.get("flight_mode", "?"),
            "link_quality": obj.get("link_quality", 0),
            "_format": "json",
        }
    except Exception:
        return None


def decode_telemetry(raw: bytes) -> dict | None:
    return _decode_telemetry_fb(raw) or _decode_telemetry_json(raw)


def _decode_command(raw: bytes) -> dict | None:
    try:
        from guidance.wire_format import WIRE_FORMAT_FLATBUFFERS, decode_command
        cmd = decode_command(raw, WIRE_FORMAT_FLATBUFFERS)
        return {
            "command_id": cmd.command_id,
            "command_type": cmd.command_type.value if hasattr(cmd.command_type, "value") else str(cmd.command_type),
            "target": str(cmd.target),
            "_format": "flatbuf",
        }
    except Exception:
        pass
    try:
        obj = json.loads(raw.decode("utf-8"))
        return {
            "command_id": obj.get("command_id", "?"),
            "command_type": obj.get("command_type", "?"),
            "target": str(obj.get("target", {})),
            "_format": "json",
        }
    except Exception:
        return None


def _decode_ack(raw: bytes) -> dict | None:
    try:
        from guidance.wire_format import WIRE_FORMAT_FLATBUFFERS, decode_ack
        ack = decode_ack(raw, WIRE_FORMAT_FLATBUFFERS)
        return {
            "command_id": ack.command_id,
            "status": ack.status.value if hasattr(ack.status, "value") else str(ack.status),
            "reason": ack.reason,
            "_format": "flatbuf",
        }
    except Exception:
        pass
    try:
        obj = json.loads(raw.decode("utf-8"))
        return {
            "command_id": obj.get("command_id", "?"),
            "status": obj.get("status", "?"),
            "reason": obj.get("reason", ""),
            "_format": "json",
        }
    except Exception:
        return None


def _decode_supervisor(raw: bytes) -> dict | None:
    try:
        obj = json.loads(raw.decode("utf-8"))
        return {
            "state": obj.get("state", "?"),
            "action": obj.get("action", "?"),
            "reason": obj.get("reason", ""),
        }
    except Exception:
        return None


def _decode_sim_state(raw: bytes) -> dict | None:
    return _decode_telemetry_json(raw)


DECODERS: dict[str, Callable] = {
    "vehicle/telemetry/fc": decode_telemetry,
    "guidance/mission/command": _decode_command,
    "guidance/mission/ack": _decode_ack,
    "guidance/supervisor/signal": _decode_supervisor,
    "sim/mujoco/state": _decode_sim_state,
}


def _hex_dump(raw: bytes, max_bytes: int = 64) -> str:
    """Return a hex-dump string of raw bytes."""
    chunk = raw[:max_bytes]
    hex_str = " ".join(f"{b:02x}" for b in chunk)
    if len(raw) > max_bytes:
        hex_str += f" ... ({len(raw)} total)"
    return hex_str


# ═══════════════════════════════════════════════════════════════════════════════
# DataStore — thread-safe message collection
# ═══════════════════════════════════════════════════════════════════════════════

class DataStore:
    """Collects all Zenoh messages in a thread-safe manner."""

    def __init__(self, cmd_log_size: int = 50):
        self.lock = threading.Lock()

        # Per-topic stats
        self.msg_counts: dict[str, int] = defaultdict(int)
        self.last_seen: dict[str, float] = {}
        self.last_raw: dict[str, bytes] = {}
        self.last_decoded: dict[str, dict | None] = {}
        self.last_size: dict[str, int] = {}

        # Telemetry (latest)
        self.telemetry: dict | None = None

        # Heartbeats (last-seen times)
        self.heartbeats: dict[str, float] = {}

        # Supervisor
        self.supervisor: dict | None = None

        # Command/ack log
        self.cmd_log: deque[dict] = deque(maxlen=cmd_log_size)

        # FSM transitions log
        self.fsm_log: deque[str] = deque(maxlen=20)
        self._prev_mode: str | None = None
        self._prev_sup_state: str | None = None

    def record(self, topic: str, raw: bytes):
        now = time.time()
        decoder = DECODERS.get(topic)
        decoded = decoder(raw) if decoder else None

        with self.lock:
            self.msg_counts[topic] += 1
            self.last_seen[topic] = now
            self.last_raw[topic] = raw
            self.last_decoded[topic] = decoded
            self.last_size[topic] = len(raw)

            # Heartbeat tracking
            if topic in HEARTBEAT_COMPONENTS:
                self.heartbeats[topic] = now

            # Telemetry
            if topic == "vehicle/telemetry/fc" and decoded:
                self.telemetry = decoded
                mode = decoded.get("flight_mode", "?")
                if mode != self._prev_mode and self._prev_mode is not None:
                    ts = datetime.now(timezone.utc).strftime("%H:%M:%S")
                    self.fsm_log.append(f"[{ts}] MODE {self._prev_mode} → {mode}")
                self._prev_mode = mode

            # Sim telemetry → also update telemetry gauge if no FC telemetry
            if topic == "sim/mujoco/state" and decoded and self.telemetry is None:
                self.telemetry = decoded

            # Supervisor
            if topic == "guidance/supervisor/signal" and decoded:
                state = decoded.get("state", "?")
                if state != self._prev_sup_state and self._prev_sup_state is not None:
                    ts = datetime.now(timezone.utc).strftime("%H:%M:%S")
                    self.fsm_log.append(
                        f"[{ts}] SUP {self._prev_sup_state} → {state} ({decoded.get('action', '?')})"
                    )
                self._prev_sup_state = state
                self.supervisor = decoded

            # Command log
            if topic == "guidance/mission/command" and decoded:
                ts = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]
                self.cmd_log.append({
                    "ts": ts,
                    "dir": "CMD→",
                    "id": decoded.get("command_id", "?")[:12],
                    "type": decoded.get("command_type", "?"),
                    "detail": decoded.get("target", "")[:30],
                })

            if topic == "guidance/mission/ack" and decoded:
                ts = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]
                self.cmd_log.append({
                    "ts": ts,
                    "dir": "←ACK",
                    "id": decoded.get("command_id", "?")[:12],
                    "type": decoded.get("status", "?"),
                    "detail": decoded.get("reason", "")[:30],
                })


# ═══════════════════════════════════════════════════════════════════════════════
# Rendering — each panel is a function returning a list of strings
# ═══════════════════════════════════════════════════════════════════════════════

def render_topic_table(store: DataStore, width: int) -> list[str]:
    """Panel 1: Zenoh Topic Table."""
    lines: list[str] = []
    lines.append(box_top(width, "① Zenoh Topic Table"))

    header = f" {'Topic':<36s} {'Cnt':>5s} {'Age':>6s} {'Bytes':>5s}  Last Payload"
    lines.append(box_row(f"{C_HEADER}{header}{RESET}", width))
    lines.append(box_row(f"{C_DIM}{hline(width - 4)}{RESET}", width))

    now = time.time()
    with store.lock:
        for topic in TOPIC_MAP:
            cnt = store.msg_counts.get(topic, 0)
            age = now - store.last_seen[topic] if topic in store.last_seen else float("inf")
            sz = store.last_size.get(topic, 0)
            dec = store.last_decoded.get(topic)

            age_str = f"{age:.1f}s" if age < 9999 else "  —  "
            age_c = C_OK if age < 2 else (C_WARN if age < 5 else C_ERR)

            payload = ""
            if dec:
                # Summarise the decoded dict compactly
                items = [f"{k}={v}" for k, v in dec.items() if not k.startswith("_")]
                payload = " ".join(items)
            elif topic in store.last_raw:
                payload = f"<{sz}B raw>"
            else:
                payload = f"{C_DIM}waiting...{RESET}"

            max_payload = width - 62
            if len(_strip_ansi(payload)) > max_payload:
                payload = payload[:max_payload - 1] + "…"

            row = (
                f" {C_DIM}{topic:<36s}{RESET} "
                f"{C_VAL}{cnt:>5d}{RESET} "
                f"{age_c}{age_str:>6s}{RESET} "
                f"{C_DIM}{sz:>5d}{RESET}  "
                f"{payload}"
            )
            lines.append(box_row(row, width))

    lines.append(box_bot(width))
    return lines


def render_telemetry(store: DataStore, width: int) -> list[str]:
    """Panel 2: Telemetry Gauges."""
    lines: list[str] = []
    lines.append(box_top(width, "② Telemetry"))

    with store.lock:
        t = store.telemetry

    if not t:
        lines.append(box_row(f"  {C_DIM}No telemetry received yet{RESET}", width))
        lines.append(box_bot(width))
        return lines

    lat = t.get("lat_deg", 0)
    lon = t.get("lon_deg", 0)
    alt = t.get("alt_m", 0)
    vn = t.get("vn_mps", 0)
    ve = t.get("ve_mps", 0)
    vd = t.get("vd_mps", 0)
    bat = t.get("battery_pct", 0)
    mode = t.get("flight_mode", "?")
    link = t.get("link_quality", 0)
    fmt = t.get("_format", "?")
    speed = math.sqrt(vn**2 + ve**2 + vd**2)

    lines.append(box_row(f"  {C_HEADER}GPS{RESET}  {C_VAL}{lat:+.6f}°{RESET} / {C_VAL}{lon:+.6f}°{RESET}", width))
    lines.append(box_row(f"  {C_HEADER}ALT{RESET}  {C_VAL}{alt:>7.1f} m{RESET}    {C_HEADER}SPD{RESET}  {C_VAL}{speed:>5.1f} m/s{RESET}", width))
    lines.append(box_row(f"  {C_HEADER}VEL{RESET}  N={C_VAL}{vn:+.2f}{RESET} E={C_VAL}{ve:+.2f}{RESET} D={C_VAL}{vd:+.2f}{RESET}", width))

    bat_bar = bar(bat, 100, 20, bat_color)
    lines.append(box_row(f"  {C_HEADER}BAT{RESET}  {bat_bar} {C_VAL}{bat:5.1f}%{RESET}", width))

    mode_c = C_OK if mode in ("GUIDED", "AUTO") else (C_WARN if mode in ("LOITER", "RTL") else C_ERR)
    lines.append(box_row(f"  {C_HEADER}MODE{RESET} {mode_c}{mode}{RESET}  {C_DIM}link={link:.2f} wire={fmt}{RESET}", width))

    lines.append(box_bot(width))
    return lines


def render_fsm(store: DataStore, width: int) -> list[str]:
    """Panel 3: FSM State Panel."""
    lines: list[str] = []
    lines.append(box_top(width, "③ FSM / Supervisor"))

    with store.lock:
        sup = store.supervisor
        fsm_entries = list(store.fsm_log)

    if sup:
        state = sup.get("state", "?")
        action = sup.get("action", "?")
        reason = sup.get("reason", "")
        sc = C_OK if state == "nominal" else (C_WARN if state in ("degraded",) else C_ERR)
        lines.append(box_row(f"  Supervisor: {sc}{state}{RESET}  action={C_VAL}{action}{RESET}  {C_DIM}{reason[:40]}{RESET}", width))
    else:
        lines.append(box_row(f"  {C_DIM}Supervisor: waiting for signal...{RESET}", width))

    lines.append(box_row(f"  {C_HEADER}Transition Log:{RESET}", width))
    if fsm_entries:
        for entry in fsm_entries[-8:]:
            lines.append(box_row(f"    {C_DIM}{entry}{RESET}", width))
    else:
        lines.append(box_row(f"    {C_DIM}(no transitions yet){RESET}", width))

    lines.append(box_bot(width))
    return lines


def render_cmd_log(store: DataStore, width: int) -> list[str]:
    """Panel 4: Command / Ack Log."""
    lines: list[str] = []
    lines.append(box_top(width, "④ Command / Ack Log"))

    header = f" {'Time':<12s} {'Dir':<5s} {'ID':<14s} {'Type':<16s} Detail"
    lines.append(box_row(f"{C_HEADER}{header}{RESET}", width))

    with store.lock:
        entries = list(store.cmd_log)

    if entries:
        for e in entries[-10:]:
            dir_c = C_OK if "CMD" in e["dir"] else fg(147)
            row = (
                f" {C_DIM}{e['ts']:<12s}{RESET} "
                f"{dir_c}{e['dir']:<5s}{RESET} "
                f"{C_VAL}{e['id']:<14s}{RESET} "
                f"{e['type']:<16s} "
                f"{C_DIM}{e['detail']}{RESET}"
            )
            lines.append(box_row(row, width))
    else:
        lines.append(box_row(f"  {C_DIM}(no commands yet){RESET}", width))

    lines.append(box_bot(width))
    return lines


def render_heartbeats(store: DataStore, width: int) -> list[str]:
    """Panel 5: Heartbeat Monitor."""
    lines: list[str] = []
    lines.append(box_top(width, "⑤ Heartbeat Monitor"))

    now = time.time()
    with store.lock:
        hb_snapshot = dict(store.heartbeats)

    for topic, label in HEARTBEAT_COMPONENTS.items():
        last = hb_snapshot.get(topic)
        if last is None:
            status = f"{C_HB_DEAD}● DEAD{RESET}"
            age_str = "never"
        else:
            age = now - last
            if age < 2.0:
                status = f"{C_HB_ALIVE}● ALIVE{RESET}"
            elif age < 5.0:
                status = f"{C_HB_STALE}● STALE{RESET}"
            else:
                status = f"{C_HB_DEAD}● DEAD{RESET}"
            age_str = f"{age:.1f}s"
        lines.append(box_row(f"  {label:<20s} {status:<22s} {C_DIM}age={age_str}{RESET}", width))

    lines.append(box_bot(width))
    return lines


def render_flatbuf_inspector(store: DataStore, width: int, inspect_topic: str) -> list[str]:
    """Panel 6: FlatBuffers Inspector — hex + decoded fields."""
    lines: list[str] = []
    lines.append(box_top(width, f"⑥ FlatBuf Inspector [{inspect_topic}]"))

    with store.lock:
        raw = store.last_raw.get(inspect_topic)
        decoded = store.last_decoded.get(inspect_topic)
        sz = store.last_size.get(inspect_topic, 0)

    if raw is None:
        lines.append(box_row(f"  {C_DIM}No data on '{inspect_topic}' yet{RESET}", width))
        lines.append(box_bot(width))
        return lines

    # Hex dump
    lines.append(box_row(f"  {C_HEADER}Raw ({sz} bytes):{RESET}", width))
    hex_str = _hex_dump(raw, max_bytes=48)
    max_hex = width - 6
    if len(hex_str) > max_hex:
        hex_str = hex_str[:max_hex - 3] + "..."
    lines.append(box_row(f"  {C_DIM}{hex_str}{RESET}", width))

    # Decoded fields
    lines.append(box_row(f"  {C_HEADER}Decoded:{RESET}", width))
    if decoded:
        for k, v in decoded.items():
            if k.startswith("_"):
                continue
            val_str = str(v)
            if len(val_str) > width - 25:
                val_str = val_str[:width - 28] + "..."
            lines.append(box_row(f"    {C_DIM}{k:<20s}{RESET} {C_VAL}{val_str}{RESET}", width))
    else:
        # Try generic JSON
        try:
            obj = json.loads(raw.decode("utf-8"))
            for k, v in obj.items():
                val_str = json.dumps(v, separators=(", ", ": ")) if isinstance(v, (dict, list)) else str(v)
                if len(val_str) > width - 25:
                    val_str = val_str[:width - 28] + "..."
                lines.append(box_row(f"    {C_DIM}{k:<20s}{RESET} {C_VAL}{val_str}{RESET}", width))
        except Exception:
            lines.append(box_row(f"  {C_DIM}(could not decode){RESET}", width))

    lines.append(box_bot(width))
    return lines


# ═══════════════════════════════════════════════════════════════════════════════
# Keyboard input (non-blocking)
# ═══════════════════════════════════════════════════════════════════════════════

class KeyReader:
    """Non-blocking single-char reader for stdin."""

    def __init__(self):
        self._old_settings = None

    def start(self):
        try:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass

    def stop(self):
        if self._old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass

    def read_char(self) -> str | None:
        """Return a single char if available, else None."""
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        except Exception:
            pass
        return None


# ═══════════════════════════════════════════════════════════════════════════════
# Main dashboard loop
# ═══════════════════════════════════════════════════════════════════════════════

INSPECT_TOPICS = list(TOPIC_MAP.keys())


def get_terminal_size() -> tuple[int, int]:
    """Return (cols, rows)."""
    try:
        cols, rows = os.get_terminal_size()
        return cols, rows
    except OSError:
        return 120, 40


def main():
    import zenoh

    parser = argparse.ArgumentParser(description="BROS Flight Stack Command Center")
    parser.add_argument("--refresh", type=float, default=1.0, help="Refresh interval in seconds")
    parser.add_argument("--inspect", default="vehicle/telemetry/fc", help="Default topic for FlatBuf inspector")
    args = parser.parse_args()

    store = DataStore()
    inspect_idx = 0
    for i, t in enumerate(INSPECT_TOPICS):
        if t == args.inspect:
            inspect_idx = i
            break

    # Open Zenoh
    try:
        session = zenoh.open(zenoh.Config.from_env())
    except Exception:
        session = zenoh.open(zenoh.Config())

    def on_sample(sample):
        topic = str(sample.key_expr)
        raw = bytes(sample.payload.to_bytes())
        store.record(topic, raw)

    sub = session.declare_subscriber("**", on_sample)

    # Keyboard
    keys = KeyReader()
    keys.start()

    running = True
    print(HIDE_CURSOR, end="", flush=True)

    try:
        while running:
            ch = keys.read_char()
            if ch:
                if ch in ("q", "Q", "\x03"):  # q or Ctrl-C
                    break
                elif ch == "i":
                    inspect_idx = (inspect_idx + 1) % len(INSPECT_TOPICS)
                elif ch in "123456789":
                    idx = int(ch) - 1
                    if idx < len(INSPECT_TOPICS):
                        inspect_idx = idx

            cols, rows = get_terminal_size()
            W = max(80, cols)

            # Build the frame
            frame_lines: list[str] = []

            # Title bar
            ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
            title = f"  {C_TITLE}BROS Command Center{RESET}  {C_DIM}{ts}{RESET}  {C_DIM}[q]uit [i]nspect-topic{RESET}"
            frame_lines.append(title)
            frame_lines.append("")

            # Panel 1: Topic table (full width)
            frame_lines.extend(render_topic_table(store, W))
            frame_lines.append("")

            # Panel 2+5 side by side if wide enough, else stacked
            half = W // 2
            if W >= 120:
                tel_lines = render_telemetry(store, half)
                hb_lines = render_heartbeats(store, half)
                max_h = max(len(tel_lines), len(hb_lines))
                for i in range(max_h):
                    left = tel_lines[i] if i < len(tel_lines) else " " * half
                    right = hb_lines[i] if i < len(hb_lines) else ""
                    frame_lines.append(left + " " + right)
            else:
                frame_lines.extend(render_telemetry(store, W))
                frame_lines.append("")
                frame_lines.extend(render_heartbeats(store, W))

            frame_lines.append("")

            # Panel 3: FSM
            frame_lines.extend(render_fsm(store, W))
            frame_lines.append("")

            # Panel 4: Command log
            frame_lines.extend(render_cmd_log(store, W))
            frame_lines.append("")

            # Panel 6: FlatBuf inspector
            inspect_topic = INSPECT_TOPICS[inspect_idx]
            frame_lines.extend(render_flatbuf_inspector(store, W, inspect_topic))

            # Render: clear + draw
            output = CLEAR_SCREEN + HOME
            # Truncate to terminal height
            visible = frame_lines[:rows - 1]
            output += "\n".join(visible)
            sys.stdout.write(output)
            sys.stdout.flush()

            time.sleep(args.refresh)

    except KeyboardInterrupt:
        pass
    finally:
        keys.stop()
        print(SHOW_CURSOR, end="", flush=True)
        print(f"\n{RESET}[cmd_center] shutdown")
        session.close()


if __name__ == "__main__":
    main()
