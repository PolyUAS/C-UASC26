#!/usr/bin/env python3
"""
Zenoh Topic Monitor — BROS Flight Stack Debug Dashboard

Subscribes to ALL Zenoh topics in the BROS topic map and displays a
live console view of data flowing between nodes. Useful for:
  - Verifying heartbeat delivery from sim/SITL to navigation
  - Watching vehicle telemetry updates (FlatBuffers or JSON)
  - Monitoring command dispatch and ack flow
  - Observing supervisor state escalation

Usage:
  python scripts/zenoh_topic_monitor.py            # all topics
  python scripts/zenoh_topic_monitor.py --topics vehicle/**  # glob filter
  python scripts/zenoh_topic_monitor.py --json     # force JSON decode
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from threading import Lock

# ── Topic map ────────────────────────────────────────────────────────────────

TOPIC_MAP = {
    "vehicle/telemetry/fc": "VehicleTelemetry (FlatBuf/JSON)",
    "vehicle/heartbeat/fc": "FC heartbeat",
    "vehicle/heartbeat/camera": "Camera heartbeat",
    "vehicle/heartbeat/gimbal": "Gimbal heartbeat",
    "vehicle/heartbeat/air_unit": "Air-unit heartbeat",
    "guidance/mission/command": "MissionCommand (FlatBuf/JSON)",
    "guidance/mission/ack": "CommandAck (FlatBuf/JSON)",
    "guidance/supervisor/signal": "SupervisorSignal (JSON)",
    "control/out/state": "Control StateVector (FlatBuf)",
    "sim/mujoco/state": "MuJoCo sim state (JSON)",
}


# ── Decoders (best-effort) ───────────────────────────────────────────────────

def _try_decode_vehicle_telemetry(raw: bytes) -> str | None:
    """Attempt FlatBuffers decode, fall back to JSON."""
    try:
        from guidance.generated.vehicle.VehicleTelemetry import (
            VehicleTelemetry as FBTel,
        )

        fb = FBTel.GetRootAs(raw, 0)
        mode = fb.FlightMode()
        if isinstance(mode, bytes):
            mode = mode.decode("utf-8")
        return (
            f"lat={fb.LatDeg():.6f} lon={fb.LonDeg():.6f} alt={fb.AltM():.1f}m "
            f"vn={fb.VnMps():.2f} ve={fb.VeMps():.2f} vd={fb.VdMps():.2f} "
            f"bat={fb.BatteryPct():.0f}% mode={mode} link={fb.LinkQuality():.2f}"
        )
    except Exception:
        pass
    try:
        obj = json.loads(raw.decode("utf-8"))
        return json.dumps(obj, separators=(", ", ": "))
    except Exception:
        return None


def _try_decode_command(raw: bytes) -> str | None:
    try:
        from guidance.wire_format import WIRE_FORMAT_FLATBUFFERS, decode_command

        cmd = decode_command(raw, WIRE_FORMAT_FLATBUFFERS)
        return f"id={cmd.command_id} type={cmd.command_type.value} target={cmd.target}"
    except Exception:
        pass
    try:
        obj = json.loads(raw.decode("utf-8"))
        return f"id={obj.get('command_id')} type={obj.get('command_type')} target={obj.get('target')}"
    except Exception:
        return None


def _try_decode_ack(raw: bytes) -> str | None:
    try:
        from guidance.wire_format import WIRE_FORMAT_FLATBUFFERS, decode_ack

        ack = decode_ack(raw, WIRE_FORMAT_FLATBUFFERS)
        return f"id={ack.command_id} status={ack.status.value} reason={ack.reason}"
    except Exception:
        pass
    try:
        obj = json.loads(raw.decode("utf-8"))
        return f"id={obj.get('command_id')} status={obj.get('status')} reason={obj.get('reason')}"
    except Exception:
        return None


def _try_decode_supervisor(raw: bytes) -> str | None:
    try:
        obj = json.loads(raw.decode("utf-8"))
        return f"state={obj.get('state')} action={obj.get('action')} reason={obj.get('reason')}"
    except Exception:
        return None


def _try_decode_sim_state(raw: bytes) -> str | None:
    try:
        obj = json.loads(raw.decode("utf-8"))
        pos = obj.get("position", {})
        vel = obj.get("velocity_ned", {})
        return (
            f"lat={pos.get('lat_deg', 0):.6f} lon={pos.get('lon_deg', 0):.6f} "
            f"alt={pos.get('alt_m', 0):.1f}m bat={obj.get('battery_pct', 0):.0f}% "
            f"mode={obj.get('flight_mode', '?')}"
        )
    except Exception:
        return None


DECODERS: dict[str, callable] = {
    "vehicle/telemetry/fc": _try_decode_vehicle_telemetry,
    "guidance/mission/command": _try_decode_command,
    "guidance/mission/ack": _try_decode_ack,
    "guidance/supervisor/signal": _try_decode_supervisor,
    "sim/mujoco/state": _try_decode_sim_state,
}


# ── Stats tracker ────────────────────────────────────────────────────────────

class TopicStats:
    def __init__(self):
        self.lock = Lock()
        self.msg_counts: dict[str, int] = defaultdict(int)
        self.last_seen: dict[str, float] = {}
        self.last_payload: dict[str, str] = {}
        self.last_raw_size: dict[str, int] = {}

    def record(self, topic: str, decoded: str, raw_size: int):
        with self.lock:
            self.msg_counts[topic] += 1
            self.last_seen[topic] = time.time()
            self.last_payload[topic] = decoded
            self.last_raw_size[topic] = raw_size

    def snapshot(self) -> list[dict]:
        with self.lock:
            now = time.time()
            rows = []
            for topic in sorted(TOPIC_MAP.keys()):
                count = self.msg_counts.get(topic, 0)
                age = now - self.last_seen[topic] if topic in self.last_seen else float("inf")
                rows.append({
                    "topic": topic,
                    "label": TOPIC_MAP.get(topic, ""),
                    "count": count,
                    "age_s": age,
                    "size": self.last_raw_size.get(topic, 0),
                    "payload": self.last_payload.get(topic, "—"),
                })
            return rows


# ── Main ─────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description="BROS Zenoh topic monitor")
    p.add_argument(
        "--topics",
        default="**",
        help="Zenoh key-expression glob (default: ** = all)",
    )
    p.add_argument(
        "--json",
        action="store_true",
        help="Force JSON decoding (skip FlatBuffers attempt)",
    )
    p.add_argument(
        "--dashboard",
        action="store_true",
        help="Continuous dashboard mode (clears screen each refresh)",
    )
    p.add_argument(
        "--refresh",
        type=float,
        default=1.0,
        help="Dashboard refresh interval in seconds (only with --dashboard)",
    )
    return p.parse_args()


def _decode_sample(topic: str, raw: bytes, force_json: bool) -> str:
    if not force_json:
        decoder = DECODERS.get(topic)
        if decoder:
            result = decoder(raw)
            if result:
                return result

    # Heartbeat topics — just show the byte
    if "heartbeat" in topic:
        return f"beat ({len(raw)}B)"

    # Generic JSON attempt
    try:
        obj = json.loads(raw.decode("utf-8"))
        return json.dumps(obj, separators=(", ", ": "))
    except Exception:
        return f"<{len(raw)} bytes>"


def main():
    import zenoh

    args = _parse_args()
    stats = TopicStats()

    try:
        session = zenoh.open(zenoh.Config.from_env())
    except Exception:
        session = zenoh.open(zenoh.Config())

    def _on_sample(sample):
        topic = str(sample.key_expr)
        raw = bytes(sample.payload.to_bytes())
        decoded = _decode_sample(topic, raw, args.json)
        stats.record(topic, decoded, len(raw))

        if not args.dashboard:
            ts = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]
            print(f"[{ts}] {topic:40s} | {decoded}")

    sub = session.declare_subscriber(args.topics, _on_sample)

    print(f"[zenoh-monitor] subscribed to '{args.topics}' — waiting for messages...")

    try:
        if args.dashboard:
            while True:
                time.sleep(args.refresh)
                rows = stats.snapshot()
                # Clear screen
                print("\033[2J\033[H", end="")
                ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
                print(f"╔══ BROS Zenoh Topic Monitor ══  {ts}")
                print(f"╠{'═' * 42}╦{'═' * 7}╦{'═' * 7}╦{'═' * 60}")
                print(f"║ {'Topic':<40s} ║ {'Count':>5s} ║ {'Age':>5s} ║ Last Payload")
                print(f"╠{'═' * 42}╬{'═' * 7}╬{'═' * 7}╬{'═' * 60}")
                for r in rows:
                    age_str = f"{r['age_s']:.1f}s" if r["age_s"] < 9999 else "never"
                    payload_trunc = r["payload"][:58]
                    print(
                        f"║ {r['topic']:<40s} ║ {r['count']:>5d} ║ {age_str:>5s} ║ {payload_trunc}"
                    )
                print(f"╚{'═' * 42}╩{'═' * 7}╩{'═' * 7}╩{'═' * 60}")
        else:
            while True:
                time.sleep(60)
    except KeyboardInterrupt:
        print("\n[zenoh-monitor] shutting down")
    finally:
        session.close()


if __name__ == "__main__":
    main()
