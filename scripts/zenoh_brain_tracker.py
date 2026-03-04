#!/usr/bin/env python3
"""
Zenoh Brain State Tracker — monitors drone FSM state transitions.

Subscribes to telemetry and supervisor signals to infer the flight-brain
state and logs every transition with timestamps. Useful for verifying
mission FSM logic during sim runs.

Usage:
  python scripts/zenoh_brain_tracker.py
  python scripts/zenoh_brain_tracker.py --log brain_log.jsonl
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path


VEHICLE_TELEMETRY_TOPIC = "vehicle/telemetry/fc"
SUPERVISOR_SIGNAL_TOPIC = "guidance/supervisor/signal"
MISSION_COMMAND_TOPIC = "guidance/mission/command"
MISSION_ACK_TOPIC = "guidance/mission/ack"
HEARTBEAT_TOPICS = [
    "vehicle/heartbeat/fc",
    "vehicle/heartbeat/camera",
    "vehicle/heartbeat/gimbal",
    "vehicle/heartbeat/air_unit",
]


class BrainStateTracker:
    """Tracks observable drone brain states from Zenoh traffic."""

    def __init__(self, log_path: str | None = None):
        self.flight_mode: str = "UNKNOWN"
        self.supervisor_state: str = "unknown"
        self.supervisor_action: str = "unknown"
        self.last_command: str = "—"
        self.last_ack_status: str = "—"
        self.battery_pct: float = 0.0
        self.altitude_m: float = 0.0
        self.heartbeat_ages: dict[str, float] = {}
        self.transition_log: list[dict] = []
        self.log_file = None
        if log_path:
            p = Path(log_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = p.open("a", encoding="utf-8")

    def _record_transition(self, event: str, detail: str):
        ts = datetime.now(timezone.utc).isoformat()
        entry = {
            "ts": ts,
            "event": event,
            "detail": detail,
            "flight_mode": self.flight_mode,
            "supervisor": f"{self.supervisor_state}/{self.supervisor_action}",
            "battery": self.battery_pct,
            "alt": self.altitude_m,
        }
        self.transition_log.append(entry)
        line = (
            f"[{ts[11:23]}] {event:24s} | {detail:50s} | "
            f"mode={self.flight_mode} sup={self.supervisor_state}/{self.supervisor_action} "
            f"bat={self.battery_pct:.0f}% alt={self.altitude_m:.1f}m"
        )
        print(line)
        if self.log_file:
            self.log_file.write(json.dumps(entry, separators=(",", ":")) + "\n")
            self.log_file.flush()

    def on_telemetry(self, raw: bytes):
        """Decode vehicle telemetry and detect mode changes."""
        flight_mode = None
        battery = None
        alt = None

        # Try FlatBuffers first
        try:
            from guidance.generated.vehicle.VehicleTelemetry import (
                VehicleTelemetry as FBTel,
            )
            fb = FBTel.GetRootAs(raw, 0)
            mode = fb.FlightMode()
            if isinstance(mode, bytes):
                mode = mode.decode("utf-8")
            flight_mode = mode
            battery = fb.BatteryPct()
            alt = fb.AltM()
        except Exception:
            pass

        # Fall back to JSON
        if flight_mode is None:
            try:
                obj = json.loads(raw.decode("utf-8"))
                flight_mode = obj.get("flight_mode", self.flight_mode)
                battery = obj.get("battery_pct", self.battery_pct)
                alt = obj.get("alt_m", self.altitude_m)
            except Exception:
                return

        if battery is not None:
            self.battery_pct = battery
        if alt is not None:
            self.altitude_m = alt

        if flight_mode and flight_mode != self.flight_mode:
            old = self.flight_mode
            self.flight_mode = flight_mode
            self._record_transition("FLIGHT_MODE_CHANGE", f"{old} → {flight_mode}")

    def on_supervisor_signal(self, raw: bytes):
        try:
            obj = json.loads(raw.decode("utf-8"))
        except Exception:
            return
        new_state = obj.get("state", self.supervisor_state)
        new_action = obj.get("action", self.supervisor_action)
        reason = obj.get("reason", "")

        if new_state != self.supervisor_state or new_action != self.supervisor_action:
            old = f"{self.supervisor_state}/{self.supervisor_action}"
            self.supervisor_state = new_state
            self.supervisor_action = new_action
            self._record_transition(
                "SUPERVISOR_CHANGE",
                f"{old} → {new_state}/{new_action} ({reason})",
            )

    def on_command(self, raw: bytes):
        try:
            obj = json.loads(raw.decode("utf-8"))
        except Exception:
            # Try FlatBuffers
            try:
                from guidance.wire_format import WIRE_FORMAT_FLATBUFFERS, decode_command
                cmd = decode_command(raw, WIRE_FORMAT_FLATBUFFERS)
                self.last_command = cmd.command_type.value
                self._record_transition("COMMAND_DISPATCHED", f"type={cmd.command_type.value} id={cmd.command_id}")
                return
            except Exception:
                return
        self.last_command = obj.get("command_type", "?")
        self._record_transition(
            "COMMAND_DISPATCHED",
            f"type={obj.get('command_type')} id={obj.get('command_id')}",
        )

    def on_ack(self, raw: bytes):
        try:
            obj = json.loads(raw.decode("utf-8"))
        except Exception:
            try:
                from guidance.wire_format import WIRE_FORMAT_FLATBUFFERS, decode_ack
                ack = decode_ack(raw, WIRE_FORMAT_FLATBUFFERS)
                self.last_ack_status = ack.status.value
                self._record_transition("COMMAND_ACK", f"status={ack.status.value} id={ack.command_id}")
                return
            except Exception:
                return
        self.last_ack_status = obj.get("status", "?")
        self._record_transition(
            "COMMAND_ACK",
            f"status={obj.get('status')} id={obj.get('command_id')} reason={obj.get('reason', '-')}",
        )

    def on_heartbeat(self, topic: str):
        self.heartbeat_ages[topic] = time.time()


def main():
    import zenoh

    parser = argparse.ArgumentParser(description="BROS brain state tracker")
    parser.add_argument("--log", default=None, help="Path to JSONL log file")
    args = parser.parse_args()

    tracker = BrainStateTracker(log_path=args.log)

    try:
        session = zenoh.open(zenoh.Config.from_env())
    except Exception:
        session = zenoh.open(zenoh.Config())

    session.declare_subscriber(
        VEHICLE_TELEMETRY_TOPIC,
        lambda s: tracker.on_telemetry(bytes(s.payload.to_bytes())),
    )
    session.declare_subscriber(
        SUPERVISOR_SIGNAL_TOPIC,
        lambda s: tracker.on_supervisor_signal(bytes(s.payload.to_bytes())),
    )
    session.declare_subscriber(
        MISSION_COMMAND_TOPIC,
        lambda s: tracker.on_command(bytes(s.payload.to_bytes())),
    )
    session.declare_subscriber(
        MISSION_ACK_TOPIC,
        lambda s: tracker.on_ack(bytes(s.payload.to_bytes())),
    )
    for hb_topic in HEARTBEAT_TOPICS:
        session.declare_subscriber(
            hb_topic,
            lambda s, t=hb_topic: tracker.on_heartbeat(t),
        )

    print("[brain-tracker] Monitoring flight-brain state transitions...")
    print(f"{'─' * 120}")

    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        print(f"\n[brain-tracker] {len(tracker.transition_log)} transitions recorded.")
    finally:
        session.close()


if __name__ == "__main__":
    main()
