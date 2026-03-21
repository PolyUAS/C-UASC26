#!/usr/bin/env python3
"""
Zenoh Command Injector — publish test MissionCommands to the flight stack.

Provides pre-built command sequences for testing the full sim pipeline:
  arm → takeoff → goto → hold → rtl → land

Usage:
  python scripts/zenoh_inject_command.py arm
  python scripts/zenoh_inject_command.py takeoff --alt 30
  python scripts/zenoh_inject_command.py goto --lat 40.7128 --lon -74.0060 --alt 50
  python scripts/zenoh_inject_command.py sequence   # runs full arm→takeoff→goto→rtl sequence
  python scripts/zenoh_inject_command.py --list      # show available commands
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid


MISSION_COMMAND_TOPIC = "guidance/mission/command"
MISSION_ACK_TOPIC = "guidance/mission/ack"


def _make_command(
    command_type: str,
    target: dict | None = None,
    timeout_sec: float = 5.0,
    mission_id: str = "test-mission",
) -> dict:
    return {
        "command_id": f"inject-{uuid.uuid4().hex[:8]}",
        "mission_id": mission_id,
        "command_type": command_type,
        "target": target or {},
        "timeout_sec": timeout_sec,
        "safety_mode": "normal",
    }


PRESET_COMMANDS = {
    "arm": lambda a: _make_command("arm"),
    "takeoff": lambda a: _make_command("takeoff", {"altitude_m": a.alt}),
    "goto": lambda a: _make_command("goto", {"lat_deg": a.lat, "lon_deg": a.lon, "alt_m": a.alt}),
    "hold": lambda a: _make_command("hold"),
    "rtl": lambda a: _make_command("rtl"),
    "land": lambda a: _make_command("land"),
    "payload_release": lambda a: _make_command("payload_release"),
    "payload_secure": lambda a: _make_command("payload_secure"),
    "abort": lambda a: _make_command("mission_abort"),
}

SEQUENCE_STEPS = [
    ("arm", 2.0),
    ("takeoff", 3.0),
    ("goto", 5.0),
    ("hold", 3.0),
    ("rtl", 5.0),
]


def _publish_and_wait_ack(session, pub, ack_sub, command: dict, timeout: float = 5.0) -> dict | None:
    """Publish command and wait for matching ack."""
    payload = json.dumps(command, separators=(",", ":"), sort_keys=True).encode("utf-8")
    pub.put(payload)
    cmd_id = command["command_id"]
    print(f"  → Published {command['command_type']:20s}  id={cmd_id}")

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            sample = ack_sub.try_recv()
            if sample is None:
                time.sleep(0.05)
                continue
            ack_raw = sample.payload.to_string()
            ack = json.loads(ack_raw)
            if ack.get("command_id") == cmd_id:
                print(f"  ← Ack: status={ack.get('status')} reason={ack.get('reason', '-')}")
                return ack
        except Exception:
            time.sleep(0.05)

    print(f"  ← Ack: TIMEOUT after {timeout}s")
    return None


def _run_sequence(session, pub, ack_sub, args):
    """Run the preset arm→takeoff→goto→rtl sequence."""
    print(f"[inject] Running test sequence ({len(SEQUENCE_STEPS)} steps)\n")

    for step_name, delay in SEQUENCE_STEPS:
        builder = PRESET_COMMANDS[step_name]
        command = builder(args)
        _publish_and_wait_ack(session, pub, ack_sub, command)
        print(f"  ... waiting {delay}s before next step\n")
        time.sleep(delay)

    print("[inject] Sequence complete.")


def _parse_args():
    p = argparse.ArgumentParser(description="BROS Zenoh command injector")
    p.add_argument(
        "command",
        nargs="?",
        default="sequence",
        choices=list(PRESET_COMMANDS.keys()) + ["sequence"],
        help="Command type to inject (default: sequence)",
    )
    p.add_argument("--lat", type=float, default=40.7128, help="Target latitude (goto)")
    p.add_argument("--lon", type=float, default=-74.0060, help="Target longitude (goto)")
    p.add_argument("--alt", type=float, default=30.0, help="Target altitude")
    p.add_argument("--list", action="store_true", help="List available commands")
    p.add_argument(
        "--topic",
        default=MISSION_COMMAND_TOPIC,
        help="Zenoh topic to publish to",
    )
    return p.parse_args()


def main():
    import zenoh

    args = _parse_args()

    if args.list:
        print("Available commands:")
        for name in PRESET_COMMANDS:
            print(f"  {name}")
        print("  sequence  (arm → takeoff → goto → hold → rtl)")
        return

    try:
        session = zenoh.open(zenoh.Config.from_env())
    except Exception:
        session = zenoh.open(zenoh.Config())

    pub = session.declare_publisher(args.topic)
    ack_sub = session.declare_subscriber(MISSION_ACK_TOPIC)

    # Allow Zenoh discovery
    time.sleep(0.5)

    if args.command == "sequence":
        _run_sequence(session, pub, ack_sub, args)
    else:
        builder = PRESET_COMMANDS[args.command]
        command = builder(args)
        _publish_and_wait_ack(session, pub, ack_sub, command)

    session.close()


if __name__ == "__main__":
    main()
