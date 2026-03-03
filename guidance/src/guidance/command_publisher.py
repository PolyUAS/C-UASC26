import argparse
from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

from guidance.bridge import MISSION_COMMAND_TOPIC
from guidance.contracts import CommandType, MissionCommand


def mission_command_to_json(command: MissionCommand) -> str:
    payload = asdict(command)
    payload["command_type"] = command.command_type.value
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def command_from_mapping(data: dict[str, Any]) -> MissionCommand:
    return MissionCommand(
        command_id=str(data["command_id"]),
        mission_id=str(data["mission_id"]),
        command_type=CommandType(str(data["command_type"])),
        target=dict(data.get("target", {})),
        timeout_sec=float(data.get("timeout_sec", 2.0)),
        safety_mode=str(data.get("safety_mode", "normal")),
    )


def load_command_from_file(file_path: str | Path) -> MissionCommand:
    data = json.loads(Path(file_path).read_text(encoding="utf-8"))
    return command_from_mapping(data)


def publish_command(command: MissionCommand, topic: str = MISSION_COMMAND_TOPIC) -> str:
    import zenoh

    payload = mission_command_to_json(command)

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    try:
        publisher = session.declare_publisher(topic)
        publisher.put(payload)
    finally:
        session.close()

    return payload


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a mission command over Zenoh")
    parser.add_argument(
        "--file",
        required=True,
        help="Path to JSON mission command file",
    )
    parser.add_argument(
        "--topic",
        default=MISSION_COMMAND_TOPIC,
        help="Zenoh topic to publish mission command payload",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    command = load_command_from_file(args.file)
    payload = publish_command(command, topic=args.topic)
    print(f"Published mission command: {payload}")


if __name__ == "__main__":
    main()
