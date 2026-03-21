import argparse
from pathlib import Path
from typing import Any

from guidance.bridge import MISSION_COMMAND_TOPIC
from guidance.contracts import MissionCommand
from guidance.wire_format import (
    WIRE_FORMAT_JSON,
    encode_command,
    mapping_to_command,
)


def mission_command_to_json(command: MissionCommand) -> str:
    return encode_command(command, WIRE_FORMAT_JSON).decode("utf-8")


def command_from_mapping(data: dict[str, Any]) -> MissionCommand:
    return mapping_to_command(data)


def load_command_from_file(file_path: str | Path) -> MissionCommand:
    import json

    data = json.loads(Path(file_path).read_text(encoding="utf-8"))
    return command_from_mapping(data)


def publish_command(
    command: MissionCommand,
    topic: str = MISSION_COMMAND_TOPIC,
    wire_format: str = WIRE_FORMAT_JSON,
) -> bytes:
    import zenoh

    payload = encode_command(command, wire_format)

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
    parser.add_argument(
        "--wire-format",
        default=WIRE_FORMAT_JSON,
        choices=["json", "flexbuffers"],
        help="Payload wire format for published command",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    command = load_command_from_file(args.file)
    payload = publish_command(command, topic=args.topic, wire_format=args.wire_format)
    if args.wire_format == WIRE_FORMAT_JSON:
        print(f"Published mission command: {payload.decode('utf-8')}")
    else:
        print(f"Published mission command ({args.wire_format}): {len(payload)} bytes")


if __name__ == "__main__":
    main()
