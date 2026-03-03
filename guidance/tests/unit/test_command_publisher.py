import json

from guidance.command_publisher import (
    command_from_mapping,
    load_command_from_file,
    mission_command_to_json,
)
from guidance.contracts import CommandType


def test_command_from_mapping_builds_command():
    payload = {
        "command_id": "cmd-201",
        "mission_id": "mission-drop",
        "command_type": "rtl",
    }

    command = command_from_mapping(payload)

    assert command.command_id == "cmd-201"
    assert command.command_type == CommandType.RTL
    assert command.timeout_sec == 2.0


def test_mission_command_to_json_uses_string_command_type():
    command = command_from_mapping(
        {
            "command_id": "cmd-202",
            "mission_id": "mission-waypoint",
            "command_type": "arm",
        }
    )

    payload = json.loads(mission_command_to_json(command))

    assert payload["command_type"] == "arm"
    assert payload["command_id"] == "cmd-202"


def test_load_command_from_file_reads_fixture(tmp_path):
    fixture = tmp_path / "command.json"
    fixture.write_text(
        json.dumps(
            {
                "command_id": "cmd-203",
                "mission_id": "mission-waypoint",
                "command_type": "takeoff",
                "target": {"altitude_m": 42.0},
            }
        ),
        encoding="utf-8",
    )

    command = load_command_from_file(fixture)

    assert command.command_type == CommandType.TAKEOFF
    assert command.target["altitude_m"] == 42.0
