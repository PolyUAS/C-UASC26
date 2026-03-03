import json

from guidance.contracts import AckStatus, CommandAck, CommandType, MissionCommand
from guidance.replay import JsonlReplayLogger


def test_jsonl_replay_logger_writes_dispatch_event(tmp_path):
    log_path = tmp_path / "replay" / "events.jsonl"
    logger = JsonlReplayLogger(log_path)

    command = MissionCommand(
        command_id="cmd-10",
        mission_id="mission-circuit",
        command_type=CommandType.TAKEOFF,
        target={"altitude_m": 30.0},
    )
    ack = CommandAck(command_id="cmd-10", status=AckStatus.ACCEPTED)

    logger.log_dispatch(command, ack)

    lines = log_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1

    payload = json.loads(lines[0])
    assert payload["kind"] == "dispatch"
    assert payload["command"]["command_id"] == "cmd-10"
    assert payload["command"]["command_type"] == "takeoff"
    assert payload["ack"]["status"] == "accepted"
