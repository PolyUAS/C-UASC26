from guidance.contracts import AckStatus, CommandAck, CommandType, MissionCommand
from guidance.wire_format import (
    WIRE_FORMAT_FLEXBUFFERS,
    WIRE_FORMAT_JSON,
    decode_ack,
    decode_command,
    encode_ack,
    encode_command,
)


def _sample_command() -> MissionCommand:
    return MissionCommand(
        command_id="cmd-wire-1",
        mission_id="mission-waypoint",
        command_type=CommandType.GOTO,
        target={"lat_deg": 32.1, "lon_deg": -117.2, "alt_m": 40.0},
        timeout_sec=3.0,
        safety_mode="strict",
    )


def test_json_command_roundtrip():
    command = _sample_command()

    encoded = encode_command(command, WIRE_FORMAT_JSON)
    decoded = decode_command(encoded, WIRE_FORMAT_JSON)

    assert decoded.command_id == command.command_id
    assert decoded.command_type == command.command_type
    assert decoded.target["alt_m"] == 40.0


def test_flexbuffers_command_roundtrip():
    command = _sample_command()

    encoded = encode_command(command, WIRE_FORMAT_FLEXBUFFERS)
    decoded = decode_command(encoded, WIRE_FORMAT_FLEXBUFFERS)

    assert decoded.command_id == command.command_id
    assert decoded.command_type == command.command_type
    assert decoded.target["lat_deg"] == 32.1


def test_json_ack_roundtrip():
    ack = CommandAck(command_id="cmd-wire-2", status=AckStatus.COMPLETED, reason="ok")

    encoded = encode_ack(ack, WIRE_FORMAT_JSON)
    decoded = decode_ack(encoded, WIRE_FORMAT_JSON)

    assert decoded.command_id == ack.command_id
    assert decoded.status == AckStatus.COMPLETED
    assert decoded.reason == "ok"


def test_flexbuffers_ack_roundtrip():
    ack = CommandAck(command_id="cmd-wire-3", status=AckStatus.FAILED, reason="timeout")

    encoded = encode_ack(ack, WIRE_FORMAT_FLEXBUFFERS)
    decoded = decode_ack(encoded, WIRE_FORMAT_FLEXBUFFERS)

    assert decoded.command_id == ack.command_id
    assert decoded.status == AckStatus.FAILED
    assert decoded.reason == "timeout"
