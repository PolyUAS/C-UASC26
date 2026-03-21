from guidance.contracts import (
    AckStatus,
    CommandAck,
    CommandType,
    MissionCommand,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
    VehicleTelemetry,
)
from guidance.wire_format import (
    WIRE_FORMAT_FLATBUFFERS,
    WIRE_FORMAT_FLEXBUFFERS,
    WIRE_FORMAT_JSON,
    decode_ack,
    decode_command,
    decode_supervisor_signal,
    decode_vehicle_telemetry_flatbuf,
    encode_ack,
    encode_command,
    encode_supervisor_signal,
    encode_vehicle_telemetry_flatbuf,
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


def test_json_supervisor_signal_roundtrip():
    signal = SupervisorSignal(
        timestamp_ms=700,
        state=SupervisorState.CRITICAL,
        action=SupervisorAction.RTL,
        reason="critical_link",
    )

    encoded = encode_supervisor_signal(signal, WIRE_FORMAT_JSON)
    decoded = decode_supervisor_signal(encoded, WIRE_FORMAT_JSON)

    assert decoded.timestamp_ms == 700
    assert decoded.state == SupervisorState.CRITICAL
    assert decoded.action == SupervisorAction.RTL


def test_flexbuffers_supervisor_signal_roundtrip():
    signal = SupervisorSignal(
        timestamp_ms=701,
        state=SupervisorState.DEGRADED,
        action=SupervisorAction.HOLD,
        reason="degraded_link",
    )

    encoded = encode_supervisor_signal(signal, WIRE_FORMAT_FLEXBUFFERS)
    decoded = decode_supervisor_signal(encoded, WIRE_FORMAT_FLEXBUFFERS)

    assert decoded.timestamp_ms == 701
    assert decoded.state == SupervisorState.DEGRADED
    assert decoded.action == SupervisorAction.HOLD


# ─────────────────────────────────────────────────────────────────────────────
# FlatBuffers roundtrip tests
# ─────────────────────────────────────────────────────────────────────────────

def test_flatbuffers_command_roundtrip():
    command = _sample_command()

    encoded = encode_command(command, WIRE_FORMAT_FLATBUFFERS)
    decoded = decode_command(encoded, WIRE_FORMAT_FLATBUFFERS)

    assert decoded.command_id == command.command_id
    assert decoded.mission_id == command.mission_id
    assert decoded.command_type == command.command_type
    assert decoded.target.get("lat_deg") == 32.1
    assert decoded.target.get("lon_deg") == -117.2
    assert abs(decoded.target.get("alt_m", 0) - 40.0) < 0.01
    assert abs(decoded.timeout_sec - 3.0) < 0.01
    assert decoded.safety_mode == "strict"


def test_flatbuffers_ack_roundtrip():
    ack = CommandAck(command_id="cmd-fb-1", status=AckStatus.COMPLETED, reason="done")

    encoded = encode_ack(ack, WIRE_FORMAT_FLATBUFFERS)
    decoded = decode_ack(encoded, WIRE_FORMAT_FLATBUFFERS)

    assert decoded.command_id == "cmd-fb-1"
    assert decoded.status == AckStatus.COMPLETED
    assert decoded.reason == "done"


def test_flatbuffers_supervisor_signal_roundtrip():
    signal = SupervisorSignal(
        timestamp_ms=800,
        state=SupervisorState.FAILSAFE,
        action=SupervisorAction.ABORT,
        reason="total_loss",
    )

    encoded = encode_supervisor_signal(signal, WIRE_FORMAT_FLATBUFFERS)
    decoded = decode_supervisor_signal(encoded, WIRE_FORMAT_FLATBUFFERS)

    assert decoded.timestamp_ms == 800
    assert decoded.state == SupervisorState.FAILSAFE
    assert decoded.action == SupervisorAction.ABORT
    assert decoded.reason == "total_loss"


def test_flatbuffers_vehicle_telemetry_roundtrip():
    telem = VehicleTelemetry(
        timestamp_ms=12345,
        lat_deg=40.7128,
        lon_deg=-74.0060,
        alt_m=100.5,
        vn_mps=5.0,
        ve_mps=3.0,
        vd_mps=-1.0,
        battery_pct=87.5,
        flight_mode="GUIDED",
        link_quality=0.95,
    )

    encoded = encode_vehicle_telemetry_flatbuf(telem)
    decoded = decode_vehicle_telemetry_flatbuf(encoded)

    assert decoded.timestamp_ms == 12345
    assert abs(decoded.lat_deg - 40.7128) < 1e-6
    assert abs(decoded.lon_deg - (-74.0060)) < 1e-6
    assert abs(decoded.alt_m - 100.5) < 1e-3
    assert abs(decoded.vn_mps - 5.0) < 0.01
    assert abs(decoded.battery_pct - 87.5) < 0.01
    assert decoded.flight_mode == "GUIDED"
    assert abs(decoded.link_quality - 0.95) < 0.01


def test_flatbuffers_ack_no_reason():
    """Ack with None reason should roundtrip cleanly."""
    ack = CommandAck(command_id="cmd-fb-nr", status=AckStatus.ACCEPTED, reason=None)

    encoded = encode_ack(ack, WIRE_FORMAT_FLATBUFFERS)
    decoded = decode_ack(encoded, WIRE_FORMAT_FLATBUFFERS)

    assert decoded.command_id == "cmd-fb-nr"
    assert decoded.status == AckStatus.ACCEPTED
    # reason encodes as "" which decodes back to None (empty → None)
