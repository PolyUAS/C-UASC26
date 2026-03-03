from dataclasses import asdict
import json

from guidance.contracts import (
    AckStatus,
    CommandAck,
    CommandType,
    MissionCommand,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
)


WIRE_FORMAT_JSON = "json"
WIRE_FORMAT_FLEXBUFFERS = "flexbuffers"
SUPPORTED_WIRE_FORMATS = {WIRE_FORMAT_JSON, WIRE_FORMAT_FLEXBUFFERS}


def _require_supported(wire_format: str):
    if wire_format not in SUPPORTED_WIRE_FORMATS:
        raise ValueError(f"unsupported_wire_format:{wire_format}")


def _require_flexbuffers():
    try:
        from flatbuffers import flexbuffers
    except Exception as error:
        return None
    return flexbuffers


def command_to_mapping(command: MissionCommand) -> dict:
    payload = asdict(command)
    payload["command_type"] = command.command_type.value
    return payload


def mapping_to_command(payload: dict) -> MissionCommand:
    return MissionCommand(
        command_id=str(payload["command_id"]),
        mission_id=str(payload["mission_id"]),
        command_type=CommandType(str(payload["command_type"])),
        target=dict(payload.get("target", {})),
        timeout_sec=float(payload.get("timeout_sec", 2.0)),
        safety_mode=str(payload.get("safety_mode", "normal")),
    )


def ack_to_mapping(ack: CommandAck) -> dict:
    payload = asdict(ack)
    payload["status"] = ack.status.value
    return payload


def mapping_to_ack(payload: dict) -> CommandAck:
    return CommandAck(
        command_id=str(payload["command_id"]),
        status=AckStatus(str(payload["status"])),
        reason=payload.get("reason"),
    )


def supervisor_signal_to_mapping(signal: SupervisorSignal) -> dict:
    payload = asdict(signal)
    payload["state"] = signal.state.value
    payload["action"] = signal.action.value
    return payload


def mapping_to_supervisor_signal(payload: dict) -> SupervisorSignal:
    return SupervisorSignal(
        timestamp_ms=int(payload["timestamp_ms"]),
        state=SupervisorState(str(payload["state"])),
        action=SupervisorAction(str(payload["action"])),
        reason=str(payload.get("reason", "")),
    )


def encode_command(command: MissionCommand, wire_format: str) -> bytes:
    _require_supported(wire_format)
    payload = command_to_mapping(command)

    if wire_format == WIRE_FORMAT_JSON:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return flexbuffers.Dumps(payload)


def decode_command(raw_payload: bytes | str, wire_format: str) -> MissionCommand:
    _require_supported(wire_format)

    if wire_format == WIRE_FORMAT_JSON:
        text = raw_payload if isinstance(raw_payload, str) else raw_payload.decode("utf-8")
        payload = json.loads(text)
        return mapping_to_command(payload)

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        text = raw_payload if isinstance(raw_payload, str) else raw_payload.decode("utf-8")
        payload = json.loads(text)
        return mapping_to_command(payload)
    data = raw_payload.encode("utf-8") if isinstance(raw_payload, str) else raw_payload
    payload = dict(flexbuffers.Loads(data))
    return mapping_to_command(payload)


def encode_ack(ack: CommandAck, wire_format: str) -> bytes:
    _require_supported(wire_format)
    payload = ack_to_mapping(ack)

    if wire_format == WIRE_FORMAT_JSON:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return flexbuffers.Dumps(payload)


def decode_ack(raw_payload: bytes | str, wire_format: str) -> CommandAck:
    _require_supported(wire_format)

    if wire_format == WIRE_FORMAT_JSON:
        text = raw_payload if isinstance(raw_payload, str) else raw_payload.decode("utf-8")
        payload = json.loads(text)
        return mapping_to_ack(payload)

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        text = raw_payload if isinstance(raw_payload, str) else raw_payload.decode("utf-8")
        payload = json.loads(text)
        return mapping_to_ack(payload)
    data = raw_payload.encode("utf-8") if isinstance(raw_payload, str) else raw_payload
    payload = dict(flexbuffers.Loads(data))
    return mapping_to_ack(payload)


def encode_supervisor_signal(signal: SupervisorSignal, wire_format: str) -> bytes:
    _require_supported(wire_format)
    payload = supervisor_signal_to_mapping(signal)

    if wire_format == WIRE_FORMAT_JSON:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return flexbuffers.Dumps(payload)


def decode_supervisor_signal(raw_payload: bytes | str, wire_format: str) -> SupervisorSignal:
    _require_supported(wire_format)

    if wire_format == WIRE_FORMAT_JSON:
        text = raw_payload if isinstance(raw_payload, str) else raw_payload.decode("utf-8")
        payload = json.loads(text)
        return mapping_to_supervisor_signal(payload)

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        text = raw_payload if isinstance(raw_payload, str) else raw_payload.decode("utf-8")
        payload = json.loads(text)
        return mapping_to_supervisor_signal(payload)
    data = raw_payload.encode("utf-8") if isinstance(raw_payload, str) else raw_payload
    payload = dict(flexbuffers.Loads(data))
    return mapping_to_supervisor_signal(payload)
