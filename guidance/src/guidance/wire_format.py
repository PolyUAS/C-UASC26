from dataclasses import asdict
import json
import os

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


WIRE_FORMAT_JSON = "json"
WIRE_FORMAT_FLEXBUFFERS = "flexbuffers"
WIRE_FORMAT_FLATBUFFERS = "flatbuffers"
SUPPORTED_WIRE_FORMATS = {WIRE_FORMAT_JSON, WIRE_FORMAT_FLEXBUFFERS, WIRE_FORMAT_FLATBUFFERS}


# ── FlatBuffers enum mapping tables ──────────────────────────────────────────
# Map domain enums ↔ FlatBuffers integer constants.

_COMMAND_TYPE_TO_FB: dict[str, int] = {
    "arm": 0, "takeoff": 1, "goto": 2, "hold": 3, "rtl": 4,
    "land": 5, "payload_release": 6, "payload_secure": 7,
    "camera_point": 8, "camera_track": 9, "mission_abort": 10,
}
_FB_TO_COMMAND_TYPE: dict[int, str] = {v: k for k, v in _COMMAND_TYPE_TO_FB.items()}

_ACK_STATUS_TO_FB: dict[str, int] = {
    "accepted": 0, "rejected": 1, "in_progress": 2, "completed": 3, "failed": 4,
}
_FB_TO_ACK_STATUS: dict[int, str] = {v: k for k, v in _ACK_STATUS_TO_FB.items()}

_SUPERVISOR_STATE_TO_FB: dict[str, int] = {
    "nominal": 0, "degraded": 1, "critical": 2, "failsafe": 3,
}
_FB_TO_SUPERVISOR_STATE: dict[int, str] = {v: k for k, v in _SUPERVISOR_STATE_TO_FB.items()}

_SUPERVISOR_ACTION_TO_FB: dict[str, int] = {
    "continue": 0, "hold": 1, "rtl": 2, "abort": 3,
}
_FB_TO_SUPERVISOR_ACTION: dict[int, str] = {v: k for k, v in _SUPERVISOR_ACTION_TO_FB.items()}


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

    if wire_format == WIRE_FORMAT_FLATBUFFERS:
        return _encode_command_flatbuf(command)

    payload = command_to_mapping(command)

    if wire_format == WIRE_FORMAT_JSON:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return flexbuffers.Dumps(payload)


def decode_command(raw_payload: bytes | str, wire_format: str) -> MissionCommand:
    _require_supported(wire_format)

    if wire_format == WIRE_FORMAT_FLATBUFFERS:
        data = raw_payload if isinstance(raw_payload, bytes) else raw_payload.encode("utf-8")
        return _decode_command_flatbuf(data)

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

    if wire_format == WIRE_FORMAT_FLATBUFFERS:
        return _encode_ack_flatbuf(ack)

    payload = ack_to_mapping(ack)

    if wire_format == WIRE_FORMAT_JSON:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return flexbuffers.Dumps(payload)


def decode_ack(raw_payload: bytes | str, wire_format: str) -> CommandAck:
    _require_supported(wire_format)

    if wire_format == WIRE_FORMAT_FLATBUFFERS:
        data = raw_payload if isinstance(raw_payload, bytes) else raw_payload.encode("utf-8")
        return _decode_ack_flatbuf(data)

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

    if wire_format == WIRE_FORMAT_FLATBUFFERS:
        return _encode_supervisor_signal_flatbuf(signal)

    payload = supervisor_signal_to_mapping(signal)

    if wire_format == WIRE_FORMAT_JSON:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")

    flexbuffers = _require_flexbuffers()
    if flexbuffers is None:
        return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return flexbuffers.Dumps(payload)


def decode_supervisor_signal(raw_payload: bytes | str, wire_format: str) -> SupervisorSignal:
    _require_supported(wire_format)

    if wire_format == WIRE_FORMAT_FLATBUFFERS:
        data = raw_payload if isinstance(raw_payload, bytes) else raw_payload.encode("utf-8")
        return _decode_supervisor_signal_flatbuf(data)

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


# ── FlatBuffers encode/decode helpers ────────────────────────────────────────

def _fb_str(val) -> str:
    """Decode a FlatBuffers string field (may return bytes or str)."""
    if isinstance(val, bytes):
        return val.decode("utf-8")
    return str(val) if val is not None else ""


def _encode_command_flatbuf(command: MissionCommand) -> bytes:
    import flatbuffers
    from guidance.generated.guidance import MissionCommand as MC

    builder = flatbuffers.Builder(256)
    cmd_id = builder.CreateString(command.command_id)
    mission_id = builder.CreateString(command.mission_id)
    safety = builder.CreateString(command.safety_mode)

    MC.MissionCommandStart(builder)
    MC.MissionCommandAddCommandId(builder, cmd_id)
    MC.MissionCommandAddMissionId(builder, mission_id)
    MC.MissionCommandAddCommandType(builder, _COMMAND_TYPE_TO_FB.get(command.command_type.value, 0))
    MC.MissionCommandAddSafetyMode(builder, safety)
    MC.MissionCommandAddTimeoutSec(builder, float(command.timeout_sec))

    target = command.target or {}
    if "lat_deg" in target:
        MC.MissionCommandAddTargetLatDeg(builder, float(target["lat_deg"]))
    if "lon_deg" in target:
        MC.MissionCommandAddTargetLonDeg(builder, float(target["lon_deg"]))
    if "alt_m" in target:
        MC.MissionCommandAddTargetAltM(builder, float(target["alt_m"]))

    fb_cmd = MC.MissionCommandEnd(builder)
    builder.Finish(fb_cmd)
    return bytes(builder.Output())


def _decode_command_flatbuf(data: bytes) -> MissionCommand:
    from guidance.generated.guidance.MissionCommand import MissionCommand as FBCmd

    fb = FBCmd.GetRootAs(data, 0)
    ct_val = _FB_TO_COMMAND_TYPE.get(fb.CommandType(), "goto")
    target: dict = {}
    if fb.TargetLatDeg() != 0.0:
        target["lat_deg"] = fb.TargetLatDeg()
    if fb.TargetLonDeg() != 0.0:
        target["lon_deg"] = fb.TargetLonDeg()
    if fb.TargetAltM() != 0.0:
        target["alt_m"] = fb.TargetAltM()

    return MissionCommand(
        command_id=_fb_str(fb.CommandId()),
        mission_id=_fb_str(fb.MissionId()),
        command_type=CommandType(ct_val),
        target=target,
        timeout_sec=float(fb.TimeoutSec()) if fb.TimeoutSec() else 2.0,
        safety_mode=_fb_str(fb.SafetyMode()) or "normal",
    )


def _encode_ack_flatbuf(ack: CommandAck) -> bytes:
    import flatbuffers
    from guidance.generated.guidance import CommandAck as CA

    builder = flatbuffers.Builder(128)
    cmd_id = builder.CreateString(ack.command_id)
    reason_off = builder.CreateString(ack.reason or "") if ack.reason else None

    CA.CommandAckStart(builder)
    CA.CommandAckAddCommandId(builder, cmd_id)
    CA.CommandAckAddStatus(builder, _ACK_STATUS_TO_FB.get(ack.status.value, 0))
    if reason_off is not None:
        CA.CommandAckAddReason(builder, reason_off)
    fb_ack = CA.CommandAckEnd(builder)
    builder.Finish(fb_ack)
    return bytes(builder.Output())


def _decode_ack_flatbuf(data: bytes) -> CommandAck:
    from guidance.generated.guidance.CommandAck import CommandAck as FBAck

    fb = FBAck.GetRootAs(data, 0)
    status_val = _FB_TO_ACK_STATUS.get(fb.Status(), "accepted")
    reason = _fb_str(fb.Reason()) if fb.Reason() else None
    return CommandAck(
        command_id=_fb_str(fb.CommandId()),
        status=AckStatus(status_val),
        reason=reason if reason else None,
    )


def _encode_supervisor_signal_flatbuf(signal: SupervisorSignal) -> bytes:
    import flatbuffers
    from guidance.generated.guidance import SupervisorSignal as SS

    builder = flatbuffers.Builder(128)
    reason_off = builder.CreateString(signal.reason or "")

    SS.SupervisorSignalStart(builder)
    SS.SupervisorSignalAddTimestampMs(builder, signal.timestamp_ms)
    SS.SupervisorSignalAddState(builder, _SUPERVISOR_STATE_TO_FB.get(signal.state.value, 0))
    SS.SupervisorSignalAddAction(builder, _SUPERVISOR_ACTION_TO_FB.get(signal.action.value, 0))
    SS.SupervisorSignalAddReason(builder, reason_off)
    fb_sig = SS.SupervisorSignalEnd(builder)
    builder.Finish(fb_sig)
    return bytes(builder.Output())


def _decode_supervisor_signal_flatbuf(data: bytes) -> SupervisorSignal:
    from guidance.generated.guidance.SupervisorSignal import SupervisorSignal as FBSig

    fb = FBSig.GetRootAs(data, 0)
    state_val = _FB_TO_SUPERVISOR_STATE.get(fb.State(), "nominal")
    action_val = _FB_TO_SUPERVISOR_ACTION.get(fb.Action(), "continue")
    return SupervisorSignal(
        timestamp_ms=fb.TimestampMs(),
        state=SupervisorState(state_val),
        action=SupervisorAction(action_val),
        reason=_fb_str(fb.Reason()),
    )


# ── VehicleTelemetry FlatBuffers (public, not gated by wire_format param) ────

def encode_vehicle_telemetry_flatbuf(telemetry: VehicleTelemetry) -> bytes:
    """Encode a VehicleTelemetry to FlatBuffers bytes."""
    import flatbuffers
    from guidance.generated.vehicle import VehicleTelemetry as VT

    builder = flatbuffers.Builder(256)
    mode = builder.CreateString(telemetry.flight_mode)

    VT.VehicleTelemetryStart(builder)
    VT.VehicleTelemetryAddTimestampMs(builder, telemetry.timestamp_ms)
    VT.VehicleTelemetryAddLatDeg(builder, telemetry.lat_deg)
    VT.VehicleTelemetryAddLonDeg(builder, telemetry.lon_deg)
    VT.VehicleTelemetryAddAltM(builder, telemetry.alt_m)
    VT.VehicleTelemetryAddVnMps(builder, float(telemetry.vn_mps))
    VT.VehicleTelemetryAddVeMps(builder, float(telemetry.ve_mps))
    VT.VehicleTelemetryAddVdMps(builder, float(telemetry.vd_mps))
    VT.VehicleTelemetryAddBatteryPct(builder, float(telemetry.battery_pct))
    VT.VehicleTelemetryAddFlightMode(builder, mode)
    VT.VehicleTelemetryAddLinkQuality(builder, float(telemetry.link_quality))
    fb_tel = VT.VehicleTelemetryEnd(builder)
    builder.Finish(fb_tel)
    return bytes(builder.Output())


def decode_vehicle_telemetry_flatbuf(data: bytes) -> VehicleTelemetry:
    """Decode FlatBuffers bytes to a VehicleTelemetry."""
    from guidance.generated.vehicle.VehicleTelemetry import VehicleTelemetry as FBTel

    fb = FBTel.GetRootAs(data, 0)
    return VehicleTelemetry(
        timestamp_ms=fb.TimestampMs(),
        lat_deg=fb.LatDeg(),
        lon_deg=fb.LonDeg(),
        alt_m=fb.AltM(),
        vn_mps=fb.VnMps(),
        ve_mps=fb.VeMps(),
        vd_mps=fb.VdMps(),
        battery_pct=fb.BatteryPct(),
        flight_mode=_fb_str(fb.FlightMode()),
        link_quality=fb.LinkQuality(),
    )
