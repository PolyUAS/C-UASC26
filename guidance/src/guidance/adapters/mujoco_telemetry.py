from dataclasses import asdict
import json
import os
from typing import Any

from guidance.contracts import VehicleTelemetry


SIM_MUJOCO_STATE_TOPIC = "sim/mujoco/state"
VEHICLE_TELEMETRY_FC_TOPIC = "vehicle/telemetry/fc"


class MuJoCoTelemetryAdapter:
    def from_state_mapping(self, state: dict[str, Any]) -> VehicleTelemetry:
        position = state.get("position", {})
        velocity_ned = state.get("velocity_ned", {})

        return VehicleTelemetry(
            timestamp_ms=int(state.get("timestamp_ms", 0)),
            lat_deg=float(position.get("lat_deg", state.get("lat_deg", 0.0))),
            lon_deg=float(position.get("lon_deg", state.get("lon_deg", 0.0))),
            alt_m=float(position.get("alt_m", state.get("alt_m", 0.0))),
            vn_mps=float(velocity_ned.get("vn_mps", state.get("vn_mps", 0.0))),
            ve_mps=float(velocity_ned.get("ve_mps", state.get("ve_mps", 0.0))),
            vd_mps=float(velocity_ned.get("vd_mps", state.get("vd_mps", 0.0))),
            battery_pct=float(state.get("battery_pct", 100.0)),
            flight_mode=str(state.get("flight_mode", "SIM")),
            link_quality=float(state.get("link_quality", 1.0)),
        )

    def from_state_payload(self, raw_payload: bytes | str) -> VehicleTelemetry:
        if isinstance(raw_payload, bytes):
            raw_payload = raw_payload.decode("utf-8")
        state = json.loads(raw_payload)
        return self.from_state_mapping(state)

    @staticmethod
    def to_telemetry_payload(telemetry: VehicleTelemetry) -> bytes:
        """Encode telemetry for Zenoh publishing.

        Uses FlatBuffers if GUIDANCE_WIRE_FORMAT=flatbuffers,
        otherwise falls back to JSON.
        """
        wire_format = os.environ.get("GUIDANCE_WIRE_FORMAT", "json")
        if wire_format == "flatbuffers":
            try:
                from guidance.wire_format import encode_vehicle_telemetry_flatbuf
                return encode_vehicle_telemetry_flatbuf(telemetry)
            except ImportError:
                pass
        return json.dumps(asdict(telemetry), separators=(",", ":"), sort_keys=True).encode("utf-8")

    @staticmethod
    def to_telemetry_payload_json(telemetry: VehicleTelemetry) -> bytes:
        """Always JSON — used for replay/debugging."""
        return json.dumps(asdict(telemetry), separators=(",", ":"), sort_keys=True).encode("utf-8")

    @staticmethod
    def to_telemetry_payload_flatbuf(telemetry: VehicleTelemetry) -> bytes:
        """Always FlatBuffers — for production wire format."""
        from guidance.wire_format import encode_vehicle_telemetry_flatbuf
        return encode_vehicle_telemetry_flatbuf(telemetry)


class MuJoCoTelemetryZenohBridge:
    def __init__(self, session: Any, adapter: MuJoCoTelemetryAdapter):
        self.session = session
        self.adapter = adapter
        self.state_sub = self.session.declare_subscriber(SIM_MUJOCO_STATE_TOPIC)
        self.telemetry_pub = self.session.declare_publisher(VEHICLE_TELEMETRY_FC_TOPIC)

    def process_once(self, raw_state_payload: bytes | str) -> VehicleTelemetry:
        telemetry = self.adapter.from_state_payload(raw_state_payload)
        self.telemetry_pub.put(self.adapter.to_telemetry_payload(telemetry))
        return telemetry

    def run_forever(self):
        while True:
            sample = self.state_sub.recv()
            payload = sample.payload
            to_bytes = getattr(payload, "to_bytes", None)
            raw_state_payload = to_bytes() if callable(to_bytes) else payload.to_string().encode("utf-8")
            self.process_once(raw_state_payload)
