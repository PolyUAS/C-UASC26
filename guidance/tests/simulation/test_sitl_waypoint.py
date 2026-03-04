"""Simulation-level waypoint mission test with MuJoCo telemetry adapter.

Tests the full sim data pipeline: sim state JSON → MuJoCoTelemetryAdapter →
VehicleTelemetry → FlatBuffers encode/decode roundtrip — without Zenoh.
"""

import json

from guidance.adapters.mujoco_telemetry import MuJoCoTelemetryAdapter
from guidance.contracts import VehicleTelemetry
from guidance.wire_format import (
    decode_vehicle_telemetry_flatbuf,
    encode_vehicle_telemetry_flatbuf,
)


# ── Sim state → telemetry adapter → FlatBuffers roundtrip ───────────────────


def _make_sim_state(
    lat: float = 40.7128,
    lon: float = -74.0060,
    alt: float = 30.0,
    vn: float = 5.0,
    ve: float = 3.0,
    vd: float = -0.5,
    battery: float = 88.0,
    mode: str = "GUIDED",
    ts: int = 1000,
) -> dict:
    """Build a sim/mujoco/state JSON payload matching mujoco-sim publisher format."""
    return {
        "timestamp_ms": ts,
        "position": {"lat_deg": lat, "lon_deg": lon, "alt_m": alt},
        "velocity_ned": {"vn_mps": vn, "ve_mps": ve, "vd_mps": vd},
        "battery_pct": battery,
        "flight_mode": mode,
        "link_quality": 1.0,
    }


def test_sim_state_to_telemetry_roundtrip():
    """Full pipeline: sim JSON → adapter → FlatBuf encode → FlatBuf decode."""
    adapter = MuJoCoTelemetryAdapter()
    state = _make_sim_state(lat=40.7128, lon=-74.006, alt=45.0)
    raw_payload = json.dumps(state).encode("utf-8")

    telemetry = adapter.from_state_payload(raw_payload)
    assert isinstance(telemetry, VehicleTelemetry)
    assert telemetry.lat_deg == 40.7128
    assert telemetry.alt_m == 45.0
    assert telemetry.flight_mode == "GUIDED"

    # Encode to FlatBuffers and decode back
    fb_bytes = encode_vehicle_telemetry_flatbuf(telemetry)
    decoded = decode_vehicle_telemetry_flatbuf(fb_bytes)
    assert abs(decoded.lat_deg - 40.7128) < 1e-6
    assert abs(decoded.alt_m - 45.0) < 1e-3
    assert decoded.flight_mode == "GUIDED"


def test_sim_state_flat_fields_fallback():
    """Adapter handles flat fields (no nested position/velocity_ned)."""
    adapter = MuJoCoTelemetryAdapter()
    state_flat = {
        "timestamp_ms": 2000,
        "lat_deg": 32.88,
        "lon_deg": -117.23,
        "alt_m": 50.0,
        "vn_mps": 1.0,
        "ve_mps": 2.0,
        "vd_mps": 0.0,
        "battery_pct": 95.0,
        "flight_mode": "AUTO",
        "link_quality": 0.99,
    }
    telemetry = adapter.from_state_mapping(state_flat)
    assert telemetry.lat_deg == 32.88
    assert telemetry.flight_mode == "AUTO"


def test_multiple_waypoints_telemetry_sequence():
    """Simulate telemetry updates at each waypoint in a sequence."""
    adapter = MuJoCoTelemetryAdapter()
    waypoints = [
        (40.7128, -74.0060, 30.0),
        (40.7135, -74.0055, 35.0),
        (40.7140, -74.0050, 30.0),
    ]

    telemetry_log = []
    for i, (lat, lon, alt) in enumerate(waypoints):
        state = _make_sim_state(lat=lat, lon=lon, alt=alt, ts=1000 * (i + 1))
        raw = json.dumps(state).encode("utf-8")
        telem = adapter.from_state_payload(raw)
        telemetry_log.append(telem)

        # Verify FlatBuffers roundtrip at each point
        fb = encode_vehicle_telemetry_flatbuf(telem)
        decoded = decode_vehicle_telemetry_flatbuf(fb)
        assert abs(decoded.lat_deg - lat) < 1e-6
        assert abs(decoded.alt_m - alt) < 1e-3

    assert len(telemetry_log) == 3
    assert telemetry_log[0].timestamp_ms == 1000
    assert telemetry_log[2].timestamp_ms == 3000


def test_sim_battery_drain_tracking():
    """Simulate battery drain over successive telemetry updates."""
    adapter = MuJoCoTelemetryAdapter()
    battery_levels = [100.0, 95.0, 85.0, 70.0, 50.0, 20.0, 5.0]

    for batt in battery_levels:
        state = _make_sim_state(battery=batt)
        telem = adapter.from_state_mapping(state)
        assert telem.battery_pct == batt

        fb = encode_vehicle_telemetry_flatbuf(telem)
        decoded = decode_vehicle_telemetry_flatbuf(fb)
        assert abs(decoded.battery_pct - batt) < 0.01


def test_sim_mode_transitions():
    """Simulate flight mode changes through a mission lifecycle."""
    adapter = MuJoCoTelemetryAdapter()
    modes = ["STABILIZE", "GUIDED", "AUTO", "RTL", "LAND"]

    for mode in modes:
        state = _make_sim_state(mode=mode)
        telem = adapter.from_state_mapping(state)
        assert telem.flight_mode == mode

        fb = encode_vehicle_telemetry_flatbuf(telem)
        decoded = decode_vehicle_telemetry_flatbuf(fb)
        assert decoded.flight_mode == mode
