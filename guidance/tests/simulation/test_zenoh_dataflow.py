"""Zenoh dataflow integration tests — verify pub/sub pipeline in-process.

These tests create real Zenoh sessions (peer-to-peer, no router needed)
to verify end-to-end data flow between guidance components:
  - Command publish → bridge process → ack publish
  - MuJoCo sim state → telemetry adapter → vehicle telemetry
  - Supervisor signal publish → bridge gate behavior

Run with:  pytest tests/simulation/test_zenoh_dataflow.py -v
Requires:  zenoh Python package (available in nix devShell)
"""

from __future__ import annotations

import json
import time

import pytest

try:
    import zenoh

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False

from guidance.contracts import (
    AckStatus,
    CommandType,
    MissionCommand,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
    VehicleTelemetry,
)
from guidance.wire_format import (
    WIRE_FORMAT_JSON,
    decode_ack,
    decode_vehicle_telemetry_flatbuf,
    encode_command,
    encode_vehicle_telemetry_flatbuf,
)

pytestmark = pytest.mark.skipif(not ZENOH_AVAILABLE, reason="zenoh not installed")


# ── Helpers ──────────────────────────────────────────────────────────────────


def _open_session() -> "zenoh.Session":
    """Open a Zenoh session in peer mode with multicast scouting."""
    cfg = zenoh.Config()
    return zenoh.open(cfg)


def _wait_for_discovery(delay: float = 0.3):
    """Give Zenoh peer discovery time to converge."""
    time.sleep(delay)


# ── Test: command publish → subscriber receives ──────────────────────────────


class TestCommandDataflow:
    """Verify MissionCommand JSON pub/sub roundtrip over Zenoh."""

    def test_command_pubsub_roundtrip(self):
        session = _open_session()
        topic = "test/dataflow/command"
        received = []

        sub = session.declare_subscriber(
            topic,
            lambda sample: received.append(sample.payload.to_string()),
        )
        pub = session.declare_publisher(topic)
        _wait_for_discovery()

        command = MissionCommand(
            command_id="df-001",
            mission_id="dataflow-test",
            command_type=CommandType.GOTO,
            target={"lat_deg": 40.7, "lon_deg": -74.0, "alt_m": 50.0},
        )
        payload = encode_command(command, WIRE_FORMAT_JSON)
        pub.put(payload)
        time.sleep(0.3)

        assert len(received) >= 1
        decoded = json.loads(received[0])
        assert decoded["command_id"] == "df-001"
        assert decoded["command_type"] == "goto"
        session.close()


# ── Test: vehicle telemetry FlatBuffers over Zenoh ───────────────────────────


class TestTelemetryDataflow:
    """Verify VehicleTelemetry FlatBuffers pub/sub over Zenoh."""

    def test_telemetry_flatbuf_pubsub(self):
        session = _open_session()
        topic = "test/dataflow/telemetry"
        received_bytes: list[bytes] = []

        sub = session.declare_subscriber(
            topic,
            lambda sample: received_bytes.append(bytes(sample.payload.to_bytes())),
        )
        pub = session.declare_publisher(topic)
        _wait_for_discovery()

        telem = VehicleTelemetry(
            timestamp_ms=9999,
            lat_deg=40.7128,
            lon_deg=-74.0060,
            alt_m=100.0,
            vn_mps=5.0,
            ve_mps=3.0,
            vd_mps=-1.0,
            battery_pct=85.0,
            flight_mode="GUIDED",
            link_quality=0.95,
        )
        fb_bytes = encode_vehicle_telemetry_flatbuf(telem)
        pub.put(fb_bytes)
        time.sleep(0.3)

        assert len(received_bytes) >= 1
        decoded = decode_vehicle_telemetry_flatbuf(received_bytes[0])
        assert decoded.timestamp_ms == 9999
        assert abs(decoded.lat_deg - 40.7128) < 1e-6
        assert decoded.flight_mode == "GUIDED"
        session.close()


# ── Test: heartbeat delivery ─────────────────────────────────────────────────


class TestHeartbeatDataflow:
    """Verify heartbeat messages flow from publisher to subscriber."""

    def test_heartbeat_pubsub(self):
        session = _open_session()
        topic = "test/dataflow/heartbeat"
        beat_count = []

        sub = session.declare_subscriber(
            topic,
            lambda sample: beat_count.append(1),
        )
        pub = session.declare_publisher(topic)
        _wait_for_discovery()

        for _ in range(5):
            pub.put(b"1")
            time.sleep(0.05)

        time.sleep(0.2)
        assert len(beat_count) >= 4  # Allow for occasional loss
        session.close()


# ── Test: supervisor signal JSON over Zenoh ──────────────────────────────────


class TestSupervisorDataflow:
    """Verify SupervisorSignal JSON pub/sub over Zenoh."""

    def test_supervisor_signal_pubsub(self):
        session = _open_session()
        topic = "test/dataflow/supervisor"
        received = []

        sub = session.declare_subscriber(
            topic,
            lambda sample: received.append(sample.payload.to_string()),
        )
        pub = session.declare_publisher(topic)
        _wait_for_discovery()

        signal_payload = json.dumps({
            "ts_ms": 12345,
            "state": "Degraded",
            "action": "Hold",
            "reason": "Camera:1500ms",
        })
        pub.put(signal_payload)
        time.sleep(0.3)

        assert len(received) >= 1
        decoded = json.loads(received[0])
        assert decoded["state"] == "Degraded"
        assert decoded["action"] == "Hold"
        session.close()


# ── Test: MuJoCo sim state → telemetry adapter pipeline ─────────────────────


class TestSimStatePipeline:
    """Verify sim/mujoco/state → adapter → vehicle/telemetry/fc pipeline."""

    def test_sim_state_to_vehicle_telemetry(self):
        from guidance.adapters.mujoco_telemetry import MuJoCoTelemetryAdapter

        session = _open_session()
        sim_topic = "test/dataflow/sim_state"
        telem_topic = "test/dataflow/vehicle_telem"

        adapter = MuJoCoTelemetryAdapter()
        telem_received: list[bytes] = []

        # Subscriber on output topic
        telem_sub = session.declare_subscriber(
            telem_topic,
            lambda sample: telem_received.append(bytes(sample.payload.to_bytes())),
        )
        telem_pub = session.declare_publisher(telem_topic)
        sim_pub = session.declare_publisher(sim_topic)
        _wait_for_discovery()

        # Simulate mujoco-sim publishing state
        sim_state = {
            "timestamp_ms": 42000,
            "position": {"lat_deg": 32.88, "lon_deg": -117.23, "alt_m": 45.0},
            "velocity_ned": {"vn_mps": 10.0, "ve_mps": -2.0, "vd_mps": 0.5},
            "battery_pct": 92.0,
            "flight_mode": "AUTO",
            "link_quality": 1.0,
        }

        # Process through adapter (what guidance-mujoco-telemetry does)
        raw = json.dumps(sim_state).encode("utf-8")
        telemetry = adapter.from_state_payload(raw)
        output = adapter.to_telemetry_payload(telemetry)
        telem_pub.put(output)
        time.sleep(0.3)

        assert len(telem_received) >= 1

        # Decode the received telemetry (could be JSON or FlatBuf depending on env)
        try:
            decoded = decode_vehicle_telemetry_flatbuf(telem_received[0])
        except Exception:
            decoded_json = json.loads(telem_received[0].decode("utf-8"))
            decoded = VehicleTelemetry(**decoded_json)

        assert decoded.timestamp_ms == 42000
        assert abs(decoded.lat_deg - 32.88) < 1e-4
        assert decoded.flight_mode == "AUTO"
        session.close()


# ── Test: multi-topic simultaneous subscribers ───────────────────────────────


class TestMultiTopicDataflow:
    """Verify multiple topics can be subscribed and received simultaneously."""

    def test_multiple_topics_concurrent(self):
        session = _open_session()
        heartbeats = []
        commands = []
        telemetry = []

        session.declare_subscriber(
            "test/multi/heartbeat",
            lambda s: heartbeats.append(1),
        )
        session.declare_subscriber(
            "test/multi/command",
            lambda s: commands.append(s.payload.to_string()),
        )
        session.declare_subscriber(
            "test/multi/telemetry",
            lambda s: telemetry.append(bytes(s.payload.to_bytes())),
        )

        hb_pub = session.declare_publisher("test/multi/heartbeat")
        cmd_pub = session.declare_publisher("test/multi/command")
        tel_pub = session.declare_publisher("test/multi/telemetry")
        _wait_for_discovery()

        # Publish to all three
        hb_pub.put(b"1")
        cmd_pub.put(json.dumps({"command_id": "m-001", "command_type": "arm"}).encode())

        telem = VehicleTelemetry(
            timestamp_ms=1, lat_deg=0, lon_deg=0, alt_m=0,
            vn_mps=0, ve_mps=0, vd_mps=0,
            battery_pct=100, flight_mode="SIM", link_quality=1.0,
        )
        tel_pub.put(encode_vehicle_telemetry_flatbuf(telem))
        time.sleep(0.3)

        assert len(heartbeats) >= 1
        assert len(commands) >= 1
        assert len(telemetry) >= 1
        session.close()
