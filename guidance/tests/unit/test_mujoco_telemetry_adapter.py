import json

from guidance.adapters.mujoco_telemetry import (
    MuJoCoTelemetryAdapter,
    MuJoCoTelemetryZenohBridge,
)


class FakePublisher:
    def __init__(self):
        self.messages: list[bytes] = []

    def put(self, payload: bytes):
        self.messages.append(payload)


class FakeSubscriber:
    def recv(self):
        raise RuntimeError("unused_in_unit_test")


class FakeSession:
    def __init__(self, publisher: FakePublisher):
        self.publisher = publisher

    def declare_subscriber(self, _topic: str):
        return FakeSubscriber()

    def declare_publisher(self, _topic: str):
        return self.publisher


def test_mujoco_adapter_maps_nested_state_fields():
    adapter = MuJoCoTelemetryAdapter()
    state = {
        "timestamp_ms": 123,
        "position": {"lat_deg": 32.88, "lon_deg": -117.23, "alt_m": 45.5},
        "velocity_ned": {"vn_mps": 12.0, "ve_mps": -3.5, "vd_mps": 0.2},
        "battery_pct": 88.0,
        "flight_mode": "AUTO",
        "link_quality": 0.95,
    }

    telemetry = adapter.from_state_mapping(state)

    assert telemetry.timestamp_ms == 123
    assert telemetry.lat_deg == 32.88
    assert telemetry.ve_mps == -3.5
    assert telemetry.flight_mode == "AUTO"


def test_mujoco_bridge_process_once_publishes_vehicle_telemetry():
    publisher = FakePublisher()
    adapter = MuJoCoTelemetryAdapter()
    bridge = MuJoCoTelemetryZenohBridge(session=FakeSession(publisher), adapter=adapter)

    state_payload = json.dumps(
        {
            "timestamp_ms": 456,
            "lat_deg": 32.1,
            "lon_deg": -117.2,
            "alt_m": 41.0,
            "vn_mps": 5.0,
            "ve_mps": 0.0,
            "vd_mps": -0.5,
            "battery_pct": 91.0,
            "flight_mode": "GUIDED",
            "link_quality": 0.99,
        }
    ).encode("utf-8")

    telemetry = bridge.process_once(state_payload)

    assert telemetry.timestamp_ms == 456
    assert len(publisher.messages) == 1
    published = json.loads(publisher.messages[0].decode("utf-8"))
    assert published["flight_mode"] == "GUIDED"
    assert published["battery_pct"] == 91.0
