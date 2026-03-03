import json

from guidance.bridge import MissionZenohBridge, ack_to_json, parse_command_payload
from guidance.contracts import AckStatus, CommandAck, CommandType


class FakePublisher:
    def __init__(self):
        self.messages: list[str] = []

    def put(self, payload: str):
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


class FakeAdapter:
    def __init__(self):
        self.dispatched = []

    def dispatch(self, command):
        self.dispatched.append(command)
        return CommandAck(command_id=command.command_id, status=AckStatus.ACCEPTED)


def test_parse_command_payload_builds_mission_command():
    payload = json.dumps(
        {
            "command_id": "cmd-101",
            "mission_id": "mission-waypoint",
            "command_type": "goto",
            "target": {"lat_deg": 32.0, "lon_deg": -117.0, "alt_m": 45.0},
            "timeout_sec": 3.5,
            "safety_mode": "strict",
        }
    )

    command = parse_command_payload(payload)

    assert command.command_id == "cmd-101"
    assert command.command_type == CommandType.GOTO
    assert command.target["alt_m"] == 45.0
    assert command.safety_mode == "strict"


def test_ack_to_json_serializes_enum_status():
    ack = CommandAck(command_id="cmd-102", status=AckStatus.FAILED, reason="x")

    payload = json.loads(ack_to_json(ack))

    assert payload["command_id"] == "cmd-102"
    assert payload["status"] == "failed"
    assert payload["reason"] == "x"


def test_process_once_dispatches_and_publishes_ack():
    publisher = FakePublisher()
    adapter = FakeAdapter()
    session = FakeSession(publisher)
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    payload = json.dumps(
        {
            "command_id": "cmd-103",
            "mission_id": "mission-waypoint",
            "command_type": "arm",
        }
    )

    ack = bridge.process_once(payload)

    assert ack.status == AckStatus.ACCEPTED
    assert len(adapter.dispatched) == 1
    published_payload = json.loads(publisher.messages[-1])
    assert published_payload["command_id"] == "cmd-103"
    assert published_payload["status"] == "accepted"


def test_process_once_rejects_invalid_json_payload():
    publisher = FakePublisher()
    adapter = FakeAdapter()
    session = FakeSession(publisher)
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    ack = bridge.process_once("not-json")

    assert ack.status == AckStatus.REJECTED
    assert ack.command_id == "unknown"
    published_payload = json.loads(publisher.messages[-1])
    assert published_payload["status"] == "rejected"
