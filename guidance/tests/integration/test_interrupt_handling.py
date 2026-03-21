"""Integration tests: supervisor interrupt handling across guidance components."""

import json

from guidance.bridge import MissionZenohBridge
from guidance.contracts import (
    AckStatus,
    CommandType,
    MissionCommand,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
)
from guidance.supervisor import MissionSupervisor, SupervisorThresholds
from guidance.contracts import ComponentHealth


# ── Fake helpers ─────────────────────────────────────────────────────────────


class FakePublisher:
    def __init__(self):
        self.messages: list = []

    def put(self, payload):
        self.messages.append(payload)


class FakeSubscriber:
    def recv(self):
        raise RuntimeError("unused")


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
        from guidance.contracts import AckStatus, CommandAck

        self.dispatched.append(command)
        return CommandAck(command_id=command.command_id, status=AckStatus.ACCEPTED)


# ── Supervisor → Bridge integration ─────────────────────────────────────────


def test_supervisor_degraded_blocks_goto_on_bridge():
    """When supervisor is DEGRADED/HOLD, the bridge should block GOTO commands."""
    pub = FakePublisher()
    adapter = FakeAdapter()
    bridge = MissionZenohBridge(session=FakeSession(pub), adapter=adapter)

    # Set supervisor to DEGRADED/HOLD
    bridge.set_supervisor_signal(
        SupervisorSignal(
            timestamp_ms=100,
            state=SupervisorState.DEGRADED,
            action=SupervisorAction.HOLD,
            reason="camera_stale",
        )
    )

    cmd = json.dumps({
        "command_id": "int-001",
        "mission_id": "test",
        "command_type": "goto",
        "target": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0},
    })
    ack = bridge.process_once(cmd)
    assert ack.status == AckStatus.REJECTED
    assert "supervisor_blocked" in (ack.reason or "")
    assert len(adapter.dispatched) == 0


def test_supervisor_degraded_allows_hold_command():
    """HOLD commands are allowed even under DEGRADED/HOLD supervisor."""
    pub = FakePublisher()
    adapter = FakeAdapter()
    bridge = MissionZenohBridge(session=FakeSession(pub), adapter=adapter)

    bridge.set_supervisor_signal(
        SupervisorSignal(
            timestamp_ms=100,
            state=SupervisorState.DEGRADED,
            action=SupervisorAction.HOLD,
            reason="camera_stale",
        )
    )

    cmd = json.dumps({
        "command_id": "int-002",
        "mission_id": "test",
        "command_type": "hold",
    })
    ack = bridge.process_once(cmd)
    assert ack.status == AckStatus.ACCEPTED
    assert len(adapter.dispatched) == 1


def test_supervisor_critical_allows_rtl():
    """RTL commands are allowed under CRITICAL/RTL supervisor."""
    pub = FakePublisher()
    adapter = FakeAdapter()
    bridge = MissionZenohBridge(session=FakeSession(pub), adapter=adapter)

    bridge.set_supervisor_signal(
        SupervisorSignal(
            timestamp_ms=200,
            state=SupervisorState.CRITICAL,
            action=SupervisorAction.RTL,
            reason="fc_critical",
        )
    )

    cmd = json.dumps({
        "command_id": "int-003",
        "mission_id": "test",
        "command_type": "rtl",
    })
    ack = bridge.process_once(cmd)
    assert ack.status == AckStatus.ACCEPTED


def test_supervisor_critical_blocks_arm():
    """ARM commands are blocked under CRITICAL/RTL supervisor."""
    pub = FakePublisher()
    adapter = FakeAdapter()
    bridge = MissionZenohBridge(session=FakeSession(pub), adapter=adapter)

    bridge.set_supervisor_signal(
        SupervisorSignal(
            timestamp_ms=200,
            state=SupervisorState.CRITICAL,
            action=SupervisorAction.RTL,
            reason="fc_critical",
        )
    )

    cmd = json.dumps({
        "command_id": "int-004",
        "mission_id": "test",
        "command_type": "arm",
    })
    ack = bridge.process_once(cmd)
    assert ack.status == AckStatus.REJECTED


def test_supervisor_failsafe_only_allows_mission_abort():
    """Under FAILSAFE/ABORT, only MISSION_ABORT is allowed."""
    pub = FakePublisher()
    adapter = FakeAdapter()
    bridge = MissionZenohBridge(session=FakeSession(pub), adapter=adapter)

    bridge.set_supervisor_signal(
        SupervisorSignal(
            timestamp_ms=300,
            state=SupervisorState.FAILSAFE,
            action=SupervisorAction.ABORT,
            reason="total_loss",
        )
    )

    # RTL should be blocked
    rtl_cmd = json.dumps({
        "command_id": "int-005",
        "mission_id": "test",
        "command_type": "rtl",
    })
    ack = bridge.process_once(rtl_cmd)
    assert ack.status == AckStatus.REJECTED

    # MISSION_ABORT should pass
    abort_cmd = json.dumps({
        "command_id": "int-006",
        "mission_id": "test",
        "command_type": "mission_abort",
    })
    ack = bridge.process_once(abort_cmd)
    assert ack.status == AckStatus.ACCEPTED


# ── MissionSupervisor health evaluation ──────────────────────────────────────


def test_supervisor_evaluates_nominal_with_fresh_heartbeats():
    sup = MissionSupervisor()
    health = [
        ComponentHealth(component_name="fc", heartbeat_age_ms=100, severity=SupervisorState.NOMINAL),
        ComponentHealth(component_name="camera", heartbeat_age_ms=200, severity=SupervisorState.NOMINAL),
    ]
    signal = sup.evaluate(health, timestamp_ms=1000)
    assert signal.state == SupervisorState.NOMINAL
    assert signal.action == SupervisorAction.CONTINUE


def test_supervisor_escalates_to_degraded_on_stale_component():
    sup = MissionSupervisor()
    health = [
        ComponentHealth(component_name="fc", heartbeat_age_ms=100, severity=SupervisorState.NOMINAL),
        ComponentHealth(component_name="camera", heartbeat_age_ms=1500, severity=SupervisorState.DEGRADED),
    ]
    signal = sup.evaluate(health, timestamp_ms=2000)
    assert signal.state == SupervisorState.DEGRADED
    assert signal.action == SupervisorAction.HOLD


def test_supervisor_escalates_to_critical():
    sup = MissionSupervisor()
    health = [
        ComponentHealth(component_name="fc", heartbeat_age_ms=4000, severity=SupervisorState.CRITICAL),
    ]
    signal = sup.evaluate(health, timestamp_ms=3000)
    assert signal.state == SupervisorState.CRITICAL
    assert signal.action == SupervisorAction.RTL


def test_supervisor_escalates_to_failsafe():
    sup = MissionSupervisor()
    health = [
        ComponentHealth(component_name="fc", heartbeat_age_ms=8000, severity=SupervisorState.FAILSAFE),
    ]
    signal = sup.evaluate(health, timestamp_ms=4000)
    assert signal.state == SupervisorState.FAILSAFE
    assert signal.action == SupervisorAction.ABORT


def test_supervisor_custom_thresholds():
    thresholds = SupervisorThresholds(
        degraded_heartbeat_ms=500,
        critical_heartbeat_ms=1000,
        failsafe_heartbeat_ms=2000,
    )
    sup = MissionSupervisor(thresholds=thresholds)
    health = [
        ComponentHealth(component_name="fc", heartbeat_age_ms=600, severity=SupervisorState.NOMINAL),
    ]
    signal = sup.evaluate(health, timestamp_ms=5000)
    assert signal.state == SupervisorState.DEGRADED
