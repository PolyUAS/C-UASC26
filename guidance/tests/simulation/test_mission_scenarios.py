"""Simulation-level mission scenario tests.

These test full mission lifecycles using the guidance components
without requiring Zenoh or MAVLink — they exercise the FSM logic,
supervisor integration, and bridge command processing in-process.
"""

import json

from guidance.bridge import MissionZenohBridge
from guidance.contracts import (
    AckStatus,
    CommandType,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
)
from guidance.masterFC import MasterFlightController
from guidance.missionFSM import MissionState


# ── Fakes ────────────────────────────────────────────────────────────────────


class FakePublisher:
    def __init__(self):
        self.messages: list = []

    def put(self, payload):
        self.messages.append(payload)


class FakeSubscriber:
    def recv(self):
        raise RuntimeError("unused")


class FakeSession:
    def __init__(self):
        self.pub = FakePublisher()

    def declare_subscriber(self, _topic: str):
        return FakeSubscriber()

    def declare_publisher(self, _topic: str):
        return self.pub


class RecordingAdapter:
    """Records dispatched commands and returns ACCEPTED for all."""

    def __init__(self):
        self.dispatched: list = []

    def dispatch(self, command):
        from guidance.contracts import CommandAck

        self.dispatched.append(command)
        return CommandAck(command_id=command.command_id, status=AckStatus.ACCEPTED)


# ── Scenario: nominal arm → takeoff → goto → rtl ────────────────────────────


def test_nominal_mission_sequence():
    """Simulate a nominal arm → takeoff → goto → rtl command sequence through the bridge."""
    session = FakeSession()
    adapter = RecordingAdapter()
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    commands = [
        {"command_id": "s-001", "mission_id": "sim-test", "command_type": "arm"},
        {"command_id": "s-002", "mission_id": "sim-test", "command_type": "takeoff", "target": {"altitude_m": 30.0}},
        {"command_id": "s-003", "mission_id": "sim-test", "command_type": "goto",
         "target": {"lat_deg": 40.7, "lon_deg": -74.0, "alt_m": 50.0}},
        {"command_id": "s-004", "mission_id": "sim-test", "command_type": "rtl"},
    ]

    acks = []
    for cmd in commands:
        payload = json.dumps(cmd)
        ack = bridge.process_once(payload)
        acks.append(ack)

    assert all(a.status == AckStatus.ACCEPTED for a in acks)
    assert len(adapter.dispatched) == 4
    assert adapter.dispatched[0].command_type == CommandType.ARM
    assert adapter.dispatched[1].command_type == CommandType.TAKEOFF
    assert adapter.dispatched[2].command_type == CommandType.GOTO
    assert adapter.dispatched[3].command_type == CommandType.RTL


# ── Scenario: supervisor escalation during mission ───────────────────────────


def test_supervisor_escalation_blocks_goto_mid_mission():
    """Supervisor HOLD signal should block GOTO but allow RTL."""
    session = FakeSession()
    adapter = RecordingAdapter()
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    # Phase 1: ARM succeeds
    ack = bridge.process_once(json.dumps({
        "command_id": "e-001", "mission_id": "sim-test", "command_type": "arm",
    }))
    assert ack.status == AckStatus.ACCEPTED

    # Phase 2: Supervisor escalates to HOLD
    bridge.set_supervisor_signal(SupervisorSignal(
        timestamp_ms=100,
        state=SupervisorState.DEGRADED,
        action=SupervisorAction.HOLD,
        reason="camera_dropout",
    ))

    # Phase 3: GOTO blocked
    ack = bridge.process_once(json.dumps({
        "command_id": "e-002", "mission_id": "sim-test", "command_type": "goto",
        "target": {"lat_deg": 1.0, "lon_deg": 2.0, "alt_m": 3.0},
    }))
    assert ack.status == AckStatus.REJECTED
    assert "supervisor_blocked" in (ack.reason or "")

    # Phase 4: RTL still allowed under HOLD
    ack = bridge.process_once(json.dumps({
        "command_id": "e-003", "mission_id": "sim-test", "command_type": "rtl",
    }))
    assert ack.status == AckStatus.ACCEPTED


# ── Scenario: complete mission lifecycle with FSM ────────────────────────────


def test_waypoint_fsm_lifecycle():
    """Test the WaypointNavigationFSM through a simulated lifecycle."""
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mission = mc.missions["waypoint"]

    # Patch stub integration points
    mission._verify_logger_attached = lambda: None

    # Mission starts in IDLE/INITIALIZING
    assert mission.current_state == MissionState.IDLE
    assert mission.wp_state.value == "initializing"

    # Start mission
    mission.start_mission()
    assert mission.current_state == MissionState.PREFLIGHT

    # Simulate logger attachment and tick
    mission.judge_logger_attached = True
    mc.tick()  # INITIALIZING → ROUTE_PLANNING
    assert mission.wp_state.value == "route_planning"

    # Set up waypoints and tick
    mission.waypoints = [{"lat": 1}, {"lat": 2}]
    mc.tick()  # ROUTE_PLANNING → TAKEOFF
    assert mission.wp_state.value == "takeoff"


# ── Scenario: failsafe abort terminates mission ─────────────────────────────


def test_failsafe_abort_terminates_mission():
    mc = MasterFlightController()
    mc.select_mission("waypoint")

    failsafe = SupervisorSignal(
        timestamp_ms=500,
        state=SupervisorState.FAILSAFE,
        action=SupervisorAction.ABORT,
        reason="all_links_lost",
    )
    mc.tick(supervisor_signal=failsafe)
    assert mc.current_phase == MasterFlightController.FlightPhase.POST_FLIGHT


# ── Scenario: invalid payload handling ───────────────────────────────────────


def test_malformed_json_rejected():
    session = FakeSession()
    adapter = RecordingAdapter()
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    ack = bridge.process_once("not valid json at all")
    assert ack.status == AckStatus.REJECTED
    assert ack.command_id == "unknown"


def test_missing_required_fields_rejected():
    session = FakeSession()
    adapter = RecordingAdapter()
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    ack = bridge.process_once(json.dumps({"command_id": "x"}))
    assert ack.status == AckStatus.REJECTED
