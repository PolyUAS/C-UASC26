"""Tests for valid/invalid FSM state transitions across all mission states."""

from guidance.missionFSM import MissionState, MissionStateMachine


class StubMission(MissionStateMachine):
    def __init__(self):
        super().__init__("transition_test")

    def execute_mission_logic(self):
        pass

    def calculate_score(self) -> float:
        return 0.0

    def get_telemetry_requirements(self) -> dict:
        return {}


# ── Valid start_mission transitions ──────────────────────────────────────────


def test_idle_to_preflight_via_start():
    fsm = StubMission()
    assert fsm.current_state == MissionState.IDLE
    fsm.start_mission()
    assert fsm.current_state == MissionState.PREFLIGHT


def test_cannot_start_from_preflight():
    fsm = StubMission()
    fsm.current_state = MissionState.PREFLIGHT
    fsm.start_mission()
    assert fsm.current_state == MissionState.PREFLIGHT


def test_cannot_start_from_executing():
    fsm = StubMission()
    fsm.current_state = MissionState.EXECUTING
    fsm.start_mission()
    assert fsm.current_state == MissionState.EXECUTING


# ── Valid abort transitions ──────────────────────────────────────────────────


def test_abort_from_preflight():
    fsm = StubMission()
    fsm.current_state = MissionState.PREFLIGHT
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


def test_abort_from_takeoff():
    fsm = StubMission()
    fsm.current_state = MissionState.TAKEOFF
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


def test_abort_from_executing():
    fsm = StubMission()
    fsm.current_state = MissionState.EXECUTING
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


def test_abort_from_rtl():
    fsm = StubMission()
    fsm.current_state = MissionState.RTL
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


def test_abort_from_landing():
    fsm = StubMission()
    fsm.current_state = MissionState.LANDING
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


# ── Terminal states block abort ──────────────────────────────────────────────


def test_abort_blocked_from_complete():
    fsm = StubMission()
    fsm.current_state = MissionState.COMPLETE
    fsm.abort_mission()
    assert fsm.current_state == MissionState.COMPLETE


def test_abort_blocked_from_aborted():
    fsm = StubMission()
    fsm.current_state = MissionState.ABORTED
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


def test_abort_blocked_from_emergency():
    fsm = StubMission()
    fsm.current_state = MissionState.EMERGENCY
    fsm.abort_mission()
    assert fsm.current_state == MissionState.EMERGENCY


# ── Emergency transitions ────────────────────────────────────────────────────


def test_emergency_from_executing():
    fsm = StubMission()
    fsm.current_state = MissionState.EXECUTING
    fsm.handle_emergency()
    assert fsm.current_state == MissionState.EMERGENCY


def test_emergency_from_takeoff():
    fsm = StubMission()
    fsm.current_state = MissionState.TAKEOFF
    fsm.handle_emergency()
    assert fsm.current_state == MissionState.EMERGENCY


def test_emergency_from_idle():
    fsm = StubMission()
    fsm.current_state = MissionState.IDLE
    fsm.handle_emergency()
    assert fsm.current_state == MissionState.EMERGENCY


# ── All MissionState enum values exist ───────────────────────────────────────


def test_all_mission_states_reachable():
    expected = {
        "idle", "preflight", "takeoff", "executing",
        "rtl", "landing", "complete", "aborted", "emergency",
    }
    actual = {s.value for s in MissionState}
    assert actual == expected


# ── Time-limit triggers RTL from any non-terminal state ──────────────────────


def test_time_limit_rtl_from_executing():
    import time

    fsm = StubMission()
    fsm.time_limit = 0.001
    fsm.current_state = MissionState.EXECUTING
    fsm.start_time = time.time() - 1.0
    assert fsm.check_time_limit() is False
    assert fsm.current_state == MissionState.RTL
