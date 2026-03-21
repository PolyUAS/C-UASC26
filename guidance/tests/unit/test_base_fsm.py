"""Tests for MissionStateMachine base class — geofence, time limits, state lifecycle."""

import time as _time

from guidance.missionFSM import MissionState, MissionStateMachine
from tests.fixtures.test_data import (
    GEOFENCE_BOUNDS,
    POSITION_INSIDE_FENCE,
    POSITION_OUTSIDE_FENCE_ALT,
    POSITION_OUTSIDE_FENCE_LAT,
)


class StubMission(MissionStateMachine):
    """Concrete subclass for testing the abstract base."""

    def __init__(self, **kwargs):
        super().__init__("stub_mission", **kwargs)
        self.logic_calls = 0

    def execute_mission_logic(self):
        self.logic_calls += 1

    def calculate_score(self) -> float:
        return 0.0

    def get_telemetry_requirements(self) -> dict:
        return {}


# ── Initial state ────────────────────────────────────────────────────────────


def test_initial_state_is_idle():
    fsm = StubMission()
    assert fsm.current_state == MissionState.IDLE


def test_start_mission_transitions_to_preflight():
    fsm = StubMission()
    fsm.start_mission()
    assert fsm.current_state == MissionState.PREFLIGHT
    assert fsm.start_time is not None


def test_start_mission_rejected_from_non_idle():
    fsm = StubMission()
    fsm.current_state = MissionState.EXECUTING
    fsm.start_mission()
    # State should not change
    assert fsm.current_state == MissionState.EXECUTING


# ── Abort ────────────────────────────────────────────────────────────────────


def test_abort_from_executing():
    fsm = StubMission()
    fsm.current_state = MissionState.EXECUTING
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


def test_abort_noop_from_complete():
    fsm = StubMission()
    fsm.current_state = MissionState.COMPLETE
    fsm.abort_mission()
    assert fsm.current_state == MissionState.COMPLETE


def test_abort_noop_from_aborted():
    fsm = StubMission()
    fsm.current_state = MissionState.ABORTED
    fsm.abort_mission()
    assert fsm.current_state == MissionState.ABORTED


# ── Geofence ─────────────────────────────────────────────────────────────────


def test_geofence_passes_when_inside():
    fsm = StubMission()
    fsm.set_geofence(**GEOFENCE_BOUNDS)
    assert fsm.check_geofence(POSITION_INSIDE_FENCE) is True
    assert fsm.geofence_breached is False


def test_geofence_trips_on_lat_breach():
    fsm = StubMission()
    fsm.set_geofence(**GEOFENCE_BOUNDS)
    assert fsm.check_geofence(POSITION_OUTSIDE_FENCE_LAT) is False
    assert fsm.geofence_breached is True
    assert fsm.current_state == MissionState.EMERGENCY


def test_geofence_trips_on_alt_breach():
    fsm = StubMission()
    fsm.set_geofence(**GEOFENCE_BOUNDS)
    fsm.current_state = MissionState.EXECUTING
    assert fsm.check_geofence(POSITION_OUTSIDE_FENCE_ALT) is False
    assert fsm.current_state == MissionState.EMERGENCY


def test_geofence_passes_when_not_set():
    fsm = StubMission()
    # No geofence configured — always within bounds
    assert fsm.check_geofence(POSITION_OUTSIDE_FENCE_LAT) is True


# ── Time limit ───────────────────────────────────────────────────────────────


def test_time_limit_passes_when_within():
    fsm = StubMission(time_limit=600.0)
    fsm.start_mission()
    assert fsm.check_time_limit() is True


def test_time_limit_triggers_rtl_when_exceeded():
    fsm = StubMission(time_limit=0.001)
    fsm.start_mission()
    _time.sleep(0.01)
    assert fsm.check_time_limit() is False
    assert fsm.current_state == MissionState.RTL


def test_time_limit_passes_when_not_started():
    fsm = StubMission(time_limit=0.001)
    # start_time is None
    assert fsm.check_time_limit() is True


# ── Emergency handler ────────────────────────────────────────────────────────


def test_emergency_handler_sets_emergency_state():
    fsm = StubMission()
    fsm.current_state = MissionState.EXECUTING
    fsm.handle_emergency()
    assert fsm.current_state == MissionState.EMERGENCY


# ── Default overrides ────────────────────────────────────────────────────────


def test_default_score_is_zero():
    fsm = StubMission()
    assert fsm.calculate_score() == 0.0


def test_default_telemetry_requirements_empty():
    fsm = StubMission()
    assert fsm.get_telemetry_requirements() == {}
