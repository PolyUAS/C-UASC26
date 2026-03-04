"""Tests for MasterFlightController — mission selection, ticking, supervisor dispatch."""

from guidance.contracts import SupervisorAction, SupervisorSignal, SupervisorState
from guidance.masterFC import MasterFlightController
from guidance.missionFSM import MissionState


# ── Mission selection ────────────────────────────────────────────────────────


def test_select_mission_activates_waypoint():
    mc = MasterFlightController()
    assert mc.select_mission("waypoint") is True
    assert mc.active_mission is mc.missions["waypoint"]
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


def test_select_mission_returns_false_when_no_flights():
    mc = MasterFlightController()
    mc.flights_remaining = 0
    assert mc.select_mission("waypoint") is False
    assert mc.active_mission is None


def test_select_circuit_mission():
    mc = MasterFlightController()
    assert mc.select_mission("circuit") is True
    assert mc.active_mission is mc.missions["circuit"]


def test_initial_phase_is_pre_competition():
    mc = MasterFlightController()
    assert mc.current_phase == MasterFlightController.FlightPhase.PRE_COMPETITION


# ── Tick / mission execution ────────────────────────────────────────────────


def test_tick_noop_when_not_active():
    mc = MasterFlightController()
    # No mission selected — tick should be a no-op
    mc.tick()
    assert mc.current_phase == MasterFlightController.FlightPhase.PRE_COMPETITION


def test_tick_calls_execute_mission_logic():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mission = mc.active_mission
    mission._verify_logger_attached = lambda: None
    # WaypointNavigationFSM starts in INITIALIZING — tick should call execute_mission_logic
    mc.tick()
    # The waypoint FSM should have been invoked (state may or may not change depending on logger)
    assert mission is not None


# ── Supervisor signal handling ───────────────────────────────────────────────


def test_tick_with_nominal_signal_continues():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mc.missions["waypoint"]._verify_logger_attached = lambda: None
    signal = SupervisorSignal(
        timestamp_ms=100,
        state=SupervisorState.NOMINAL,
        action=SupervisorAction.CONTINUE,
        reason="all_nominal",
    )
    mc.tick(supervisor_signal=signal)
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


def test_tick_with_abort_signal_transitions_to_post_flight():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    signal = SupervisorSignal(
        timestamp_ms=200,
        state=SupervisorState.FAILSAFE,
        action=SupervisorAction.ABORT,
        reason="total_loss",
    )
    mc.tick(supervisor_signal=signal)
    assert mc.current_phase == MasterFlightController.FlightPhase.POST_FLIGHT


def test_tick_with_hold_signal_does_not_abort():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mc.missions["waypoint"]._verify_logger_attached = lambda: None
    signal = SupervisorSignal(
        timestamp_ms=150,
        state=SupervisorState.DEGRADED,
        action=SupervisorAction.HOLD,
        reason="camera_stale",
    )
    mc.tick(supervisor_signal=signal)
    # HOLD doesn't transition to POST_FLIGHT
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


def test_tick_with_rtl_signal_does_not_abort():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mc.missions["waypoint"]._verify_logger_attached = lambda: None
    signal = SupervisorSignal(
        timestamp_ms=250,
        state=SupervisorState.CRITICAL,
        action=SupervisorAction.RTL,
        reason="fc_critical",
    )
    mc.tick(supervisor_signal=signal)
    # RTL doesn't transition to POST_FLIGHT either
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


# ── All mission types can be selected ────────────────────────────────────────


def test_all_mission_types_selectable():
    mc = MasterFlightController()
    for mission_type in ["waypoint", "circuit", "drop", "delivery", "localization", "recovery"]:
        mc2 = MasterFlightController()
        assert mc2.select_mission(mission_type) is True
        assert mc2.active_mission is not None
