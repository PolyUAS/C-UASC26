"""Integration tests: MasterFlightController ↔ mission FSM coordination."""

from guidance.contracts import SupervisorAction, SupervisorSignal, SupervisorState
from guidance.masterFC import MasterFlightController
from guidance.missionFSM import MissionState
from guidance.waypoint import WaypointNavigationFSM


# ── Master selects and ticks mission ─────────────────────────────────────────


def test_master_ticks_waypoint_fsm():
    """MasterFC.tick() drives the active mission's execute_mission_logic."""
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mission = mc.missions["waypoint"]
    assert isinstance(mission, WaypointNavigationFSM)

    # Patch stub integration point
    mission._verify_logger_attached = lambda: None

    # First tick on waypoint FSM in INITIALIZING state
    mc.tick()
    # Logger not attached, so FSM stays in INITIALIZING (but execute_mission_logic was called)
    assert mission.wp_state == WaypointNavigationFSM.WaypointState.INITIALIZING


def test_master_ticks_circuit_fsm():
    mc = MasterFlightController()
    mc.select_mission("circuit")
    mc.tick()
    # Circuit FSM's execute_mission_logic is a no-op, so just verify no crash
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


def test_master_abort_stops_mission_ticking():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    abort_signal = SupervisorSignal(
        timestamp_ms=1,
        state=SupervisorState.FAILSAFE,
        action=SupervisorAction.ABORT,
        reason="total_loss",
    )
    mc.tick(supervisor_signal=abort_signal)
    assert mc.current_phase == MasterFlightController.FlightPhase.POST_FLIGHT

    # Further ticks should be no-ops (not MISSION_ACTIVE)
    mc.tick()
    assert mc.current_phase == MasterFlightController.FlightPhase.POST_FLIGHT


# ── Supervisor signal propagation ────────────────────────────────────────────


def test_hold_signal_does_not_abort_mission():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mc.missions["waypoint"]._verify_logger_attached = lambda: None
    hold = SupervisorSignal(
        timestamp_ms=10,
        state=SupervisorState.DEGRADED,
        action=SupervisorAction.HOLD,
        reason="camera_stale",
    )
    mc.tick(supervisor_signal=hold)
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


def test_rtl_signal_does_not_abort_mission():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mc.missions["waypoint"]._verify_logger_attached = lambda: None
    rtl = SupervisorSignal(
        timestamp_ms=20,
        state=SupervisorState.CRITICAL,
        action=SupervisorAction.RTL,
        reason="fc_critical",
    )
    mc.tick(supervisor_signal=rtl)
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


def test_continue_signal_passes_through():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mc.missions["waypoint"]._verify_logger_attached = lambda: None
    cont = SupervisorSignal(
        timestamp_ms=30,
        state=SupervisorState.NOMINAL,
        action=SupervisorAction.CONTINUE,
        reason="all_nominal",
    )
    mc.tick(supervisor_signal=cont)
    assert mc.current_phase == MasterFlightController.FlightPhase.MISSION_ACTIVE


# ── Successive missions ──────────────────────────────────────────────────────


def test_can_switch_missions():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    assert mc.active_mission is mc.missions["waypoint"]

    mc.select_mission("circuit")
    assert mc.active_mission is mc.missions["circuit"]


# ── Geofence integration ────────────────────────────────────────────────────


def test_waypoint_geofence_triggers_emergency_during_tick():
    mc = MasterFlightController()
    mc.select_mission("waypoint")
    mission = mc.missions["waypoint"]
    mission.set_geofence(
        min_lat=40.710,
        max_lat=40.720,
        min_lon=-74.010,
        max_lon=-74.000,
        max_alt=120.0,
    )
    # Simulate a geofence breach check
    result = mission.check_geofence({"lat": 40.750, "lon": -74.005, "alt": 50.0})
    assert result is False
    assert mission.current_state == MissionState.EMERGENCY
