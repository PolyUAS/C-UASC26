"""Tests for WaypointNavigationFSM — sub-state machine, scoring, telemetry reqs."""

from guidance.missionFSM import MissionState
from guidance.waypoint import WaypointNavigationFSM


# ── Initial state ────────────────────────────────────────────────────────────


def test_initial_wp_state_is_initializing():
    fsm = WaypointNavigationFSM()
    assert fsm.wp_state == WaypointNavigationFSM.WaypointState.INITIALIZING


def test_initial_mission_state_is_idle():
    fsm = WaypointNavigationFSM()
    assert fsm.current_state == MissionState.IDLE


def test_mission_name():
    fsm = WaypointNavigationFSM()
    assert fsm.mission_name == "Waypoint Navigation"


def test_time_limit_default_600():
    fsm = WaypointNavigationFSM()
    assert fsm.time_limit == 600.0


# ── Route planning ───────────────────────────────────────────────────────────


def test_optimize_route_returns_sequential_indices():
    fsm = WaypointNavigationFSM()
    waypoints = [{"lat": 1}, {"lat": 2}, {"lat": 3}]
    route = fsm._optimize_route(waypoints)
    assert route == [0, 1, 2]


def test_route_planning_after_initializing():
    """If judge logger is attached, FSM moves to ROUTE_PLANNING then TAKEOFF."""
    fsm = WaypointNavigationFSM()
    fsm.waypoints = [{"lat": 1}, {"lat": 2}]

    # _verify_logger_attached is a stub integration point — mock it
    fsm._verify_logger_attached = lambda: None

    # Simulate logger attached
    fsm.judge_logger_attached = True
    fsm.execute_mission_logic()  # Should transition INITIALIZING → ROUTE_PLANNING

    assert fsm.wp_state == WaypointNavigationFSM.WaypointState.ROUTE_PLANNING

    # Next tick: ROUTE_PLANNING → TAKEOFF
    fsm.execute_mission_logic()
    assert fsm.wp_state == WaypointNavigationFSM.WaypointState.TAKEOFF


# ── Scoring ──────────────────────────────────────────────────────────────────


def test_score_zero_when_not_all_waypoints_visited():
    fsm = WaypointNavigationFSM()
    fsm.waypoints = [1, 2, 3]
    fsm.waypoint_errors = {0: 1.0, 1: 2.0}  # missing index 2
    assert fsm.calculate_score() == 0.0


def test_score_correct_when_all_visited():
    import math

    fsm = WaypointNavigationFSM()
    fsm.waypoints = [1, 2]
    fsm.waypoint_errors = {0: 3.0, 1: 4.0}
    expected = math.sqrt(3.0**2 + 4.0**2)
    assert abs(fsm.calculate_score() - expected) < 1e-6


def test_score_zero_when_no_waypoints():
    fsm = WaypointNavigationFSM()
    fsm.waypoints = []
    fsm.waypoint_errors = {}
    assert fsm.calculate_score() == 0.0


# ── Telemetry requirements ───────────────────────────────────────────────────


def test_telemetry_requirements_include_judge_logger():
    reqs = WaypointNavigationFSM.telemetry_requirements()
    assert reqs["judge_logger"] is True
    assert reqs["gps_required"] is True


def test_instance_telemetry_requirements():
    fsm = WaypointNavigationFSM()
    reqs = fsm.get_telemetry_requirements()
    assert "altitude_monitoring" in reqs


# ── WaypointState enum completeness ─────────────────────────────────────────


def test_all_waypoint_states_in_handler_table():
    fsm = WaypointNavigationFSM()
    for state in WaypointNavigationFSM.WaypointState:
        assert state in fsm._state_handlers, f"Missing handler for {state}"


# ── Waypoint verification and counting ──────────────────────────────────────


def test_waypoint_verification_records_error():
    fsm = WaypointNavigationFSM()
    fsm.waypoints = [{"lat": 1}, {"lat": 2}]
    fsm.planned_route = [0, 1]
    fsm.current_waypoint_idx = 0

    # Force into verification state
    fsm.wp_state = WaypointNavigationFSM.WaypointState.WAYPOINT_VERIFICATION

    # Mock distance function to return a test value
    fsm._get_min_distance_to_waypoint = lambda idx: 1.5
    fsm._command_next_waypoint = lambda: None

    fsm.execute_mission_logic()

    assert 0 in fsm.waypoints_reached
    assert fsm.waypoint_errors[0] == 1.5
    assert fsm.current_waypoint_idx == 1
    assert fsm.wp_state == WaypointNavigationFSM.WaypointState.NAVIGATING


def test_last_waypoint_verification_transitions_to_all_complete():
    fsm = WaypointNavigationFSM()
    fsm.waypoints = [{"lat": 1}]
    fsm.planned_route = [0]
    fsm.current_waypoint_idx = 0
    fsm.wp_state = WaypointNavigationFSM.WaypointState.WAYPOINT_VERIFICATION
    fsm._get_min_distance_to_waypoint = lambda idx: 0.5
    fsm._command_rtl = lambda: None

    fsm.execute_mission_logic()

    assert fsm.wp_state == WaypointNavigationFSM.WaypointState.ALL_WAYPOINTS_COMPLETE


# ── Constants ────────────────────────────────────────────────────────────────


def test_approach_radius():
    assert WaypointNavigationFSM.APPROACH_RADIUS == 15.0


def test_waypoint_threshold():
    assert WaypointNavigationFSM.WAYPOINT_THRESHOLD == 2.0
