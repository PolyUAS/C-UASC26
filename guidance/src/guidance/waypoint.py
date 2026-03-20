# LKS
#
# Waypoint navigation FSM for unordered waypoint mission
# This FSM handles the complex logic of navigating to a set of unordered waypoints, including:
# - Route optimization (TSP)
# - State transitions for takeoff, navigation, approach, verification, and return
# - Judge logger verification
# - Performance tracking for scoring
# This FSM is one piece of the 6 event FSMs, each of which inherits from the
# base MissionStateMachine and implements its own mission logic and scoring.

import math
from enum import Enum
from typing import Any, Callable, Dict, List

from guidance.missionFSM import MissionState, StateMachineBaseClass


class WaypointNavigationFSM(StateMachineBaseClass):
    """Handles unordered waypoint navigation"""

    APPROACH_RADIUS = 15.0
    WAYPOINT_THRESHOLD = 2.0
    TELEMETRY_REQUIREMENTS: Dict[str, bool] = {
        "judge_logger": True,
        "gps_required": True,
        "altitude_monitoring": True,
    }

    class WaypointState(Enum):
        INITIALIZING = "initializing"
        ROUTE_PLANNING = "route_planning"
        TAKEOFF = "takeoff"
        NAVIGATING = "navigating"
        APPROACHING_WAYPOINT = "approaching_waypoint"
        WAYPOINT_VERIFICATION = "waypoint_verification"
        ALL_WAYPOINTS_COMPLETE = "all_waypoints_complete"
        RETURNING_HOME = "returning_home"
        LANDING = "landing"
        MISSION_COMPLETE = "mission_complete"

    def __init__(self):
        super().__init__("Waypoint Navigation", time_limit=600.0)
        self.wp_state = self.WaypointState.INITIALIZING

        # Mission-specific data
        self.waypoints: List[Any] = []  # Unordered list from judges
        self.planned_route: List[int] = []  # Optimized order
        self.current_waypoint_idx = 0
        self.waypoints_reached = set()

        # Judge logger tracking
        self.judge_logger_attached = False
        self.judge_logger_verified = False

        # Performance tracking
        self.waypoint_errors: Dict[int, float] = {}  # Distance error at each WP

        self._state_handlers: Dict[
            WaypointNavigationFSM.WaypointState, Callable[[], None]
        ] = {
            self.WaypointState.INITIALIZING: self._handle_initializing,
            self.WaypointState.ROUTE_PLANNING: self._handle_route_planning,
            self.WaypointState.TAKEOFF: self._handle_takeoff,
            self.WaypointState.NAVIGATING: self._handle_navigating,
            self.WaypointState.APPROACHING_WAYPOINT: self._handle_approaching_waypoint,
            self.WaypointState.WAYPOINT_VERIFICATION: self._handle_waypoint_verification,
            self.WaypointState.ALL_WAYPOINTS_COMPLETE: self._handle_all_waypoints_complete,
            self.WaypointState.RETURNING_HOME: self._handle_returning_home,
            self.WaypointState.LANDING: self._handle_landing,
            self.WaypointState.MISSION_COMPLETE: self._handle_mission_complete,
        }

    def execute_mission_logic(self):
        """Main state machine logic via per-state dispatch table."""
        handler = self._state_handlers.get(self.wp_state)
        if handler is not None:
            handler()

    def _handle_initializing(self):
        self._verify_logger_attached()
        if self.judge_logger_attached:
            self.wp_state = self.WaypointState.ROUTE_PLANNING

    def _handle_route_planning(self):
        self.planned_route = self._optimize_route(self.waypoints)
        self.wp_state = self.WaypointState.TAKEOFF

    def _handle_takeoff(self):
        if self._is_airborne():
            self.wp_state = self.WaypointState.NAVIGATING
            self._command_next_waypoint()

    def _handle_navigating(self):
        distance = self._distance_to_current_waypoint()
        if distance < self.APPROACH_RADIUS:
            self.wp_state = self.WaypointState.APPROACHING_WAYPOINT

        check_geofence_breach = getattr(self, "check_geofence_breach", None)
        if callable(check_geofence_breach) and check_geofence_breach():
            self.handle_emergency()

    def _handle_approaching_waypoint(self):
        distance = self._distance_to_current_waypoint()
        if distance < self.WAYPOINT_THRESHOLD:
            self.wp_state = self.WaypointState.WAYPOINT_VERIFICATION

    def _handle_waypoint_verification(self):
        wp_idx = self.planned_route[self.current_waypoint_idx]
        self.waypoint_errors[wp_idx] = self._get_min_distance_to_waypoint(wp_idx)
        self.waypoints_reached.add(wp_idx)

        self.current_waypoint_idx += 1
        if self.current_waypoint_idx >= len(self.planned_route):
            self.wp_state = self.WaypointState.ALL_WAYPOINTS_COMPLETE
        else:
            self.wp_state = self.WaypointState.NAVIGATING
            self._command_next_waypoint()

    def _handle_all_waypoints_complete(self):
        self._command_rtl()
        self.wp_state = self.WaypointState.RETURNING_HOME

    def _handle_returning_home(self):
        if self._is_near_home():
            self.wp_state = self.WaypointState.LANDING

    def _handle_landing(self):
        if self._is_landed():
            self.wp_state = self.WaypointState.MISSION_COMPLETE
            self.current_state = MissionState.COMPLETE

    def _handle_mission_complete(self):
        return

    def _optimize_route(self, waypoints: List[Any]) -> List[int]:
        """
        TSP optimization for waypoint order
        Critical for performance - poor route = longer flight time

        Options:
        1. Nearest neighbor (fast, ~90% optimal)
        2. 2-opt improvement (better, reasonable time)
        3. Genetic algorithm (best, may be too slow for 7 points)
        """
        # Implementation depends on performance requirements
        # For 7 waypoints, brute force is even feasible (7! = 5040 permutations)
        return list(range(len(waypoints)))

    def calculate_score(self) -> float:
        """
        Score formula from rules:
        X = sqrt(sum of squared errors)
        Z = 10 - (X - M) / S

        Note: This needs data from ALL teams to compute M and S
        Returns raw X value for judge scoring
        """
        if len(self.waypoint_errors) != len(self.waypoints):
            return 0.0  # Didn't complete all waypoints

        squared_errors = sum(err**2 for err in self.waypoint_errors.values())
        return math.sqrt(squared_errors)

    @classmethod
    def telemetry_requirements(cls) -> Dict[str, bool]:
        return dict(cls.TELEMETRY_REQUIREMENTS)

    def get_telemetry_requirements(self) -> dict:
        return self.telemetry_requirements()
