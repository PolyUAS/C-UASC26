class WaypointNavigationFSM(MissionStateMachine):
    """Handles unordered waypoint navigation"""
    
    class WaypointState(Enum):
        PLANNING_ROUTE = "planning"
        NAVIGATING = "navigating"
        APPROACHING_WAYPOINT = "approaching"
        WAYPOINT_REACHED = "reached"
        
    def __init__(self):
        super().__init__("Waypoint Navigation", time_limit=600.0)
        self.waypoint_state = None
        self.waypoints_visited = set()
        self.current_target_waypoint = None
        self.judge_logger_attached = False
        
    def execute_mission_logic(self):
        """State transitions specific to waypoint nav"""
        if self.waypoint_state == self.WaypointState.PLANNING_ROUTE:
            self._plan_optimal_route()
        elif self.waypoint_state == self.WaypointState.NAVIGATING:
            self._navigate_to_next_waypoint()
        # etc.