class MasterFlightController:
    """Orchestrates all missions and flight operations"""
    
    class FlightPhase(Enum):
        PRE_COMPETITION = "pre_competition"
        QUALIFICATION = "qualification"
        MISSION_READY = "mission_ready"
        MISSION_ACTIVE = "mission_active"
        POST_FLIGHT = "post_flight"
        
    def __init__(self):
        self.current_phase = self.FlightPhase.PRE_COMPETITION
        self.active_mission: Optional[MissionStateMachine] = None
        self.qualified = False
        self.flights_remaining = 4
        
        # Initialize all mission FSMs
        self.missions = {
            'waypoint': WaypointNavigationFSM(),
            'circuit': CircuitTimeTrialFSM(),
            'drop': PackageDropFSM(),
            'delivery': PackageDeliveryFSM(),
            'localization': TargetLocalizationFSM(),
            'recovery': PackageRecoveryFSM()
        }
        
    def select_mission(self, mission_type: str) -> bool:
        """Team selects which mission to attempt"""
        if self.flights_remaining <= 0:
            return False
        
        self.active_mission = self.missions[mission_type]
        self.current_phase = self.FlightPhase.MISSION_ACTIVE
        return True