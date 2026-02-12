# ;~]

from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional


class MissionState(Enum):
    """Common states across all missions"""

    IDLE = "idle"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    EXECUTING = "executing"
    RTL = "rtl"
    LANDING = "landing"
    COMPLETE = "complete"
    ABORTED = "aborted"
    EMERGENCY = "emergency"


class MissionStateMachine(ABC):
    """Base class for all mission FSMs"""

    def __init__(self, mission_name: str, time_limit: float = 600.0):
        self.mission_name = mission_name
        self.time_limit = time_limit  # 10 minutes default
        self.current_state = MissionState.IDLE
        self.start_time: Optional[float] = None
        self.geofence_breached = False

    # Concrete methods (shared logic)
    def check_geofence(self, position) -> bool:
        """All missions need this"""
        # Implementation
        pass

    def handle_emergency(self):
        """Standardized emergency response"""
        self.current_state = MissionState.EMERGENCY
        # Implementation
        pass

    def check_time_limit(self) -> bool:
        """All missions have 10 min limit"""
        # Implementation
        pass

    # Abstract methods (must implement)
    @abstractmethod
    def execute_mission_logic(self):
        """Mission-specific execution"""
        pass

    @abstractmethod
    def calculate_score(self) -> float:
        """Each mission scores differently"""
        pass

    @abstractmethod
    def get_telemetry_requirements(self) -> dict:
        """What data each mission needs to log"""
        pass
