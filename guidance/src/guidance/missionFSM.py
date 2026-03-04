# LKS

from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional
import time
import time


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
        self.geofence_bounds: Optional[dict] = None  # {"min_lat", "max_lat", "min_lon", "max_lon", "max_alt"}

    def set_geofence(self, min_lat: float, max_lat: float, min_lon: float, max_lon: float, max_alt: float):
        """Define the geofence boundaries"""
        self.geofence_bounds = {
            "min_lat": min_lat,
            "max_lat": max_lat,
            "min_lon": min_lon,
            "max_lon": max_lon,
            "max_alt": max_alt,
        }

    # Concrete methods (shared logic)
    def check_geofence(self, position: dict) -> bool:
        """
        All missions need this.
        position should be a dict with keys: 'lat', 'lon', 'alt'
        Returns True if within geofence, False if breached.
        """
        if self.geofence_bounds is None:
            # No geofence defined — assume safe
            return True

        lat = position.get("lat", 0.0)
        lon = position.get("lon", 0.0)
        alt = position.get("alt", 0.0)

        within = (
            self.geofence_bounds["min_lat"] <= lat <= self.geofence_bounds["max_lat"]
            and self.geofence_bounds["min_lon"] <= lon <= self.geofence_bounds["max_lon"]
            and alt <= self.geofence_bounds["max_alt"]
        )

        if not within:
            self.geofence_breached = True
            self.handle_emergency()

        return within

    def handle_emergency(self):
        """Standardized emergency response — transition to EMERGENCY and initiate RTL"""
        previous_state = self.current_state
        self.current_state = MissionState.EMERGENCY
        print(
            f"[{self.mission_name}] EMERGENCY triggered! "
            f"Transitioning from {previous_state.value} to {self.current_state.value}. "
            f"Geofence breached: {self.geofence_breached}"
        )

    def check_time_limit(self) -> bool:
        """
        All missions have a time limit (default 10 min).
        Returns True if within time limit, False if exceeded.
        """

        if self.start_time is None:
            return True

        elapsed = time.time() - self.start_time
        if elapsed >= self.time_limit:
            print(
                f"[{self.mission_name}] Time limit of {self.time_limit}s exceeded "
                f"(elapsed: {elapsed:.1f}s). Initiating RTL."
            )
            self.current_state = MissionState.RTL
            return False

        return True

    def start_mission(self):
        """Begin the mission timer and transition from IDLE to PREFLIGHT"""

        if self.current_state != MissionState.IDLE:
            print(f"[{self.mission_name}] Cannot start mission from state {self.current_state.value}")
            return

        self.start_time = time.time()
        self.current_state = MissionState.PREFLIGHT
        print(f"[{self.mission_name}] Mission started. State: {self.current_state.value}")

    def abort_mission(self):
        """Abort the mission from any non-terminal state"""
        terminal_states = {MissionState.COMPLETE, MissionState.ABORTED, MissionState.EMERGENCY}
        if self.current_state in terminal_states:
            print(f"[{self.mission_name}] Already in terminal state {self.current_state.value}")
            return

        print(f"[{self.mission_name}] Aborting mission from state {self.current_state.value}")
        self.current_state = MissionState.ABORTED

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
