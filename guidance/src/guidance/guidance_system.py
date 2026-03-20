from enum import Enum
from typing import Type

from zenoh.ext import Miss

from guidance.missionFSM import StateMachineBaseClass


class CommonStates(dict):
    """Base states available to all missions, stored as a name → value mapping."""

    IDLE = "idle"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    EXECUTING = "executing"
    RTL = "rtl"
    LANDING = "landing"
    COMPLETE = "complete"
    ABORTED = "aborted"
    EMERGENCY = "emergency"

    def __init__(self):
        super().__init__(
            {
                k: v
                for k, v in vars(self.__class__).items()
                if not k.startswith("_") and isinstance(v, str)
            }
        )


class Guidance_System:
    TERMINAL_STATE_NAMES = {"COMPLETE", "ABORTED", "EMERGENCY"}

    current_mission: StateMachineBaseClass | None = None

    def __init__(self):

        pass

    def start_mission(self, mission: StateMachineBaseClass):
        self.current_mission = mission

        self.current_mission.start_mission()


# --- Geofence ---


def set_geofence(
    self,
    min_lat: float,
    max_lat: float,
    min_lon: float,
    max_lon: float,
    max_alt: float,
):
    self.geofence_bounds = {
        "min_lat": min_lat,
        "max_lat": max_lat,
        "min_lon": min_lon,
        "max_lon": max_lon,
        "max_alt": max_alt,
    }

    def check_geofence(self, position: dict) -> bool:
        if self.geofence_bounds is None:
            return True

        within = (
            self.geofence_bounds["min_lat"]
            <= position.get("lat", 0.0)
            <= self.geofence_bounds["max_lat"]
            and self.geofence_bounds["min_lon"]
            <= position.get("lon", 0.0)
            <= self.geofence_bounds["max_lon"]
            and position.get("alt", 0.0) <= self.geofence_bounds["max_alt"]
        )

        if not within:
            self.geofence_breached = True
            self.handle_emergency()

        return within

    @property
    def terminal_states(self) -> set:
        return {self.StateEnum[name] for name in self.TERMINAL_STATE_NAMES}
