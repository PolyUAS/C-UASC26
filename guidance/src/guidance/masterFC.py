from enum import Enum
from typing import Optional

from guidance.contracts import SupervisorAction, SupervisorSignal
from guidance.missionFSM import MissionStateMachine
from guidance.missions import (
    CircuitTimeTrialFSM,
    PackageDeliveryFSM,
    PackageDropFSM,
    PackageRecoveryFSM,
    TargetLocalizationFSM,
)
from guidance.supervisor import MissionSupervisor
from guidance.waypoint import WaypointNavigationFSM


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
        self.supervisor = MissionSupervisor()
        self.qualified = False
        self.flights_remaining = 4

        # Initialize all mission FSMs
        self.missions = {
            "waypoint": WaypointNavigationFSM(),
            "circuit": CircuitTimeTrialFSM(),
            "drop": PackageDropFSM(),
            "delivery": PackageDeliveryFSM(),
            "localization": TargetLocalizationFSM(),
            "recovery": PackageRecoveryFSM(),
        }

    def select_mission(self, mission_type: str) -> bool:
        """Team selects which mission to attempt"""
        if self.flights_remaining <= 0:
            return False

        self.active_mission = self.missions[mission_type]
        self.current_phase = self.FlightPhase.MISSION_ACTIVE
        return True

    def tick(self, supervisor_signal: Optional[SupervisorSignal] = None):
        if self.current_phase != self.FlightPhase.MISSION_ACTIVE or self.active_mission is None:
            return

        if supervisor_signal is not None:
            self._apply_supervisor_signal(supervisor_signal)
            if supervisor_signal.action == SupervisorAction.ABORT:
                return

        self.active_mission.execute_mission_logic()

    def _apply_supervisor_signal(self, signal: SupervisorSignal):
        if signal.action == SupervisorAction.CONTINUE:
            return
        if signal.action == SupervisorAction.HOLD:
            return
        if signal.action == SupervisorAction.RTL:
            return
        self.current_phase = self.FlightPhase.POST_FLIGHT
