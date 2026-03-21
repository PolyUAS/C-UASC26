from enum import Enum

from statemachine import State

from guidance.missionFSM import AbstractFSMBaseClass


class PackageDeliveryFSM(AbstractFSMBaseClass):
    PLAN_CIRCUIT = State()

    def __init__(self):
        super().__init__()

    def execute_mission_logic(self):
        return

    def calculate_score(self) -> float:
        return 0.0

    def get_telemetry_requirements(self) -> dict:
        return {
            "gps_required": True,
            "airspeed_monitoring": True,
            "lap_event_log": True,
        }
