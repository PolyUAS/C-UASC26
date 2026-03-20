from enum import Enum

from guidance.missionFSM import StateMachineBaseClass


class TargetLocalizationFSM(StateMachineBaseClass):
    class State(Enum):
        SEARCH_PATTERN = "search_pattern"
        DETECT = "detect"
        TRACK = "track"
        GEOLOCALIZE = "geolocalize"
        CONFIDENCE_GATE = "confidence_gate"
        REPORT = "report"

    def __init__(self):
        super().__init__("Target ID Localization", time_limit=600.0)
        self.state = self.State.SEARCH_PATTERN

    def execute_mission_logic(self):
        return

    def calculate_score(self) -> float:
        return 0.0

    def get_telemetry_requirements(self) -> dict:
        return {
            "camera_required": True,
            "gimbal_required": True,
            "perception_log": True,
        }
