from enum import Enum

from zenoh import Session

from guidance.missionFSM import AbstractFSMBaseClass


class TargetLocalizationFSM(AbstractFSMBaseClass):
    def __init__(
        self,
        session: Session,
    ):
        super().__init__()

    def execute_mission_logic(self):
        return

    def get_telemetry_requirements(self) -> dict:
        return {
            "camera_required": True,
            "gimbal_required": True,
            "perception_log": True,
        }
