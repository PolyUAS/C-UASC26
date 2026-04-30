from enum import Enum

from statemachine import State

from guidance.missionFSM import AbstractFSMBaseClass, AbstractFSMMetaclass

class PackageOpsMode(Enum):
    DROP = "drop"
    DELIVERY = "delivery"
    RECOVERY = "recovery"

class PackageOpsFSM(AbstractFSMMetaclass):
    class State(Enum):
        TARGET_ASSIGN = "target_assign"
        APPROACH = "approach"
        STABILIZE = "stabilize"
        ACTUATE = "actuate"
        VERIFY = "verify"
        EGRESS = "egress"

    def __init__(self, mode: PackageOpsMode):
        super().__init__(f"Package Ops ({mode.value})", time_limit=600.0)
        self.mode = mode
        self.state = self.State.TARGET_ASSIGN

    def execute_mission_logic(self):
        return

    def calculate_score(self) -> float:
        return 0.0

    def get_telemetry_requirements(self) -> dict:
        return {
            "gps_required": True,
            "payload_status": True,
            "camera_required": self.mode != PackageOpsMode.DROP,
        }

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
            "lap_event_log": True,        }


class PackageDropFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.DROP)

class PackageRecoveryFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.RECOVERY)
