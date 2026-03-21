from enum import Enum

from guidance.missionFSM import MissionStateMachine


class CircuitTimeTrialFSM(MissionStateMachine):
    class State(Enum):
        PLAN_CIRCUIT = "plan_circuit"
        TAKEOFF = "takeoff"
        ACQUIRE_GATE = "acquire_gate"
        LAP_EXECUTION = "lap_execution"
        LAP_VALIDATION = "lap_validation"
        FINISH_RTL = "finish_rtl"

    def __init__(self):
        super().__init__("Circuit Time Trial", time_limit=600.0)
        self.state = self.State.PLAN_CIRCUIT

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


class TargetLocalizationFSM(MissionStateMachine):
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


class PackageOpsMode(Enum):
    DROP = "drop"
    DELIVERY = "delivery"
    RECOVERY = "recovery"


class PackageOpsFSM(MissionStateMachine):
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


class PackageDropFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.DROP)


class PackageDeliveryFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.DELIVERY)


class PackageRecoveryFSM(PackageOpsFSM):
    def __init__(self):
        super().__init__(PackageOpsMode.RECOVERY)
