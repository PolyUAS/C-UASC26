from enum import Enum

from guidance.missionFSM import StateMachineBaseClass


class CircuitTimeTrialFSM(StateMachineBaseClass):
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
