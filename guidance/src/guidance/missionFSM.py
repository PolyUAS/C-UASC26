from abc import ABC


class AbstractMissionStateMachine(ABC):
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
        pass

    def set_geofence(self, )
