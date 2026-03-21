from abc import ABC, ABCMeta

from statemachine import State, StateMachine

from guidance.generated.Geofence.Coords import Coords
from guidance.generated.Geofence.GeofencePacket import GeofencePacket


class AbstractFSMMetaclass(ABCMeta, type(StateMachine)):
    pass


class AbstractFSMBaseClass(ABC, StateMachine, metaclass=AbstractFSMMetaclass):
    IDLE = State(initial=True)
    PREFLIGHT = State()
    TAKEOFF = State()
    EXECUTING = State()
    RTL = State()
    LANDING = State()
    COMPLETE = State(final=True)
    ABORTED = State(final=True)
    EMERGENCY = State()

    geofence: list[Coords] = []

    def __init__(self):
        pass

    def set_geofence(self, packet: GeofencePacket):
        coords_len = packet.PointsLength()
        if coords_len is not None:
            self.geofence = []
            for i in range(coords_len):
                pt = packet.Points(i)
                if pt is not None:
                    self.geofence.append(pt)
