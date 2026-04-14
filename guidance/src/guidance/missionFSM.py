from abc import ABC, ABCMeta

from statemachine import State, StateMachine

from guidance.generated.Geofence.Coords import Coords
from guidance.generated.Geofence.GeofencePacket import GeofencePacket


class AbstractFSMMetaclass(ABCMeta, type(StateMachine)):
    pass


class AbstractFSMBaseClass(ABC, StateMachine, metaclass=AbstractFSMMetaclass):
    INIT = State(initial=True)
    LAUNCH = State()

    ############################################
    # Things go here in non-abstract classes
    ############################################

    COMPLETE = State(final=True)
    LAND = State()
    RETURN_TO_HOME = State()

    INIT.to(LAUNCH)
    RETURN_TO_HOME.to(LAND)
    LAND.to(COMPLETE)

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
