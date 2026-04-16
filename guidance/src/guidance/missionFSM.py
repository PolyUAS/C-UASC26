from abc import ABC, ABCMeta

import flatbuffers
from statemachine import State, StateMachine
from zenoh import Session

import guidance.generated.waypoint.Waypoint as Waypoint
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

    def __init__(self, session: Session):
        self.session = session

        self.target_pos_pub = session.declare_publisher("control/in/target_pos")

        pass

    def set_geofence(self, packet: GeofencePacket):
        coords_len = packet.PointsLength()
        if coords_len is not None:
            self.geofence = []
            for i in range(coords_len):
                pt = packet.Points(i)
                if pt is not None:
                    self.geofence.append(pt)

    # TODO MAKE SMART THREAD WITH OBSERVER PATTERN HERE
    def navigate_waypoint(
        self, lat: int, lon: int, alt: int, hdg: float, a_rad: float, hold_time: float
    ) -> None:
        """Navigate to a waypoint with the given latitude, longitude, altitude, and heading."""
        builder = flatbuffers.Builder(0)

        Waypoint.Start(builder)
        Waypoint.AddLat(builder, lat)
        Waypoint.AddLon(builder, lon)
        Waypoint.AddAlt(builder, alt)
        Waypoint.AddHeading(builder, hdg)
        Waypoint.AddAcceptanceRadius(builder, a_rad)
        Waypoint.AddHoldTime(builder, hold_time)
        waypoint = Waypoint.End(builder)

        builder.Finish(waypoint)
        payload = bytes(builder.Output())

        self.target_pos_pub.put(payload)
