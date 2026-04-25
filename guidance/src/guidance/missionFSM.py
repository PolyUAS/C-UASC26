import asyncio
import math
from abc import ABC, ABCMeta

import flatbuffers
from statemachine import State, StateMachine
from zenoh import Session

import guidance.generated.Position.GlobalPositionInt as GlobalPositionInt
import guidance.generated.waypoint.Waypoint as Waypoint
from guidance.generated.Geofence.Coords import Coords
from guidance.generated.Geofence.GeofencePacket import GeofencePacket
from guidance.geo_utils import Data_Waypoint


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

    goto_lock = asyncio.Lock()

    def __init__(self, session: Session):
        super().__init__()
        self.session = session

        self.target_pos_pub = session.declare_publisher("control/in/target_pos")

        self.position_sub = session.declare_subscriber("control/telem/gl_pos_int")

        pass

    def set_geofence(self, packet: GeofencePacket):
        coords_len = packet.PointsLength()
        if coords_len is not None:
            self.geofence = []
            for i in range(coords_len):
                pt = packet.Points(i)
                if pt is not None:
                    self.geofence.append(pt)

    async def navigate_waypoint(
        self,
        waypoint: Data_Waypoint,
        hdg: float,
        a_rad: float,
        hold_time: float = 1.0,
        timeout: float | None = 60.0,
    ) -> None:
        """
        Navigate to a waypoint with the given latitude, longitude, altitude, and heading.

        waypoint lat + lon MUST BE in degE7, and alt in mm
        """
        builder = flatbuffers.Builder(0)

        Waypoint.Start(builder)
        Waypoint.AddLat(builder, waypoint.lat)
        Waypoint.AddLon(builder, waypoint.lon)
        Waypoint.AddAlt(builder, waypoint.alt)
        Waypoint.AddHeading(builder, hdg)
        Waypoint.AddAcceptanceRadius(builder, a_rad)
        Waypoint.AddHoldTime(builder, hold_time)
        waypoint = Waypoint.End(builder)

        builder.Finish(waypoint)
        payload = bytes(builder.Output())

        async with self.goto_lock:
            self.target_pos_pub.put(payload)

            t = waypoint

            d = math.inf

            async with asyncio.timeout(timeout):
                while a_rad <= d:  # while we have not breached the radius
                    s = self.position_sub.try_recv()
                    if s is not None:
                        payload = s.payload
                        pos = GlobalPositionInt.GlobalPositionInt.GetRootAs(payload)

                        p = Data_Waypoint(pos.Lat(), pos.Lon(), pos.Alt())

                        d = p.distance_to(t)
                    await asyncio.sleep(0.300)  # 300 ms async delay
        # return from goto
