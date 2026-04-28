import asyncio
import math
import os
from abc import ABC, ABCMeta
from os import environ

import flatbuffers
from statemachine import State, StateMachine
from statemachine.transition_list import TransitionList
from zenoh import Session

import guidance.generated.Control_Squid.Squidarm as SquidarmFB
import guidance.generated.Position.GlobalPositionInt as GlobalPositionInt
import guidance.generated.waypoint.FrameType as FrameTypeFB
import guidance.generated.waypoint.Waypoint as WaypointFB
from guidance.generated.Geofence.Coords import Coords
from guidance.generated.Geofence.GeofencePacket import GeofencePacket
from guidance.geo_utils import WaypointStruct


class AbstractFSMMetaclass(ABCMeta, type(StateMachine)):
    pass


class AbstractFSMBaseClass(ABC, StateMachine, metaclass=AbstractFSMMetaclass):
    drone_name = os.environ.get("DRONE_NAME")

    INIT = State(initial=True)
    LAUNCH = State()

    ############################################
    # Things go here in non-abstract classes
    ############################################

    COMPLETE = State(final=True)
    LAND = State()
    RETURN_TO_HOME = State()

    on_go: TransitionList = INIT.to(LAUNCH)
    on_return: TransitionList = RETURN_TO_HOME.to(LAND)
    on_finish: TransitionList = LAND.to(COMPLETE)

    geofence: list[Coords] = []

    goto_lock = asyncio.Lock()

    def __init__(self, session: Session, home: WaypointStruct):
        """
        Initializer for the FSM base class.
        home must have an alt that is in the global frame.
        """
        super().__init__()
        self.session = session

        self.target_pos_pub = session.declare_publisher("control/in/target_pos")

        self.position_sub = session.declare_subscriber("control/telem/gl_pos_int")

        payload_input_sub_str = ""
        if os.environ.get("DRONE_NAME") == "HORUS":
            payload_input_sub_str = "control/in/squidarm"
        elif os.environ.get("DRONE_NAME") == "OSIRUS":
            payload_input_sub_str = "control/in/payload"

        self.payload_sub = session.declare_publisher(payload_input_sub_str)

    def set_geofence(self, packet: GeofencePacket):
        coords_len = packet.PointsLength()
        if coords_len:  # guards against None and 0
            self.geofence = []
            for i in range(coords_len):
                pt = packet.Points(i)
                if pt is not None:
                    self.geofence.append(pt)

    async def navigate_waypoint(
        self,
        waypoint: WaypointStruct,
        a_rad: float,  # a_rad is in meters
        frame: int = FrameTypeFB.FrameType.GLOBAL,
        hdg: float = math.nan,
        hold_time: float = 1.0,
        timeout: float | None = 60.0,
    ) -> bool:
        """
        Navigate to a waypoint (in ```frame```) with the given heading and acceptance radius.

        returns: success value of the navigation command

        waypoint lat + lon MUST BE in degE7, and alt in mm
        ```a_rad``` (acceptance radius) is in meters
        ```hdg``` is in degrees (or ```math.nan``` for default)
        """
        builder = flatbuffers.Builder(0)

        WaypointFB.Start(builder)
        WaypointFB.AddLat(builder, waypoint.lat)
        WaypointFB.AddLon(builder, waypoint.lon)
        WaypointFB.AddAlt(builder, waypoint.alt)
        WaypointFB.AddHeading(builder, hdg)
        WaypointFB.AddAcceptanceRadius(builder, a_rad)
        WaypointFB.AddHoldTime(builder, hold_time)
        WaypointFB.AddFrame(builder, frame)
        waypoint = WaypointFB.End(builder)

        builder.Finish(waypoint)
        payload = bytes(builder.Output())

        async with self.goto_lock:
            self.target_pos_pub.put(payload)

            t = waypoint

            d = math.inf

            async with asyncio.timeout(timeout):  # whole-motion timeout
                while a_rad <= d:  # while we have not breached the radius
                    async with asyncio.timeout(0.300):  # 300 ms packet recv timeout
                        s = self.position_sub.try_recv()

                    if s is not None:
                        payload = s.payload
                        pos = GlobalPositionInt.GlobalPositionInt.GetRootAs(payload)

                        p = WaypointStruct(pos.Lat(), pos.Lon(), pos.Alt())

                        d = p.distance_to(t)

                    await asyncio.sleep(0.300)  # 300 ms async delay
            return a_rad <= d  # returns success value
        # return from goto

    def set_payload(self, x: float, y: float):
        if self.drone_name == "OSIRUS":
            y = 0.0

        builder = flatbuffers.Builder(0)
        SquidarmFB.Start(builder)
        SquidarmFB.AddX(x, builder)
        SquidarmFB.AddY(y, builder)
        p = SquidarmFB.End(builder)

        builder.Finish(p)
        payload = bytes(builder.Output())

        self.payload_sub.put(payload)
