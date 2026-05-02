import asyncio
import math
import os
from abc import ABC, ABCMeta, abstractmethod
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

    geofence: list[Coords] = []

    goto_lock = asyncio.Lock()

    home: WaypointStruct

    def __init__(self, session: Session, home: WaypointStruct):
        """
        Initializer for the FSM base class.
        home must have an alt that is in the global frame.
        """
        super().__init__()
        self.session = session

        self.home = home

        self.target_pos_pub = session.declare_publisher("control/in/target_pos")

        self.position_sub = session.declare_subscriber("control/telem/gl_pos_int")

        payload_input_sub_str = "control/in/squidarm"
        if os.environ.get("DRONE_NAME") == "HORUS":
            payload_input_sub_str = "control/in/squidarm"
        elif os.environ.get("DRONE_NAME") == "OSIRUS":
            payload_input_sub_str = "control/in/payload"

        self.payload_sub = session.declare_publisher(payload_input_sub_str)

    def _reset_home(self):

        buf = self.position_sub.recv()

        w = WaypointFB.Waypoint.GetRootAs(buf)

        self.home = WaypointStruct(w.Lat(), w.Lon(), w.Alt())

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
        frame: int = FrameTypeFB.FrameType.GLOBAL_RELATIVE_ALT,
        hdg: float = math.nan,
        speed: float = -1.0,  # m/s, -1 for dflt
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
        WaypointFB.AddAlt(builder, waypoint.alt / 1000.0)
        WaypointFB.AddHeading(builder, hdg)
        WaypointFB.AddAcceptanceRadius(builder, a_rad)
        WaypointFB.AddSpeed(builder, speed)
        WaypointFB.AddFrame(builder, frame)

        packet = WaypointFB.End(builder)

        builder.Finish(packet)
        payload = bytes(builder.Output())

        # print(f"lat: {waypoint.lat} lon: {waypoint.lon} alt: {waypoint.alt}")

        async with self.goto_lock:
            self.target_pos_pub.put(payload)
            t = waypoint
            d = math.inf
            try:
                async with asyncio.timeout(timeout):
                    while d > a_rad:
                        try:
                            async with asyncio.timeout(0.300):
                                s = self.position_sub.recv()
                            if s is not None:
                                payload = s.payload.to_bytes()
                                pos = GlobalPositionInt.GlobalPositionInt.GetRootAs(
                                    payload
                                )
                                p = WaypointStruct(pos.Lat(), pos.Lon(), pos.Alt())
                                d = p.distance_to(t)
                        except TimeoutError:
                            pass  # no packet this tick, keep waiting
                        await asyncio.sleep(0.1)
            except TimeoutError:
                return False  # overall timeout → mission failure
            return d <= a_rad
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

    @abstractmethod
    async def run(self) -> None:
        """
        Main mission logic. Subclasses must implement this.
        Should drive the FSM from LAUNCH through to COMPLETE.
        """
