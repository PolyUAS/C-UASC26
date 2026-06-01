import math
import os
import threading
import time
import traceback
from abc import ABC, ABCMeta, abstractmethod

import flatbuffers
import zenoh
from statemachine import State, StateMachine
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

    home: WaypointStruct

    pos: WaypointStruct

    def __init__(self, session: Session, home: WaypointStruct):
        """
        Initializer for the FSM base class.
        home must have an alt that is in the global frame.
        """
        self.session = session
        self.home = home
        self.pos = home

        self.target_pos_pub = session.declare_publisher("control/in/target_pos")

        # Background thread keeps self.pos current via a blocking subscriber.
        self._position_sub = session.declare_subscriber("control/telem/gl_pos_int")

        self.geofence_queryier = session.declare_querier("control/query/geofence")

        payload_input_sub_str = "control/in/squidarm"
        if os.environ.get("DRONE_NAME") == "HORUS":
            payload_input_sub_str = "control/in/squidarm"
        elif os.environ.get("DRONE_NAME") == "OSIRUS":
            payload_input_sub_str = "control/in/payload"

        self.payload_sub = session.declare_publisher(payload_input_sub_str)

        # Daemon thread — dies automatically when the main thread exits.
        self._stop_pos_thread = threading.Event()
        self._pos_thread = threading.Thread(
            target=self._run_pos_thread, daemon=True, name="pos-updater"
        )
        self._pos_thread.start()
        super().__init__()

    # ------------------------------------------------------------------
    # Position thread
    # ------------------------------------------------------------------

    def _run_pos_thread(self) -> None:
        """
        Blocking background thread that continuously reads position samples
        and updates self.pos.  Runs until _stop_pos_thread is set.
        """
        while not self._stop_pos_thread.is_set():
            try:
                sample = self._position_sub.recv()  # blocks until a message arrives
                if sample is None:
                    continue
                payload = bytes(sample.payload.to_bytes())
                pos_fb = GlobalPositionInt.GlobalPositionInt.GetRootAs(payload)
                self.pos = WaypointStruct(
                    pos_fb.Lat(), pos_fb.Lon(), pos_fb.AltAboveHome()
                )
            except Exception as e:
                print(f"Position decode error: {type(e).__name__}: {e}")
                traceback.print_exc()

    def _reset_home(self, wp: WaypointStruct) -> None:
        self.home = wp

    # ------------------------------------------------------------------
    # Geofence
    # ------------------------------------------------------------------

    def set_geofence(self, packet: GeofencePacket) -> None:
        coords_len = packet.PointsLength()
        if coords_len:
            self.geofence = []
            for i in range(coords_len):
                pt = packet.Points(i)
                if pt is not None:
                    self.geofence.append(pt)

    def update_geofence(self) -> None:
        pack = self.geofence_queryier.get().recv()
        pack = pack.ok
        if pack is not None:
            geo = GeofencePacket.GetRootAs(pack.payload.to_bytes())
            self.set_geofence(geo)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def navigate_waypoint(
        self,
        waypoint: WaypointStruct,
        a_rad: float,  # acceptance radius in meters
        frame: int = FrameTypeFB.FrameType.GLOBAL_RELATIVE_ALT,
        hdg: float = math.nan,
        speed: float = -1.0,  # m/s, -1 for default
        timeout: float | None = 10.0,
    ) -> bool:
        """
        Navigate to a waypoint (in ``frame``) with the given heading and
        acceptance radius.  Blocks the calling thread until arrival or timeout.

        Returns True on arrival, False on timeout.

        waypoint lat + lon MUST BE in degE7, alt in mm.
        a_rad (acceptance radius) is in meters.
        hdg is in degrees (or math.nan for default).
        """

        def _build_payload() -> bytes:
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
            return bytes(builder.Output())

        payload = _build_payload()
        self.target_pos_pub.put(payload)

        t = waypoint
        d = math.inf
        last_print = time.time()

        deadline = None if timeout is None else time.time() + timeout

        temp_sub = self.session.declare_subscriber("control/telem/gl_pos_int")

        while d > a_rad:
            if deadline is not None and time.time() > deadline:
                print(
                    f"GUIDANCE: navigation timeout after {timeout:.0f}s "
                    f"at {d:.2f}m from target"
                )
                temp_sub.undeclare()
                return False
            s = temp_sub.try_recv()

            if s is not None:
                p = bytes(s.payload.to_bytes())
                pos_fb = GlobalPositionInt.GlobalPositionInt.GetRootAs(p)
                x = WaypointStruct(pos_fb.Lat(), pos_fb.Lon(), pos_fb.AltAboveHome())
                print(f"x is {x}")
                self.pos = x
            else:
                pass
                # print("NOTHINGBURGERRRRRR")
            d = self.pos.distance_to(t)

            if time.time() - last_print > 0.5:
                print(
                    f"GUIDANCE: dist to target = {d:.2f}m (pos={self.pos}, target={t})"
                )
                last_print = time.time()

            time.sleep(0.05)
        temp_sub.undeclare()
        return True

    # ------------------------------------------------------------------
    # Payload
    # ------------------------------------------------------------------

    def set_payload(self, x: float, y: float) -> None:
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

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def run(self) -> None:
        """
        Main mission logic.  Subclasses must implement this.
        Should drive the FSM from LAUNCH through to COMPLETE.
        """
