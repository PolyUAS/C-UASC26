import sys
from pathlib import Path

_GENERATED = Path(__file__).parent / "generated"
sys.path.insert(0, str(_GENERATED))
import math
import time
from time import sleep

import flatbuffers
import zenoh

import guidance2.generated.waypoint.FrameType as FrameTypeFB
import guidance2.generated.waypoint.Waypoint as WaypointFB
from guidance2.generated.Geofence.Coords import Coords
from guidance2.generated.Geofence.GeofencePacket import GeofencePacket
from guidance2.generated.Position.GlobalPositionInt import GlobalPositionInt
from guidance2.geo_utils import WaypointStruct
from guidance2.lawnmower import generate_lawnmower_sweep

pos: WaypointStruct
goto: zenoh.Publisher
geofence: list[Coords] = []


def update_geofence(pack: zenoh.Reply) -> None:
    p = pack.ok
    if p is not None:
        geo = GeofencePacket.GetRootAs(p.payload.to_bytes())
        set_geofence(geo)


def update_pos(sample: zenoh.Sample):
    global pos
    b = sample.payload.to_bytes()
    packet = GlobalPositionInt.GetRootAs(b)
    pos = WaypointStruct(packet.Lat(), packet.Lon(), packet.AltAboveHome() * 1.0)


def navigate_waypoint(
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
    global goto

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
    goto.put(payload)

    t = waypoint
    d = math.inf
    last_print = time.time()

    deadline = None if timeout is None else time.time() + timeout

    while d > a_rad:
        if deadline is not None and time.time() > deadline:
            print(
                f"GUIDANCE: navigation timeout after {timeout:.0f}s "
                f"at {d:.2f}m from target"
            )

            return False

            # print("NOTHINGBURGERRRRRR")
        d = pos.distance_to(t)

        if time.time() - last_print > 0.5:
            # print(
            #     f"GUIDANCE: dist to target = {d:.2f}m (pos={self.pos}, target={t})"
            # )
            last_print = time.time()

        time.sleep(0.05)

    return True


def _run_object_localization_lawnmower() -> None:
    global geofence
    global pos
    print("starting object-localization lawnmower sweep")
    print(f"geofence = {geofence}")

    if len(geofence) < 3:
        print("no valid geofence was received; skipping lawnmower sweep")
        return

    sweep_points = generate_lawnmower_sweep(
        geofence=geofence,
        alt_mm=pos.alt,
        spacing_m=8.0,
        margin_m=8.0,
        sample_step_m=1.0,
    )

    for pt in sweep_points:
        print(f"pt! = lat: {pt.lat}, lon: {pt.lon}, alt: {pt.alt}")

    print(f"generated {len(sweep_points)} sweep waypoints")

    for i, waypoint in enumerate(sweep_points):
        print(f"going to sweep waypoint {i + 1}/{len(sweep_points)}")
        print(
            f"→ SENT WAYPOINT {i + 1}: lat={waypoint.lat}, "
            f"lon={waypoint.lon}, alt={waypoint.alt}"
        )

        success = navigate_waypoint(
            waypoint,
            a_rad=2.0,
            timeout=60.0,
        )


def set_geofence(packet: GeofencePacket) -> None:
    global geofence
    geofence = []

    for i in range(packet.PointsLength()):
        pt = packet.Points(i)
        if pt is not None:
            geofence.append(pt)


def main():
    global pos
    global goto

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    sub = session.declare_subscriber("control/telem/gl_pos_int")

    sample = sub.recv()
    sub.undeclare()

    b = sample.payload.to_bytes()

    packet = GlobalPositionInt.GetRootAs(b)

    home = WaypointStruct(packet.Lat(), packet.Lon(), packet.AltAboveHome() * 1.0)
    pos = home

    session.declare_subscriber("control/telem/gl_pos_int", update_pos)

    geofence_queryier = session.declare_querier("control/query/geofence")

    # Wait for subscribers to be ready
    goto = session.declare_publisher("control/in/target_pos")
    sleep(0.5)

    pack = geofence_queryier.get().recv()
    update_geofence(pack)

    _run_object_localization_lawnmower()

    while pos.alt < 12_000:
        pass

    session.close()


if __name__ == "__main__":
    main()
