"""
guidance - Python Zenoh Node
"""

import asyncio
from time import sleep

import zenoh

from guidance.generated.Position.GlobalPositionInt import GlobalPositionInt
from guidance.generated.waypoint import FrameType, Waypoint
from guidance.geo_utils import WaypointStruct
from guidance.missions import BAD_FSM


def main():

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    sub = session.declare_subscriber("control/telem/gl_pos_int")
    sleep(0.5)

    sample = sub.recv()

    b = sample.payload.to_bytes()

    packet = GlobalPositionInt.GetRootAs(b)

    home = WaypointStruct(packet.Lat(), packet.Lon(), packet.AltAboveHome() * 1.0)

    fsm = BAD_FSM(session, home)
    print("init fsm")

    # Wait for subscribers to be ready
    sleep(0.5)

    print("running fsm")

    asyncio.run(fsm.run())

    session.close()


if __name__ == "__main__":
    main()
