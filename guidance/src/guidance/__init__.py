"""
guidance - Python Zenoh Node
"""

import asyncio
from time import sleep

import flatbuffers
import zenoh

import guidance.generated.gimbal.State
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
    # only the first sample is needed: undeclare so its FIFO is not filled and never drained
    sub.undeclare()

    b = sample.payload.to_bytes()

    packet = GlobalPositionInt.GetRootAs(b)

    home = WaypointStruct(packet.Lat(), packet.Lon(), packet.AltAboveHome() * 1.0)

    fsm = BAD_FSM(session, home)
    print("init fsm")

    # publ = session.declare_publisher("control/telem/gimbal")

    # Wait for subscribers to be ready
    sleep(0.5)

    # i = -135.0
    # j = -160.0

    # while True:
    #     builder = flatbuffers.Builder(256)

    #     guidance.generated.gimbal.State.Start(builder)
    #     guidance.generated.gimbal.State.AddPitch(builder, i)
    #     guidance.generated.gimbal.State.AddYaw(builder, j)

    #     packet = guidance.generated.gimbal.State.End(builder)

    #     builder.Finish(packet)
    #     payload = bytes(builder.Output())
    #     publ.put(payload)
    #     i += 1.0
    #     j += 1.0

    #     if i > 45.0:
    #         i = -135.0
    #     if j > 160.0:
    #         j = -160.0

    print("running fsm")

    asyncio.run(fsm.run())

    session.close()


if __name__ == "__main__":
    main()
