"""
guidance - Python Zenoh Node
"""

import asyncio
from time import sleep

import flatbuffers
import zenoh

import guidance.generated.Control_Squid.Squidarm as SquidarmFB
import guidance.generated.Position.GlobalPositionInt as GlobalPositionInt
from guidance.geo_utils import WaypointStruct
from guidance.missions import BAD_FSM
from guidance.missions.lawnmower_fsm import LAWNMOWER_FSM


def main():

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    sub = session.declare_subscriber("control/telem/gl_pos_int")
    sleep(0.5)

    sample = sub.recv()

    b = sample.payload.to_bytes()

    packet = GlobalPositionInt.GlobalPositionInt.GetRootAs(b)

    home = WaypointStruct(packet.Lat(), packet.Lon(), packet.AltAboveHome() * 1.0)

    fsm = LAWNMOWER_FSM(session, home)

    print("init fsm")

    publ = session.declare_publisher("control/telem/gimbal")

    # Wait for subscribers to be ready
    sleep(0.5)

    # i = -135.0
    # j = -160.0

    # while True:
    #     builder = flatbuffers.Builder(0)

    #     State.Start(builder)
    #     State.AddPitch(builder, i)
    #     State.AddYaw(builder, j)

    #     packet = State.End(builder)

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

    fsm.run()

    session.close()


def gigasniffer():
    import time

    import zenoh

    config = zenoh.Config.from_env()

    with zenoh.open(config) as session:

        def callback(sample):
            print(
                "RX:", sample.key_expr, bytes(sample.payload).decode(errors="replace")
            )
            if sample is None:
                return
            if sample.key_expr == "control/telem/gl_pos_int":
                payload = bytes(sample.payload.to_bytes())
                pos_fb = GlobalPositionInt.GlobalPositionInt.GetRootAs(payload)
                pos = WaypointStruct(pos_fb.Lat(), pos_fb.Lon(), pos_fb.AltAboveHome())
                print(f"pos got: {pos}")

        sub = session.declare_subscriber("**", callback)

        print("listening...")
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
