import os
import sys
from time import sleep

import flatbuffers
import zenoh

import hello_world.Gimbal as Gimbal
import hello_world.Gimbal.State as State


def main():
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    pub = session.declare_publisher("control/in/gimbal")

    # Wait for subscribers to be ready
    sleep(0.5)

    # Now publish
    pub.put("Hello, from Python!")

    for i in range(-1350, 1350):
        builder = flatbuffers.Builder(1024)

        State.Start(builder)
        State.AddPitch(builder, 0.0)
        State.AddRoll(builder, 0.0)
        State.AddYaw(builder, i / 10.0)
        State.AddZoom(builder, 1.0)
        state = State.End(builder)

        builder.Finish(state)

        buf = builder.Output()

        pub.put(buf)

    session.close()


if __name__ == "__main__":
    main()
