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

    # pub = session.declare_publisher("control/in/gimbal")

    pub = session.declare_querier("control/in/gimbal")

    # Wait for subscribers to be ready
    sleep(0.5)

    # Now publish

    # inside the loop — or replace the loop with:
    # Sweep yaw from -135° to +135° in steps of 5° (for faster visible testing)
    for i in range(-1350, 1351, 50):  # i = decidegrees → -135.0 ... +135.0
        builder = flatbuffers.Builder(1024)

        # ── Start building the State table ──
        State.StateStart(builder)

        State.StateAddYaw(builder, float(i) / 10.0)  # degrees as float
        State.StateAddPitch(builder, 0.0)
        State.StateAddRoll(builder, 0.0)
        State.StateAddZoom(builder, 1.0)  # or whatever zoom you want

        state_offset = State.StateEnd(builder)

        # Finish the buffer (sets root type)
        builder.Finish(state_offset)

        # Get bytes and publish
        buf = builder.Output()
        pub.get(payload=buf)

        print(f"Published yaw = {i / 10.0:>6.1f}°")
        sleep(0.3)  # adjust delay — smaller = faster sweep, larger = easier to observe

    session.close()


if __name__ == "__main__":
    main()
