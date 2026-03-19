import os
import sys
from random import sample
from time import sleep

import flatbuffers
import zenoh

import hello_world.Geofence.GeofencePacket as GeofencePacket
import hello_world.Gimbal as Gimbal
import hello_world.Gimbal.State as State


def main():
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    # pub = session.declare_publisher("control/in/gimbal")

    pub = session.declare_publisher("control/in/gimbal")

    # Wait for subscribers to be ready
    sleep(0.5)

    # Now publish
    #
    geofenceQ = session.declare_querier("control/query/geofence")

    try:
        geofence_q = session.declare_querier("control/query/geofence")
        print("Querying current geofence...")

        replies = geofence_q.get()  # synchronous get (waits for replies)

        if not replies:
            print("→ No replies received")
        else:
            for reply in replies:
                if reply.ok:
                    sample = reply.ok
                    try:
                        pkt: GeofencePacket.GeofencePacket = (
                            GeofencePacket.GeofencePacket.GetRootAs(
                                sample.payload.to_bytes(), 0
                            )
                        )
                        print(
                            "  Geofence reply received (size:",
                            len(sample.payload.to_bytes()),
                            "bytes)",
                        )
                        print(str(pkt))
                        # Add more printing / parsing if you know the schema
                    except Exception as e:
                        print("  Cannot parse geofence FlatBuffer:", e)
                else:
                    print("  Query error:", reply.err)
    except Exception as e:
        print("Geofence query failed:", e)

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
        pub.put(buf)

        print(f"Published yaw = {i / 10.0:>6.1f}°")
        sleep(0.3)  # adjust delay — smaller = faster sweep, larger = easier to observe

    session.close()


if __name__ == "__main__":
    main()
