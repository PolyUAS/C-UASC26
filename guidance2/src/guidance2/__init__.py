import os
import sys
from time import sleep

import zenoh


def main():
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    sub = session.declare_subscriber("control/telem/gl_pos_int")

    sample = sub.recv()

    b = sample.payload.to_bytes()

    packet = GlobalPositionInt.GlobalPositionInt.GetRootAs(b)

    home = WaypointStruct(packet.Lat(), packet.Lon(), packet.AltAboveHome() * 1.0)

    # Wait for subscribers to be ready
    sleep(0.5)

    # Now publish
    pub.put("Hello, from Python!")
    print("Python → Published")
    print("Python → Waiting for Rust message...")

    sample = sub.recv()

    print("Python ← Received:", sample.payload.to_string())

    print("Python done!")
    session.close()


if __name__ == "__main__":
    main()
