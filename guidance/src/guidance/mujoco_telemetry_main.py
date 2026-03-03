from guidance.adapters.mujoco_telemetry import (
    MuJoCoTelemetryAdapter,
    MuJoCoTelemetryZenohBridge,
)


def main():
    import zenoh

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    adapter = MuJoCoTelemetryAdapter()
    bridge = MuJoCoTelemetryZenohBridge(session=session, adapter=adapter)

    bridge.run_forever()


if __name__ == "__main__":
    main()
