import os

from guidance.adapters.mavlink_pymavlink import PymavlinkSitlClient
from guidance.adapters.mavlink_sitl import SitlMissionAdapter
from guidance.bridge import MissionZenohBridge
from guidance.replay import JsonlReplayLogger


def main():
    import zenoh

    connection_string = os.getenv("MAVLINK_CONNECTION", "udpin:127.0.0.1:14550")
    baud = int(os.getenv("MAVLINK_BAUD", "57600"))
    replay_path = os.getenv("GUIDANCE_REPLAY_PATH", "logs/mission_dispatch.jsonl")

    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    client = PymavlinkSitlClient.connect(connection_string=connection_string, baud=baud)
    logger = JsonlReplayLogger(replay_path)
    adapter = SitlMissionAdapter(client=client, event_logger=logger)
    bridge = MissionZenohBridge(session=session, adapter=adapter)

    bridge.run_forever()


if __name__ == "__main__":
    main()
