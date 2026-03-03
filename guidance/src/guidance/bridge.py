from dataclasses import asdict
import json
from typing import Any

from guidance.adapters.mavlink_sitl import SitlMissionAdapter
from guidance.contracts import AckStatus, CommandAck, CommandType, MissionCommand


MISSION_COMMAND_TOPIC = "guidance/mission/command"
MISSION_ACK_TOPIC = "guidance/mission/ack"


def parse_command_payload(raw_payload: str) -> MissionCommand:
    payload = json.loads(raw_payload)
    return MissionCommand(
        command_id=str(payload["command_id"]),
        mission_id=str(payload["mission_id"]),
        command_type=CommandType(str(payload["command_type"])),
        target=dict(payload.get("target", {})),
        timeout_sec=float(payload.get("timeout_sec", 2.0)),
        safety_mode=str(payload.get("safety_mode", "normal")),
    )


def ack_to_json(ack: CommandAck) -> str:
    payload = asdict(ack)
    payload["status"] = ack.status.value
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


class MissionZenohBridge:
    def __init__(self, session: Any, adapter: SitlMissionAdapter):
        self.session = session
        self.adapter = adapter
        self.command_sub = self.session.declare_subscriber(MISSION_COMMAND_TOPIC)
        self.ack_pub = self.session.declare_publisher(MISSION_ACK_TOPIC)

    def process_once(self, raw_payload: str) -> CommandAck:
        try:
            command = parse_command_payload(raw_payload)
        except Exception as error:
            ack = CommandAck(
                command_id="unknown",
                status=AckStatus.REJECTED,
                reason=f"invalid_payload:{error}",
            )
            self.ack_pub.put(ack_to_json(ack))
            return ack

        ack = self.adapter.dispatch(command)
        self.ack_pub.put(ack_to_json(ack))
        return ack

    def run_forever(self):
        while True:
            sample = self.command_sub.recv()
            raw_payload = sample.payload.to_string()
            self.process_once(raw_payload)
