from typing import Any

from guidance.adapters.mavlink_sitl import SitlMissionAdapter
from guidance.contracts import AckStatus, CommandAck, CommandType, MissionCommand
from guidance.wire_format import (
    WIRE_FORMAT_JSON,
    decode_command,
    encode_ack,
)


MISSION_COMMAND_TOPIC = "guidance/mission/command"
MISSION_ACK_TOPIC = "guidance/mission/ack"


def parse_command_payload(raw_payload: str) -> MissionCommand:
    return decode_command(raw_payload, WIRE_FORMAT_JSON)


def ack_to_json(ack: CommandAck) -> str:
    return encode_ack(ack, WIRE_FORMAT_JSON).decode("utf-8")


class MissionZenohBridge:
    def __init__(self, session: Any, adapter: SitlMissionAdapter, wire_format: str = WIRE_FORMAT_JSON):
        self.session = session
        self.adapter = adapter
        self.wire_format = wire_format
        self.command_sub = self.session.declare_subscriber(MISSION_COMMAND_TOPIC)
        self.ack_pub = self.session.declare_publisher(MISSION_ACK_TOPIC)

    def process_once(self, raw_payload: bytes | str) -> CommandAck:
        try:
            command = decode_command(raw_payload, self.wire_format)
        except Exception as error:
            ack = CommandAck(
                command_id="unknown",
                status=AckStatus.REJECTED,
                reason=f"invalid_payload:{error}",
            )
            self.ack_pub.put(encode_ack(ack, self.wire_format))
            return ack

        ack = self.adapter.dispatch(command)
        self.ack_pub.put(encode_ack(ack, self.wire_format))
        return ack

    def run_forever(self):
        while True:
            sample = self.command_sub.recv()
            payload = sample.payload
            if self.wire_format == WIRE_FORMAT_JSON:
                raw_payload = payload.to_string()
            else:
                to_bytes = getattr(payload, "to_bytes", None)
                raw_payload = to_bytes() if callable(to_bytes) else payload.to_string().encode("utf-8")
            self.process_once(raw_payload)
