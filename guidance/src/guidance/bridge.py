from typing import Any

from guidance.adapters.mavlink_sitl import SitlMissionAdapter
from guidance.contracts import (
    AckStatus,
    CommandAck,
    CommandType,
    MissionCommand,
    SupervisorAction,
    SupervisorSignal,
)
from guidance.wire_format import (
    WIRE_FORMAT_JSON,
    decode_command,
    decode_supervisor_signal,
    encode_ack,
)


MISSION_COMMAND_TOPIC = "guidance/mission/command"
MISSION_ACK_TOPIC = "guidance/mission/ack"
SUPERVISOR_SIGNAL_TOPIC = "guidance/supervisor/signal"


def parse_command_payload(raw_payload: str) -> MissionCommand:
    return decode_command(raw_payload, WIRE_FORMAT_JSON)


def ack_to_json(ack: CommandAck) -> str:
    return encode_ack(ack, WIRE_FORMAT_JSON).decode("utf-8")


class MissionZenohBridge:
    def __init__(self, session: Any, adapter: SitlMissionAdapter, wire_format: str = WIRE_FORMAT_JSON):
        self.session = session
        self.adapter = adapter
        self.wire_format = wire_format
        self.last_supervisor_signal: SupervisorSignal | None = None
        self.command_sub = self.session.declare_subscriber(MISSION_COMMAND_TOPIC)
        self.supervisor_sub = self.session.declare_subscriber(SUPERVISOR_SIGNAL_TOPIC)
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

        block_reason = self._command_block_reason(command)
        if block_reason is not None:
            ack = CommandAck(
                command_id=command.command_id,
                status=AckStatus.REJECTED,
                reason=block_reason,
            )
            self.ack_pub.put(encode_ack(ack, self.wire_format))
            return ack

        ack = self.adapter.dispatch(command)
        self.ack_pub.put(encode_ack(ack, self.wire_format))
        return ack

    def run_forever(self):
        while True:
            self._poll_supervisor_nonblocking()
            sample = self.command_sub.recv()
            payload = sample.payload
            if self.wire_format == WIRE_FORMAT_JSON:
                raw_payload = payload.to_string()
            else:
                to_bytes = getattr(payload, "to_bytes", None)
                raw_payload = to_bytes() if callable(to_bytes) else payload.to_string().encode("utf-8")
            self.process_once(raw_payload)

    def set_supervisor_signal(self, signal: SupervisorSignal):
        self.last_supervisor_signal = signal

    def _poll_supervisor_nonblocking(self):
        try_recv = getattr(self.supervisor_sub, "try_recv", None)
        if not callable(try_recv):
            return

        while True:
            sample = try_recv()
            if sample is None:
                return
            payload = sample.payload
            if self.wire_format == WIRE_FORMAT_JSON:
                raw_payload = payload.to_string()
            else:
                to_bytes = getattr(payload, "to_bytes", None)
                raw_payload = to_bytes() if callable(to_bytes) else payload.to_string().encode("utf-8")
            try:
                signal = decode_supervisor_signal(raw_payload, self.wire_format)
                self.last_supervisor_signal = signal
            except Exception:
                return

    def _command_block_reason(self, command: MissionCommand) -> str | None:
        signal = self.last_supervisor_signal
        if signal is None or signal.action == SupervisorAction.CONTINUE:
            return None

        if self._is_allowed_under_supervisor(command.command_type, signal.action):
            return None

        return (
            f"supervisor_blocked:action={signal.action.value};"
            f"state={signal.state.value};reason={signal.reason}"
        )

    @staticmethod
    def _is_allowed_under_supervisor(command_type: CommandType, action: SupervisorAction) -> bool:
        if action == SupervisorAction.HOLD:
            return command_type in {
                CommandType.HOLD,
                CommandType.RTL,
                CommandType.LAND,
                CommandType.MISSION_ABORT,
            }
        if action == SupervisorAction.RTL:
            return command_type in {
                CommandType.RTL,
                CommandType.LAND,
                CommandType.MISSION_ABORT,
            }
        if action == SupervisorAction.ABORT:
            return command_type == CommandType.MISSION_ABORT
        return True
