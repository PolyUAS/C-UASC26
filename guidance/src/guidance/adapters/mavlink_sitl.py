from dataclasses import dataclass
from typing import Protocol

from guidance.contracts import AckStatus, CommandAck, CommandType, MissionCommand


class MavlinkClient(Protocol):
    def arm(self) -> None: ...

    def takeoff(self, altitude_m: float) -> None: ...

    def goto(self, lat_deg: float, lon_deg: float, alt_m: float) -> None: ...

    def hold(self) -> None: ...

    def rtl(self) -> None: ...

    def land(self) -> None: ...

    def payload_release(self) -> None: ...

    def payload_secure(self) -> None: ...


class AdapterEventLogger(Protocol):
    def log_dispatch(self, command: MissionCommand, ack: CommandAck) -> None: ...


@dataclass(slots=True)
class SitlMissionAdapter:
    client: MavlinkClient
    event_logger: AdapterEventLogger | None = None

    def dispatch(self, command: MissionCommand) -> CommandAck:
        ack: CommandAck
        try:
            self._dispatch_impl(command)
        except (KeyError, TypeError, ValueError) as error:
            ack = CommandAck(
                command_id=command.command_id,
                status=AckStatus.REJECTED,
                reason=f"invalid_command:{error}",
            )
            self._log_dispatch(command, ack)
            return ack
        except Exception as error:
            ack = CommandAck(
                command_id=command.command_id,
                status=AckStatus.FAILED,
                reason=f"adapter_error:{error}",
            )
            self._log_dispatch(command, ack)
            return ack

        ack = CommandAck(
            command_id=command.command_id,
            status=AckStatus.ACCEPTED,
        )
        self._log_dispatch(command, ack)
        return ack

    def _dispatch_impl(self, command: MissionCommand) -> None:
        command_type = command.command_type

        if command_type == CommandType.ARM:
            self.client.arm()
            return

        if command_type == CommandType.TAKEOFF:
            altitude_m = float(command.target["altitude_m"])
            self.client.takeoff(altitude_m)
            return

        if command_type == CommandType.GOTO:
            lat_deg = float(command.target["lat_deg"])
            lon_deg = float(command.target["lon_deg"])
            alt_m = float(command.target["alt_m"])
            self.client.goto(lat_deg, lon_deg, alt_m)
            return

        if command_type == CommandType.HOLD:
            self.client.hold()
            return

        if command_type == CommandType.RTL:
            self.client.rtl()
            return

        if command_type == CommandType.LAND:
            self.client.land()
            return

        if command_type == CommandType.PAYLOAD_RELEASE:
            self.client.payload_release()
            return

        if command_type == CommandType.PAYLOAD_SECURE:
            self.client.payload_secure()
            return

        raise ValueError(f"unsupported_command_type:{command_type.value}")

    def _log_dispatch(self, command: MissionCommand, ack: CommandAck) -> None:
        if self.event_logger is None:
            return
        self.event_logger.log_dispatch(command, ack)
