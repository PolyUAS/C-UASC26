from dataclasses import dataclass
from time import time
from typing import Any


@dataclass(slots=True)
class PymavlinkSitlClient:
    connection: Any
    mavlink: Any
    target_system: int
    target_component: int

    @classmethod
    def connect(
        cls,
        connection_string: str,
        baud: int = 57600,
        source_system: int = 255,
        heartbeat_timeout_sec: float = 20.0,
    ):
        from pymavlink import mavutil

        connection = mavutil.mavlink_connection(
            connection_string,
            baud=baud,
            source_system=source_system,
        )
        connection.wait_heartbeat(timeout=heartbeat_timeout_sec)
        return cls(
            connection=connection,
            mavlink=mavutil.mavlink,
            target_system=connection.target_system,
            target_component=connection.target_component,
        )

    def arm(self) -> None:
        self._command_long(
            self.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1,
        )

    def takeoff(self, altitude_m: float) -> None:
        self._command_long(
            self.mavlink.MAV_CMD_NAV_TAKEOFF,
            param7=altitude_m,
        )

    def goto(self, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        self.connection.mav.set_position_target_global_int_send(
            self._boot_time_ms(),
            self.target_system,
            self.target_component,
            self.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            self._position_only_type_mask(),
            int(lat_deg * 1e7),
            int(lon_deg * 1e7),
            float(alt_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def hold(self) -> None:
        self._set_mode("LOITER")

    def rtl(self) -> None:
        self._command_long(self.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def land(self) -> None:
        self._command_long(self.mavlink.MAV_CMD_NAV_LAND)

    def payload_release(self) -> None:
        self._command_long(
            self.mavlink.MAV_CMD_DO_SET_SERVO,
            param1=9,
            param2=2000,
        )

    def payload_secure(self) -> None:
        self._command_long(
            self.mavlink.MAV_CMD_DO_SET_SERVO,
            param1=9,
            param2=1000,
        )

    def _set_mode(self, mode_name: str) -> None:
        if hasattr(self.connection, "set_mode"):
            self.connection.set_mode(mode_name)
            return
        raise RuntimeError("connection_missing_set_mode")

    def _command_long(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )

    @staticmethod
    def _boot_time_ms() -> int:
        return int(time() * 1000)

    @staticmethod
    def _position_only_type_mask() -> int:
        return 0b110111111000
