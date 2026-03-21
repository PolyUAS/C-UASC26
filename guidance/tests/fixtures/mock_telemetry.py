# Mock telemetry data and providers for tests
import json
from typing import Any

from guidance.contracts import VehicleTelemetry


class FakeTelemetryProvider:
    """Generates configurable telemetry data for tests."""

    def __init__(
        self,
        lat: float = 40.7128,
        lon: float = -74.0060,
        alt: float = 30.0,
        mode: str = "GUIDED",
        battery: float = 100.0,
    ):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.mode = mode
        self.battery = battery
        self._tick = 0

    def next_telemetry(self) -> VehicleTelemetry:
        self._tick += 1
        return VehicleTelemetry(
            timestamp_ms=self._tick * 20,  # 50 Hz
            lat_deg=self.lat,
            lon_deg=self.lon,
            alt_m=self.alt,
            vn_mps=0.0,
            ve_mps=0.0,
            vd_mps=0.0,
            battery_pct=self.battery,
            flight_mode=self.mode,
            link_quality=1.0,
        )

    def next_sim_state_json(self) -> bytes:
        """Return bytes matching mujoco-sim publisher format."""
        self._tick += 1
        state = {
            "timestamp_ms": self._tick * 20,
            "position": {"lat_deg": self.lat, "lon_deg": self.lon, "alt_m": self.alt},
            "velocity_ned": {"vn_mps": 0.0, "ve_mps": 0.0, "vd_mps": 0.0},
            "battery_pct": self.battery,
            "flight_mode": self.mode,
            "link_quality": 1.0,
        }
        return json.dumps(state, separators=(",", ":")).encode("utf-8")

    def drain_battery(self, amount: float = 1.0):
        self.battery = max(0.0, self.battery - amount)

    def set_position(self, lat: float, lon: float, alt: float):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def set_mode(self, mode: str):
        self.mode = mode
