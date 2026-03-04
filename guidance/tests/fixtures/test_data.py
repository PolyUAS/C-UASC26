# Shared static test data for guidance test suites
from guidance.contracts import (
    AckStatus,
    CommandAck,
    CommandType,
    MissionCommand,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
    VehicleTelemetry,
)


# ── Waypoints (unordered waypoint mission) ───────────────────────────────────

WAYPOINTS_UNORDERED = [
    {"lat": 40.7128, "lon": -74.0060, "alt": 30.0, "name": "WP-A"},
    {"lat": 40.7135, "lon": -74.0055, "alt": 35.0, "name": "WP-B"},
    {"lat": 40.7140, "lon": -74.0050, "alt": 30.0, "name": "WP-C"},
    {"lat": 40.7145, "lon": -74.0045, "alt": 40.0, "name": "WP-D"},
]

HOME_POSITION = {"lat": 40.7128, "lon": -74.0060, "alt": 0.0}

# ── Geofence boundaries ─────────────────────────────────────────────────────

GEOFENCE_BOUNDS = {
    "min_lat": 40.710,
    "max_lat": 40.720,
    "min_lon": -74.010,
    "max_lon": -74.000,
    "max_alt": 120.0,
}

POSITION_INSIDE_FENCE = {"lat": 40.7130, "lon": -74.0050, "alt": 50.0}
POSITION_OUTSIDE_FENCE_LAT = {"lat": 40.730, "lon": -74.0050, "alt": 50.0}
POSITION_OUTSIDE_FENCE_ALT = {"lat": 40.7130, "lon": -74.0050, "alt": 150.0}

# ── Sample commands ──────────────────────────────────────────────────────────

SAMPLE_ARM_COMMAND = MissionCommand(
    command_id="test-arm-001",
    mission_id="test-mission",
    command_type=CommandType.ARM,
)

SAMPLE_TAKEOFF_COMMAND = MissionCommand(
    command_id="test-takeoff-001",
    mission_id="test-mission",
    command_type=CommandType.TAKEOFF,
    target={"altitude_m": 30.0},
    timeout_sec=10.0,
)

SAMPLE_GOTO_COMMAND = MissionCommand(
    command_id="test-goto-001",
    mission_id="test-mission",
    command_type=CommandType.GOTO,
    target={"lat_deg": 40.7128, "lon_deg": -74.0060, "alt_m": 50.0},
    timeout_sec=15.0,
    safety_mode="strict",
)

SAMPLE_RTL_COMMAND = MissionCommand(
    command_id="test-rtl-001",
    mission_id="test-mission",
    command_type=CommandType.RTL,
)

SAMPLE_ABORT_COMMAND = MissionCommand(
    command_id="test-abort-001",
    mission_id="test-mission",
    command_type=CommandType.MISSION_ABORT,
)

# ── Sample telemetry ─────────────────────────────────────────────────────────

SAMPLE_TELEMETRY = VehicleTelemetry(
    timestamp_ms=1000,
    lat_deg=40.7128,
    lon_deg=-74.0060,
    alt_m=30.0,
    vn_mps=5.0,
    ve_mps=3.0,
    vd_mps=-0.5,
    battery_pct=88.0,
    flight_mode="GUIDED",
    link_quality=0.95,
)

LOW_BATTERY_TELEMETRY = VehicleTelemetry(
    timestamp_ms=2000,
    lat_deg=40.7128,
    lon_deg=-74.0060,
    alt_m=30.0,
    vn_mps=0.0,
    ve_mps=0.0,
    vd_mps=0.0,
    battery_pct=5.0,
    flight_mode="RTL",
    link_quality=0.50,
)

# ── Supervisor signals ───────────────────────────────────────────────────────

NOMINAL_SIGNAL = SupervisorSignal(
    timestamp_ms=100,
    state=SupervisorState.NOMINAL,
    action=SupervisorAction.CONTINUE,
    reason="all_components_nominal",
)

DEGRADED_HOLD_SIGNAL = SupervisorSignal(
    timestamp_ms=200,
    state=SupervisorState.DEGRADED,
    action=SupervisorAction.HOLD,
    reason="Camera:1500ms",
)

CRITICAL_RTL_SIGNAL = SupervisorSignal(
    timestamp_ms=300,
    state=SupervisorState.CRITICAL,
    action=SupervisorAction.RTL,
    reason="FlightController:4000ms",
)

FAILSAFE_ABORT_SIGNAL = SupervisorSignal(
    timestamp_ms=400,
    state=SupervisorState.FAILSAFE,
    action=SupervisorAction.ABORT,
    reason="FlightController:8000ms",
)

# ── Sample acks ──────────────────────────────────────────────────────────────

ACCEPTED_ACK = CommandAck(
    command_id="test-cmd-001",
    status=AckStatus.ACCEPTED,
)

REJECTED_ACK = CommandAck(
    command_id="test-cmd-002",
    status=AckStatus.REJECTED,
    reason="supervisor_blocked",
)

FAILED_ACK = CommandAck(
    command_id="test-cmd-003",
    status=AckStatus.FAILED,
    reason="adapter_error:link_down",
)
