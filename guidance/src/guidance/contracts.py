from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


class CommandType(Enum):
    ARM = "arm"
    TAKEOFF = "takeoff"
    GOTO = "goto"
    HOLD = "hold"
    RTL = "rtl"
    LAND = "land"
    PAYLOAD_RELEASE = "payload_release"
    PAYLOAD_SECURE = "payload_secure"
    CAMERA_POINT = "camera_point"
    CAMERA_TRACK = "camera_track"
    MISSION_ABORT = "mission_abort"


class AckStatus(Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    COMPLETED = "completed"
    TIMED_OUT = "timed_out"
    FAILED = "failed"


class SupervisorState(Enum):
    NOMINAL = "nominal"
    DEGRADED = "degraded"
    CRITICAL = "critical"
    FAILSAFE = "failsafe"


class SupervisorAction(Enum):
    CONTINUE = "continue"
    HOLD = "hold"
    RTL = "rtl"
    ABORT = "abort"


@dataclass(slots=True)
class MissionCommand:
    command_id: str
    mission_id: str
    command_type: CommandType
    target: Dict[str, Any] = field(default_factory=dict)
    timeout_sec: float = 2.0
    safety_mode: str = "normal"


@dataclass(slots=True)
class CommandAck:
    command_id: str
    status: AckStatus
    reason: Optional[str] = None


@dataclass(slots=True)
class VehicleTelemetry:
    timestamp_ms: int
    lat_deg: float
    lon_deg: float
    alt_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    battery_pct: float
    flight_mode: str
    link_quality: float


@dataclass(slots=True)
class PerceptionTelemetry:
    timestamp_ms: int
    source: str
    frame_id: str
    confidence: float
    payload: Dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class ComponentHealth:
    component_name: str
    heartbeat_age_ms: int
    severity: SupervisorState
    diagnostics: Dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class SupervisorSignal:
    timestamp_ms: int
    state: SupervisorState
    action: SupervisorAction
    reason: str
