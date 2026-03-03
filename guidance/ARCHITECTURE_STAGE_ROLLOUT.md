# Guidance Rollout Plan (Stages 1-6)

## Current physical topology and constraints

- Jetson `<->` camera/gimbal over Ethernet (`RTSP` for video, `UART` control exposed over network serial bridge if required).
- Jetson `<->` flight controller (FC) over `MAVLink` on direct `UART` cable.
- FC `<->` air unit over `MAVLink` on `UART` cable.

This topology is supported as-is, while still allowing software redesign to reduce coupling and improve failure handling.

## BROS alignment notes

- Reproducibility-first workflow: all build/test/run commands execute through module `nix develop` shells.
- Two-language strategy: Python for mission and orchestration ergonomics, Rust for lower-level reliability and memory safety.
- Pub/sub middleware: Zenoh topics are the default process boundary between guidance, navigation, and future nodes.
- Serialization target: FlatBuffers remains the long-term payload format for low-overhead transport; JSON is currently used for early-stage bridge bring-up and debugging.

## Layered software architecture

1. **Mission layer (Python / guidance)**
   - Event FSMs and scoring logic.
   - Mission-level intent creation (not direct device byte protocol logic).
2. **Autonomy adapter layer (Python)**
   - Converts mission intents into FC actions via MAVLink/ArduPilot APIs.
   - Aggregates telemetry for mission FSMs.
3. **Device reliability layer (Rust / navigation)**
   - Watchdog, health checks, device liveness, link supervision.
   - Supervisory escalation to mission layer (`DEGRADED`, `CRITICAL`, `FAILSAFE`).
4. **Transport layer (Zenoh + FlatBuffers)**
   - Typed cross-process messages, command ack/nack, heartbeats.

## Stage 1: Control contract and schemas

Deliverables:
- Canonical mission command envelope.
- Canonical telemetry envelope.
- Health and watchdog status messages.
- Topic map for Zenoh keys.

Required contracts:
- `MissionCommand`: command ID, mission ID, type, target, timeout, safety mode.
- `CommandAck`: command ID, status (`ACCEPTED`, `REJECTED`, `COMPLETED`, `TIMED_OUT`, `FAILED`), reason.
- `VehicleTelemetry`: position, velocity, battery, mode, link quality.
- `PerceptionTelemetry`: detection payloads, confidence, source timestamp.
- `ComponentHealth`: component name, heartbeat age, severity, diagnostics.
- `SupervisorSignal`: global state and requested action (`CONTINUE`, `HOLD`, `RTL`, `ABORT`).

## Stage 2: Python mission/supervisor interfaces

Deliverables:
- Mission FSM contracts with unified lifecycle and per-event task states.
- Consolidated package operation state machine modes (`DROP`, `DELIVERY`, `RECOVERY`).
- Supervisor preemption policy that can interrupt any mission FSM.

Mission split:
- Standalone: `CircuitTimeTrialFSM`, `TargetLocalizationFSM`.
- Consolidated: `PackageOpsFSM(mode=DROP|DELIVERY|RECOVERY)`.

## Stage 3: Rust lower-level interface and watchdog

Deliverables:
- Rust daemon for FC/camera/gimbal/air-unit health monitoring.
- Watchdog timers per component + aggregate supervisory output.
- Rust publisher/subscriber for contract topics and heartbeat enforcement.

Minimum checks:
- FC MAVLink heartbeat freshness.
- Camera RTSP stream freshness.
- Gimbal control channel responsiveness.
- Air-unit telemetry continuity via FC bridge.

## Stage 4: SITL integration (ArduPilot + MAVLink)

Deliverables:
- SITL harness where Python mission intents command simulated vehicle behavior.
- Contract replay logs from Zenoh for deterministic FSM debugging.
- Safety test scenarios: geofence breach, low battery, telemetry dropout, stale command.

Implemented scaffold:
- `guidance.bridge_main` provides a mission command bridge that subscribes to `guidance/mission/command`, dispatches through a MAVLink adapter, publishes `guidance/mission/ack`, and appends JSONL replay logs.

## Stage 5: HITL integration with SIYI hardware path

Deliverables:
- End-to-end health supervision with physical SIYI FC, air unit, and camera links.
- Latency and timeout thresholds tuned from measured hardware behavior.
- Recovery actions validated: hold, RTL, abort.

## Stage 6: Mission-by-mission enablement

Order:
1. Waypoint (already partially built).
2. Circuit time trial.
3. Target ID localization.
4. Package ops mode: drop.
5. Package ops mode: delivery.
6. Package ops mode: recovery.

For each mission, complete:
- Unit tests for task states and transitions.
- Integration tests for mission + supervisor arbitration.
- SITL pass criteria before hardware test.

## Initial topic map (Zenoh)

- `guidance/mission/command`
- `guidance/mission/ack`
- `guidance/supervisor/signal`
- `vehicle/telemetry/fc`
- `vehicle/telemetry/perception`
- `vehicle/health/component`
- `vehicle/watchdog/heartbeat`

## Hardware-aware redesign notes

- Keep FC as single source of truth for flight authority whenever possible.
- Place camera/gimbal direct controls behind Rust reliability service even if mission logic remains in Python.
- Prevent split-brain control by requiring supervisor grant before mission command dispatch.