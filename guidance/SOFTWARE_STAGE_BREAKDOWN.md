# Software Stage Breakdown (4-6)

This document focuses on software execution first, with hardware connection deferred until physical integration is scheduled.

## Stage 4 (SITL / Simulation Integration)

### Current state

- Implemented mission command bridge runtime (`guidance-bridge`) from Zenoh command topic to MAVLink adapter dispatch.
- Implemented supervisor-gated bridge behavior driven by `guidance/supervisor/signal`.
- Implemented mission command publisher (`guidance-publish-command`) with command JSON fixtures.
- Implemented acknowledgement monitor (`guidance-ack-monitor`).
- Implemented replay logger to JSONL for deterministic playback.
- Implemented optional wire-format path: JSON (default) and FlexBuffers.
- Implemented MuJoCo telemetry adapter stub (`guidance-mujoco-telemetry`) from `sim/mujoco/state` to `vehicle/telemetry/fc`.

### Remaining software tasks

1. Add mission state telemetry stream topic(s) for state transition observability.
2. Add supervisor action test scenarios in full simulation timelines (hold/rtl/abort cases).
3. Add deterministic scenario runner that replays command/telemetry timelines.
4. Expand simulation adapters for:
   - MuJoCo vehicle dynamics state output (stub implemented, requires simulator topic source hookup).
   - Optional Genesis vision stream output.
   - Optional Gymnasium training loop policy output.
5. Add acceptance tests for simulation cases:
   - nominal mission progression,
   - stale command timeout,
   - geofence breach,
   - telemetry dropout,
   - forced RTL/abort.

### Simulation connection points

- `sim/telemetry -> guidance` via Zenoh topic mapped into `VehicleTelemetry`.
- `sim/perception -> guidance` via Zenoh topic mapped into `PerceptionTelemetry`.
- `guidance/mission/command -> sim/actuation` via bridge and adapter.
- `guidance/supervisor/signal -> sim/failsafe` to validate preemption behavior.

## Stage 5 (Pre-HITL Software Hardening)

Stage 5 software work can be done before plugging hardware in.

### Goals

- Freeze interfaces for FC, camera, gimbal, and air-unit proxies.
- Harden fault-handling and observability before physical tests.

### Tasks

1. Introduce component contract tests for FC/camera/gimbal/air-unit adapters.
2. Add latency budget checks and timeout assertions in CI.
3. Add health model calibration from simulation traces.
4. Add log schema versioning and replay compatibility checks.
5. Add watchdog escalation integration tests (degraded -> critical -> failsafe).

### Exit criteria

- No interface churn in mission command/ack contract.
- Stable replay compatibility across commits.
- Simulation stress suite passing at target rates.

## Stage 6 (Mission Enablement, Software Side)

### Current status

- Waypoint FSM has partial mission logic.
- Other mission FSMs are scaffolded and await detailed state logic.

### Per-mission software sequence

1. Implement task-state transitions and guards.
2. Bind each transition to explicit command intents.
3. Add unit tests for each transition path.
4. Add simulation integration tests with command/ack and telemetry assertions.
5. Add supervisor preemption tests.

### Mission implementation order

1. Waypoint (complete existing logic + adapter bindings).
2. Circuit time trial.
3. Target localization.
4. Package drop.
5. Package delivery.
6. Package recovery.

## Recommended immediate next sprint

1. Add mission-state telemetry topic and structured state snapshots.
2. Add scenario runner for deterministic timeline playback.
3. Add Genesis perception adapter stub and topic bridge.
4. Add Gymnasium policy command adapter stub.