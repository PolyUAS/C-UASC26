# Development Workflow (Nix-first)

Use Nix shells for every build/test/lint command so toolchains match CI and avoid host dependency drift.

## General rules

- Run commands with `nix develop -c ...` in the module directory (`guidance`, `navigation`, etc.).
- Prefer `bash --noprofile --norc -lc '...'` when executing multiple commands to avoid host shell startup side effects.
- Keep Rust and Python checks scoped to their module shell.

## Guidance (Python)

From repo root:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'python -m compileall src && pytest -q'
```

## Navigation (Rust)

From repo root:

```bash
cd navigation
nix develop -c bash --noprofile --norc -lc 'cargo fmt --check && cargo clippy -- -D warnings && cargo check'
```

## Zenoh node runs

Use module-specific shells for direct node runs:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'python -m guidance'
```

Run mission command/ack bridge (SITL-oriented):

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'MAVLINK_CONNECTION=udpin:127.0.0.1:14550 GUIDANCE_REPLAY_PATH=logs/mission_dispatch.jsonl guidance-bridge'
```

Run bridge with optional FlexBuffers wire format:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'GUIDANCE_WIRE_FORMAT=flexbuffers MAVLINK_CONNECTION=udpin:127.0.0.1:14550 guidance-bridge'
```

Publish a command fixture to the bridge:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'guidance-publish-command --file examples/mission_commands/arm.json'
```

Publish in FlexBuffers wire format:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'guidance-publish-command --wire-format flexbuffers --file examples/mission_commands/goto.json'
```

Monitor acknowledgement messages:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'guidance-ack-monitor'
```

Run MuJoCo telemetry adapter stub:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'guidance-mujoco-telemetry'
```

Publish a supervisor signal (JSON example) to test bridge gating:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'python - <<"PY"
import zenoh, json
payload = {
	"timestamp_ms": 1,
	"state": "critical",
	"action": "rtl",
	"reason": "simulated_link_loss"
}
session = zenoh.open(zenoh.Config().from_env())
pub = session.declare_publisher("guidance/supervisor/signal")
pub.put(json.dumps(payload))
session.close()
PY'
```

Recommended 3-terminal loop:

1. Terminal A: ArduPilot SITL + MAVLink endpoint (`udpin:127.0.0.1:14550`).
2. Terminal B: `guidance-bridge` process.
3. Terminal C: `guidance-publish-command --file examples/mission_commands/goto.json`.

Recommended 5-terminal simulation orchestration loop:

1. Terminal A: ArduPilot SITL + MAVLink endpoint (`udpin:127.0.0.1:14550`).
2. Terminal B: `guidance-bridge`.
3. Terminal C: `guidance-ack-monitor`.
4. Terminal D: `guidance-mujoco-telemetry`.
5. Terminal E: simulator publishers (`sim/mujoco/state`) + `guidance-publish-command` commands.

Watch for:
- Ack messages on `guidance/mission/ack`.
- Replay lines appended at `logs/mission_dispatch.jsonl`.

```bash
cd navigation
nix develop -c bash --noprofile --norc -lc 'cargo run'
```

## SITL-stage practice

- Keep mission logic (`guidance`) and watchdog/device reliability (`navigation`) as separate processes.
- Exchange intents/health over Zenoh topics defined in `guidance/ARCHITECTURE_STAGE_ROLLOUT.md`.
- Validate with simulated adapters before hardware wiring changes.
