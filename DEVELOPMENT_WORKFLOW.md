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

Publish a command fixture to the bridge:

```bash
cd guidance
nix develop -c bash --noprofile --norc -lc 'guidance-publish-command --file examples/mission_commands/arm.json'
```

Recommended 3-terminal loop:

1. Terminal A: ArduPilot SITL + MAVLink endpoint (`udpin:127.0.0.1:14550`).
2. Terminal B: `guidance-bridge` process.
3. Terminal C: `guidance-publish-command --file examples/mission_commands/goto.json`.

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
