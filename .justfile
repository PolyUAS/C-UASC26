alias r := run
alias b := build
alias c := clean
alias d := develop
alias n := new

# default target, lists all targets available
list:
    @just --list

# runs the default master nix flake target
run:
    @nix run

# builds the nix targets
build:
    @nix build

# removes all build artifacts and binaries
clean:
    @rm -rf result target
    @ ./.build_utils/clean.py

# runs clean and destroys the machine's nix cache (used to free all system storage memory related to this project)
nuke: clean
    @nix-collect-garbage

# creates a new node from the script, with NAME as the node name, and BUILD as the build type (currently python or rust)
new NAME BUILD:
    @ ./.build_utils/new_node.py {{ NAME }} {{ BUILD }}

# opens the nix shell
develop:
    @nix develop

# --- Sim stack (MuJoCo physics, no hardware) ---

# Start full sim stack (mujoco-sim + navigation)
sim-run:
    nix run .#sim-stack

# Run only MuJoCo sim node
sim-mujoco:
    cd mujoco-sim && nix develop -c bash --noprofile --norc -lc 'cargo run'

# Run only guidance MuJoCo telemetry adapter
sim-telemetry:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m guidance.adapters.mujoco_telemetry'

# Run guidance bridge in SIM mode
sim-bridge:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'SIM_MODE=1 MAVLINK_CONNECTION=sim python -m guidance.bridge_main'

# --- SITL stack (ArduPilot SITL + control, start SITL externally first) ---

# Start full SITL stack (control + navigation)
sitl-run:
    nix run .#sitl-stack

# Run only control node (MAVLink bridge)
sitl-control:
    cd control && nix develop -c bash --noprofile --norc -lc 'MAVLINK_CONN="${MAVLINK_CONN:-tcp:127.0.0.1:5760}" cargo run'

# --- Shared tooling ---

# Watch vehicle telemetry topic (FlatBuffers)
watch-telemetry:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -c "\
    import zenoh, time;\
    from guidance.generated.vehicle import VehicleTelemetry;\
    session = zenoh.open(zenoh.Config());\
    sub = session.declare_subscriber(\"vehicle/telemetry/fc\");\
    print(\"Watching vehicle/telemetry/fc (FlatBuffers)...\");\
    while True:\
        s = sub.recv();\
        buf = bytes(s.payload.to_bytes());\
        t = VehicleTelemetry.VehicleTelemetry.GetRootAs(buf, 0);\
        print(f\"alt={t.AltM():.1f}m vn={t.VnMps():.2f} mode={t.FlightMode()}\")\
    "'

# Run navigation watchdog
watch-nav:
    cd navigation && nix develop -c bash --noprofile --norc -lc 'cargo run'

# Inject a goto command from JSON file
inject-goto:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m guidance.command_publisher'

# Watch command acks
watch-ack:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m guidance.ack_monitor'

# Build all flight stack modules
build-all:
    nix build .#guidance
    nix build .#navigation
    nix build .#control
    nix build .#mujoco-sim

# Run all guidance tests
test-guidance:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m pytest tests/ -v'

# Run only unit tests
test-unit:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m pytest tests/unit/ -v'

# Run only integration tests
test-integration:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m pytest tests/integration/ -v'

# Run only simulation tests (includes Zenoh dataflow)
test-sim:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m pytest tests/simulation/ -v'

# Run Zenoh dataflow test bench
test-dataflow:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python -m pytest tests/simulation/test_zenoh_dataflow.py -v'

# Generate Python FlatBuffers code from schemas
generate-flatbuffers:
    nix develop -c bash --noprofile --norc -lc 'bash scripts/generate_flatbuffers.sh'

# --- Debug / monitoring tools ---

# Live Zenoh topic dashboard (all topics, refreshes every 1s)
monitor-dashboard:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_topic_monitor.py --dashboard'

# Stream Zenoh messages to stdout (firehose mode)
monitor-stream:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_topic_monitor.py'

# Track flight-brain state transitions
monitor-brain:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_brain_tracker.py'

# Track brain state and log to file
monitor-brain-log:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_brain_tracker.py --log logs/brain_transitions.jsonl'

# Inject a test command sequence (arm → takeoff → goto → hold → rtl)
inject-sequence:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_inject_command.py sequence'

# Inject a single arm command
inject-arm:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_inject_command.py arm'

# Inject a single takeoff command
inject-takeoff:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_inject_command.py takeoff --alt 30'

# Inject a goto command with custom coordinates
inject-goto-custom LAT LON ALT:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/zenoh_inject_command.py goto --lat {{ LAT }} --lon {{ LON }} --alt {{ ALT }}'

# --- Command Center & Viewers ---

# Launch the full Command Center TUI dashboard (all 6 panels)
cmd-center:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/cmd_center.py'

# Command Center with faster refresh
cmd-center-fast:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/cmd_center.py --refresh 0.3'

# Command Center inspecting a specific topic
cmd-center-inspect TOPIC:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'python ../scripts/cmd_center.py --inspect {{ TOPIC }}'

# Launch MuJoCo 3D viewer (requires mujoco Python package)
viewer-3d:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'uv sync --group viewer && python ../scripts/mujoco_viewer.py'

# MuJoCo viewer with flight trail enabled
viewer-3d-trail:
    cd guidance && nix develop -c bash --noprofile --norc -lc 'uv sync --group viewer && python ../scripts/mujoco_viewer.py --trail'

# Full sim experience: sim-stack + command center + viewer (use with tmux or separate terminals)
full-sim-demo:
    @echo "=== BROS Full Sim Demo ==="
    @echo "Run these in separate terminals:"
    @echo "  Terminal 1: just sim-run        # MuJoCo sim + navigation"
    @echo "  Terminal 2: just cmd-center     # Command Center dashboard"
    @echo "  Terminal 3: just viewer-3d      # MuJoCo 3D viewer"
    @echo "  Terminal 4: just inject-sequence # Send test commands"
