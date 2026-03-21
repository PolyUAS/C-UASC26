#!/usr/bin/env bash
# Generate Python FlatBuffers code from shared schemas.
# Intended to be run from the repo root or from a Nix shell that has flatc.
#
# Output goes to guidance/src/guidance/generated/ which is importable as:
#   from guidance.generated.vehicle import VehicleTelemetry
#   from guidance.generated.guidance import MissionCommand, CommandAck, ...

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCHEMA_DIR="$REPO_ROOT/schemas"
OUT_DIR="$REPO_ROOT/guidance/src/guidance/generated"

# Clean previous output
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

# Generate
flatc --python -o "$OUT_DIR" \
    "$SCHEMA_DIR/vehicle_telemetry.fbs" \
    "$SCHEMA_DIR/mission_command.fbs" \
    "$SCHEMA_DIR/command_ack.fbs" \
    "$SCHEMA_DIR/supervisor_signal.fbs" \
    "$SCHEMA_DIR/gimbal_state.fbs"

# flatc creates namespace subdirectories (vehicle/, guidance/, gimbal/)
# Ensure each has __init__.py for Python package imports
for dir in "$OUT_DIR"/*/; do
    if [ -d "$dir" ]; then
        touch "$dir/__init__.py"
    fi
done

# Root __init__.py
cat > "$OUT_DIR/__init__.py" << 'EOF'
"""Auto-generated FlatBuffers Python code.

Do NOT edit. Regenerate via:
    scripts/generate_flatbuffers.sh

Or from the guidance dev shell:
    cd $REPO_ROOT && ./scripts/generate_flatbuffers.sh
"""
EOF

echo "FlatBuffers Python codegen complete → $OUT_DIR"
find "$OUT_DIR" -name '*.py' | sort
