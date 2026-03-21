#!/usr/bin/env bash
set -euo pipefail

# cmd_center.sh - Spin up parallel terminal processes for each just command
# Usage: ./scripts/cmd_center.sh <recipe1> <recipe2> ... <recipeN>
# Example: ./scripts/cmd_center.sh build run test lint

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
JUSTFILE="$PROJECT_ROOT/.justfile"
LOG_DIR="$PROJECT_ROOT/.cmd_center_logs"
SESSION_NAME="cmd_center_$$"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

usage() {
    echo -e "${CYAN}Usage:${NC} $0 <command1> [command2] ... [commandN]"
    echo ""
    echo "Spins up a separate process/terminal for each specified just recipe,"
    echo "ensuring 'nix develop' is entered before running 'j <command>'."
    echo ""
    echo -e "${CYAN}Options:${NC}"
    echo "  -l, --list       List available just recipes from .justfile"
    echo "  -t, --tmux       Use tmux split panes (default if tmux is available)"
    echo "  -b, --background Run all commands in background processes with log files"
    echo "  -h, --help       Show this help message"
    echo ""
    echo -e "${CYAN}Examples:${NC}"
    echo "  $0 build run test"
    echo "  $0 --background build lint test"
    echo "  $0 --list"
}

list_recipes() {
    if [[ -f "$JUSTFILE" ]]; then
        echo -e "${CYAN}Available recipes in .justfile:${NC}"
        grep -E '^[a-zA-Z_][a-zA-Z0-9_-]*\s*:' "$JUSTFILE" | sed 's/\s*:.*//' | sort | while read -r recipe; do
            echo "  - $recipe"
        done
    else
        echo -e "${RED}Error: .justfile not found at $JUSTFILE${NC}" >&2
        exit 1
    fi
}

validate_recipe() {
    local recipe="$1"
    if [[ -f "$JUSTFILE" ]]; then
        if ! grep -qE "^${recipe}\s*:" "$JUSTFILE"; then
            echo -e "${YELLOW}Warning: '${recipe}' not found in .justfile (will attempt anyway)${NC}" >&2
        fi
    fi
}

# Generate the nix develop + just command wrapper
gen_nix_just_cmd() {
    local recipe="$1"
    # Uses nix develop to drop into the dev shell, then runs `j <recipe>`
    # 'j' is assumed to be an alias for 'just' set up inside the nix develop shell
    cat <<EOF
cd "$PROJECT_ROOT" && nix develop --command bash -c 'just $recipe'
EOF
}

run_with_tmux() {
    local commands=("$@")
    local num_cmds=${#commands[@]}

    if ! command -v tmux &>/dev/null; then
        echo -e "${RED}Error: tmux is not installed. Use --background mode instead.${NC}" >&2
        exit 1
    fi

    # Kill existing session with same name if it exists
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

    # Create a new tmux session with the first command
    local first_cmd
    first_cmd="$(gen_nix_just_cmd "${commands[0]}")"
    tmux new-session -d -s "$SESSION_NAME" -n "${commands[0]}" "bash -c '${first_cmd}; echo -e \"\\n${GREEN}[${commands[0]}] finished. Press enter to close.${NC}\"; read'"

    # Create additional panes/windows for remaining commands
    for ((i = 1; i < num_cmds; i++)); do
        local cmd
        cmd="$(gen_nix_just_cmd "${commands[$i]}")"
        tmux new-window -t "$SESSION_NAME" -n "${commands[$i]}" "bash -c '${cmd}; echo -e \"\\n${GREEN}[${commands[$i]}] finished. Press enter to close.${NC}\"; read'"
    done

    echo -e "${GREEN}Started tmux session '${SESSION_NAME}' with ${num_cmds} command(s):${NC}"
    for cmd in "${commands[@]}"; do
        echo -e "  ${CYAN}→${NC} j $cmd"
    done
    echo ""
    echo -e "Attach with: ${YELLOW}tmux attach -t ${SESSION_NAME}${NC}"
    echo -e "Kill all with: ${YELLOW}tmux kill-session -t ${SESSION_NAME}${NC}"

    # Auto-attach
    tmux attach -t "$SESSION_NAME"
}

run_in_background() {
    local commands=("$@")

    mkdir -p "$LOG_DIR"

    local pids=()
    local timestamp
    timestamp="$(date +%Y%m%d_%H%M%S)"

    echo -e "${GREEN}Starting ${#commands[@]} command(s) in background...${NC}"
    echo -e "${CYAN}Logs directory:${NC} $LOG_DIR"
    echo ""

    for recipe in "${commands[@]}"; do
        local logfile="$LOG_DIR/${recipe}_${timestamp}.log"
        local cmd
        cmd="$(gen_nix_just_cmd "$recipe")"

        echo -e "  ${CYAN}→${NC} j ${recipe} (PID: pending, log: ${logfile})"

        bash -c "$cmd" > "$logfile" 2>&1 &
        local pid=$!
        pids+=("$pid")
        echo -e "    ${GREEN}Started${NC} PID: ${pid}"
    done

    echo ""
    echo -e "${GREEN}All commands launched.${NC}"
    echo -e "${YELLOW}PIDs:${NC} ${pids[*]}"
    echo ""

    # Store PIDs for cleanup
    local pidfile="$LOG_DIR/pids_${timestamp}.txt"
    printf '%s\n' "${pids[@]}" > "$pidfile"
    echo -e "PID file: ${pidfile}"
    echo -e "Kill all: ${YELLOW}kill ${pids[*]}${NC}"
    echo ""

    # Wait and report
    echo -e "${CYAN}Waiting for all processes to complete...${NC}"
    echo -e "Press Ctrl+C to stop waiting (processes will continue in background)."
    echo ""

    local all_success=true
    for i in "${!pids[@]}"; do
        local pid="${pids[$i]}"
        local recipe="${commands[$i]}"
        if wait "$pid" 2>/dev/null; then
            echo -e "  ${GREEN}✓${NC} ${recipe} (PID ${pid}) completed successfully"
        else
            echo -e "  ${RED}✗${NC} ${recipe} (PID ${pid}) failed (exit code: $?)"
            all_success=false
        fi
    done

    echo ""
    if $all_success; then
        echo -e "${GREEN}All commands completed successfully!${NC}"
        exit 0
    else
        echo -e "${RED}Some commands failed. Check logs in ${LOG_DIR}${NC}"
        exit 1
    fi
}

# --- Main ---

MODE="tmux"  # default

# Parse flags
RECIPES=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -l|--list)
            list_recipes
            exit 0
            ;;
        -t|--tmux)
            MODE="tmux"
            shift
            ;;
        -b|--background)
            MODE="background"
            shift
            ;;
        -*)
            echo -e "${RED}Unknown option: $1${NC}" >&2
            usage
            exit 1
            ;;
        *)
            RECIPES+=("$1")
            shift
            ;;
    esac
done

if [[ ${#RECIPES[@]} -eq 0 ]]; then
    echo -e "${RED}Error: No commands specified.${NC}" >&2
    echo ""
    usage
    list_recipes 2>/dev/null || true
    exit 1
fi

# Validate recipes
for recipe in "${RECIPES[@]}"; do
    validate_recipe "$recipe"
done

echo -e "${GREEN}Command Center${NC} - launching ${#RECIPES[@]} process(es)"
echo -e "Mode: ${YELLOW}${MODE}${NC}"
echo ""

case "$MODE" in
    tmux)
        run_with_tmux "${RECIPES[@]}"
        ;;
    background)
        run_in_background "${RECIPES[@]}"
        ;;
esac