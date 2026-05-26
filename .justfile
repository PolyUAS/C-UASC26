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

# creates a new node from the script, with NAME as the node name, and TEMPLATE as the template name inside of .build_utils/templates
new NAME TEMPLATE:
    @ ./.build_utils/create.py {{ NAME }} {{ TEMPLATE }}

# opens the nix shell
develop:
    @nix develop

ARDUPILOT_DIR := justfile_directory() + "/ardupilot"
ARDUPILOT_REPO := "https://github.com/ArduPilot/ardupilot.git"
SITL_DEPS := "pexpect empy MAVProxy future gnureadline wxPython matplotlib PyOpenGL pymavlink pillow requests opencv-python"
VEHICLE := "ArduCopter"
SITL_ARGS := "-w "

clone-ardupilot:
    @if [ ! -e {{ ARDUPILOT_DIR }} ]; then \
        git clone --recurse-submodules {{ ARDUPILOT_REPO }} {{ ARDUPILOT_DIR }}; \
    elif [ -d {{ ARDUPILOT_DIR }}/.git ]; then \
        cd {{ ARDUPILOT_DIR }} && git submodule update --init --recursive; \
    else \
        echo "{{ ARDUPILOT_DIR }} exists but is not a git repo"; \
        exit 1; \
    fi

setup-sitl: clone-ardupilot
    @cd {{ ARDUPILOT_DIR }} && \
        UV_VENV_CLEAR=1 uv venv .venv --python 3.12 && \
        uv pip install {{ SITL_DEPS }}

sitl: setup-sitl
    @cd {{ ARDUPILOT_DIR }} && \
        uv run python Tools/autotest/sim_vehicle.py -v {{ VEHICLE }} {{ SITL_ARGS }}

MAVPROXY_ARGS := "--master=udp:127.0.0.1:14550 --console --map  --out=udp:127.0.0.1:14551"

mavproxy:
    @cd {{ ARDUPILOT_DIR }} && \
        uv run python -m MAVProxy.mavproxy {{ MAVPROXY_ARGS }}
