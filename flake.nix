{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    rust-demo-sub.url = "./rust_sub";
    python-demo-sub.url = "./python_sub";
    simpledemo.url = "./simpledemo";

    vision-node.url = "./vision";

    control.url = "./control";
    guidance.url = "./guidance";
    navigation.url = "./navigation";

    mujoco-sim.url = "./mujoco-sim";
  };

  outputs = inputs @ { flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];

      imports = [
        # ./module.nix
        # inputs.foo.flakeModule
      ];

      perSystem = { config, self', inputs', pkgs, system, ... }: let
        # Allows definition of system-specific attributes
        # without needing to declare the system explicitly!
        #
        # Quick rundown of the provided arguments:
        # - config is a reference to the full configuration, lazily evaluated
        # - self' is the outputs as provided here, without system. (self'.packages.default)
        # - inputs' is the input without needing to specify system (inputs'.foo.packages.bar)
        # - pkgs is an instance of nixpkgs for your specific system
        # - system is the system this configuration is for

        # This is equivalent to packages.<system>.default

        # Generate shared Zenoh config (customize as needed; could derive from template)
        sharedConfig = pkgs.writeText "zenoh-config.json" ''
          {
            "mode": "peer",
            "listen": { "endpoints": ["tcp/127.0.0.1:0"] },
            "scouting": {
              "multicast": {
                "enabled": true
              }
            }
          }
        '';
      in {
        # Expose subproject packages for composition
        packages = {
          rust_demo = inputs.rust-demo-sub.packages.${system}.default;
          python_demo = inputs.python-demo-sub.packages.${system}.default;
          hello_world = inputs.simpledemo.packages.${system}.default;
          vision = inputs.vision-node.packages.${system}.default;

          control = inputs.control.packages.${system}.default;

          # Flight stack modules
          guidance   = inputs.guidance.packages.${system}.default;
          navigation = inputs.navigation.packages.${system}.default;

          mujoco-sim = inputs.mujoco-sim.packages.${system}.default;

          # Launcher: Spins up all with shared config
          demo = pkgs.writeShellApplication {
            name = "demo-ping-pong-zenoh";
            runtimeInputs = [
              self'.packages.rust_demo
              self'.packages.python_demo
            ];
            text = ''
              export ZENOH_CONFIG=${sharedConfig}
              echo "Launching with shared config: $ZENOH_CONFIG"
              # Start processes in background
              hello &
              PYTHON_PID=$!
              rust-zenoh-app &
              RUST_PID=$!
              # Trap EXIT and SIGINT (Ctrl+C)
              trap 'kill $PYTHON_PID $RUST_PID 2>/dev/null' EXIT SIGINT
              # Wait for both processes
              wait $PYTHON_PID $RUST_PID
            '';
          };

          # Sim stack: MuJoCo physics + guidance bridge + navigation (no hardware)
          sim-stack = pkgs.writeShellApplication {
            name = "sim-stack";
            runtimeInputs = [
              self'.packages.mujoco-sim
              self'.packages.navigation
            ];
            text = ''
              export ZENOH_CONFIG=${sharedConfig}
              export SIM_MODE=1
              echo "[sim-stack] Starting MuJoCo sim stack..."

              PIDS=""
              cleanup() {
                echo "[sim-stack] Shutting down..."
                for pid in $PIDS; do
                  kill "$pid" 2>/dev/null || true
                done
              }
              trap cleanup EXIT SIGINT SIGTERM

              # 1. MuJoCo sim (publishes sim/mujoco/state + heartbeats)
              mujoco-sim &
              PIDS="$PIDS $!"

              # 2. Navigation watchdog
              navigation &
              PIDS="$PIDS $!"

              echo "[sim-stack] All processes started. Ctrl+C to stop."
              wait
            '';
          };

          # SITL stack: ArduPilot SITL + control node + guidance + navigation
          sitl-stack = pkgs.writeShellApplication {
            name = "sitl-stack";
            runtimeInputs = [
              self'.packages.control
              self'.packages.navigation
            ];
            text = ''
              export ZENOH_CONFIG=${sharedConfig}
              MAVLINK_CONN="''${MAVLINK_CONN:-tcp:127.0.0.1:5760}"
              export MAVLINK_CONN
              echo "[sitl-stack] Starting SITL stack (MAVLINK_CONN=$MAVLINK_CONN)..."

              PIDS=""
              cleanup() {
                echo "[sitl-stack] Shutting down..."
                for pid in $PIDS; do
                  kill "$pid" 2>/dev/null || true
                done
              }
              trap cleanup EXIT SIGINT SIGTERM

              # 1. Control bridge (MAVLink ↔ Zenoh)
              control &
              PIDS="$PIDS $!"

              # 2. Navigation watchdog
              navigation &
              PIDS="$PIDS $!"

              echo "[sitl-stack] All processes started. Ctrl+C to stop."
              wait
            '';
          };

          default = self'.packages.demo;

          reset_jetson_network = pkgs.writeShellApplication {
            name = "reset_jetson_network";
            runtimeInputs = [

            ];
            text = ''
                sudo ip addr flush dev enP8p1s0
                sudo ip addr add 192.168.144.30/24 dev enP8p1s0
                sudo ip link set enP8p1s0 up
            '';
          };


        };

        devShells.default = pkgs.mkShell {
          packages = [
            self'.packages.demo
            self'.packages.rust_demo
            self'.packages.python_demo
            self'.packages.hello_world

            self'.packages.vision

            self'.packages.reset_jetson_network
            self'.packages.control

            pkgs.just
            pkgs.uv
            pkgs.flatbuffers
            pkgs.htop
          ];
          env.ZENOH_CONFIG = sharedConfig;
          env.MAVLINK_CONN = "serial:/dev/ttyTHS1:256000";
          shellHook = ''
            export ZENOH_CONFIG=${sharedConfig}
            alias j="just"
            echo "Run 'demo-ping-pong-zenoh' to start demo."
            echo "just is aliased to j"
          '';
        };
      };
      flake = {
        # The usual flake attributes can be defined here, including
        # system-agnostic and/or arbitrary outputs.
      };
    };
}
