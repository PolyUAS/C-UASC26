{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";

    zenohd.url = ./zenohd;

    vision.url = ./vision;

    control.url = ./control;
    guidance.url = ./guidance;
    navigation.url = ./navigation;

  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      imports = [
        # ./module.nix
        # inputs.foo.flakeModule
      ];

      perSystem =
        {
          config,
          self',
          inputs',
          pkgs,
          system,
          ...
        }:
        let
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
              mode: "client",
              connect: {
                endpoints: ["tcp/127.0.0.1:7447"]
              },
              scouting: {
                multicast: {
                  enabled: false
                }
              }
            }
          '';

          routerCfg = pkgs.writeText "zenoh-router-cfg.json" ''
            {
              mode: "router",
              listen: {
                endpoints: ["tcp/127.0.0.1:7447"]
              }
            }
          '';
        in
        {
          # Expose subproject packages for composition
          packages = {

            vision = inputs.vision.packages.${system}.default;

            control = inputs.control.packages.${system}.default;

            zenohd = inputs.zenohd.packages.${system}.default;

            # Flight stack modules
            guidance = inputs.guidance.packages.${system}.default;
            navigation = inputs.navigation.packages.${system}.default;

            # Launcher: Spins up all with shared config

            generate_schemas = pkgs.writeShellApplication {
              name = "generate_schemas";
              runtimeInputs = [
                pkgs.flatbuffers
              ];
              text = ''
                for dir in */; do
                    [ -d "$dir" ] || continue

                    # strip trailing slash for cleaner paths
                    d="$dir"

                    # check if either pattern exists
                    if [ -d "$d/src/generated" ]; then
                        flatc --rust   -o "$d/src/generated/"     schemas/* 2>/dev/null
                    fi

                    if [ -d "$d/src/$d/generated" ] && [ ! -d "$d/src/generated" ]; then
                        flatc --python -o "$d/src/$d/generated/" schemas/* 2>/dev/null
                    fi
                done
              '';
            };

            default = pkgs.writeShellApplication {
              name = "run_CUASC26";
              runtimeInputs = with self'.packages; [
                zenohd

                control
                guidance
                # navigation
                # vision
                #
                jetson_set_dev_power
                jetson_reset_power
              ];

              text = ''
                export MAVLINK_CONN="serial:/dev/ttyTHS1:256000"
                # export MAVLINK_CONN="udpin:0.0.0.0:14551";


                # export DRONE_NAME="HORUS"
                export DRONE_NAME="OSIRUS"


                jetson_set_dev_power
                export ZENOH_CONFIG=${sharedConfig}
                echo "Launching with shared config: $ZENOH_CONFIG"

                zenohd -c ${routerCfg} 1>/dev/null &
                ZENOH_PID=$!

                sleep 3

                control &
                CONTROL_PID=$!

                guidance &
                GUIDANCE_PID=$!

                trap 'kill $ZENOH_PID $GUIDANCE_PID $CONTROL_PID 2>/dev/null && jetson_reset_power' EXIT INT TERM

                wait $CONTROL_PID $GUIDANCE_PID

              '';
            };

            jetson_set_dev_power = pkgs.writeShellApplication {
              name = "jetson_set_dev_power";
              runtimeInputs = [ ];

              text = ''
                sudo nvpmodel -m 2
              '';
            };

            jetson_reset_power = pkgs.writeShellApplication {
              name = "jetson_reset_power";
              runtimeInputs = [ ];
              text = ''
                sudo nvpmodel -m 0
              '';
            };

            sitl = pkgs.writeShellApplication {
              name = "sitl_CUASC26";
              runtimeInputs = with self'.packages; [
                zenohd

                control
                guidance
                # navigation
                # vision
                #
                # jetson_set_dev_power
                # jetson_reset_power
              ];

              text = ''
                # export MAVLINK_CONN="serial:/dev/ttyTHS1:256000"
                export MAVLINK_CONN="udpin:0.0.0.0:14551";


                # export DRONE_NAME="HORUS"
                export DRONE_NAME="OSIRUS"


                # jetson_set_dev_power
                export ZENOH_CONFIG=${sharedConfig}
                echo "Launching with shared config: $ZENOH_CONFIG"

                zenohd -c ${routerCfg} 1>/dev/null &
                ZENOH_PID=$!

                sleep 3

                control &
                CONTROL_PID=$!

                guidance &
                GUIDANCE_PID=$!

                trap 'kill $ZENOH_PID $GUIDANCE_PID $CONTROL_PID 2>/dev/null' EXIT INT TERM

                wait $CONTROL_PID $GUIDANCE_PID

              '';
            };

          };

          devShells.default = pkgs.mkShell {
            packages = [

              self'.packages.vision

              self'.packages.control

              self'.packages.zenohd
              self'.packages.generate_schemas
              self'.packages.default

              pkgs.just
              pkgs.uv
              pkgs.flatbuffers

              pkgs.btop

            ];
            env.ZENOH_CONFIG = sharedConfig;

            env.MAVLINK_CONN = "serial:/dev/ttyTHS1:256000";
            # env.MAVLINK_CONN = "udpin:0.0.0.0:14550";

            env.ZENOHD_CONFIG = routerCfg;
            env.GIMBAL_OFFSET_MM = "152,0,0";

            # env.DRONE_NAME = "HORUS";
            env.DRONE_NAME = "OSIRUS";

            shellHook = ''
              export ZENOH_CONFIG=${sharedConfig}
              alias j="just"
              echo "just is aliased to j"
            '';
          };


          test_adhoc = pkgs.writeShellApplication {
            name = "test_adhoc";
            runtimeInputs = [];
            text = ''
                sudo nmcli device wifi hotspot \
                    ifname wlP1p1s0 \
                    ssid JetsonAP \
                    password jetson123
            '';
          };


        };
      flake = {
        # The usual flake attributes can be defined here, including
        # system-agnostic and/or arbitrary outputs.
      };
    };
}
