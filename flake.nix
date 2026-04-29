{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = github:NixOS/nixpkgs/nixos-unstable;
    flake-parts.url = github:hercules-ci/flake-parts;



    zenohd.url = ./zenohd;

    vision.url = ./vision;

    control.url = ./control;
    guidance.url = ./guidance;
    navigation.url = ./navigation;

    mujoco-sim.url = ./mujoco-sim;
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



          vision = inputs.vision.packages.${system}.default;

          control = inputs.control.packages.${system}.default;

          zenohd = inputs.zenohd.packages.${system}.default;


          # Flight stack modules
          guidance   = inputs.guidance.packages.${system}.default;
          navigation = inputs.navigation.packages.${system}.default;

          mujoco-sim = inputs.mujoco-sim.packages.${system}.default;

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



          default = self'.packages.reset_jetson_network;

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

            self'.packages.vision

            self'.packages.reset_jetson_network
            self'.packages.control

            pkgs.just
            pkgs.uv
            pkgs.flatbuffers
            self'.packages.zenohd
            pkgs.btop
            self'.packages.generate_schemas


          ];
          env.ZENOH_CONFIG = sharedConfig;
          env.MAVLINK_CONN = "serial:/dev/ttyTHS1:256000";


          env.DRONE_NAME = "HORUS";
          # env.DRONE_NAME = "OSIRUS";

          shellHook = ''
            export ZENOH_CONFIG=${sharedConfig}
            alias j="just"
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
