{
  description = "MuJoCo drone simulation node -- Rust";

  inputs = {
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    naersk.url = "github:nix-community/naersk/master";
  };

  outputs = { flake-parts, ... } @ inputs: flake-parts.lib.mkFlake { inherit inputs; } {
    imports = [];

    perSystem = { config, self', inputs', pkgs, system, ... }:
      let
        schemasDir = ../schemas;
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            git
            cargo
            rustc
            rustfmt
            pre-commit
            rustPackages.clippy
            mujoco
            libGL
            libGLU
            libx11
            libxrandr
          ];
          RUST_SRC_PATH = pkgs.rustPlatform.rustLibSrc;
          MUJOCO_DIR = "${pkgs.mujoco}";
          shellHook = ''
            export LD_LIBRARY_PATH="${pkgs.mujoco}/lib:${pkgs.libGL}/lib:$LD_LIBRARY_PATH"
          '';
        };

        packages.default =
          let
            naersk-lib = pkgs.callPackage inputs.naersk {};
          in
          naersk-lib.buildPackage {
            src = ./.;
            pname = "mujoco-sim";
            nativeBuildInputs = with pkgs; [ git mujoco ];
            buildInputs = with pkgs; [ mujoco libGL ];
            MUJOCO_DIR = "${pkgs.mujoco}";
            SCHEMAS_DIR = "${schemasDir}";
          };
      };

    flake = {};

    systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
  };
}
