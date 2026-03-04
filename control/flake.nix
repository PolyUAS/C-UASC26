{
  description = "control systems node -- Rust";

  inputs = {
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable"; # This should point to whichever nixpkgs rev you want.
    naersk.url = "github:nix-community/naersk/master";

  };

  outputs = { flake-parts, ... } @ inputs: flake-parts.lib.mkFlake { inherit inputs; } {
    imports = [
      # ./module.nix
      # inputs.foo.flakeModule
    ];

    perSystem = { config, self', inputs', pkgs, system, ... }: {

      devShells.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          git
          cargo
          rustc
          rustfmt
          pre-commit
          rustPackages.clippy
          flatbuffers
        ];
        RUST_SRC_PATH = pkgs.rustPlatform.rustLibSrc;
      };

      packages.default =
        let
          naersk-lib = pkgs.callPackage inputs.naersk { };
          schemasDir = ../schemas;
        in
        naersk-lib.buildPackage {
          src = ./.;
          pname = "control";
          nativeBuildInputs = [ pkgs.git pkgs.flatbuffers ];
          SCHEMAS_DIR = "${schemasDir}";
        };
    };

    flake = {
      # The usual flake attributes can be defined here, including
      # system-agnostic and/or arbitrary outputs.
    };

    # Declared systems that your flake supports. These will be enumerated in perSystem
    systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
  };
}
