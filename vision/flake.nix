{
  description = "vision — Python Zenoh Node";
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-24.11";
    pyproject-nix = {
      url = "github:pyproject-nix/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    uv2nix = {
      url = "github:pyproject-nix/uv2nix";
      inputs.pyproject-nix.follows = "pyproject-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    pyproject-build-systems = {
      url = "github:pyproject-nix/build-system-pkgs";
      inputs.pyproject-nix.follows = "pyproject-nix";
      inputs.uv2nix.follows = "uv2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs = {
    nixpkgs,
    pyproject-nix,
    uv2nix,
    pyproject-build-systems,
    ...
  }:
    let
      inherit (nixpkgs) lib;
      forAllSystems = lib.genAttrs lib.systems.flakeExposed;
      workspace = uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ./.; };
      overlay = workspace.mkPyprojectOverlay {
        sourcePreference = "wheel";
      };
      editableOverlay = workspace.mkEditablePyprojectOverlay {
        root = "$REPO_ROOT";
      };

      # Patch nvidia-cufile to ignore libibverbs (GPUDirect RDMA, not present on Jetson)
      # Patch all nvidia-* CUDA wheel packages that have cross-package .so dependencies.
      # These wheels are designed to resolve each other at runtime, but Nix's auto-patchelf
      # runs at build time and can't see across derivation boundaries.
      pyprojectOverrides = final: prev:
        let
          pkgs = nixpkgs.legacyPackages.${final.python.stdenv.hostPlatform.system};
          ignoreAutoPatchelf = pkg: pkg.overrideAttrs (_: {
            autoPatchelfIgnoreMissingDeps = true;
          });
          needsIgnore = name:
            lib.hasPrefix "nvidia-" name ||
            lib.hasPrefix "torch"    name ||
            name == "torch"           ||
            name == "torchvision"     ||
            name == "onnxruntime-gpu";
        in
        lib.mapAttrs
          (name: pkg:
            if needsIgnore name
            then ignoreAutoPatchelf pkg
            else pkg
          )
          prev;


      pythonSets = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          python = pkgs.python310;
        in
        (pkgs.callPackage pyproject-nix.build.packages {
          inherit python;
        }).overrideScope
          (
            lib.composeManyExtensions [
              pyproject-build-systems.overlays.wheel
              overlay
              pyprojectOverrides  # <-- added here
            ]
          )
      );
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          pythonSet = pythonSets.${system}.overrideScope editableOverlay;
          virtualenv = pythonSet.mkVirtualEnv "vision-dev-env" workspace.deps.all;
        in
        {
          default = pkgs.mkShell {
            packages = [
              virtualenv
              pkgs.uv

            ];

            buildInputs = with pkgs; [
              # opencv4
              # gstreamer
              # ...
            ];
            env = {
              UV_NO_SYNC = "1";
              UV_PYTHON = pythonSet.python.interpreter;
              UV_PYTHON_DOWNLOADS = "never";
              UV_SKIP_WHEEL_FILENAME_CHECK = "1";

              LD_LIBRARY_PATH = lib.makeLibraryPath [
                  pkgs.libGL
                  pkgs.libGLU
              ] + ":/usr/local/cuda-12.6/targets/aarch64-linux/lib:$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu";



            };
            shellHook = ''
              unset PYTHONPATH
              export REPO_ROOT=$(git rev-parse --show-toplevel)

            '';
          };
        }
      );
      packages = forAllSystems (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          env = pythonSets.${system}.mkVirtualEnv "vision-env" workspace.deps.default;
        in {
          default = pkgs.runCommand "vision-wrapped" {
            nativeBuildInputs = [ pkgs.makeWrapper ];
          } ''
            mkdir -p $out/bin
            makeWrapper ${env}/bin/vision $out/bin/vision \
              --prefix LD_LIBRARY_PATH : ${pkgs.glibc}/lib \
              --prefix LD_LIBRARY_PATH : ${pkgs.libxcrypt}/lib \
              --prefix LD_LIBRARY_PATH : /usr/local/cuda-12.6/targets/aarch64-linux/lib \
              --suffix LD_LIBRARY_PATH : /usr/lib/aarch64-linux-gnu \
              --prefix LD_LIBRARY_PATH : ${lib.makeLibraryPath [ pkgs.libGL pkgs.libGLU ]}
          '';
        }
      );
    };
}
