{
  description = "utils for setup + system interfacing + stuff";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
  };

  outputs = inputs: inputs.flake-parts.lib.mkFlake { inherit inputs; } {
    systems = [ "aarch64-linux" ];
    perSystem = { pkgs, self', ... }: {

      packages = {
        setup_weird_stuff = pkgs.writeShellApplication {
          name = "setup_weird_stuff";
          runtimeInputs = [
            self'.packages.jetson_reset_network
            self'.packages.jetson_set_dev_power
          ];
          text = ''
              set -euo pipefail

              PKG="cudss"
              DEB="cudss-local-tegra-repo-ubuntu2204-0.6.0_0.6.0-1_arm64.deb"
              URL="https://developer.download.nvidia.com/compute/cudss/0.6.0/local_installers/$DEB"
              REPO_DIR="/var/cudss-local-tegra-repo-ubuntu2204-0.6.0"
              KEYRING_DST="/usr/share/keyrings"

              if dpkg -s "$PKG" >/dev/null 2>&1; then
                  echo "$PKG is already installed."
                  exit 0
              fi

              echo "$PKG is not installed. Installing..."

              if [ ! -f "$DEB" ]; then
                  wget -O "$DEB" "$URL"
              fi

              sudo dpkg -i "$DEB"

              if compgen -G "$REPO_DIR/cudss-*-keyring.gpg" >/dev/null; then
                  sudo cp "$REPO_DIR"/cudss-*-keyring.gpg "$KEYRING_DST"/
              else
                  echo "Keyring file not found in $REPO_DIR" >&2
                  exit 1
              fi

              sudo apt-get update
              sudo apt-get -y install "$PKG"

              echo "$PKG installation complete."


              jetson_reset_network

              jetson_set_dev_power
          '';
        };

        default = self'.packages.setup_weird_stuff;


        jetson_reset_network = pkgs.writeShellApplication {
          name = "jetson_reset_network";
          runtimeInputs = [

          ];
          text = ''
              sudo ip addr flush dev enP8p1s0
              sudo ip addr add 192.168.144.30/24 dev enP8p1s0
              sudo ip link set enP8p1s0 up
          '';
        };

        jetson_set_dev_power = pkgs.writeShellApplication {
          name = "jetson_set_dev_power";
          runtimeInputs = [];

          text = ''
            sudo nvpmodel -m 2
          '';
        };

      };

      devShells.default = pkgs.mkShell {
        packages = with pkgs; [
          self'.packages.default
        ];
      };

    };
  };
}
