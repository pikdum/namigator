{
  description = "Dev shell for building and testing namigator";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in
    {
      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          # cmake finds the compiler from the stdenv that mkShell pulls in.
          nativeBuildInputs = [
            pkgs.cmake
            pkgs.git
          ];

          # python3 provides both the interpreter used by smoke_tests.py and
          # the dev headers pybind11 needs to build the mapbuild/pathfind modules.
          buildInputs = [
            pkgs.python3
          ];

          # The vendored recastnavigation/pybind11 submodules declare a
          # cmake_minimum_required below what CMake 4.x still supports. This
          # lets their old policy scopes configure without patching submodules.
          CMAKE_POLICY_VERSION_MINIMUM = "3.5";

          shellHook = ''
            # auto-load local paths (WOW_DATA, NAV_DATA) for the manual tests.
            # gitignored; copy .env.example -> .env. run from the repo root.
            if [ -f .env ]; then
              set -a
              . ./.env
              set +a
            fi

            echo "namigator dev shell"
            echo "  git submodule update --init --recursive   # fetch stormlib, pybind11, recastnavigation"
            echo "  cmake -B build -DCMAKE_BUILD_TYPE=Release"
            echo "  cmake --build build"
            echo "  cmake --install build --prefix install && (cd install && test/smoke_tests.py)"
          '';
        };
      });
    };
}
