# namigator — agent working notes

Pathfinding/navmesh library for WoW 1.12 server data. C++17 + CMake, with
optional Python bindings (pybind11). This file is the quick-start for getting
a session productive; `CLAUDE.md` is a symlink to it.

## Dev environment (NixOS)

All build/test tooling lives in a flake dev shell — do not expect cmake, gcc,
or python on the bare system.

- Enter it: `nix develop` (or let direnv auto-load it via `.envrc`).
- Provides: `cmake`, `gcc` (C++17), `python3` (+ dev headers for pybind11), `git`.
- The shell exports `CMAKE_POLICY_VERSION_MINIMUM=3.5` so the vendored
  `recastnavigation`/`stormlib` submodules (which declare a pre-3.5
  `cmake_minimum_required`) still configure under CMake 4.x.
- The shell auto-sources `.env` (gitignored) from the repo root for local
  paths. Copy `.env.example` → `.env`. Variables:
  - `WOW_DATA` — your 1.12.1 client's `Data/` dir (the folder of `*.MPQ`s).
  - `NAV_DATA` — a pre-built MapBuilder output dir (optional; for fast manual tests).

Run commands either inside `nix develop` or as `nix develop --command bash -c '…'`
(the shell hook, including the `.env` load, runs for both).

## First-time setup

The `stormlib` and `pybind11` submodules are usually not initialized on a fresh
checkout — the build needs them:

```
git submodule update --init --recursive
```

## Build

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

Python modules are produced at:
- `build/MapBuilder/mapbuild.cpython-*.so`
- `build/pathfind/pathfind.cpython-*.so`

## Smoke tests (self-contained — no client data needed)

Uses the bundled `test/test_map.mpq`. This is the CI-equivalent check.

```
PYTHONPATH=build/MapBuilder:build/pathfind python3 test/smoke_tests.py
```

## Manual tests (need a real client + ideally pre-built nav data)

`test/manual_tests.py` builds/queries the real `Azeroth` continent. Its own
module path (`../lib`) is stale, so set `PYTHONPATH` as below.

Fast path — skip the multi-minute Azeroth build, run only the query asserts
against pre-built maps:

```
PYTHONPATH=build/MapBuilder:build/pathfind \
  python3 test/manual_tests.py -d "$WOW_DATA" -n "$NAV_DATA"
```

Full path — build Azeroth from the client into a temp dir, then query (slow;
also runs `build_bvh`, which asserts a minimum game-object count):

```
PYTHONPATH=build/MapBuilder:build/pathfind \
  python3 test/manual_tests.py -d "$WOW_DATA" -j "$(nproc)"
```

## Building navigation maps yourself (CLI)

```
MapBuilder --data "$WOW_DATA" --output <out> --bvh --threads "$(nproc)"
MapBuilder --data "$WOW_DATA" --output <out> --map Azeroth --threads "$(nproc)"
```

(thistle_tea's flake exposes this as `nix run .#maps -- <WOW_DIR> <OUT_DIR>`.)

## Open issue: zone/area on open terrain inside a WMO

`Map::ZoneAndArea` determines area by ray-casting to a surface and reading that
surface's area. On open ADT terrain that lies inside a WMO's footprint (e.g. the
Stormwind interior), the downward ray misses the WMO, so it falls back to the
per-chunk ADT area — returning Elwynn Forest (`12/12`) instead of Stormwind City
(`1519/1519`).

- Vertical rays (both up and down) miss the WMO at such points, and the
  full-WMO AABB is far too coarse to use as a containment test. A real fix needs
  proper WMO-footprint containment.
- The interior Stormwind assertion in `test/manual_tests.py` is currently
  **skipped** pending that fix.
- The related issue #78 case (a cave under an ADT hole, where *both* queries
  miss) is fixed by the upward-ray fallback and has a regression check that
  asserts `12/34`.
