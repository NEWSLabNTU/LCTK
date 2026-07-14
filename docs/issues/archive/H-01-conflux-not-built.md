# H-01 · `conflux_py` is never built → solver nodes ImportError at startup

- **Severity:** High
- **Area:** build system / setup
- **Status:** Fixed (2026-07-11)
- **Verified:** Yes (confirmed against live source, 2026-07-09)
- **Location:**
  - `justfile:22, 29` (`--ignore-paths ros/conflux`)
  - `.envrc:37-43` (sources only `$PWD/install/setup.bash`)
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:28` (and advanced / lidar_to_lidar solvers) `from conflux_py import ...`

## Problem

The root `just build` explicitly excludes `ros/conflux` (`--ignore-paths ros/conflux`), and nothing in `setup/`, the root `justfile`, or the README builds it — only a code comment (`justfile:22`) mentions that it must be built separately. `conflux`'s own justfile runs a bare `colcon build` producing a separate `ros/conflux/install/` tree, which the launch recipes never source (`.envrc` and the recipes source only the top-level `install/`). Yet all three solver nodes import `conflux_py`.

## Failure scenario

A user follows the documented setup and build steps. The build succeeds, `just demo` launches, and each solver node dies immediately with `ModuleNotFoundError: No module named 'conflux_py'`. The pipeline appears to start but produces no calibration.

## Suggested fix

Add a build step that builds `ros/conflux` and merges it into the workspace install tree (or source `ros/conflux/install/setup.bash` in the launch recipes and `.envrc`). Document it in README/CLAUDE.md. Ideally make `just build` build conflux with its required toolchain rather than ignoring it.

## Resolution (2026-07-11)

The pipeline only needs two of the conflux packages: `conflux_cpp` (whose CMake
builds `libconflux_ffi.so`) and `conflux_py` (the ctypes wrapper the solver nodes
import). Neither uses the git `rclrs`; only `conflux` (the standalone node) and
`conflux-ros2` do, which LCTK does not need.

Two problems were fixed:

1. **`just build` never built conflux.** Added a `build-conflux` recipe that runs
   `colcon build --packages-select conflux_cpp conflux_py`, and made `build` depend
   on it (so `conflux_py` exists before the solver packages that depend on it). The
   main build now uses `--packages-ignore conflux conflux_cpp conflux_py` (the old
   `--ignore-paths ros/conflux` was ineffective — it did not exclude anything).

2. **The real build failure was setuptools, not a toolchain conflict.** A user-pip
   `setuptools 80` shadowed Ubuntu/Humble's apt `setuptools 59.6.0`; colcon's
   `--symlink-install` runs `setup.py develop --editable`, which setuptools 80
   rejects (`error: option --editable not recognized`), failing every `ament_python`
   package. Removing the user-level setuptools restores 59.6.0 and the builds pass.
   This env fix is machine-level, not in the repo; it should be added to
   setup/CONTRIBUTING (pin `setuptools<80` for the ROS 2 Humble build).

Verified: from a conflux-clean tree, `just build` produces `build-conflux` (2 pkgs)
+ main (13 pkgs), 0 failures, `libconflux_ffi.so` installed, and
`from conflux_py import ROS2Synchronizer` succeeds under `install/`.
