# H-01 · `conflux_py` is never built → solver nodes ImportError at startup

- **Severity:** High
- **Area:** build system / setup
- **Status:** Open
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
