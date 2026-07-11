# L-08 · Stale README & docs misdirect new users

- **Severity:** Low
- **Area:** documentation
- **Status:** Partially fixed (2026-07-11)
- **Verified:** Static review
- **Location:**
  - `README.md:13, 18-19, 116, 142`
  - `ros/lctk_launch/config/README.md`
  - `book/src/user-guide/lidar-camera.md`, `book/src/user-guide/configuration.md`

## Problem

Multiple stale references:
- `README.md` tells users to run `./setup-dev-env.sh` (does not exist; actual `./setup.sh`) and `just sample-sensor-data start` / `just lidar-camera start` (recipe names/args do not exist; actual `just sample-data`, `just lidar-camera` with no `start`).
- `README.md:142` build command omits `--ignore-paths ros/conflux`, contradicting the working justfile.
- Web UI port documented as `http://localhost:8080`, actual is `:8000`.
- `max_icp_iterations` shown as both `100` (lidar-camera.md) and `10` (configuration.md).
- `config/README.md` references `board_pattern.json5` (actual `board_detector.json5`) and package `calib_launch` (actual `lctk_launch`); the package README documents an older launch interface.

## Failure scenario

A new user copy-pastes any quick-start command and it errors or builds the wrong thing.

## Suggested fix

Sweep README, config/README, and the book against the current justfile and launch files; add a CI doc-lint or a smoke test that runs the documented commands.

## Partial resolution (2026-07-11)
Fixed the highest-impact drift: README Quick Start (`./setup.sh`, `just sample-data`
/ `just demo` instead of the nonexistent `setup-dev-env.sh` / `... start`), the web
UI port (`:8000`) and `max_icp_iterations` default (10) across the book, and
`config/README.md` (`lctk_launch`, `board_detector.json5`). The rest of README.md
still documents an older `just sample-sensor-data {start|stop|status|logs}` /
`just lidar-camera start` interface that no longer exists — it needs a broader
rewrite against the current justfile (tracked here).
