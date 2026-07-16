# L-09 · Setup fragility, no export tooling, dump JSON mislabeled as "calibration"

- **Severity:** Low
- **Area:** setup scripts / advanced_extrinsic_solver / demo recipe
- **Status:** Fixed (2026-07-16)
- **Verified:** Static review
- **Location:**
  - `setup/scripts/install-ros2.sh:26` (curl `releases/latest`), `install-rust.sh`, `install-cuda.sh:12`
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:632-691` (`dump_detections`)
  - `justfile:103-114` (`demo` recipe)

## Problem

- ROS / Rust / CUDA installers are fetched live from GitHub at runtime with no pinning, rate-limit handling, or failure handling. A GitHub API hiccup or format change breaks setup.
- `dump_detections` and the interactive controller "Save" (`p` key → `~/detections.json`) write a solver **re-load** JSON (raw rvec/tvec correspondences), not a usable extrinsic — misleading as an "export the calibration result".
- The `demo` recipe does not forward `enable_overlay` / `enable_judge`, so it defaults them to `false` while `lidar-camera` / `calibrate` default them to `true` — inconsistent behavior between the two "run it" paths.

## Failure scenario

GitHub API changes break a fresh setup; a user expects the dumped JSON to be their calibration result; `just demo` behaves differently from `just lidar-camera`.

## Suggested fix

Pin installer versions (or vendor a known-good source), rename/clearly document the dump JSON as a solver session file (see [gap-autoware-export.md](./gap-autoware-export.md) for the real export), and make `demo` forward the same defaults as `lidar-camera`.

## Partial resolution (2026-07-11)
`just demo` now forwards `enable_overlay`/`enable_judge` (wired through
demo.launch.py), so it matches `just lidar-camera` instead of silently disabling
them. The runtime-fetched installer pinning is folded into the Phase-4
dependency/vuln work (docs/roadmap/phase-4-*), and the "dump_detections is not an
extract" clarification is covered by [gap-autoware-export.md](./gap-autoware-export.md).

## Update (2026-07-14)
Added a numpy pin to `setup/scripts/install-colcon-rust.sh` (mirroring the setuptools
one): a user-pip numpy shadowing the apt numpy that Humble is built against is now
removed at setup time, matching the build's `_check-python-env` guard. The remaining
runtime-fetched installer version-pinning (curl `releases/latest`) is still tracked
under Phase-4; the dump-JSON "not an extract" clarification remains in
[gap-autoware-export.md](./gap-autoware-export.md).

## Update (2026-07-16) — third pip-shadowing instance: scipy

A user-pip scipy 1.15 (requires numpy >= 1.23) over apt numpy 1.21 broke every
`scipy.optimize` import with `TypeError: 'numpy._DTypeMeta' object is not subscriptable`,
failing the M-13 pose-weighting test. Fixed the same way as the previous two instances:
removed the pip package, and extended the `_check-python-env` guard, the installer
(`install-colcon-rust.sh`), and the CLAUDE.md table to cover scipy.

## Resolution (2026-07-16) — installer pinning (the last open part)

- `install-ros2.sh`: `ros-apt-source` no longer resolved via a live `releases/latest`
  API call; pinned to 1.2.0 (env-overridable via `ROS_APT_VERSION`), download has
  retries and a clear stale-pin error. Pinned URL verified reachable (HTTP 200).
- `install-rust.sh`: `cargo-ament-build` (0.1.11) and `cargo-nextest` (0.9.137) pinned
  to the versions this workspace builds with, `--locked`, env-overridable; rustup fetch
  gets retries; toolchain pinning delegated to `rust-toolchain.toml`.
- `install-cuda.sh`: keyring version parameterized (`CUDA_KEYRING_VERSION`, default the
  previously hardcoded 1.0-1) with retries and a stale-pin error.

With the export mislabeling covered by `lctk_autoware_export` (gap-autoware-export) and
the demo-recipe defaults fixed on 2026-07-11, every part of this issue is closed.
