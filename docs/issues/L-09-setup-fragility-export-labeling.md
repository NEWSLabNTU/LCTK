# L-09 · Setup fragility, no export tooling, dump JSON mislabeled as "calibration"

- **Severity:** Low
- **Area:** setup scripts / advanced_extrinsic_solver / demo recipe
- **Status:** Partially fixed (2026-07-11)
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
