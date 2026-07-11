# L-03 · `pnp-solver` panics on failed solve and truncates distortion to 5 coefficients

- **Severity:** Low (no production caller today — PnP is done in Python)
- **Area:** pnp-solver crate
- **Status:** Fixed (2026-07-11)
- **Verified:** Static review
- **Location:** `rust/pnp-solver/src/lib.rs:66-75, 96, 102-118`

## Problem

- `calib3d::solve_pnp(...).unwrap()` and `OpenCvPose{...}.try_to_cv().unwrap()` crash the process on a degenerate or too-small correspondence set (empty input is guarded, but `<4` points / collinear / coplanar-with-ITERATIVE are not).
- Distortion is truncated to the first 5 coefficients, silently dropping rational-polynomial k4–k6 for 8-coeff cameras (inconsistent with `aruco-detector`, which uses all of `camera_info.d`).

## Failure scenario

If this crate is ever wired into the live path: process crash on sensor data, and biased poses on rational-polynomial cameras.

## Suggested fix

Return `Result` instead of `unwrap()`, validate correspondence count, and pass the full distortion vector.

## Resolution (2026-07-11)
`pnp-solver` now returns `None` (with a warning) instead of unwrapping a failed
`solve_pnp` or pose conversion, and passes the full distortion vector rather than
truncating to 5 coefficients.
