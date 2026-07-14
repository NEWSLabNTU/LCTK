# L-12 · Three dead crates, one of which is better than the live code it isn't replacing

- **Severity:** Low
- **Area:** rust/pnp-solver, rust/calibration-quality, rust/dynamic-calibration, rust/aruco-detector
- **Status:** Deferred to H-09 (2026-07-13) — decision coupled to the H-09 metrics work, in progress
- **Verified:** Yes (confirmed against live source, 2026-07-12)

## Problem

### `rust/pnp-solver` — zero dependents

No `Cargo.toml` in the workspace depends on it (`grep -rn "pnp-solver" --include=Cargo.toml`
returns only its own manifest). Yet it is strictly better than the Python that does the real
work:

- it exposes `PnpMethod::{ITERATIVE, EPNP, IPPE, SQPNP}` (`src/lib.rs:31-42`) — including
  `IPPE`, which is the correct choice for a planar target;
- it passes the **real** distortion vector, handling OpenCV's 4/5/8/12/14-length forms
  (`src/lib.rs:68-78`), which the live solver does not ([M-11](./M-11-solvers-ignore-distortion.md));
- it guards against the empty-input panic (`src/lib.rs:98`).

### `rust/calibration-quality` — reachable only from dead code

Defines `CalibrationMetrics { reprojection_error, consistency_score, num_inliers, inlier_ratio,
geometric_error, statistical_metrics }` (`metrics.rs:9-27`) — exactly the vocabulary
[H-09](./H-09-no-extrinsic-quality-metric.md) is asking for. It is depended on only by
`rust/dynamic-calibration`, which nothing depends on. Also, its `reprojection_error` is
misnamed: it computes a 3D-3D residual `‖T·source − target‖²` (`metrics.rs:63-70`), not an
image-plane reprojection.

### `Detector::estimate_pose` / `PoseEstimation::fit_icp` — zero callers

`rust/aruco-detector/src/multi_aruco.rs:83-104` wraps `aruco::estimate_pose_single_markers`,
and `:151-255` implements an ICP-based pose regression. Neither is called from anywhere. The
live path (`rust/aruco-locator/src/lib.rs:74-97`) only extracts corners, so **no camera-frame
board pose is ever computed**, even though the marker size is known and the machinery exists.

## Failure scenario

Not a runtime failure — a maintenance and design hazard. Three crates and two methods carry
implied blessing ("this is how the project does PnP / quality / marker pose") while the actual
pipeline does something different and, in places, worse. A contributor fixing
`rust/pnp-solver` changes nothing.

## Suggested fix

This is a decision, not a mechanical cleanup, and it should be made *with*
[phase 5](../roadmap/phase-5-stable-extrinsic-solution.md) rather than before it:

- **`pnp-solver`**: either delete it, or promote it — port the solver's PnP core to it and call
  it via a Python binding. Phase 5 wants method selection and honest distortion handling anyway.
- **`calibration-quality`**: fix the `reprojection_error` misnomer and make it the home of the
  H-09 metrics, or delete it and both of the crates above it.
- **`estimate_pose`**: phase 5.3 wants a camera-frame board pose (for plane-normal
  correspondences). That is exactly what this method provides. Wire it up rather than delete it.

Whatever is decided, nothing should stay in the tree unreferenced.

## Disposition (2026-07-13) — deferred to H-09

Confirmed against the current tree: `rust/pnp-solver` has zero Cargo dependents;
`rust/dynamic-calibration` has zero dependents and is the only thing that depends on
`rust/calibration-quality`; `Detector::estimate_pose` and `PoseEstimation::fit_icp`
have zero callers. `calibration-quality`'s `reprojection_error` does compute a 3D-3D
residual — `(transform * source - target).norm_squared()`, metres², not pixels.

Per the suggested fix, this is a decision to make *with* H-09 (delete vs promote
`pnp-solver`; delete vs repurpose `calibration-quality` as the H-09 metrics home;
wire up `estimate_pose` for phase 5.3's camera-frame board pose). H-09 is in progress
by another agent and may reuse this code, so nothing is deleted or renamed here to
avoid colliding with that work. Left as a recorded decision for the H-09 change to
resolve; nothing should remain unreferenced once H-09 lands.
