# H-09 · The extrinsic solution has no quality metric of any kind

- **Severity:** High
- **Area:** advanced_extrinsic_solver, extrinsic_solver_node
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1333-1366` (`_solve_pnp`)
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1092-1145` (`_solve_from_buffer` result handling)
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:670-730` (`_solve_pnp_educational`)

## Problem

`_solve_pnp` returns `(success, rvec, tvec)` and nothing else. `cv2.projectPoints`,
`cv2.solvePnPRansac`, `cv2.solvePnPRefineLM` and `cv2.solvePnPRefineVVS` do not appear anywhere
in `ros/` or `rust/` outside `pointcloud_image_overlay` (verified by grep). After the solve:

- no reprojection error (mean, RMS, per-corner, or per-pose),
- no covariance — every `covariance` field emitted by the pipeline is `[0.0; 36]`
  (`aruco_locator_node/src/main.rs:103`, `lidar_board_detector/src/main.rs:1801`),
- no condition number / observability measure,
- no cross-validation.

`last_solve_status` is a free-text string (`"Calibration successful"`), and
`GetBufferStatus` returns only `buffer_size`, `total_correspondences`, `is_publishing`,
`last_solve_status`. `total_correspondences` is a raw count, which is actively misleading —
see [H-07](./H-07-no-pose-diversity-gate.md) — because 20 frames of a stationary board report
320 "correspondences" while carrying the information of one.

The only quality signal that exists anywhere is the LiDAR-side ICP loss, and it never reaches
the solver ([M-13](./M-13-icp-quality-not-propagated.md)).

## Failure scenario

The pipeline cannot distinguish a well-conditioned calibration from a degenerate one. Both
report `"Calibration successful"`. The only feedback the operator has is eyeballing the point
cloud overlay — which is precisely the check that *passes* on the board and *fails* in the
background, and which no automated gate replicates. A bad extrinsic can be dumped to JSON and
shipped to Autoware with no recorded evidence of its quality.

## Suggested fix

All of these are cheap and computable from data the solver already has:

1. **Reprojection residuals.** `cv2.projectPoints(all_object_points, rvec, tvec, K, dist)` →
   per-corner error vector. Report mean / RMS / max, plus a per-pose breakdown. ~10 lines.
2. **Leave-one-pose-out cross-validation.** Solve on `K−1` poses, evaluate reprojection RMSE on
   the held-out pose; repeat. The spread between train-RMSE and holdout-RMSE is the direct
   numeric proxy for "does this transform generalise off the board". This is the construction
   behind the VOQ score of Tsai et al. (ITSC 2021), whose whole subject is calibration that
   overfits its sample poses.
3. **Covariance and conditioning.** `Σ ≈ σ²(JᵀJ)⁻¹` from the PnP Jacobian gives a per-DoF
   standard deviation; `cond(JᵀJ)` exposes the near-null rotation direction of
   [H-07](./H-07-no-pose-diversity-gate.md) as a single number.
4. Surface all of the above in `GetBufferStatus`, in the interactive controller, and in the
   `dump_detections` JSON, so a saved calibration carries its own quality record.

Note `rust/calibration-quality/` already defines a `CalibrationMetrics` struct
(`metrics.rs:9-27`) — but it is dead code, and its `reprojection_error` is actually a 3D-3D
residual, not an image-plane one. See [L-12](./L-12-dead-solver-crates.md).
