# M-12 · No outlier rejection and no LM refinement in the extrinsic solve

- **Severity:** Medium
- **Area:** lidar_to_camera_solver
- **Status:** Fixed (2026-08-18) — maintained continuous and manual paths share SQPnP, LM refinement, float64, quality reporting, and covariance weighting
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1333-1366` (`_solve_pnp`, `SOLVEPNP_SQPNP`)
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:718-723` (`SOLVEPNP_ITERATIVE`)

## Problem

The advanced solver makes a single direct `cv2.solvePnP(..., flags=cv2.SOLVEPNP_SQPNP)` call
over the concatenated correspondence set. There is:

- **no RANSAC** — `solvePnPRansac` appears nowhere in the repo,
- **no refinement** — no `solvePnPRefineLM`, no `solvePnPRefineVVS`, no re-solve with
  `useExtrinsicGuess=True`. SQPnP is a direct global solver, so the advanced path performs
  *zero* nonlinear polish of the reprojection cost,
- **no weighting** — every one of the `16 × N` corners enters with equal weight,
- **no residual gating** — nothing looks at per-corner error, because nothing computes it
  ([H-09](./H-09-no-extrinsic-quality-metric.md)).

## Failure scenario

One bad pose in the buffer corrupts the whole calibration and there is no way to notice or
exclude it. Bad poses are easy to produce: a partially occluded board, a grazing-incidence view
where ICP settles into a poor local minimum, a frame where the LiDAR board pose snapped to the
wrong origin corner ([M-14](../M-14-corner-order-brittle.md)). All 16 of that pose's corners are
outliers *together* (they share one rigid `T_board`), which is exactly the correlated-outlier
regime least-squares handles worst.

Note the *default* solver (`extrinsic_solver_node`) uses `SOLVEPNP_ITERATIVE`, which does run
LM — but only on a single detection pair, with no buffer at all.

## Suggested fix

Layered, in increasing order of change:

1. `solvePnPRefineLM` (or `RefineVVS`) after the SQPnP initialisation. One line, strictly better.
2. Compute per-corner reprojection residuals (H-09) and reject corners above a threshold, then
   re-solve. Reject at **pose granularity**, not corner granularity — the errors within a pose
   are correlated, so a per-corner reject is statistically wrong.
3. IRLS with a Huber/Cauchy kernel over pose-grouped residuals, or `solvePnPRansac` with the
   pose as the sampling unit.
4. Weight each pose by its ICP quality once that is available
   ([M-13](./M-13-icp-quality-not-propagated.md)).

## Resolution (2026-07-13) — LM refinement (fix item 1)

The advanced solver's `_solve_pnp` now runs `cv2.solvePnPRefineLM` after the
`SOLVEPNP_SQPNP` initialization, so the reprojection cost gets a nonlinear polish
it previously never received (SQPnP is a direct global solver). The refinement is
guarded — if it raises, the SQPnP result is kept and a warning is logged. The
standard `extrinsic_solver_node` already uses `SOLVEPNP_ITERATIVE`, which runs LM
internally, so it was not touched.

Items 2–4 (per-corner/per-pose residual gating, IRLS/RANSAC, ICP-quality weighting)
are intentionally left: residuals are the [H-09](./H-09-no-extrinsic-quality-metric.md)
metric surface and per-pose ICP quality is [M-13](./M-13-icp-quality-not-propagated.md),
both in progress by other agents. Doing the gating here would collide with the
residual computation being added under H-09.

Verified: `tmp/test_m12_refine.py` confirms RefineLM never worsens the reprojection
RMSE on a synthetic noisy scene; `just build` + `just test` green.

## Final resolution (2026-08-18) — estimator asymmetry removed

Diamond-frame Phase 2 Stage 2 made `lidar_to_camera_solver` the only config-driven camera solver
and added `solver_mode=continuous` on its maintained backend. Both `continuous` and `manual` now use
the same float64 correspondence path, SQPnP initialization, LM refinement, quality report, and
board-pose covariance weighting. The weaker `extrinsic_solver_node` remains in-tree only until
Stage 3 deletes it; no launch or justfile path can select it.

Focused coverage proves the continuous policy replaces rather than accumulates its latest pair and
calls `SOLVEPNP_SQPNP` plus `solvePnPRefineLM`. Full verification passed: `just build`, 240 Rust
tests, and 181 Python tests.

Pose-grouped RANSAC/IRLS remains a possible estimator enhancement, explicitly out of scope for the
diamond-frame spec. It is no longer evidence of different or unrefined maintained solver paths, so
this issue is closed.
