# H-09 · The extrinsic solution has no quality metric of any kind

- **Severity:** High
- **Area:** advanced_extrinsic_solver, extrinsic_solver_node
- **Status:** Fixed (2026-07-14)
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

**Design, with measurements:**
[docs/superpowers/specs/2026-07-13-h09-extrinsic-quality-metric-design.md](../../superpowers/specs/2026-07-13-h09-extrinsic-quality-metric-design.md).
The plan below was **revised on 2026-07-13 after simulating it** — one of the metrics originally
proposed here does not work at all, and another is actively misleading.

1. **Reprojection residuals** — report, but **never alone and never to rank**. Measured: the
   degenerate capture scores a *lower* RMSE (8.77 px) than the well-spread one (10.88 px) while
   being 13× worse in rotation, and the single-pose solve scores the best RMSE of all (0.125 px)
   while being the worst-conditioned. Reprojection error does not merely fail to catch the
   problem — it inverts the ranking.
2. **~~Leave-one-pose-out cross-validation~~ — CUT.** Measured holdout/train ratio: 1.1×
   (degenerate) vs 1.3× (well-spread). Flat. When the board is held still the held-out pose is
   *identical* to the training poses, so the model predicts it perfectly. LOO is structurally blind
   to this failure.
3. **Conditioning — the discriminator.** `cond(JᵀJ)` = 4.6e4 (degenerate) vs 2.4e2 (well-spread).
   Nearly free: `cv2.projectPoints` already returns the Jacobian. The per-DoF σ from
   `Σ ≈ σ²(JᵀJ)⁻¹` separates too but **under-reports ~4×**, because it assumes all noise is in the
   pixels and cannot see the ICP 3D error ([M-13](./M-13-icp-quality-not-propagated.md)).
4. **Subset resampling — the honest uncertainty.** Parameter spread over all C(N,3) pose subsets:
   ±5.77° / ±311 mm (degenerate) vs ±1.08° / ±52 mm (well-spread), and it predicts the true error.
   At N = 9–10 this is 120 solves — milliseconds. Tsai et al.'s construction; gives a covariance
   with no ground truth.
5. **Diversity statistics** — normal spread, depth range, image coverage. These say *what to do
   next*.
6. Surface it all in `last_solve_status` (already a string — no message change), in the logs with
   guidance, and in the `dump_detections` JSON.

Note `rust/calibration-quality/` already defines a `CalibrationMetrics` struct
(`metrics.rs:9-27`) — but it is dead code, and its `reprojection_error` is actually a 3D-3D
residual, not an image-plane one. See [L-12](./L-12-dead-solver-crates.md).

## Resolution (2026-07-14)

New pure-Python package `ros/lctk_quality/`, imported by both solvers. Surfaced through
`last_solve_status` (already a string — no message change, no `lctk_interfaces` rebuild), through
warnings that say what to DO, and through a `"quality"` block in the `dump_detections` JSON.
Nothing rejects: C-04 was a gate with an unreachable threshold that silently discarded every
detection for months.

**The design was rebuilt twice, both times because a metric was measured rather than assumed.**

1. *Reprojection RMSE inverts.* A degenerate capture scores a **better** RMSE than a good one
   (8.77 px vs 10.88 px in simulation; 3.46 px vs 8.12 px on the real field capture), and the
   single-pose solve `just demo` produces scores the best of all (0.125 px) while being the
   worst-conditioned thing the pipeline can make. Reported, never ranked on.
2. *Leave-one-out CV is blind here and was cut.* Holdout/train ratio measured **1.1× degenerate vs
   1.3× good** — flat. When the board is held still the held-out pose is *identical* to the training
   poses, so it is predicted perfectly.
3. *Resampled uncertainty inverts too, if fed frames.* The real scene-2 capture — **one board
   placement filmed nine times** — reports **±0.22° / ±9 mm**, the most confident number in the
   suite, for a capture that cannot constrain the extrinsic at all. Repeated frames of a static
   board carry correlated error, so every subset agrees. Resampling measures variance; a degenerate
   capture has low variance and high bias.

So the pipeline is: **frames → distinct placements → diversity (the gate) → residuals →
conditioning → spread**. `N` is the number of distinct board placements, never the frame count, and
`resampling` refuses to emit a number below 4 of them. Board-normal span is the primary signal — the
only one that separates cleanly on real data (1.7–3.0° degenerate vs 41.4° diverse).

**Verified against the capture that produced the shipped production extrinsic**
(`data/2022-10-14-otobrite-calibration`, whose `solve-extrinsics.sh` used exactly two poses):

```
DEGENERATE | 2 placements (18 frames) | uncert n/a (too few placements) | normals 41deg | rms 8.12px
  2 distinct placements (aim for 10+); move the board to a new spot and re-capture
  depth range is 0.10 m (aim for 1.5+); move the board nearer and farther
  Reprojection error (8.12 px) is NOT evidence of quality here
```

And on the live `just demo` pipeline, which had been reporting nothing but success:

```
[WARN] Single-pose solve: the extrinsic is under-constrained BY CONSTRUCTION.
       reprojection rms 1.27 px, cond(JtJ) 2e+04
```

10 tests, including two that exist purely to stop the design regressing to either trap.
Settles [L-13](./L-13-calibration-metrics-msg-dead.md) and [L-12](./L-12-dead-solver-crates.md) in
principle — the dead scaffolding should now be deleted.
