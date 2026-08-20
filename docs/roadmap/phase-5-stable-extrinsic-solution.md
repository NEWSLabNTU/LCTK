# Phase 5: Stable Extrinsic Solutions

## Overview

The LiDAR-camera extrinsic produced today is unstable in a specific, reproducible way: the
point-cloud overlay lines up on the calibration board and on the ArUco markers, but background
points are visibly tilted. Adding more buffered image/point-cloud pairs makes the solution
*less noisy* without making it *less wrong*.

This phase explains why that happens, fixes the systematic biases that make it worse, and
replaces "eyeball the overlay" with a number that fails when the background tilts.

**Scope note.** The fixes divide cleanly into *defects* (tracked as issues, fixed first) and
*design work* (staged below). The defects are not optional preliminaries — two of them
([C-03](../issues/archive/C-03-double-undistortion.md), [H-08](../issues/archive/H-08-no-subpixel-corner-refinement.md))
directly poison the remedy for the design problem, so the ordering is load-bearing.

## Problem Statement

### What the solver actually solves

The 3D points fed to `cv2.solvePnP` are **not LiDAR returns**. The pipeline is:

1. `lidar_board_detector` fits the hollow-board model (1000 mm plate, 3 holes) to the cropped
   cloud by Kabsch ICP → board pose `T_board` in the LiDAR frame.
2. `aruco_locator_node` detects the 4 markers and publishes their 16 corner **pixels**.
3. `advanced_extrinsic_solver` takes the 16 *ideal model* corner coordinates from the ArUco
   config, pushes them through `T_board`, and calls that the 3D point set
   (`advanced_extrinsic_solver/main.py:1290-1331`).
4. One `solvePnP` over all buffered poses concatenated.

So every object point lies on a **500 mm coplanar patch**, wherever the board was held. All 16
corners of one pose come from a single rigid `T_board`, so their errors are perfectly
correlated: **one pose carries ~6 DoF of information, not 32 independent residuals.**

### Why the background tilts

Perturb the extrinsic by `(δθ, δt)`. A point `p` moves by `δθ × p + δt`. Write `p = p̄ + Δ`,
where `p̄` is the centroid of all accumulated correspondences:

```
motion(p) = (δθ × p̄ + δt)  +  δθ × Δ
```

Choose `δt = −δθ × p̄`. The first term vanishes identically. What remains is `δθ × Δ` —
proportional only to the spread of the correspondences about **their own centroid**.

> **A rotation about the correspondence centroid is a near-null direction of the reprojection
> cost, damped only by how far the correspondences spread from that centroid.**

Reprojection error on the board barely moves. A background point at offset `Δ` from that
centroid swings by `f·|δθ × Δ| / Z` pixels — growing with distance from the board. Board glued,
background tilted. Exactly the observed symptom.

Two consequences that determine everything below:

- **More frames of the same board placement do not help the conditioning.** They shrink the
  noise variance; they do not change `Δ`. This is why the current "cache more pairs" strategy
  plateaus.
- **Only new board placements — at new depths and new orientations — shrink the null
  direction.** And the solver currently has no idea whether it has any
  ([H-07](../issues/archive/H-07-no-pose-diversity-gate.md)).

### Why nothing catches it

There is no reprojection error, no covariance, no condition number, and no cross-validation
anywhere in the pipeline ([H-09](../issues/archive/H-09-no-extrinsic-quality-metric.md)). A degenerate
solve and a good one both report `"Calibration successful"`. The *only* check that
distinguishes them is the human looking at the overlay — and the near-null direction is
precisely the error that the board region of that overlay cannot show.

## Stage 5.0 — Fix the defects ✅ blocking

These are tracked as issues and must land before any of the staged work, because they either
bias every correspondence or actively invert the sign of the Stage 5.5 guidance.

| Issue | Why it blocks |
|-------|---------------|
| ✅ [C-03](../issues/archive/C-03-double-undistortion.md) — image undistorted twice — **fixed 2026-07-12** | Radius-dependent bias on every corner. Border poses carried the *largest* error, so "spread the board across the FoV" (Stage 5.5) would have injected more systematic error than it removed. Stage 5.5 is unblocked. |
| ✅ [H-08](../issues/archive/H-08-no-subpixel-corner-refinement.md) — no sub-pixel refinement — **fixed 2026-07-13** | `CORNER_REFINE_NONE`, and there is no redundancy to average the error away. Now `SUBPIX`, refined on the **raw** frame (where the gradients still exist) and mapped to the rectified frame with `undistortPoints`. Measured 25–60% lower corner RMSE than `NONE` across the working marker-size range; `CONTOUR` was no better than `NONE`. |
| ✅ [M-11](../issues/archive/M-11-solvers-ignore-distortion.md) — solvers hardcode `dist_coeffs = 0` — **fixed 2026-07-13** | The rectify-once contract was unstated, violated, and silently breakable. Now correct *by construction*: the corners on the wire are rectified points, so `dist_coeffs = 0` is a definition rather than an assumption. The solvers were not touched. |
| [M-14](../issues/M-14-corner-order-brittle.md) — gravity-based origin corner; duplicated corner-order logic | Produces silent 90° errors in individual poses. Poisons the buffer with correlated outliers. |
| [L-10](../issues/archive/L-10-solver-float32-precision.md) — `float32` solve | Free precision loss; makes a `cond(JᵀJ)` diagnostic unreliable. |
| ✅ [L-11](../issues/archive/L-11-detector-param-block-bugs.md) — detector param block — **fixed 2026-07-13** | Reduced pose yield. Both copies of the block are gone; `DetectorParameters` is now constructed in exactly one place, from config. |

**Exit criterion:** a board rendered through a known `K, D` recovers its corners through the full
detector path. Met — `rust/aruco-detector/tests/rectify_contract.rs` closes the distort → detect →
undistort round trip, and both underlying bugs were re-introduced to confirm the tests catch them.

**Remaining in this stage:** M-14 (gravity-based origin corner; duplicated corner-order logic) and
L-10 (`float32`).

## Stage 5.1 — Measure the instability

You cannot fix conditioning you cannot see. Everything here is computable from data the solver
already holds; none of it changes the estimate.

Tracked as [H-09](../issues/archive/H-09-no-extrinsic-quality-metric.md). **Design (with measurements):**
[2026-07-13-h09-extrinsic-quality-metric-design.md](../superpowers/specs/2026-07-13-h09-extrinsic-quality-metric-design.md).

> **This stage was re-planned on 2026-07-13 after simulating it.** Two of the metrics originally
> listed here do not work, and one is worse than useless. The list below is what the measurements
> support. See the design doc for the numbers.

1. **Reprojection residuals** — per-corner and per-pose. Report them, but **never alone and never
   to rank**. Measured: on a degenerate capture (board held still) the RMSE comes out *lower*
   (8.77 px) than on a well-spread one (10.88 px), while the extrinsic is **13× worse in rotation**.
   The single-pose default solve scores the best RMSE of all (0.125 px) and is the worst-conditioned
   thing the pipeline can produce. Reprojection error here is not merely insufficient — it inverts.
2. **~~Leave-one-pose-out cross-validation~~ — CUT.** Measured holdout/train ratio: **1.1×
   degenerate vs 1.3× well-spread**. Flat, and pointing the wrong way. The reason is structural:
   when the board is held still, the held-out pose is *identical* to the training poses, so the
   model predicts it perfectly. LOO detects failure to generalise to *different* poses, and the
   degenerate case has none. It is blind to exactly the failure it was meant to catch.
3. **Conditioning — the discriminator.** `cond(JᵀJ)`: **4.6e4** (degenerate) vs **2.4e2**
   (well-spread), a 190× gap. Nearly free: `cv2.projectPoints` already returns the Jacobian, whose
   first 6 columns are ∂proj/∂(rvec, tvec).
   The per-DoF σ from `Σ ≈ σ²(JᵀJ)⁻¹` separates too, but **under-reports by ~4×** (1.22° where the
   true error is 5.07°) because it assumes all noise is in the pixels and cannot see the ICP 3D
   error. Useful as a relative signal; dishonest as an absolute one. Label it.
4. **Subset resampling — the honest uncertainty.** Spread of the solved parameters over all C(N,3)
   pose subsets. Measured: **±5.77° / ±311 mm** (degenerate) vs **±1.08° / ±52 mm** (well-spread) —
   and it *predicts the true error* (5.77° estimated, 5.07° actual). This is Tsai et al.'s
   construction, and at N = 9–10 real poses it is 120 solves: milliseconds. It is the headline
   number the operator acts on.
5. **Diversity statistics**: board-normal angular spread, depth range, image coverage. These do not
   measure quality — they say *what to do next*, which a condition number cannot.
6. Surface it all in `last_solve_status` (already a string — no message change), in the logs with
   guidance, and in the `dump_detections` JSON, so a saved calibration carries its own quality
   record.

**Exit criterion:** a deliberately degenerate capture (board held still) and a good one (spread
poses) are separable by the reported numbers alone, with no overlay. **Met in simulation** — the
separation is 190× on conditioning and ~6× on subset spread.

## Stage 5.2 — Robust estimator

Tracked as [M-12](../issues/archive/M-12-no-robust-estimation-or-refinement.md),
[M-13](../issues/archive/M-13-icp-quality-not-propagated.md).

1. **Refine.** `solvePnPRefineLM` (or `RefineVVS`) after the SQPnP initialisation. The advanced
   path currently performs *zero* nonlinear polish of the reprojection cost.
2. **Reject at pose granularity.** Per-corner residuals now exist (5.1); reject **whole poses**,
   not individual corners. The 16 corners of a pose share one `T_board`, so they are outliers
   together — a per-corner reject is statistically wrong.
3. **Weight by board-pose quality.** M-13: put a real 6×6 covariance into `Detection3D`
   (`Σ ≈ σ²(JᵀJ)⁻¹` from the ICP correspondence Jacobian — it comes out naturally anisotropic:
   tight along the plane normal, loose in-plane) and consume it as a per-pose weight.
4. **Diversity gate.** Refuse to solve, or loudly warn, when the buffer fails the 5.1 diversity
   statistics. Raise `min_poses_required` off its current default of **2**.

**Exit criterion:** injecting one deliberately-bad pose into a good buffer changes the solved
extrinsic by less than its reported σ, and the bad pose is named in the status output.

## Stage 5.3 — Joint estimation (kill the errors-in-variables bias)

This is where the remaining *bias* — as opposed to variance — goes away.

`cv2.solvePnP` assumes the 3D points are exact and all noise is in the pixels. Here the 3D points
are model corners pushed through an ICP pose whose error is large, anisotropic, and correlated
within a pose. That is a textbook errors-in-variables problem, and least-squares on it is
biased, not merely noisy — no amount of averaging converges to the truth.

Two changes, either of which helps and which compose:

1. **Add plane/normal correspondences, not just point correspondences.**
   The marker size is known and metric, so IPPE on the ArUco corners yields the board's full
   6-DoF pose **in the camera frame**; ICP yields it **in the LiDAR frame**. That is a per-pose
   3D-3D pose pair. Rotation estimated from **normal alignment across poses** (Zhang–Pless
   style) is far better conditioned against far-field tilt than reprojection of a clustered
   point patch, because the normals span directions independently of how far the points spread.
   Zhou, Li & Kaess (IROS 2018) reduce the minimum to **one** board pose by combining line and
   plane correspondences.
   *The machinery already exists and is dead code*: `Detector::estimate_pose`
   (`rust/aruco-detector/src/multi_aruco.rs:83-104`) has zero callers
   ([L-12](../issues/archive/L-12-dead-solver-crates.md)).

2. **Bundle-adjust the board poses together with the extrinsic.**
   Stop treating `T_board` as ground truth. Optimise `{T_extrinsic, T_board^(1..K)}` jointly
   against **both** the image reprojection residual **and** the LiDAR point-to-model residual,
   each weighted by its own covariance. Every board pose is then free to move to satisfy both
   sensors, and the LiDAR's anisotropic uncertainty is modelled instead of being asserted to be
   zero. This is the direction both Zhou/Kaess and the Michigan "Improvements to Target-Based
   3D LiDAR to Camera Calibration" work converge on — the latter explicitly designing its fit
   around the premise that "the camera image data is the most accurate information in one's
   possession", and using the *known target geometry* to beat LiDAR quantisation and systematic
   range error.

**Exit criterion:** on a fixed dataset, the joint solve's held-out reprojection RMSE (5.1) is
materially below the PnP solve's, and its reported σ shrinks.

## Stage 5.4 — Validate against the background, not the board

Stages 5.1–5.3 all measure the calibration on the *target*. The failure mode is defined by what
happens *away from* the target. Close that gap directly.

Initialise from the solved extrinsic, then score it with a **targetless edge cost** on the same
recorded scenes:

- Extract LiDAR edges and image edges, and score their alignment (inverse-distance-transform of
  the image edge map, à la Levinson & Thrun). Prefer **plane-intersection** edges over
  depth-discontinuity edges — the latter suffer from beam divergence artefacts, which is the
  key robustness lesson of the Livox/HKU pixel-level self-calibration work.

Two uses, and the first is the important one:

- **As a metric.** An extrinsic that is right at the board and tilted in the background scores
  *well* on board reprojection and *badly* here. This is the number that finally replicates the
  eyeball test, and it is the accept/reject gate the pipeline has never had.
- **As a refinement.** Optionally polish the extrinsic against this cost. Livox report ~50 % of
  residuals within 1 px from single scenes, matching checkerboard methods that needed 36+ board
  poses.

`ros/pointcloud_image_overlay/` already projects the cloud into the image and is the natural
home for the scoring node.

**Exit criterion:** the background-alignment score separates a known-good extrinsic from one
perturbed by a rotation about the board centroid — the perturbation that board-only
reprojection error is blind to by construction.

## Stage 5.5 — Capture guidance and next-best-pose

Once conditioning is observable (5.1) it can be optimised during capture instead of diagnosed
afterwards. (This stage was blocked on [C-03](../issues/archive/C-03-double-undistortion.md) — telling the
operator to work the image borders would have made things worse while the frame was being rectified
twice. C-03 is fixed, so the guidance below is now safe to issue.)

Baseline guidance, from the literature (ACFR `cam_lidar_calibration`; Tsai et al.):

- **10–20 poses**, not 2.
- **≥ 1–2 m depth range** between the closest and farthest board placement.
- **Spread across the width of the FoV**, not clustered at the principal point.
- **Maximise board yaw/pitch variation** — near-parallel board normals cannot constrain
  rotation.

Then close the loop: the interactive controller already has the buffer and (after 5.1) the
covariance. Report the *current* conditioning and the direction of the weakest-constrained DoF,
and tell the operator where to put the board next to shrink it. Fold the guidance into
`ros/interactive_solver_controller` and the operator docs.

## Success criteria

The phase is done when:

1. A degenerate capture is **rejected by the pipeline**, with a message naming what is missing
   (depth spread / normal spread / coverage) — not accepted with `"Calibration successful"`.
2. Every solved extrinsic ships with a reprojection RMSE, a held-out RMSE, a 6-DoF σ, and a
   background-alignment score, all persisted into the `dump_detections` JSON.
3. The "board is right, background is tilted" outcome is **detectable without looking at the
   overlay**.
4. Perturbing the calibration by a rotation about the board centroid produces a measurable
   degradation in at least one reported metric.

## References

- Tsai, Worrall, Shan, Lohr, Nebot. *Optimising the selection of samples for robust lidar camera
  calibration* (ITSC 2021). <https://arxiv.org/abs/2103.12287> · code:
  <https://github.com/acfr/cam_lidar_calibration>
  — the VOQ score; calibration that overfits its sample poses and fails to generalise to the
  scene. Directly the failure mode of this phase.
- Zhou, Li, Kaess. *Automatic Extrinsic Calibration of a Camera and a 3D LiDAR using Line and
  Plane Correspondences* (IROS 2018).
  <https://www.ri.cmu.edu/publications/automatic-extrinsic-calibration-of-a-camera-and-a-3d-lidar-using-line-and-plane-correspondences/>
  — line + plane correspondences reduce the minimum to one board pose.
- Huang, Grizzle. *Improvements to Target-Based 3D LiDAR to Camera Calibration*.
  <https://arxiv.org/abs/1910.03126>
  — known target geometry against LiDAR quantisation/systematic range error; fit that treats the
  camera as the most accurate sensor.
- Yuan, Liu, Hu, Zhang. *Pixel-level Extrinsic Self Calibration of High Resolution LiDAR and
  Camera in Targetless Environments*. <https://arxiv.org/abs/2103.01627>
  — plane-intersection edge cost, covariance via `Σ_T = (J_Tᵀ Σ⁻¹ J_T)⁻¹`, explicit scene
  degeneracy analysis.
- Levinson, Thrun. *Automatic Online Calibration of Cameras and Lasers* (RSS 2013) —
  edge-alignment / IDT scoring, the basis of Stage 5.4.
- *Experimental Evaluation of 3D-LiDAR Camera Extrinsic Calibration*.
  <https://arxiv.org/pdf/2007.01959>
  — experimental comparison of 3D-2D (PnP) vs 3D-3D formulations.
