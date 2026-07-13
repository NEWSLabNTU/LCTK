# H-09: A Quality Metric for the Extrinsic Solve — Design

- **Date:** 2026-07-13
- **Issue:** [H-09](../../issues/H-09-no-extrinsic-quality-metric.md)
- **Also settles:** [L-13](../../issues/L-13-calibration-metrics-msg-dead.md) (dead quality scaffolding), [L-12](../../issues/L-12-dead-solver-crates.md) (dead Rust crates)
- **Enables:** [H-07](../../issues/H-07-no-pose-diversity-gate.md) (you cannot gate on conditioning you do not measure)
- **Phase:** [Phase 5, Stage 5.1](../../roadmap/phase-5-stable-extrinsic-solution.md)

## Problem

`_solve_pnp` returns `(success, rvec, tvec)`. `last_solve_status` is the free text
`"Calibration successful"`. There is no reprojection error, no covariance, no condition number, and
no cross-validation anywhere in the pipeline. A degenerate solve and a good one are indistinguishable
from the outside.

This is the same defect that produced [C-04](../../issues/C-04-board-detector-gate-unreachable.md),
one layer up: **the system has no way to tell you it is not working.**

## The evidence this design is built on

Everything below was *measured* before it was specified, with a simulation that models what the
pipeline actually does: PnP over 16 coplanar ArUco corners per pose, where the 3D points are model
corners pushed through an ICP board pose carrying realistic per-pose rigid noise (1 cm translation,
0.5° rotation — the errors-in-variables problem of [M-13](../../issues/M-13-icp-quality-not-propagated.md)),
and the image corners carry 0.1 px noise.

Ground truth: a real LiDAR→camera extrinsic. Target: 2° / 50 mm (the project's own, from
`calibration_judge`).

| case | train RMSE | cond(JᵀJ) | σ_rot (CRLB) | subset sd_rot | subset sd_trans | **TRUE rot** | **TRUE trans** |
|---|---|---|---|---|---|---|---|
| Single pose (what `just demo` runs) | **0.125 px** | 1.3e5 | 0.10° | — | — | 0.79° | 33 mm |
| Degenerate (10× board held still) | 8.77 px | 4.6e4 | 1.22° | **5.77°** | **311 mm** | **5.07°** | **230 mm** |
| Well-spread (10 poses) | 10.88 px | **2.4e2** | 0.09° | 1.08° | 52 mm | 0.38° | 19 mm |

Four conclusions, three of which contradict the obvious design:

1. **Reprojection RMSE does not merely fail to separate the cases — it inverts them.** The
   degenerate solve scores a *better* RMSE (8.77 px) than the good one (10.88 px), while being 13×
   worse in rotation and failing the 2°/50 mm target outright. The single-pose solve scores the
   *best* RMSE of all (0.125 px) and is the worst-conditioned thing the system can produce.
   **RMSE must never be reported alone, and must never be used to rank or select.** This is the
   literature's "necessary but not sufficient" condition, and it is sharper than that phrasing
   suggests: it is actively misleading.

2. **`cond(JᵀJ)` is the discriminator.** 4.6e4 (degenerate) vs 2.4e2 (well-spread) — a 190× gap,
   and the single-pose case is worst at 1.3e5. It is also nearly free (see below).

3. **Leave-one-out cross-validation does not work for this problem.** Measured holdout/train ratio:
   **1.1× degenerate vs 1.3× well-spread** — flat, and pointing the wrong way. The reason is
   structural: when the board is held still, the held-out pose is *identical* to the training poses,
   so the model predicts it perfectly. LOO detects failure to generalise to *different* poses, and
   in the degenerate case there are none. **LOO is blind to precisely the failure it was meant to
   catch, and is cut from this design.** (An earlier draft of this spec, and the phase-5 roadmap,
   both proposed it as the headline metric.)

4. **Subset resampling works, and is the honest uncertainty estimate.** Spread of the solved
   parameters over all C(N,3) pose subsets: sd_rot **5.77° vs 1.08°**, sd_trans **311 mm vs 52 mm**
   — and it *predicts the true error* (5.77° estimated against 5.07° actual). The CRLB σ from
   `σ²(JᵀJ)⁻¹` said 1.22° for the same case: a **4× underestimate**, because it assumes all noise is
   in the pixels and structurally cannot see the ICP 3D error. σ is therefore a usable *relative*
   signal but a dishonest *absolute* one, and must be labelled as such wherever it is shown.

## Architecture

New package `ros/lctk_quality/` — **pure Python, numpy + cv2 only, no `rclpy`.** Both solver nodes
import it. Unit-testable without ROS, which is what lets the metrics be pinned against synthetic
degenerate and well-conditioned cases.

```
ros/lctk_quality/lctk_quality/
    residuals.py      per-corner and per-pose reprojection error
    conditioning.py   Jacobian -> cond(JtJ), per-DoF sigma (flagged optimistic)
    resampling.py     all C(N,k) pose subsets -> empirical parameter spread
    diversity.py      board-normal spread, depth range, image coverage
    report.py         QualityReport dataclass + the one-line summary string
  test/               synthetic; no ROS, no rosbag
```

### The Jacobian is free

`cv2.projectPoints` returns `(imagePoints, jacobian)`, shape `2N × 15`. Its **first 6 columns are
exactly ∂proj/∂(rvec, tvec)** — verified numerically against finite differences to ~1e-6. So
`cond(JᵀJ)` and `Σ ≈ σ²(JᵀJ)⁻¹` fall out of a call we are already making. No analytic derivation, no
autodiff.

### Resampling

For all C(N, 3) subsets of poses (N = 9–10 in the real captures → **120 subsets**, milliseconds),
re-solve PnP and take the standard deviation of the resulting 6-vectors. This is Tsai et al.'s
construction, and it yields a covariance **with no ground truth**. It is the headline number:

> *"Your extrinsic is uncertain to ±5.8° and ±311 mm."*

`k = 3` is the minimum that determines a non-degenerate PnP; it is a parameter, not a constant.

### Diversity

Board-normal pairwise angles, board-centroid depth range, image-coverage fraction of the ArUco
patches. These do not measure quality — they explain *why* it is bad and what to do about it, which
a bare condition number cannot.

## Surfacing — no message changes

- **`last_solve_status`** (already a `string`, so no `.srv` edit and no `lctk_interfaces` rebuild):

  ```
  DEGENERATE | uncert +/-5.8deg +/-311mm | cond 4.6e4 | rms 8.77px | normals span 3deg
  ```

- **Logs** carry the *guidance*, not just the number:

  ```
  [WARN] Calibration is under-constrained. Parameter spread across pose subsets is
         +/-5.8 deg / +/-311 mm, against a 2 deg / 50 mm target.
         Board normals span only 3 deg  -> vary board yaw/pitch.
         Depth range is 0.04 m          -> move the board nearer and farther (aim for >=1.5 m).
         Reprojection error (8.77 px) is NOT evidence of quality here.
  ```

- **`dump_detections` JSON**: a new top-level `"quality"` block. v3 is ours and the loader already
  ignores unknown keys, so this is forward-compatible.

- **Gate**: `reject_on_bad_quality`, **default `false`**. Report and warn; do not block. C-04 was a
  gate whose threshold was unreachable and which silently discarded every detection for months —
  thresholds get validated against real captures *before* anything is allowed to reject.

## Both solvers

| | metrics |
|---|---|
| `advanced_extrinsic_solver` (buffered, opt-in) | everything: residuals, conditioning, subset resampling, diversity |
| `extrinsic_solver_node` (**the default `just demo` path**) | residuals + conditioning, and an explicit warning that a single-pose solve is under-constrained by construction |

The default path solves from **one** detection pair — 16 coplanar points. Measured: the best-looking
RMSE in the whole table (0.125 px) and the worst conditioning (1.3e5). It is the configuration most
likely to mislead, and today it reports `"Calibration successful"`.

## Testing

Synthetic, deterministic, no rosbag, runs in CI. The suite is the simulation above, promoted to
tests:

1. **The metric must separate degenerate from well-conditioned.** `cond(JᵀJ)` and subset spread must
   both distinguish the two cases by a wide margin. If they cannot, the metric is worthless and we
   find out in milliseconds.
2. **The null-direction test.** Perturb a good extrinsic by *a rotation about the correspondence
   centroid* — the null direction derived in H-07. Assert train RMSE barely moves while `cond` and
   the subset spread blow up. This is the bug, encoded as a test.
3. **RMSE must be shown to invert.** Pin the measured fact that the degenerate case scores a lower
   RMSE than the good one, so that nobody later "simplifies" the design down to reprojection error.
4. **The Jacobian columns are what we think they are.** Assert `jac[:, :6]` matches finite
   differences of `(rvec, tvec)`.
5. **Subset spread tracks true error.** On synthetic data with known ground truth, assert the
   estimated spread is the right order of magnitude — it is the number the operator will act on.

## Deletions (settles L-12, L-13)

Nothing in the tree should offer a fifth answer to "how good is this calibration?".

- `ros/lctk_interfaces/msg/CalibrationMetrics.msg` — built, zero users, and IoU-shaped rather than
  residual-shaped.
- `rust/calibration-quality/` — its `reprojection_error` is a 3D-3D residual in metres, not pixels.
- `rust/dynamic-calibration/` — its only consumer, itself consumed by nobody.

`ros/calibration_judge/` **stays**: it scores against a supplied ground-truth transform, which makes
it a benchmark harness, not a field metric. It is orthogonal, and it is where the 2°/50 mm target
comes from.

## Out of scope

- **Background-consistency scoring** (projecting LiDAR edges against an image edge map) — the only
  metric that measures the tilt *directly* rather than inferring it from conditioning. That is
  phase 5.4 and a separate subsystem.
- **Acting on the metric**: rejecting bad poses (M-12), weighting by ICP quality (M-13), gating on
  diversity (H-07). This spec produces the numbers; those consume them.
- **Joint / errors-in-variables estimation** (phase 5.3). Note the measurements above quantify how
  much that would buy: the 3D board-pose noise is what drives the degenerate case to 5° of error,
  and it is invisible to the CRLB σ.
