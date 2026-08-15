# M-13 · Board-pose uncertainty is measured, then thrown away before the solver

- **Severity:** Medium
- **Area:** lidar_board_detector → extrinsic solvers
- **Status:** Fixed (2026-07-14)
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/lidar_board_detector/src/main.rs:975-994` (ICP stats published as a `String` debug topic)
  - `ros/lidar_board_detector/src/main.rs:1760-1811` (`convert_board_detection_to_detection3d`, `covariance: [0.0; 36]` at `:1801`, `score: 1.0` hardcoded)
  - `rust/hollow-board-detector/src/algo.rs:880-946` (ICP loss and Kabsch step)
  - `rust/hollow-board-config/src/lib.rs:292-391` (correspondence model)

## Problem

The board detector computes a real quality signal — `IcpStatistics { iterations, initial_loss,
final_loss, min_loss, successful, convergence_reason }` plus an inlier count — and gates
acceptance on it (`icp_good_fit_threshold: 0.012`, `icp_min_inlier_points: 1000`,
`main.rs:1330`). It then publishes that signal as a **formatted `std_msgs/String` on a debug
topic** and emits the `Detection3D` with `covariance: [0.0; 36]` and `score: 1.0`.

The solver therefore treats every board pose as exact and equally trustworthy, when in fact:

- **the pose error is strongly anisotropic.** In the hollow-board correspondence model, an
  interior LiDAR point's correspondence is its own projection onto the board plane, so interior
  points contribute *only* out-of-plane residual. In-plane `(x, y, yaw)` is constrained
  **only** by the square's edges and the 3 hole rims (`hollow-board-config/src/lib.rs:339-387`).
  Out-of-plane is tightly observed; in-plane is loose.
- **the solve is an errors-in-variables problem.** PnP object points are the ideal model corners
  pushed through `T_board` ([H-07](./H-07-no-pose-diversity-gate.md)), so all of that anisotropic
  ICP error lands in the "known" 3D points — and `cv2.solvePnP` assumes 3D points are noiseless
  and all noise is in the pixels. The estimate is biased, not merely noisy.
- ICP itself has no robust kernel: Kabsch closed-form SVD, a single hard 50 mm gate
  (`icp_outlier_threshold: 0.050`, documented in-config as "effectively disabled"),
  `icp_damping_factor: 1.0`, and the loss is a **mean**, not an RMS (`algo.rs:880-885`).

## Failure scenario

A grazing-incidence board yields a pose whose in-plane component is metres-scale wrong in the
worst case and centimetres-wrong routinely, and the solver weights its 16 corners exactly the
same as a clean fronto-parallel pose's. There is no mechanism to down-weight it, and no field in
the message that could carry the information even if there were.

## Suggested fix

1. Populate `Detection3D.results[0].pose.covariance` with a real 6×6 board-pose covariance.
   The cheap version is `Σ ≈ σ²(JᵀJ)⁻¹` from the ICP correspondence Jacobian at convergence,
   which naturally comes out anisotropic (tight normal-direction, loose in-plane) and needs no
   new machinery. Put `final_loss` / inlier ratio into `score`.
2. Consume it in the solver: at minimum a per-pose scalar weight from `final_loss` and inlier
   count; properly, a Mahalanobis weighting in a joint cost.
3. Add a Huber kernel and an RMS loss to the ICP itself.

Item (1) is a prerequisite for the errors-in-variables / joint-optimisation work in
[docs/roadmap/phase-5-stable-extrinsic-solution.md](../../roadmap/phase-5-stable-extrinsic-solution.md).

## Resolution (2026-07-14)

The detector now publishes a real 6x6 board-pose covariance, and the solver consumes it.

### Producing it — the anisotropy is the whole point

`compute_pose_covariance` (`ros/lidar_board_detector/src/main.rs`) builds the information matrix
from the converged ICP correspondences. The linearisation projects each residual onto the direction
the model *actually* constrains — at a closest-point correspondence, the surface normal:

```
e_i = n_i . (p_i - q_i)
J_i = [ n_i^T , (d_i x n_i)^T ]     d_i = q_i - c   (c = the PUBLISHED pose origin)
H   = sum J_i^T J_i                 Cov = sigma^2 * H^-1
```

This reproduces the anisotropy the issue predicted, for free. Interior points project onto the board
normal, so they say **nothing** about where the board sits within its own plane; only border and
hole-rim points have an in-plane residual. A board with sparse edge returns is therefore tight
out-of-plane and nearly free in-plane, and the covariance now says so.

Two things that would each have silently produced a wrong answer:

- **`H` is routinely singular, and that is a result, not an error.** With only interior points it
  has rank 3. `try_inverse()` bails exactly on the case this covariance exists to describe, and a
  single whole-matrix fallback throws away the DoF that *are* well determined. So it is inverted
  per eigendirection: each mode gets `sigma^2 / lambda`, and an unobservable mode saturates at a
  large variance instead of collapsing to zero. **Zero is the dangerous value** — downstream it
  reads as "this pose is exact", which is the original bug.
- **The published pose is not the ICP pose.** A post-ICP fixup moves the origin to the lowest corner
  and rotates the frame by 90°·k. Rather than push the covariance through that adjoint (easy to get
  silently transposed), `J` is built about the *published* origin — the fixup is a pure
  re-parameterisation of the same plate, so the model points are unchanged and the covariance comes
  out directly in the published frame.

`score` also stops being a hardcoded `1.0`; it now decays with the ICP fit error.

### Consuming it

`BoardDetection` gains a `covariance` field. `_pose_weight` propagates it onto the ArUco corners
that pose generated (`J_corner = [I | -[c]_x]`, `Sigma_corner = J Sigma_pose J^T`) and weights the
pose by the inverse of the resulting positional sigma.

**No OpenCV PnP entry point accepts per-point weights** — not `solvePnP`, not `solvePnPRefineLM`,
not `solvePnPRefineVVS`. So M-12's LM polish slot becomes a weighted `scipy.least_squares` when a
covariance is present, and falls back to OpenCV's unweighted `RefineLM` when it is not (so an older
detector behaves exactly as before). This also completes item 4 of
[M-12](./M-12-no-robust-estimation-or-refinement.md).

### Verified

**On the live pipeline** — the covariance is on the wire and the numbers are physical:

```
score (was hardcoded 1.0):  0.557
covariance all-zero?        False
  sigma translation (mm):  x=2.45   y=15.15   z=5.55     <- 6x anisotropy, on real data
  sigma rotation   (deg):  rx=1.11  ry=0.21   rz=0.13
  symmetric?                  True
```

**Rust** (`covariance_tests` in `lidar_board_detector`, 4 tests): interior-only correspondences
report the in-plane DoF as unobservable while `z` stays tight; adding border points collapses the
in-plane variance by >100x. Symmetry and row-major order pinned.

**Python** (`test_pose_weighting.py`, 4 tests): a loose pose is weighted down; no covariance means
no behaviour change; and — the test that justifies the mechanism — **weighted refinement beats
unweighted when one pose in the buffer has a bad ICP fit.** If that had failed, the covariance would
be decoration and should have been deleted rather than maintained.

### Left

The ICP itself still has no robust kernel (Kabsch, no Huber; `avg_loss` is a mean, not an RMS).
That is a detector-algorithm change, separate from propagating the uncertainty it already has.
