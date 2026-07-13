# M-13 · Board-pose uncertainty is measured, then thrown away before the solver

- **Severity:** Medium
- **Area:** lidar_board_detector → extrinsic solvers
- **Status:** Open
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
[docs/roadmap/phase-5-stable-extrinsic-solution.md](../roadmap/phase-5-stable-extrinsic-solution.md).
