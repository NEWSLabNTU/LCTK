# 0004. The LiDAR-to-camera solver owns camera-frame board pose

- **Date:** 2026-09-03
- **Status:** accepted

## Context

`rust/aruco-detector/src/multi_aruco.rs` contains a dormant pose/ICP branch, including
`ImageDetection::estimate_pose`, `PoseEstimation`, `ImagePoseMarker`, and `fit_icp`. The repository
has no live caller for that branch. The archived [L-12 finding](../issues/archive/L-12-dead-solver-crates.md)
already identified the broader risk of retaining solver implementations that are not on the live
path.

The live `ros/lidar_to_camera_solver` node already owns the camera-frame board-pose solve and the
subsequent extrinsic solve. Rebuilding PnP or retaining a second ArUco ICP implementation would
create competing owners for the same concept and would preserve dead dependencies and tuning
surfaces such as `icp_rejection_threshold`.

## Decision

The ArUco detector is responsible for image marker detection, corner undistortion, and exposing
marker observations. It does not estimate camera-frame board pose and does not run ICP.

The `ros/lidar_to_camera_solver` node remains the sole owner of camera-frame board pose, PnP
initialization, refinement, and extrinsic solving. Delete the unused ArUco pose/ICP API and its
now-unused dependencies; do not replace it with another detector-side solver.

## Consequences

- Pose and extrinsic behavior has one live owner and one tuning surface.
- The ArUco crate becomes smaller and retains only the marker-observation contract.
- Consumers of the dormant Rust pose API will need to migrate or be removed; this is an intentional
  public API break for an unused path.
- Future camera-pose changes belong in `lidar_to_camera_solver`, with the existing solver tests and
  diagnostics providing the verification surface.
- Historical L-12 text remains archived; the implementation plan records this ADR as the decision
  that supersedes its deferred “keep for a future phase” rationale.
