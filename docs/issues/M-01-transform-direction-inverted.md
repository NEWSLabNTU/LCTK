# M-01 · Transform frame labels are inverted vs ROS TF semantics

- **Severity:** Medium
- **Area:** extrinsic solvers / TF output
- **Status:** Deferred (needs visual verification)
- **Verified:** Static review
- **Location:**
  - `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:753-768`
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1357-1370`
  - `ros/lctk_launch/lctk_launch/tf_tree_broadcaster.py`

## Problem

`cv2.solvePnP` returns `(R, t)` mapping LiDAR-frame points into the camera frame (`p_cam = R·P_lidar + t`). That raw rvec/tvec is placed into a `TransformStamped` labeled `header.frame_id = lidar`, `child_frame_id = camera`. In ROS TF, a transform labeled lidar→camera must express the camera's pose in lidar coordinates — the inverse of what is stored. The in-repo overlay node bypasses TF and uses the transform directly as rvec/tvec for `projectPoints`, so topic-consumers and TF-consumers require opposite interpretations.

## Failure scenario

Any standard `tf2` consumer — the natural Autoware ingestion route via `/tf_static` — looks up the transform and gets it pointing the wrong way. The success log even prints "LiDAR → Camera", reinforcing the ambiguity.

## Suggested fix

Publish the transform with frame labels matching TF semantics (invert `(R, t)` before building the `TransformStamped`, or relabel parent/child), and keep the overlay node's direct-rvec/tvec usage consistent with that choice. Document the convention explicitly.

## Status note (2026-07-11)

Deferred, not because it isn't real but because it cannot be verified without a
visual check. The published transform is self-consistent within LCTK (the overlay
consumes it directly as `rvec`/`tvec` for `projectPoints`), so changing the TF
labels to the correct direction requires a coordinated change in the overlay
(invert it back) and the only correctness signal is whether point clouds still
project onto the image correctly — a visual result this environment can't produce.
Recommended fix, to be done with sample data + the overlay:

1. Publish the `TransformStamped` with correct TF semantics (invert the solvePnP
   `T_cam_lidar` so `frame_id=lidar, child=camera` really is the camera pose in
   the lidar frame).
2. Update `pointcloud_image_overlay` to invert the looked-up transform back into
   the LiDAR→camera-points mapping it feeds to `projectPoints`.
3. Verify the overlay alignment is unchanged on `just demo`, then confirm a
   `tf2` lookup gives the Autoware-correct direction.

Until then this is the one blocker between the solver output and a correct
Autoware `sensor_kit_calibration.yaml` (see [gap-autoware-export.md](./gap-autoware-export.md)).
