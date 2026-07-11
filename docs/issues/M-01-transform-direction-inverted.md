# M-01 · Transform frame labels are inverted vs ROS TF semantics

- **Severity:** Medium
- **Area:** extrinsic solvers / TF output
- **Status:** Open
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
