# LiDAR-to-Camera Solver

ROS 2 LiDAR-camera extrinsic solver with one geometry and estimator backend and two operating modes.

## Modes

- `continuous` (default): replace the latest synchronized detection pair, solve immediately, and
  publish the transform. Useful for quick visual checks. One board placement is under-constrained;
  low reprojection RMS is not evidence of a trustworthy calibration.
- `manual`: retain operator-selected pairs, solve the multi-pose buffer, and expose services for
  buffer management, archive load/save, and transform adjustment.

Both modes use float64 correspondences, SQPnP initialization, LM refinement, and board-pose
covariance weighting when covariance is available. Both require the detector's
`corner_aligned_plate_center_v1` convention announcement.

## Run

```bash
# Auto-publish each latest pair
just solver_mode=continuous lidar-camera

# Multi-pose workflow
just solver_mode=manual lidar-camera
just manual-solver-controller
```

Direct invocation uses the same parameter:

```bash
ros2 run lidar_to_camera_solver lidar_to_camera_solver \
  --ros-args -p solver_mode:=manual
```

Accepted `solver_mode` values are exactly `continuous` and `manual`.

## Interfaces

Subscribed topics:

- `aruco_detections` (`vision_msgs/Detection2DArray`)
- `calibration_board_detections` (`vision_msgs/Detection3DArray`)
- camera info derived from `camera_topic`
- `/lctk/board_frame_convention` (`std_msgs/String`, latched)

Published topics:

- `extrinsic_transform` (`geometry_msgs/TransformStamped`)
- `axis_markers` (`visualization_msgs/MarkerArray`)

Manual mode exposes `add_detection`, `clear_buffer`, `get_status`, `list_buffer`,
`remove_detection`, `dump_detections`, `load_detections`, `adjust_transform`, `reset_transform`, and
`get_pose_info` below the node's private namespace.

## Build and test

```bash
just build
just test
just lint-py
```
