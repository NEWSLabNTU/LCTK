# aruco_locator_node

A ROS 2 node for detecting ArUco markers in camera images.

## Overview

This node subscribes to camera images and publishes detected ArUco marker positions. It integrates with ROS 2 to provide real-time marker detection for robotics applications including camera calibration and visual localization.

## Requirements

- ROS 2 Humble or later
- Rust 1.56 or later
- OpenCV 4.6.0
- rclrs (ROS 2 Rust client library)

## Quick Start

Always build via `just build` from the repo root (never a raw `cargo build`/`colcon build`
invocation — see the repo root `CLAUDE.md`).

```bash
just build
source install/setup.bash

# Run directly (normally launched by lctk_launch's calibrate.launch.py instead)
ros2 run aruco_locator_node aruco_locator_node \
    --ros-args \
    -p target_config:=/path/to/config/targets/hollow_1000_aruco_4_v1.json5 \
    -p aruco_detector_config_file:=/path/to/config/aruco/aruco_detector.json5
```

There is no `--intrinsics-file` CLI flag: intrinsics come from the `camera_info` topic, derived
from the resolved `image` topic's namespace (`<ns>/image` -> `<ns>/camera_info`), the
image_pipeline convention.

## ROS Topics

### Subscriptions (relative; remapped by the launch file)
- `image` (sensor_msgs/Image): Input camera images
- `<image topic's namespace>/camera_info` (sensor_msgs/CameraInfo): Derived automatically from
  the resolved `image` topic, not a separate parameter

### Publications
- `aruco_detections` (vision_msgs/Detection2DArray): Detected ArUco markers with 2D positions
- `target_identity` (lctk_interfaces/CalibrationTargetIdentity): Reliable, transient-local target
  identity. It is relative to the node namespace so a late-starting solver receives the identity
  for its camera observer.
- `image_with_detections` (sensor_msgs/Image): Debug overlay, published only when
  `debug_overlay_enabled:=true`

## ROS Parameters

- `target_config` (required): Path to a Target Definition JSON5 file. It owns dictionary, marker
  IDs, paper layout and target identity.
- `aruco_detector_config_file` (required): Path to the separate detector-tuning JSON5 (corner
  refinement, adaptive threshold) — no board geometry belongs here.
- `debug_overlay_enabled` (default: `false`): Publish `image_with_detections`.
- `use_best_effort_qos` (default: `true`): BEST_EFFORT for live sensors, RELIABLE for rosbag
  playback (mirrors the pipeline's `mode` parameter).

There is no legacy `aruco_config_file` alias any more — `target_config` is the sole, required
source of the printed pattern.

## License

MIT License
