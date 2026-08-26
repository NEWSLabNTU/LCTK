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

```bash
# Build the node
source /opt/ros/humble/setup.bash
make build_interface
source install/setup.bash
cargo build --release --manifest-path src/bin/aruco_locator_node/Cargo.toml

# Run the node
ros2 run aruco_locator_node aruco_locator_node

# Run with custom intrinsics file
ros2 run aruco_locator_node aruco_locator_node --intrinsics-file config/camera_intrinsics.yaml
```

## ROS Topics

### Subscriptions
- `/image` (sensor_msgs/Image): Input camera images
- `/camera_info` (sensor_msgs/CameraInfo): Camera calibration information

### Publications
- `/aruco_detections` (vision_msgs/Detection2DArray): Detected ArUco markers with 2D positions
- `target_identity` (lctk_interfaces/CalibrationTargetIdentity): Reliable, transient-local target
  identity. It is relative to the node namespace so a late-starting solver receives the identity
  for its camera observer.

## Configuration

Set `target_config` to a Target Definition JSON5 file. It owns dictionary, marker IDs, paper
layout and target identity; `aruco_detector_config_file` remains the separate detection-tuning
file.

For temporary compatibility with maintained pre-cutover launch files, `aruco_config_file` alone
selects the explicit `hollow_1000_aruco_4` Target Definition. Supplying both parameters is an
error, and this legacy alias is removed in W5-E1; it cannot select or define another target.

## Command Line Options

- `--intrinsics-file`: Path to camera intrinsics YAML file (optional, uses camera_info topic if not provided)

## License

MIT License
