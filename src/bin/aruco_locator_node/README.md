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

## Configuration

The node uses an ArUco pattern configuration file located at `config/aruco_pattern.json5`. This file defines the ArUco dictionary and marker properties.

## Command Line Options

- `--intrinsics-file`: Path to camera intrinsics YAML file (optional, uses camera_info topic if not provided)

## License

MIT License