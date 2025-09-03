# calibration_board_locator

A ROS 2 node for detecting calibration boards in point cloud data.

## Overview

This node processes 3D point cloud data to detect hollow calibration boards used for LiDAR-camera calibration. It identifies planar boards with specific geometric patterns and publishes their 3D positions and orientations.

## Requirements

- ROS 2 Humble or later
- Rust 1.56 or later
- rclrs (ROS 2 Rust client library)

## Quick Start

```bash
# Build the node
source /opt/ros/humble/setup.bash
make build_interface
source install/setup.bash
cargo build --release --manifest-path src/bin/calibration_board_locator/Cargo.toml

# Run the node
ros2 run calibration_board_locator calibration_board_locator

# Run with custom configuration
ros2 run calibration_board_locator calibration_board_locator \
    --ros-args -p board_detector_config:=/path/to/config.json5
```

## ROS Topics

### Subscriptions
- `/input_pointcloud` (sensor_msgs/PointCloud2): Input point cloud data

### Publications
- `/calibration_board_detections` (vision_msgs/Detection3DArray): Detected calibration boards with 3D poses

## Configuration

The node uses configuration files for:
- Board detector parameters: `config/board_detector.json5`
- ArUco pattern specifications: `config/aruco_pattern.json5`

## ROS Parameters

- `board_detector_config`: Path to board detector configuration file
- `aruco_pattern_config`: Path to ArUco pattern configuration file

## License

MIT License