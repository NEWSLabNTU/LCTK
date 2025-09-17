# extrinsic_solver

A ROS 2 node for solving extrinsic calibration parameters between LiDAR and camera sensors.

## Overview

This node combines ArUco marker detections from camera images with calibration board detections from LiDAR point clouds to compute the transformation between the two sensor coordinate frames. It uses Perspective-n-Point (PnP) algorithms to solve for the extrinsic parameters.

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
cargo build --release --manifest-path src/bin/extrinsic_solver/Cargo.toml

# Run the node
ros2 run extrinsic_solver extrinsic_solver --intrinsics-file config/camera_intrinsics.yaml

# Run with specific PnP method
ros2 run extrinsic_solver extrinsic_solver \
    --intrinsics-file config/camera_intrinsics.yaml \
    --method SQPNP
```

## ROS Topics

### Subscriptions
- `/aruco_detections` (vision_msgs/Detection2DArray): 2D ArUco marker detections from camera
- `/calibration_board_detections` (vision_msgs/Detection3DArray): 3D calibration board detections from LiDAR
- `/camera_info` (sensor_msgs/CameraInfo): Camera calibration information

### Publications
- `/extrinsic_transform` (geometry_msgs/TransformStamped): Computed transformation from LiDAR to camera

## Command Line Options

- `--intrinsics-file`: Path to camera intrinsics YAML file
- `--method`: PnP solving method (P3P, ITERATIVE, EPNP, SQPNP, etc.)
- `--output-file`: Path to save calibration results

## Configuration

The node uses ArUco pattern configuration from `config/aruco_pattern.json5` which defines the marker layout on calibration boards.

## License

MIT License