# pointcloud_image_overlay

A ROS 2 node for overlaying LiDAR point cloud data onto camera images using calibrated extrinsic parameters.

## Overview

This node visualizes the projection of 3D point cloud data onto 2D camera images using the transformation between LiDAR and camera coordinate frames. It uses Rerun for real-time visualization of the overlay, with distance-based coloring of projected points.

## Requirements

- ROS 2 Humble or later
- Rust 1.56 or later
- Rerun visualization tool
- rclrs (ROS 2 Rust client library)

## Quick Start

```bash
# Build the node
source /opt/ros/humble/setup.bash
make build_interface
source install/setup.bash
cargo build --release --manifest-path src/bin/pointcloud_image_overlay/Cargo.toml

# Run the node
ros2 run pointcloud_image_overlay pointcloud_image_overlay

# Run with custom parameters
ros2 run pointcloud_image_overlay pointcloud_image_overlay \
    --ros-args -p max_distance:=50.0
```

## ROS Topics

### Subscriptions
- `/pointcloud` (sensor_msgs/PointCloud2): Input point cloud data
- `/image` (sensor_msgs/Image): Camera image for overlay
- `/camera_info` (sensor_msgs/CameraInfo): Camera calibration parameters
- `/tf` (geometry_msgs/TransformStamped): Transform from LiDAR to camera frame

### Visualization

The node uses Rerun for visualization. Start the Rerun viewer before running the node:
```bash
rerun
```

## ROS Parameters

- `max_distance`: Maximum distance for point cloud visualization (default: 100.0 meters)

## License

MIT License