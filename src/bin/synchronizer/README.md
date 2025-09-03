# synchronizer

A ROS 2 node for time-synchronizing multiple sensor data streams.

## Overview

This node synchronizes detection messages from different sensors (ArUco markers from cameras and calibration boards from LiDAR) based on their timestamps. It ensures that corresponding detections from different sensors are processed together, which is critical for accurate sensor calibration.

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
cargo build --release --manifest-path src/bin/synchronizer/Cargo.toml

# Run the node
ros2 run synchronizer synchronizer

# Run with custom synchronization window
ros2 run synchronizer synchronizer \
    --ros-args -p sync_tolerance_ms:=100
```

## ROS Topics

### Subscriptions
- `/aruco_detections` (vision_msgs/Detection2DArray): ArUco marker detections from camera
- `/board_detections` (vision_msgs/Detection3DArray): Calibration board detections from LiDAR

### Publications
- `/synchronized/aruco_detections` (vision_msgs/Detection2DArray): Time-synchronized ArUco detections
- `/synchronized/board_detections` (vision_msgs/Detection3DArray): Time-synchronized board detections

## ROS Parameters

- `sync_tolerance_ms`: Maximum time difference (in milliseconds) for messages to be considered synchronized (default: 50ms)
- `buffer_size`: Number of messages to buffer for synchronization (default: 100)

## Synchronization Algorithm

The node uses a multi-stream synchronization algorithm that:
1. Buffers incoming messages from each stream
2. Finds the best temporal matches within the tolerance window
3. Publishes synchronized message pairs

## License

MIT License