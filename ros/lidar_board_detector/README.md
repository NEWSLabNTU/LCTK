# lidar_board_detector

A ROS 2 node for detecting calibration boards in point cloud data.

## Overview

This node processes 3D point cloud data to detect hollow calibration boards used for LiDAR-camera calibration. It identifies planar boards with specific geometric patterns and publishes their 3D positions and orientations.

## Requirements

- ROS 2 Humble or later
- Rust 1.56 or later
- rclrs (ROS 2 Rust client library)

## Quick Start

```bash
# Build the node (always via just build -- see the repo root CLAUDE.md)
just build

# Run the node directly (normally launched by lctk_launch's calibrate.launch.py instead)
ros2 run lidar_board_detector lidar_board_detector \
    --ros-args \
    -p target_config:=/path/to/config/targets/hollow_1000_aruco_4_v1.json5 \
    -p detector_config:=/path/to/config/board/hollow_1000/velodyne.json5
```

## ROS Topics

### Subscriptions
- `/input_pointcloud` (sensor_msgs/PointCloud2): Input point cloud data

### Publications
- `/calibration_board_detections` (vision_msgs/Detection3DArray): Detected calibration boards with 3D poses

## Configuration

The node reads two required, separately-scoped configuration files -- see the repo root
`CLAUDE.md`'s "Config-Driven Calibration" section for the full split:
- **Target Definition** (physical target geometry: plate, cutouts, fiducial layout), e.g.
  `config/targets/hollow_1000_aruco_4_v1.json5`
- **Detector Tuning** (sensor-specific ICP/RANSAC parameters; no geometry), e.g.
  `config/board/hollow_1000/velodyne.json5`

A crop-box config is additionally required when the Detector Tuning preset selects
`detection_mode: "bbox"` (e.g. `config/board/bbox.json5`); the shipped `bbox_free` presets don't
need one.

## ROS Parameters

- `target_config`: Path to the Target Definition file (required)
- `detector_config`: Path to the Detector Tuning preset file (required)
- `bbox_file`: Path to the crop-box config file (required only when `detector_config` selects
  `detection_mode: "bbox"`)

## License

MIT License