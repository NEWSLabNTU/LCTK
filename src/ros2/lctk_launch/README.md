# LCTK Calibration Launch Package

This package provides YAML launch files for the LCTK (LiDAR and Camera Toolkit) calibration pipeline.

## Overview

The calibration pipeline consists of the following nodes:

1. **aruco_locator_node**: Detects ArUco markers in camera images
2. **lidar_board_detector**: Detects calibration boards in point clouds
3. **synchronizer**: Synchronizes detections from both sensors using timestamps
4. **extrinsic_solver**: Solves camera-LiDAR extrinsic transformation parameters
5. **pointcloud_image_overlay**: Visualizes calibration results using Rerun

## Example Usage

### Basic Usage

```bash
ros2 launch calib_launch calibration_pipeline.launch.yaml \
    camera_topic:=/my_camera/image_raw \
    pointcloud_topic:=/my_lidar/pointcloud \
    aruco_config_file:=/path/to/aruco_pattern.json5 \
    board_config_file:=/path/to/board_detector.json5 \
    debug_mode:=true
```

### With Configuration Files

```bash
# Provide configuration files for ArUco and board detection
ros2 launch calib_launch calibration_pipeline.launch.yaml \
    aruco_config_file:=$PWD/config/aruco_pattern.json5 \
    board_config_file:=$PWD/config/board_detector.json5 \
```

### Real-world Example with ZED Camera and Velodyne LiDAR

```bash
# Complete calibration setup for ZED + Velodyne
ros2 launch calib_launch calibration_pipeline.launch.yaml \
    camera_topic:=/zed/zed_node/left/image_rect_color \
    camera_info_topic:=/zed/zed_node/left/camera_info \
    pointcloud_topic:=/velodyne_points \
    debug_mode:=true \
    sync_window_ms:=100 \
    min_distance:=2.0 \
    max_distance:=20.0
```


## Launch Arguments

### calibration_pipeline.launch.yaml

| Argument            | Default               | Description                                                                    |
|---------------------|-----------------------|--------------------------------------------------------------------------------|
| `camera_topic`      | `/camera/image_raw`   | Input camera image topic                                                       |
| `camera_info_topic` | `/camera/camera_info` | Input camera info topic (provides camera intrinsics and distortion parameters) |
| `pointcloud_topic`  | `/lidar/pointcloud`   | Input point cloud topic                                                        |
| `aruco_config_file` | `""`                  | Path to ArUco pattern JSON5 config                                             |
| `board_config_file` | `""`                  | Path to board detector JSON5 config                                            |
| `debug_mode`        | `false`               | Enable debug logging and visualization                                         |
| `sync_window_ms`    | `50`                  | Synchronization window size (milliseconds)                                     |
| `max_distance`      | `10.0`                | Maximum point cloud distance filter (meters)                                   |
| `min_distance`      | `1.0`                 | Minimum point cloud distance filter (meters)                                   |


## Output Topics

The pipeline publishes the following topics:

- `/calibration/aruco_locator/aruco_detections` - ArUco marker detections
- `/calibration/lidar_board_detector/board_detections` - Board detections
- `/calibration/synchronizer/synchronized_detections` - Synchronized detection pairs
- `/calibration/synchronizer/synchronized_pointcloud` - Synchronized point clouds
- `/calibration/synchronizer/synchronized_image` - Synchronized images
- `/calibration/extrinsic_solver/extrinsic_transform` - Camera-LiDAR transform

## Visualization

The `pointcloud_image_overlay` node uses Rerun for GPU-accelerated 3D visualization. After launching, you can view the results in the Rerun viewer at `http://localhost:9876` (if web viewer is enabled).

The visualization shows:
- Camera images as background
- Point cloud overlay with distance-based coloring
- Real-time calibration results
- Interactive 3D view of the sensor setup
