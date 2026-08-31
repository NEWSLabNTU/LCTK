# LCTK Calibration Launch Package

This package provides YAML launch files for the LCTK (LiDAR and Camera Toolkit) calibration pipeline.

## Overview

Nodes are generated dynamically from a YAML calibration config (devices, markers, calibration
pairs). For a LiDAR-camera pair the generated pipeline is:

1. **aruco_locator_node**: Detects ArUco markers in camera images
2. **lidar_board_detector**: Detects calibration boards in point clouds
3. **lidar_to_camera_solver**: Synchronizes both detection streams (via `lctk_sync` /
   Conflux) and solves the camera-LiDAR extrinsic transformation, in `continuous` (default) or
   `manual` (multi-pose buffered) mode
4. **pointcloud_image_overlay** (optional, `enable_overlay:=true`): Visualizes calibration
   results using Rerun

A LiDAR-LiDAR pair instead generates a **lidar_to_lidar_solver** node consuming both
`lidar_board_detector` outputs.

## Example Usage

### Basic Usage

```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=/path/to/your_config.yaml
```

`config_file` is a YAML document describing sensor topics/frames, calibration markers (each
naming a Target Definition + Detector Tuning preset), calibration pairs, and a required `sync:`
section. See `config/examples/sample_data.yaml` for a complete example and the "Configuration
Format" section of the repo root `CLAUDE.md` for the full schema.

### With Debug Logging and RViz

```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=/path/to/your_config.yaml \
    debug_mode:=true \
    log_level:=debug \
    enable_rviz:=true
```

### Two-LiDAR Example

```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=$(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/two_lidar.yaml
```

Equivalently, via the justfile: `just calibrate /path/to/your_config.yaml` (see the repo root
`README.md`).

## Launch Arguments

### calibrate.launch.py

| Argument         | Default      | Description                                                                 |
|------------------|--------------|------------------------------------------------------------------------------|
| `config_file`    | (required)   | Path to the YAML calibration config                                         |
| `debug_mode`     | `false`      | Enable debug topics                                                         |
| `log_level`      | `info`       | ROS log level (debug/info/warn/error/fatal)                                 |
| `mode`           | `offline`    | Transport QoS: `offline` (RELIABLE, recorded data) or `realtime` (BEST_EFFORT, live) |
| `enable_rviz`    | `true`       | Launch RViz alongside the pipeline                                          |
| `rviz_config`    | `config/rviz/calibration.rviz` | Path to RViz config file                                  |
| `solver_mode`    | `continuous` | `continuous` (latest-pair auto-solve), `manual` (multi-pose buffer), or `assisted` (auto-capture + web review) |
| `enable_overlay` | `false`      | Launch `pointcloud_image_overlay` for visual verification (one per pair)    |
| `enable_judge`   | `false`      | Launch the calibration quality judge (one per pair)                         |

(The `just calibrate` / `just demo` recipes pass their own defaults for several of these — see
the repo root `README.md`'s "Configuration Variables" section — which differ from the launch
file's own defaults above.)

The old XML arguments `aruco_config_file:=` and `board_config_file:=` no longer exist on any
maintained launch path; per-marker config is now supplied inside `config_file`'s YAML via
`target_config` and `detector_config`.

## Output Topics

Topics are namespaced per generated node, e.g. for the pair `(top_lidar, front_center)`:

- `/calibration/aruco_locator/aruco_detections` - ArUco marker detections
- `/calibration/lidar_board_detector/calibration_board_detections` - Board detections
- `/calibration/<lidar>_<camera>/extrinsic_transform` - Camera-LiDAR transform, published by
  `lidar_to_camera_solver`
- `/calibration/pointcloud_overlay` - Rerun visualization, published by
  `pointcloud_image_overlay` when `enable_overlay:=true`

## Visualization

The `pointcloud_image_overlay` node uses Rerun for GPU-accelerated 3D visualization. After launching, you can view the results in the Rerun viewer at `http://localhost:9876` (if web viewer is enabled).

The visualization shows:
- Camera images as background
- Point cloud overlay with distance-based coloring
- Real-time calibration results
- Interactive 3D view of the sensor setup
