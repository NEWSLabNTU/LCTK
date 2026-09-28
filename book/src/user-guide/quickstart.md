# Quick Start

This tutorial walks you through your first calibration using included sample data.

## Step 1: Install LCTK

Follow the [Installation guide](./installation.md) to clone the repository, install its
dependencies, and build LCTK. Return here when setup is complete.

## Step 2: Run the demo

The demo is a [session](./sessions.md) — one directory holding the sample recording and
everything needed to calibrate against it. The simplest command starts both the sample
playback and calibration graph:

```bash
just demo
```

`just demo` runs through `play_launch`, whose status page is at
`http://localhost:8000`. The justfile defaults to assisted solver mode; its review
page is at `http://localhost:8080`.

The direct launch form is useful when you do not need `play_launch`:

```bash
source install/setup.bash
ros2 launch lctk_launch session.launch.py \
    session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne
```

Direct launch defaults to continuous mode and does not provide the `play_launch` status
page. Pass `solver_mode:=assisted` if you want the review page on port 8080.

The system will:
1. Play back recorded LiDAR and camera data
2. Detect the Calibration Target in point clouds
3. Detect ArUco markers on the Calibration Target in camera images
4. Compute the LiDAR-to-camera transformation

## Step 3: Monitor Progress

In another terminal, check the calibration output:

```bash
source install/setup.bash

# Watch for calibration transform (namespace is "<lidar>_<camera>" from the session's
# device names; sample3-hollow-velodyne names them top + front_center)
ros2 topic echo /calibration/top_front_center/extrinsic_transform

# Inspect the two detection streams
ros2 topic hz /calibration/front_center/aruco_detections
ros2 topic hz /calibration/top_calibration_board/calibration_board_detections
```

When the solver has a current estimate, the transform topic contains a `TransformStamped`
message. The topic is namespaced from the session's device and marker names; other
sessions use different names.

## Step 4: Visualize (Optional)

If you have a display, `just demo` already launches RViz by default. To open RViz
separately against the default layout, run:

```bash
just rviz
```

To disable RViz for the demo, put the justfile variable **before** the recipe name:

```bash
just rviz_enabled=false demo
```

In RViz:
1. Set **Fixed Frame** to `velodyne_top`
2. Add PointCloud2: `/sensing/lidar/top/pointcloud_raw`
3. Add MarkerArray: `/calibration/top_calibration_board/debug/final_board_pose`

## What Happened?

The calibration system:
- **Detected** a Calibration Target with circular holes in the LiDAR point cloud
- **Detected** ArUco markers on the same Calibration Target in camera images
- **Solved** the 3D transformation from LiDAR frame to camera frame

## Next Steps

- **Use your own data**: See [Calibration Sessions](./sessions.md) and
  [LiDAR-Camera Calibration](./lidar-camera.md)
- **Calibrate multiple LiDARs**: See [Multi-LiDAR Calibration](./multi-lidar.md)
- **Adjust parameters**: See [Configuration](./configuration.md)
- **Troubleshoot**: See [Troubleshooting](./troubleshooting.md)

## Common Issues

**"Command not found"**: Run `source install/setup.bash` after building

**"No detections"**: Check sample data is playing with `ros2 topic list`

**"Build failed"**: See [Installation](./installation.md) for troubleshooting
