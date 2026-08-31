# Quick Start

This tutorial walks you through your first calibration using included sample data.

## Step 1: Install LCTK

```bash
cd ~/repos  # or your preferred location
git clone https://github.com/your-org/LCTK.git
cd LCTK

# Run setup (installs ROS 2, Rust, dependencies)
./setup.sh

# Reload shell after setup
source ~/.bashrc

# Build the project
just build
```

## Step 2: Run Demo

The demo is a [session](./sessions.md) — one directory holding the sample recording and
everything needed to calibrate against it. `session.launch.py` plays the data and runs the
calibration graph:

```bash
source install/setup.bash
ros2 launch lctk_launch session.launch.py \
    session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne
```

Or, with the justfile shorthand:

```bash
just demo
```

Open `http://localhost:8000` in your browser to see the web UI showing node status.

The system will:
1. Play back recorded LiDAR and camera data
2. Detect the calibration board in point clouds
3. Detect ArUco markers in camera images
4. Compute the LiDAR-to-camera transformation

## Step 3: Monitor Progress

In another terminal, check the calibration output:

```bash
source install/setup.bash

# Watch for calibration transform (namespace is "<lidar>_<camera>" from the session's
# device names; sample3-hollow-velodyne names them top + front_center)
ros2 topic echo /calibration/top_front_center/extrinsic_transform

# Check detection rates (should be >1 Hz)
ros2 topic hz /calibration/front_center/aruco_detections
ros2 topic hz /calibration/top_calibration_board/calibration_board_detections
```

When calibration succeeds, you'll see a `TransformStamped` message with the LiDAR-to-camera transformation.

## Step 4: Visualize (Optional)

If you have a display, launch RViz:

```bash
just rviz
```

Or enable RViz in the demo. Justfile variables go **before** the recipe name:

```bash
just rviz_enabled=true demo
```

In RViz:
1. Set **Fixed Frame** to `velodyne_top`
2. Add PointCloud2: `/sensing/lidar/top/pointcloud_raw`
3. Add MarkerArray: `/calibration/top_calibration_board/debug/final_board_pose`

## What Happened?

The calibration system:
- **Detected** a calibration board with circular holes in the LiDAR point cloud
- **Detected** ArUco markers on the same board in camera images
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
