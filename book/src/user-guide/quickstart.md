# Quick Start Tutorial

This tutorial walks you through your first calibration using included sample data. You'll go from installation to calibration results in ~10 minutes.

## Step 1: Install LCTK

```bash
cd ~/repos  # or your preferred location
git clone https://github.com/your-org/LCTK.git
cd LCTK

# Run interactive setup (installs ROS 2, Rust, dependencies)
make prepare

# Build the project (takes ~5-10 minutes first time)
make build
```

If build succeeds, you'll see `✓ Build complete` at the end.

## Step 2: Launch Sample Data

Open a new terminal and start the sample data player:

```bash
cd ~/repos/LCTK
make launch_lidar_camera_sample_data
```

This plays back recorded LiDAR and camera data in a loop. You should see:
- `[velodyne_driver]: Publishing packet data...`
- `[gscam_node]: Publishing images...`

Leave this running.

## Step 3: Run Calibration

Open **another terminal** and launch the calibration pipeline:

```bash
cd ~/repos/LCTK
make launch_lidar_camera_calibration
```

The system will:
1. Detect the calibration board in point clouds (LiDAR)
2. Detect ArUco markers in camera images
3. Synchronize detections
4. Compute the camera-to-LiDAR transformation

## Step 4: Monitor Progress

In a **third terminal**, check the calibration output:

```bash
# Watch for successful calibration
ros2 topic echo /calibration_transform

# Check detection rates (should be >1 Hz)
ros2 topic hz /aruco_detections
ros2 topic hz /calibration_board_detections
```

When calibration succeeds, you'll see a `TransformStamped` message with the camera-to-LiDAR transformation (translation and rotation).

## Step 5: Visualize (Optional)

```bash
rviz2
```

In RViz:
1. Set **Fixed Frame** to `velodyne`
2. Add → By Topic → `/sensing/lidar/top/pointcloud_raw`
3. Add → By Topic → `/calibration/debug/final_board_pose` (markers)
4. You'll see the point cloud and detected board visualization

## What Just Happened?

The calibration system:
- **Detected** a 1m × 1m hollow board with 4 circular holes in the LiDAR point cloud
- **Detected** ArUco markers on the same board in camera images
- **Matched** these detections in time using timestamps
- **Solved** the 3D transformation from camera frame to LiDAR frame

## Next Steps

- **Use your own data**: See [LiDAR-Camera Calibration](./lidar-camera.md)
- **Calibrate multiple LiDARs**: See [Multi-LiDAR Calibration](./multi-lidar.md)
- **Adjust parameters**: See [Configuration](./configuration.md)
- **Troubleshoot**: See [Troubleshooting](./troubleshooting.md)

## Common First-Time Issues

**"Command not found"**: Run `source install/setup.bash` after building

**"No detections"**: Check that sample data is playing (`ros2 topic list` should show `/sensing/lidar/top/pointcloud_raw`)

**"Build failed"**: See [Installation](./installation.md) for dependency troubleshooting
