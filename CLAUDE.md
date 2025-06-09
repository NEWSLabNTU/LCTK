# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LCTK (LiDAR and Camera Toolkit) is a set of libraries and tools for calibrating LiDAR and camera systems. It's primarily implemented in Rust with some shell scripts for workflow automation.

## Build Commands

The project uses a three-pass build process:

```bash
# Build the whole project (runs all three passes)
make build

# Individual build passes:
# 1. Build rclrs and common interface types for Rust
make build_ros2_rust

# 2. Build interface types in this project (after building ROS for Rust)
make build_interface

# 3. Build the rest of the project (handled by make build)

# To build a single crate:
make build_interface  # Run at least once
source install/setup.bash
cargo build --release --manifest-path src/bin/aruco_locator_node/Cargo.toml

# Clean build artifacts
make clean
# or
cargo clean

# Setup environment variables (run before build/development)
source setup/setup-env.sh
```

## Project Structure

The project is organized into:

1. **Rust Libraries** (`rust-lib/`): Core components with reusable functionality
   - `aruco-config`: Serializable types for ArUco pattern description
   - `aruco-detector`: Detection of ArUco markers in images
   - `hollow-board-config`: Serializable types for hollow-board shapes
   - `hollow-board-detector`: Detection of hollow-boards in point clouds
   - `plane-estimator`: Plane fitting algorithms for point clouds
   - `pnp-solver`: OpenCV wrapper for PnP (Perspective-n-Point) solving
   - `serde-types`: Common serializable types used across the project

2. **Rust Binaries** (`rust-bin/`): Command-line tools
   - `aruco-generator`: Generate ArUco board images
   - `find-aruco-marker`: Detect ArUco markers in images
   - `find-hollow-board`: Detect hollow boards in point clouds
   - `pcd-tool`: Process point cloud data
   - `project-to-image`: Project 3D points to camera images
   - `extrinsic_solver`: Solve extrinsic parameters between LiDAR and camera
   - `multi_wayside`: Handle multi-wayside calibration

3. **Scripts** (`scripts/`): Automation scripts for calibration workflows
   - `lidar-to-camera-calibration/`: Scripts for LiDAR to camera calibration
   - `lidar-to-lidar-calibration/`: Scripts for LiDAR to LiDAR calibration
   - `record-data/`: Scripts for data recording

## Calibration Workflow

The LiDAR-to-camera calibration workflow follows these steps:

1. Record data from LiDAR (pcap files) and cameras (mp4 files)
2. Convert pcap to pcd format (point cloud data)
3. Extract video frames to jpg images
4. Detect hollow boards in point clouds
5. Detect ArUco markers in images
6. Solve extrinsic parameters between LiDAR and camera

The main script for this workflow is in `scripts/lidar-to-camera-calibration/lidar_to_camera.sh`.

## Running Specific Tools

To run a specific tool, use cargo with the proper manifest path:

```bash
# Convert pcap to pcd
cargo run --release --manifest-path "rust-bin/pcd-tool/Cargo.toml" -- convert <input_pcap> <output_dir> <start_frame> <num_frames>

# Find hollow boards in point clouds
cargo run --release --manifest-path "rust-bin/find-hollow-board/Cargo.toml" -- --preview <input_pcd_dir> <output_dir>

# Find ArUco markers in images
cargo run --release --manifest-path "rust-bin/find-aruco-marker/Cargo.toml" -- --gui <intrinsics_yaml> <input_video> <output_dir>

# Solve extrinsic parameters
cargo run --release --manifest-path "rust-bin/extrinsic_solver/Cargo.toml" -- --method SQPNP --intrinsics-file <intrinsics_file> --output-file <output_file> --boards <board_files> --arucos <aruco_files>
```

## Data Recording

The toolkit includes utilities for data recording across multiple sensor devices:

1. Configure the connection session in `exp/record-data/config/session/`
2. Launch connections with `exp/record-data/launch.sh`
3. Configure recording recipes in `exp/record-data/config/recipes/`
4. Start recording with `exp/record-data/delegate-recipe.sh [recipe_file]`
5. Sync data with `exp/record-data/sync.sh`

## Dependencies

The project requires:
- Rust toolchain
- OpenCV 4.6.0
- CUDA 11.3 (optional)
- Python 3.8 (optional)
- ROS 2 Humble or later (for ROS 2 nodes)

The environment setup script is in `setup/setup-env.sh`.

## ROS 2 Integration

Several LCTK tools have been converted to ROS 2 nodes:

1. **aruco_locator_node**: Detects ArUco markers in camera images
   - Subscribes to: `/image` (sensor_msgs/Image)
   - Publishes to: `/aruco_detections` (vision_msgs/Detection2DArray)

2. **calibration_board_locator**: Detects calibration boards in point clouds
   - Subscribes to: `/input_pointcloud` (sensor_msgs/PointCloud2)
   - Publishes to: `/calibration_board_detections` (vision_msgs/Detection3DArray)

3. **extrinsic_solver**: Solves extrinsic parameters between LiDAR and camera
   - Subscribes to: `/aruco_detections` (vision_msgs/Detection2DArray)
   - Subscribes to: `/calibration_board_detections` (vision_msgs/Detection3DArray)
   - Publishes to: `/extrinsic_transform` (geometry_msgs/TransformStamped)

4. **pcd_tool**: Processes and converts point cloud data
   - Subscribes to: `/input_pointcloud` (sensor_msgs/PointCloud2)
   - Publishes to: `/converted_pointcloud` (sensor_msgs/PointCloud2)

### Building ROS 2 Nodes

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build interface types (includes rclrs and ROS 2 message types)
make build_interface

# Source the workspace
source install/setup.bash

# Build specific ROS 2 nodes
cargo build --release --manifest-path src/bin/aruco_locator_node/Cargo.toml

# Or build everything at once
make build
```

### Running ROS 2 Nodes

```bash
# Run ArUco locator node
ros2 run aruco_locator_node aruco_locator_node --intrinsics-file config/intrinsics.yaml

# Run calibration board locator node
ros2 run calibration_board_locator calibration_board_locator

# Run extrinsic solver node
ros2 run extrinsic_solver extrinsic_solver --intrinsics-file config/intrinsics.yaml

# Run PCD tool node
ros2 run pcd_tool pcd_tool_ros

# Launch all nodes
ros2 launch lctk_ros2 lctk_nodes.launch.py
```

## Development Memories

- The rclrs source is located at ros2_rust_ws/src/ros2_rust/rclrs
- ROS 2 Rust packages are in ros2_rust_ws/
- Please use named parameters in format string. For example, use println!("{e}") instead of println!("{}", e).
- Use cmake_minimum_required(VERSION 3.10) in CMakeLists.txt in ROS packages.
- CameraIntrinsics has been replaced with ROS sensor_msgs::msg::CameraInfo in the codebase
- Don't make Pokemon exception handlings. For example, `try: except Exception: pass`. It creates silent errors. I prefer to throw errors to the user so developers can fix it.

## Coding Style

- When creating closures that capture variables, prefer to clone variables in a local scope and move them to the closure. For example:
  ```rust
  // Preferred style
  let subscription = {
      let state = Arc::clone(&state);
      let publisher = Arc::clone(&publisher);
      
      node.create_subscription::<MessageType, _>(
          "topic_name",
          move |msg| {
              callback(msg, &state, &publisher);
          },
      )?
  };
  
  // Instead of
  let state_clone = Arc::clone(&state);
  let publisher_clone = Arc::clone(&publisher);
  let subscription = node.create_subscription::<MessageType, _>(
      "topic_name", 
      move |msg| {
          callback(msg, &state_clone, &publisher_clone);
      },
  )?;
  ```