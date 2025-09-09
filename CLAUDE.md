# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LCTK (LiDAR and Camera Toolkit) is a set of libraries and tools for calibrating LiDAR and camera systems. It's primarily implemented in Rust with some shell scripts for workflow automation.

## Setup and Build

### Quick Start

```bash
# Set up development environment (interactive)
make prepare

# Or use the setup script directly with options:
./setup-dev-env.sh -y              # Non-interactive installation
./setup-dev-env.sh -y --no-cuda    # Skip CUDA installation
./setup-dev-env.sh -y --minimal    # Minimal installation (no CUDA, no dev tools)

# Build the project
make build

# Test with sample data
make launch_sensor
```

### Known Issues and Solutions

#### empy Version Compatibility
ROS 2 Humble requires empy version 3.3.4, which is provided by Ubuntu 22.04's python3-empy package. If you encounter errors like:
- `AttributeError: module 'string' has no attribute 'split'`
- Build failures in ros2_rust_ws with empy-related errors

This typically means a newer incompatible version (4.x) was installed via pip. Fix by removing pip-installed empy and using the system package:
```bash
# Remove any pip-installed empy versions
pip3 uninstall empy

# Ensure the system package is installed (already included in ros-humble-desktop)
sudo apt-get install python3-empy
```

The setup-dev-env.sh script handles this automatically by ensuring the system package is used.

### Build Commands

The project uses a three-pass build process:

```bash
# Build the whole project (runs all three passes)
make build

# Individual build passes:
# 1. Build rclrs and common interface types for Rust
make build_ros2_rust

# 2. Build interface types in this project (after building ROS for Rust)
make build_interface

# 3. Build the rest of the project
make build_packages

# To build a single crate:
make build_interface  # Run at least once
source install/setup.bash
cargo build --release --manifest-path src/bin/aruco_locator_node/Cargo.toml

# Clean build artifacts
make clean

# Launch calibration pipelines
make launch_lidar_camera_calibration
make launch_two_lidar_calibration
```

## Project Structure

The project is organized into:

1. **Rust Libraries** (`src/lib/`): Core components with reusable functionality
   - `aruco-config`: Serializable types for ArUco pattern description
   - `aruco-detector`: Detection of ArUco markers in images
   - `aruco-generator`: Generate ArUco board images (library)
   - `hollow-board-config`: Serializable types for hollow-board shapes
   - `hollow-board-detector`: Detection of hollow-boards in point clouds
   - `plane-estimator`: Plane fitting algorithms for point clouds
   - `pnp-solver`: OpenCV wrapper for PnP (Perspective-n-Point) solving
   - `serde-types`: Common serializable types used across the project

2. **ROS 2 Nodes** (`src/bin/`): ROS 2 nodes and command-line tools
   - `aruco_generator_node`: Generate ArUco board images
   - `aruco_locator_node`: Detect ArUco markers in images
   - `calibration_board_locator`: Detect calibration boards in point clouds
   - `extrinsic_solver`: Solve extrinsic parameters between LiDAR and camera (requires SFCGAL)
   - `multi_wayside`: Handle multi-wayside calibration (requires SFCGAL)
   - `multi_wayside_node`: Multi-wayside calibration ROS node (requires SFCGAL)
   - `pointcloud_image_overlay`: Overlay point clouds on camera images
   - `synchronizer`: Synchronize multiple data streams
   - `rosbag_deck`: ROS bag playback and recording tools

3. **Scripts** (`scripts/`): Automation scripts for calibration workflows
   - `lidar-to-camera-calibration/`: Scripts for LiDAR to camera calibration
   - `lidar-to-lidar-calibration/`: Scripts for LiDAR to LiDAR calibration
   - `record-data/`: Scripts for data recording

4. **Ansible** (`ansible/`): Infrastructure-as-code for environment setup
   - `playbooks/`: Ansible playbooks for setup automation
   - `roles/`: Modular Ansible roles for each component
   - Self-contained configuration and requirements

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

The project requires Ubuntu 22.04 LTS and the following dependencies:

### Core Dependencies (installed by setup-dev-env.sh)
- ROS 2 Humble
- Rust toolchain (stable and nightly)
- OpenCV 4.5.4 or later
- GStreamer 1.20+ with plugins (base, good, bad, ugly, libav)
- C++ development headers (libstdc++-12-dev, libclang-dev, llvm-dev)
- SFCGAL library for geometric computations
- libpcap for network packet capture
- Python 3.10 with pip, venv, numpy, scipy

### Optional Dependencies
- CUDA 11.8 toolkit (for GPU acceleration)
- Development tools (gdb, valgrind, lcov, etc.)
- Poetry for Python package management

All dependencies are managed through Ansible playbooks. Run `make prepare` or `./setup-dev-env.sh` to install everything automatically.

The environment setup script is in `setup/setup-env.sh`.

### Known Build Issues and Solutions

1. **OpenCV version 0.0.0 issue**: The Makefile now automatically sets `OPENCV_PKGCONFIG_NAME=opencv4` to use the system OpenCV installation instead of the non-existent `/opt/opencv4.6.0` path.

2. **Ansible configuration errors**:
   - The ansible.builtin collection is built-in and shouldn't be in ansible-galaxy-requirements.yaml
   - Ansible needs the ANSIBLE_CONFIG environment variable set to find the correct roles path
   - The setup script now exports ANSIBLE_CONFIG to ensure roles are found
   - When Ansible is installed via pipx, pip module needs `executable: /usr/bin/pip3` to install user packages

3. **OpenCV binding generation failure**: If you see errors like "fatal error: 'memory' file not found" when building opencv-rust crates:
   - Install C++ development headers: `sudo apt-get install libstdc++-12-dev libclang-dev`
   - The Makefile already sets the correct OpenCV environment variables

4. **SFCGAL library missing**: If packages like `extrinsic_solver`, `multi_wayside`, or `multi_wayside_node` fail with "SFCGAL/capi/sfcgal_c.h: No such file or directory":
   - Install SFCGAL: `sudo apt-get install libsfcgal-dev`
   - Or exclude these packages from the build if not needed

5. **Colcon build aborts**: When one package fails in colcon build, all subsequent packages are aborted. To build other packages:
   - Fix the failing package's dependencies first, or
   - Build packages individually using cargo with their manifest paths

6. **gscam node crashes**: If the gscam node fails to play video files:
   - Install required GStreamer plugins: `sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav`
   - The camera.launch.xml file has been updated to use a simple decodebin pipeline that auto-detects video formats
   - If you get "undefined symbol: av_timecode_make_smpte_tc_string2" error:
     * This is a libav/ffmpeg version mismatch issue
     * Try: `sudo apt-get install --reinstall gstreamer1.0-libav libavutil56 libavfilter7`
     * Or use the test pattern launch file: `ros2 launch calib_launch camera_test.launch.xml`
     * Or if you have NVIDIA GPU, modify the pipeline to use nvcodec decoders

7. **rosdep missing dependencies**: The `libpcap` dependency is correctly specified in package.xml files (not `libpcap-dev`)

8. **rosdep initialization errors**:
   - `rosdep init` must be run as root (creates /etc/ros/rosdep/sources.list.d/20-default.list)
   - `rosdep update` must be run as a regular user (updates ~/.ros/rosdep/sources.cache)
   - The Ansible playbooks now check if rosdep is already initialized before attempting to run init

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

## Sample Data

The repository includes sample data for testing calibration workflows:
- `data/sampledata/3/`: Contains LiDAR pcap and video.avi files
- `data/sampledata/4/`: Contains additional LiDAR pcap files

Run sample data playback:
```bash
make launch_sensor  # Plays LiDAR and camera data in loop
```

## Development Memories

- The rclrs source is located at ros2_rust_ws/src/ros2_rust/rclrs
- ROS 2 Rust packages are in ros2_rust_ws/
- Please use named parameters in format string. For example, use println!("{e}") instead of println!("{}", e).
- Use cmake_minimum_required(VERSION 3.10) in CMakeLists.txt in ROS packages.
- CameraIntrinsics has been replaced with ROS sensor_msgs::msg::CameraInfo in the codebase
- Don't make Pokemon exception handlings. For example, `try: except Exception: pass`. It creates silent errors. I prefer to throw errors to the user so developers can fix it.
- If `source /opt/ros/humble/setup.bash` was done earlier and we would like to test Rust code only without ROS, you can run `cargo clippy --all-targets --all-features`.
- In Rust, initialize struct fields first and then construct the struct. It avoids creating a mutable struct.
- When tasks are completed, notify GNU Screen with a bell: `printf '\a'; echo "[Task Complete] <task description>"`
- Fixed colcon-cargo JSON parsing issue by modifying /home/aeon/.local/lib/python3.10/site-packages/colcon_cargo/task/cargo/build.py to use direct subprocess calls with --quiet flag. This resolves "JSONDecodeError: Expecting value: line 1 column 1" errors caused by patch warnings in cargo metadata output.
- OpenCV environment variables are set automatically in the Makefile to avoid version 0.0.0 issues.
- Dependencies are now managed through Ansible playbooks in an Autoware-style setup (setup-dev-env.sh)
- Git ignores build artifacts: ansible_collections/, build/, install/, log/, build_logs/, .cargo/, ros2_rust_ws/{build,install,log}/

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