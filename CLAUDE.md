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
make launch_lidar_camera_sample_data
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

#### Working Directory Issues
If Claude Code's working directory becomes invalid or gets lost during a session (showing empty $PWD or directory not found errors):
- This appears to be a session state issue that cannot be recovered within the current session
- Solution: Restart the Claude Code session
- All commands should be run from `/home/jetson/LCTK` (project root)
- If you see bash commands failing with directory errors, check if the working directory is valid with `pwd` before attempting fixes

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
   - `lidar_board_detector`: Detect calibration boards in point clouds
   - `extrinsic_solver`: Solve extrinsic parameters between LiDAR and camera (requires SFCGAL)
   - `multi_wayside`: Handle multi-wayside calibration (requires SFCGAL)
   - `multi_wayside_node`: Multi-wayside calibration ROS node (requires SFCGAL)
   - `pointcloud_image_overlay`: Overlay point clouds on camera images (Python)
     - Automatically derives camera_info topic from image topic following image_pipeline convention
     - Always publishes overlay images when input images are available
     - Displays error messages on output images when extrinsic calibration is missing
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

### Board Detection Pipeline

The calibration board detection in point clouds follows a multi-stage pipeline (`lidar_board_detector/src/main.rs`):

1. **Bounding Box Filtering** (`detect_bbox`): Filters points to a region of interest around the expected board location
2. **RANSAC Plane Detection** (`detect_ransac`): Detects the dominant plane in the filtered point cloud
   - Uses RANSAC algorithm to fit a plane model
   - Identifies inlier points that belong to the plane
3. **ICP Board Pose Refinement** (`detect_icp`): Refines the board pose using Iterative Closest Point
   - **PCA-based Initial Pose**: Computes initial board pose from plane inliers using Principal Component Analysis
     - Performs eigenvalue decomposition of the covariance matrix
     - Extracts orthogonal eigenvectors v1, v2, v3 (ordered by eigenvalue magnitude)
     - Applies orientation constraints: v3 points toward camera, v1 and v2 have positive z components
     - Ensures right-hand rule by swapping v1/v2 if needed (not flipping)
     - Implementation: `compute_initial_pose_pca()` uses nalgebra's `symmetric_eigen()` for eigendecomposition
   - **ICP Iterations**: Refines pose by iteratively matching model points to observed points
   - Uses Kabsch algorithm for rigid body transformation estimation

**Key Implementation Details:**
- PCA provides a robust initial guess for ICP, reducing convergence time
- The PCA implementation is self-contained using nalgebra (no external PCA libraries required)
- Initial board pose is published to debug topics immediately after PCA computation
- ICP iterations use the BoardIcpIterator API for flexible iteration control and debug visualization

### Debug Mode

The calibration system supports debug mode for detailed analysis and troubleshooting:

```bash
# Launch calibration with debug mode enabled
make launch_lidar_camera_calibration debug_mode:=true

# Or launch manually with ROS2
ros2 launch calib_launch lidar_camera_calibration.launch.xml debug_mode:=true
```

When debug mode is enabled:
- **Debug Topics**: Additional point cloud and marker topics are published:
  - `/calibration/debug/all_points`: All input points before filtering
  - `/calibration/debug/filtered_points`: Points within bounding box
  - `/calibration/debug/plane_inliers`: Points detected as part of the calibration plane
  - `/calibration/debug/plane_marker`: Circular plane visualization showing RANSAC-detected plane (centered at inlier centroid, aligned with plane normal)
  - `/calibration/debug/initial_board_marker`: Initial board pose from PCA-based alignment (published immediately after PCA computation)
  - `/calibration/debug/icp_iterations`: Board pose at each ICP iteration
  - `/calibration/debug/final_board_pose`: Final successful detection pose
  - `/calibration/debug/bbox_marker`: Bounding box visualization
  - `/calibration/debug/icp_stats`: ICP iteration statistics (if ICP iteration debug is enabled)
- **Debug Logging**: Detailed algorithm information logged at debug level:
  - RANSAC plane fitting progress and statistics
  - PCA-based pose initialization with eigenvalues and eigenvector orientations
  - ICP algorithm iteration details and convergence
  - Point cloud statistics and processing steps
- **Performance Impact**: Debug mode adds computational overhead and should only be enabled for development/troubleshooting

To view debug topics in RViz or other tools:
```bash
# List available debug topics
ros2 topic list | grep debug

# Echo debug point cloud data
ros2 topic echo /calibration/debug/all_points

# View in RViz
rviz2 -d config/debug_visualization.rviz
```

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

### Topic Naming Conventions

**Camera Info Topic Derivation (image_pipeline convention):**
- ROS 2 nodes automatically derive the `camera_info` topic from the image topic
- Convention: Replace the last component of the image topic with `camera_info`
- Examples:
  - `/sensing/camera/front_center/image_raw` → `/sensing/camera/front_center/camera_info`
  - `/my/camera/image` → `/my/camera/camera_info`
  - `/camera/compressed` → `/camera/camera_info`
- Nodes implementing this: `aruco_locator_node`, `pointcloud_image_overlay`
- No manual remapping needed for camera_info in launch files

Several LCTK tools have been converted to ROS 2 nodes:

1. **aruco_locator_node**: Detects ArUco markers in camera images
   - Subscribes to: `/image` (sensor_msgs/Image)
   - Publishes to: `/aruco_detections` (vision_msgs/Detection2DArray)

2. **lidar_board_detector**: Detects calibration boards in point clouds
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

### Rebuilding a Single ROS 2 Package

To rebuild a single ROS 2 package after making changes:

```bash
# Standard way: remove build and install directories, then rebuild
rm -rf {build,install}/PACKAGE_NAME
make build_packages
```

This ensures a clean rebuild and proper installation of the package.

### Running ROS 2 Nodes

```bash
# Run ArUco locator node
ros2 run aruco_locator_node aruco_locator_node --intrinsics-file src/ros2/lctk_launch/config/camera/front_center_camera_info.yaml

# Run calibration board locator node
ros2 run lidar_board_detector lidar_board_detector

# Run extrinsic solver node
ros2 run extrinsic_solver extrinsic_solver --intrinsics-file src/ros2/lctk_launch/config/camera/front_center_camera_info.yaml

# Run PCD tool node
ros2 run pcd_tool pcd_tool_ros

# Launch all nodes
ros2 launch lctk_ros2 lctk_nodes.launch.py
```

## DDS Configuration

The project includes a Cyclone DDS configuration file to prevent DDS packets from being sent to external networks:

- **Configuration File**: `config/dds/cyclone_local.xml`
- **Purpose**: Restricts DDS communication to localhost only
- **Features**:
  - Localhost-only network interfaces (127.0.0.1)
  - Multicast disabled for security
  - TCP transport for reliable local communication
  - No packet routing outside local network

### Usage

The DDS configuration is automatically applied by all Makefile launch targets:

```bash
make launch_lidar_camera_calibration  # Uses local DDS config
make launch_lidar_camera_sample_data  # Uses local DDS config
make launch_two_lidar_calibration     # Uses local DDS config
```

For manual usage:
```bash
export CYCLONE_DDS_URI=file://$PWD/config/dds/cyclone_local.xml
ros2 launch your_launch_file.xml
```

This ensures that ROS 2 DDS traffic remains within the local machine and doesn't leak to external networks.

## Sample Data

The repository includes sample data for testing calibration workflows:
- `data/sampledata/3/`: Contains LiDAR pcap and video.avi files
- `data/sampledata/4/`: Contains additional LiDAR pcap files

Run sample data playback:
```bash
make launch_lidar_camera_sample_data  # Plays LiDAR and camera data in loop
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
- **DEPRECATED**: Fixed colcon-cargo JSON parsing issue by modifying /home/aeon/.local/lib/python3.10/site-packages/colcon_cargo/task/cargo/build.py to use direct subprocess calls with --quiet flag. This resolves "JSONDecodeError: Expecting value: line 1 column 1" errors caused by patch warnings in cargo metadata output. (Note: We now use colcon-cargo-ros2 instead of colcon-cargo + colcon-ros-cargo)
- **Colcon Rust Integration**: Migrated from colcon-cargo + colcon-ros-cargo to colcon-cargo-ros2 for better ROS 2 integration and automatic binding generation. The old packages conflict with colcon-cargo-ros2 and must be uninstalled. The Ansible setup script now checks for these conflicts and installs colcon-cargo-ros2 via `pip install --user colcon-cargo-ros2`.
- OpenCV environment variables are set automatically to avoid version 0.0.0 issues.
- Dependencies are now managed through Ansible playbooks in an Autoware-style setup (setup-dev-env.sh)
- Git ignores build artifacts: ansible_collections/, build/, install/, log/, build_logs/, .cargo/, ros2_rust_ws/{build,install,log}/
- **Config File Parameters**: All ROS2 nodes now require mandatory config file parameters - no hardcoded defaults:
  - `aruco_locator_node`: Requires `aruco_config_file` parameter (no default path)
  - `lidar_board_detector`: Requires `board_detector_file`, `aruco_pattern_file`, and `bbox_file` parameters
  - Config files are passed from launch files to ensure explicit configuration
- **GStreamer Video Playback**: The camera.launch.xml now uses `filesrc location=$(var video_file) ! decodebin ! videoconvert` instead of test patterns
- **ROS2 Daemon Issues**: If ROS2 daemon becomes unresponsive, kill it with: `pkill -9 -f ros2-daemon`
- **Colcon Build Flags Order**: When using `colcon build` with `--packages-select`, this flag MUST come BEFORE `--cmake-args` and `--cargo-args`. Otherwise it will be treated as part of CMake or Cargo flags. Correct order:
  ```bash
  colcon build --packages-select <pkg_name> --cmake-args <args> --cargo-args <args>  # CORRECT
  colcon build --cmake-args <args> --packages-select <pkg_name>  # WRONG - treated as CMake flag
  ```
- **ROS Interface Package Resolution**: If you see errors like `error: failed to select a version for the requirement 'builtin_interfaces = "*"` with "version X.X.X is yanked" from crates.io, it means Cargo is incorrectly searching crates.io instead of using local ROS interface packages. This is controlled by `.cargo/config.toml` which is generated by rclrs. The local packages should be resolved via the cargo configuration, not from crates.io.
- **Building ROS2 Packages - IMPORTANT**:
  - **ALWAYS use `make build_packages`** to rebuild ROS2 packages in `./src/ros2/`
  - **NEVER use `colcon build --packages-select <pkg>`** for individual packages - this can break `.cargo/config.toml` and cause Cargo to fail finding interface packages
  - **For interface packages in `./src/interface/`**, use `make build_interface` instead
  - The Makefile targets properly maintain the cargo configuration for local ROS interface resolution
- **Workspace Dependencies**: ROS2 Rust dependencies use workspace.dependencies in root Cargo.toml, not patch.crates-io
- **rclrs API Migration**: Updated from v0.4.x to v0.5.x - executor.spin() now returns Vec<RclrsError> instead of Vec<Result<_, RclrsError>>
- **Detection Synchronizer Subscription Lifecycle**: Fixed critical issue where subscription objects were being dropped immediately after creation in detection_synchronizer. ROS2 subscriptions must be stored as struct fields to keep them alive:
  ```rust
  pub struct SynchronizerNode {
      _state: Arc<SynchronizerState>,
      _node: Node,
      _aruco_subscription: Subscription<Detection2DArray>,  // Store subscriptions
      _board_subscription: Subscription<Detection3DArray>, // as struct members
  }
  ```
- **Detection Synchronizer Configuration**: Optimized synchronization parameters for calibration workflows:
  - Window size: 500ms (increased from 50ms) to handle detection processing delays
  - Buffer size: 200 (increased from 50) for more robust buffering
  - Quality threshold: 50 (reduced from 128) to be more permissive with timestamp differences
  - Allow empty detections through for synchronization (calibration targets may not always be visible)
- **Calibration Pipeline Architecture**: Verified end-to-end detection synchronization flow:
  - Raw detections → Detection Synchronizer → Synchronized detections → Extrinsic Solver
  - Extrinsic solver properly remapped to consume synchronized topics instead of raw detection topics
  - Detection synchronizer shows active subscriptions and publishes to synchronized topics
- **Board Detector Performance**: Fixed ICP algorithm performance by reducing max_iterations from 20,000 to 100 in board_detector.json5, preventing minutes-long processing delays
- **Camera Info Topic Derivation**: ROS 2 nodes (aruco_locator_node, pointcloud_image_overlay) automatically derive camera_info topics from image topics following image_pipeline convention (e.g., /camera/image_raw → /camera/camera_info). No manual camera_info remapping needed in launch files
- **Board Detection Pipeline Refactoring**: The board detection process has been separated into distinct stages for better debugging and modularity:
  - `detect_bbox()`: Bounding box filtering
  - `detect_ransac()`: RANSAC plane detection
  - `detect_icp()`: ICP pose refinement with PCA-based initialization
  - Each stage has dedicated debug visualization topics
- **PCA-based Pose Initialization**: Implemented custom PCA using nalgebra's `symmetric_eigen()` for computing initial board pose from RANSAC plane inliers:
  - Computes covariance matrix and performs eigenvalue decomposition
  - Applies orientation constraints: v3 (smallest eigenvalue) points toward camera, v1 and v2 have positive z
  - Right-hand rule maintained by swapping v1/v2 (not flipping) when cross product check fails
  - Initial pose published immediately after PCA computation for debugging visibility
  - Located in `lidar_board_detector/src/main.rs::compute_initial_pose_pca()`
- **Debug Visualization Enhancements**:
  - Added `debug/plane_marker` topic showing circular RANSAC plane (semi-transparent blue disk centered at inlier centroid, aligned with plane normal)
  - Fixed `debug/initial_board_marker` to publish immediately after PCA computation instead of only on successful detection
  - All debug topics now have consistent marker lifetimes and visual styling
- **ArUco Locator Camera Info Synchronization**: The aruco_locator_node now waits for the first camera_info message before processing images, ensuring camera calibration data is available for image undistortion. The node continues to update its calibration parameters when new camera_info messages arrive.
- **Arc-Swap Implementation in lidar_board_detector**: Implemented lock-free RCU semantics using arc-swap crate (Sep 29, 2025):
  - Replaced `Arc<Mutex<BBox>>` with `Arc<ArcSwap<BBox>>` to eliminate mutex contention
  - Filter thread performs lock-free reads with `bbox.load()` for zero-wait access
  - Service handlers use atomic updates with `bbox.store()` for instant writes
  - Performance: <10ms service response times confirmed, no thread starvation
  - Files modified: `src/ros2/lidar_board_detector/Cargo.toml`, `src/main.rs`, `src/services.rs`
- **ROS2 DDS Discovery Timing Issue**: Discovered critical timing pattern with CycloneDDS localhost configuration (Sep 29, 2025):
  - **Pattern**: Services work immediately after launch but timeout after ~5s delay
  - **Root Cause**: CycloneDDS discovery lease renewal gap in localhost-only configuration
  - **Window**: 0-2s (active discovery) → 2-5s (stabilization) → 5-10s (lease renewal gap) → 10s+ (stable)
  - **Solution**: Use `ros2 service wait /service/name --timeout 30` before service calls
  - **Note**: This is a DDS configuration issue, NOT related to arc-swap implementation
- **KNOWN ISSUE - 45° Tilt in Pointcloud Overlay**: Ongoing calibration bug (Oct 2, 2025):
  - **Symptom**: Both board marker points and full pointcloud appear tilted ~45° clockwise in camera overlay image, despite 3D points being correct in RViz
  - **Confirmed Root Cause**: Corner ordering mismatch between 2D ArUco corners and 3D board model corners in extrinsic solver
  - **Location**: `src/ros2/extrinsic_solver_node/extrinsic_solver_node/main.py` in `_compute_multi_marker_corners()` (lines 501-555)
  - **Investigation Findings**:
    - Tested shift=1 (np.roll by -1): Reversed tilt to counter-clockwise direction
    - Tested shift=2, shift=3: Scattered/incorrect points
    - Tested swapping corners (0,2) and (1,3): Scattered/incorrect points
    - Tested swapping "left" and "right" marker positions in grid assignments (lines 541-544): No improvement
    - The Python implementation's `make_corners()` function matches Rust's `multi_marker_corners()` in ordering: `[right, top, left, bottom]`
    - ArUco detector outputs corners in OpenCV standard order: `[top-left, top-right, bottom-right, bottom-left]`
  - **Still Unknown**: The exact mapping between ArUco corner indices and board model corner indices that would fix the tilt
  - **Next Steps**: Need to verify the actual corner ordering from ArUco detector output vs. the expected ordering for board model corners. May need to examine wayside-portal's working implementation more carefully for subtle differences in how corners are paired.
- **Temporary Files and Scripts**: When creating temporary files or scripts during development (Oct 28, 2025):
  - **ALWAYS use Write/Edit tools** instead of bash heredocs (e.g., `cat > file << 'EOF'`)
  - **Create temp files in `$PROJECT_ROOT/tmp/`** (e.g., `/home/aeon/repos/LCTK/tmp/`)
  - Example: Scripts for validation, testing, or one-off tasks should be written to `tmp/` directory
  - Benefits: Better visibility of temp files, easier cleanup, can be version controlled if needed
  - The `tmp/` directory is gitignored by default

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
- Whenever you run a command requiring root privilege (such as sudo), stop and show the command to user so that user can run the command in another terminal.
- To rebuild a ROS2 package in Rust, use `colcon build --packages-select PKG_NAME` to rebuild that package. Don't use `cargo build` because it does not install the compiled binary to the install/ directory.
- **ALWAYS run colcon build from the project root directory**, never from subdirectories like `src/ros2/`. Running colcon build from subdirectories creates build/install directories in the wrong location (e.g., `src/ros2/{build,install}`) which causes issues when colcon/cargo runs from the project root and searches for packages in the wrong paths.
- Functional struct initialization for large structs in Rust.
- If you compile a package using colcon build --package-select or --packages-up-to, respect the COLCON_BUILD_FLAGS in Makefile.
