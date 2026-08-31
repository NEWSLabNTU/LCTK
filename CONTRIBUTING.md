# Contributing to LCTK

This document provides detailed information about the LCTK codebase structure, development guidelines, and how to contribute to the project.

## Project Structure

LCTK is organized into three main categories: pure Rust libraries, ROS 2 nodes, and launch/config
packages. The layout below is current; an older `src/lib/` + `src/bin/` + `src/ros2/` layout
predates a directory reorg to today's `rust/` + `ros/`, and any lingering reference to that old
layout is stale.

### Libraries (`rust/`)

Core reusable functionality implemented as pure Rust libraries (no ROS dependency):

- **[aruco-config](rust/aruco-config/)** - Serializable types for ArUco pattern descriptions
- **[aruco-detector](rust/aruco-detector/)** - ArUco marker detection algorithms
- **[aruco-generator](rust/aruco-generator/)** - Generate ArUco board images (library)
- **[aruco-locator](rust/aruco-locator/)** - Locating ArUco markers in images
- **[board-cluster-detector](rust/board-cluster-detector/)** - Isolates a calibration board's
  point cluster from a point cloud without a hand-tuned crop box (`bbox_free` detection)
- **[calibration-target](rust/calibration-target/)** - Validated, immutable physical
  definitions of calibration targets (Target Definition parsing, validation, semantic identity,
  and canonical board-local geometry). Supersedes the former `hollow-board-config`
- **[calibration-target-detector](rust/calibration-target-detector/)** - Calibration-target
  pose estimation (`TargetPoseEstimator`, plus the solid- and perforated-surface adapters).
  Supersedes the former `hollow-board-detector`
- **[plane-estimator](rust/plane-estimator/)** - Plane fitting algorithms for point clouds

### ROS 2 Nodes and Packages (`ros/`)

#### Rust ROS 2 Nodes
- **[aruco_generator_node](ros/aruco_generator_node/)** - Prints the ArUco board pattern from
  a Target Definition
- **[aruco_locator_node](ros/aruco_locator_node/)** - Detect ArUco markers in images
- **[lidar_board_detector](ros/lidar_board_detector/)** - Detect calibration boards in point
  clouds

#### Python ROS 2 Nodes
- **[extrinsic_solver_node](ros/extrinsic_solver_node/)** - Superseded LiDAR-camera solver;
  unreachable from config-driven launch, pending deletion
- **[lidar_to_camera_solver](ros/lidar_to_camera_solver/)** - LiDAR-camera solver
  (continuous and manual modes)
- **[interactive_solver_controller](ros/interactive_solver_controller/)** - Rich TUI driving
  `lidar_to_camera_solver`
- **[lidar_to_lidar_solver](ros/lidar_to_lidar_solver/)** - LiDAR-to-LiDAR calibration solver
- **[lctk_quality](ros/lctk_quality/)** / **[calibration_judge](ros/calibration_judge/)** -
  Extrinsic quality metric
- **[pointcloud_image_overlay](ros/pointcloud_image_overlay/)** - Overlay point clouds on
  camera images
- **[filter_box_tuner](ros/filter_box_tuner/)** - Interactive crop-box tuning for the board
  detector
- **[lctk_autoware_export](ros/lctk_autoware_export/)** - Exports a solved extrinsic into an
  Autoware `sensor_kit_calibration.yaml`
- **[lctk_sync](ros/lctk_sync/)** - Owns Conflux-backed detection-pair synchronization used by
  the solver nodes
- **[lctk_target](ros/lctk_target/)** - Python-side Target Definition loading

#### Launch Files and Configuration
- **[lctk_launch](ros/lctk_launch/)** - Config-driven launch system for calibration pipelines
- **Configuration files** - Located in `ros/lctk_launch/config/`

#### Interface Packages
- **[lctk_interfaces](ros/lctk_interfaces/)** - Custom ROS 2 message/service definitions
  (solver services, quality report)

#### Data and External Integration
- **[lctk_sample_data](ros/lctk_sample_data/)** - Sample data playback (pcap + avi)
- **[conflux](ros/conflux/)** - Git submodule: message synchronizer used by all solvers

## ROS 2 Node Details

### Core Calibration Nodes

#### ArUco Locator Node
- **Package**: `aruco_locator_node`
- **Node Name**: `/calibration/aruco_locator/aruco_locator`
- **Subscribes**:
  - `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
  - `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)
- **Publishes**:
  - `/calibration/aruco_locator/aruco_detections` (vision_msgs/Detection2DArray)
  - `/calibration/aruco_locator/image_with_detections` (sensor_msgs/Image)

#### Calibration Board Locator Node
- **Package**: `lidar_board_detector`
- **Node Name**: `/calibration/lidar_board_detector/lidar_board_detector`
- **Subscribes**:
  - `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)
- **Publishes**:
  - `/calibration/lidar_board_detector/calibration_board_detections` (vision_msgs/Detection3DArray)
- **Debug Topics** (when `debug_mode=true`):
  - `/calibration/lidar_board_detector/debug/all_points` (sensor_msgs/PointCloud2)
  - `/calibration/lidar_board_detector/debug/filtered_points` (sensor_msgs/PointCloud2)
  - `/calibration/lidar_board_detector/debug/plane_inliers` (sensor_msgs/PointCloud2)
  - `/calibration/lidar_board_detector/debug/bbox_marker` (visualization_msgs/MarkerArray)
  - `/calibration/lidar_board_detector/debug/final_board_pose` (visualization_msgs/MarkerArray)
  - `/calibration/lidar_board_detector/debug/icp_iterations` (visualization_msgs/MarkerArray)
  - `/calibration/lidar_board_detector/debug/initial_board_marker` (visualization_msgs/MarkerArray)
  - `/calibration/lidar_board_detector/debug/icp_stats` (std_msgs/String)

#### Extrinsic Solver Node
- **Package**: `extrinsic_solver_node`
- **Node Name**: `/calibration/extrinsic_solver/extrinsic_solver_node`
- **Subscribes**:
  - `/calibration/aruco_locator/aruco_detections` (vision_msgs/Detection2DArray)
  - `/calibration/lidar_board_detector/calibration_board_detections` (vision_msgs/Detection3DArray)
  - `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)
- **Publishes**:
  - `/calibration/extrinsic_solver/extrinsic_transform` (geometry_msgs/TransformStamped)
  - `/calibration/extrinsic_solver/calibration_quality` (std_msgs/String)
  - `/calibration/extrinsic_solver/image_with_detections` (sensor_msgs/Image)
- **Debug Topics**:
  - `/calibration/extrinsic_solver/debug/recent_aruco_detections` (vision_msgs/Detection2DArray)
  - `/calibration/extrinsic_solver/debug/recent_board_detections` (vision_msgs/Detection3DArray)

### Sensor Input Nodes

#### Velodyne Driver Node
- **Package**: `velodyne_driver` (external)
- **Node Name**: `/velodyne_driver_node`
- **Publishes**:
  - `/sensing/lidar/top/velodyne_packets` (velodyne_msgs/VelodyneScan)

#### Velodyne Transform Node
- **Package**: `velodyne_pointcloud` (external)
- **Node Name**: `/velodyne_transform_node`
- **Subscribes**:
  - `/sensing/lidar/top/velodyne_packets` (velodyne_msgs/VelodyneScan)
- **Publishes**:
  - `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)

#### Camera Driver Node (GSCam)
- **Package**: `gscam` (external)
- **Node Name**: `/sensing/camera/front_center/camera_driver`
- **Publishes**:
  - `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
  - `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)

### Visualization Nodes

#### Pointcloud Image Overlay
- **Package**: `pointcloud_image_overlay`
- **Node Name**: `/calibration/pointcloud_image_overlay`
- **Subscribes**:
  - `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)
  - `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
  - `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)
  - `/calibration/extrinsic_solver/extrinsic_transform` (geometry_msgs/TransformStamped)
- **Publishes**:
  - `/calibration/pointcloud_overlay` (Rerun visualization)

## Topic Naming Convention

LCTK follows a hierarchical topic naming convention:

### Sensor Data
- `/sensing/lidar/top/` - Top-mounted LiDAR sensor
- `/sensing/camera/front_center/` - Front center camera

### Calibration Pipeline
- `/calibration/aruco_locator/` - ArUco detection results
- `/calibration/lidar_board_detector/` - Board detection results
- `/calibration/extrinsic_solver/` - Calibration computation results

### Debug Topics
- `/calibration/<node_name>/debug/` - Debug information for specific nodes

## Development Guidelines

### Code Style

#### Rust Code
- Follow standard Rust formatting (`cargo fmt`)
- Use named parameters in format strings: `println!("{e}")` instead of `println!("{}", e)`
- Initialize struct fields first, then construct the struct to avoid mutable structs
- Use explicit error handling - avoid "Pokemon exception handling"

#### ROS 2 Integration
- Use `cmake_minimum_required(VERSION 3.10)` in CMakeLists.txt
- CameraIntrinsics has been replaced with ROS `sensor_msgs::msg::CameraInfo`
- Config file parameters are mandatory - no hardcoded defaults

#### Memory Management
- When creating closures that capture variables, prefer cloning variables in local scope:
  ```rust
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
  ```

### Build System

The project uses a three-pass build process:

1. **Build ROS 2 Rust packages**: `make build_ros2_rust`
2. **Build interface types**: `make build_interface`
3. **Build LCTK packages**: `make build_packages`

### Configuration Management

- Configuration files use JSON5 format for readability
- Target Definitions (physical plate geometry, cutouts, fiducial layout):
  `ros/lctk_launch/config/targets/`
- Detector Tuning presets (sensor-specific, geometry-free, per target):
  `ros/lctk_launch/config/board/<target>/`, e.g. `hollow_1000/velodyne.json5`
- ArUco detector tuning (corner refinement, adaptive threshold):
  `ros/lctk_launch/config/aruco/`
- Calibration is config-driven (`ros2 launch lctk_launch calibrate.launch.py
  config_file:=<yaml>`); XML launch arguments for these files no longer exist

### Testing

- Use `make launch_lidar_camera_sample_data` for testing with sample data
- Enable debug mode with `debug_mode=true` for detailed algorithm insights
- Use RViz for visualization validation

### Debugging

#### Debug Mode Features
When `debug_mode=true` is enabled:

1. **Additional Point Cloud Topics**:
   - `/debug/all_points` - All input points before filtering
   - `/debug/filtered_points` - Points within bounding box
   - `/debug/plane_inliers` - Points detected as part of the calibration plane

2. **Algorithm Visualization**:
   - `/debug/initial_board_marker` - Initial board pose from PCA
   - `/debug/final_board_pose` - Final refined board pose after ICP
   - `/debug/icp_iterations` - ICP algorithm progress visualization

3. **Performance Metrics**:
   - `/debug/icp_stats` - ICP algorithm statistics and convergence information

#### Logging
- Use `RCUTILS_LOGGING_SEVERITY=DEBUG` for detailed algorithm logging
- RANSAC plane fitting progress and statistics
- ICP algorithm iteration details and convergence
- Point cloud statistics and processing steps

## Known Issues and Solutions

### Build Issues
1. **OpenCV version 0.0.0**: Makefile automatically sets `OPENCV_PKGCONFIG_NAME=opencv4`
2. **Missing C++ headers**: Install with `sudo apt-get install libstdc++-12-dev libclang-dev`
3. **SFCGAL library missing**: Install with `sudo apt-get install libsfcgal-dev`

### Runtime Issues
1. **gscam node crashes**: Install GStreamer plugins
2. **rosdep initialization errors**: Use setup script for proper initialization
3. **ROS2 daemon issues**: Kill with `pkill -9 -f ros2-daemon`

### Algorithm Issues
1. **ICP convergence**: Fixed successful flag bug and improved parameters
2. **Board orientation**: Implemented PCA-based initialization with -135° rotation
3. **Correspondence matching**: Improved outlier rejection and adaptive thresholds

## Performance Optimizations

### Detection Synchronizer
- Optimized synchronization parameters for calibration workflows
- Window size: 500ms for detection processing delays
- Buffer size: 200 for robust buffering
- Quality threshold: 50 for timestamp differences

### ICP Algorithm
- Reduced max_iterations from 20,000 to 100 for performance
- Improved convergence criteria for stability
- Better outlier rejection with adaptive thresholds

## Contributing Process

1. **Fork the repository** and create a feature branch
2. **Follow coding standards** and add appropriate tests
3. **Update documentation** if adding new features
4. **Test thoroughly** with sample data and debug mode
5. **Submit a pull request** with detailed description

### Commit Guidelines
- Use clear, descriptive commit messages
- Reference issue numbers when applicable
- Keep commits focused and atomic

### Testing Guidelines
- Test with both sample data and real sensor data
- Verify debug topics work correctly
- Check RViz visualization functionality
- Ensure backwards compatibility

## Additional Resources

- **CLAUDE.md**: AI assistant instructions and detailed setup information
- **README.md**: Quick start guide and usage examples
- **Individual package READMEs**: Detailed documentation for each component
- **Launch files**: `ros/lctk_launch/launch/` (config-driven; see `calibrate.launch.py`)