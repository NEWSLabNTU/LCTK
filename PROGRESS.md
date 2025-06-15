# LCTK ROS 2 Refactoring Plan

## Project Overview

LCTK (LiDAR and Camera Toolkit) is undergoing a comprehensive refactoring to become fully ROS 2 compatible. The goal is to transform the existing command-line tools into a cohesive ROS 2 ecosystem that enables real-time calibration and processing workflows.

## Completed Refactoring (ROS 2 Nodes)

### Core Calibration Pipeline Nodes ✅

1. **`aruco_locator_node`** - ArUco marker detection in camera images
   - **Status**: ✅ Complete
   - **Language**: Rust (using rclrs)
   - **Subscribes**: `/image` (sensor_msgs/Image), `/camera_info` (sensor_msgs/CameraInfo)
   - **Publishes**: `/aruco_detections` (vision_msgs/Detection2DArray)

2. **`calibration_board_locator`** - Hollow calibration board detection in point clouds
   - **Status**: ✅ Complete
   - **Language**: Rust (using rclrs)
   - **Subscribes**: `/input_pointcloud` (sensor_msgs/PointCloud2)
   - **Publishes**: `/calibration_board_detections` (vision_msgs/Detection3DArray)

3. **`extrinsic_solver`** - Solves LiDAR-to-camera extrinsic parameters
   - **Status**: ✅ Complete
   - **Language**: Rust (using rclrs)
   - **Subscribes**: ArUco detections, board detections, camera info
   - **Publishes**: `/extrinsic_transform` (geometry_msgs/TransformStamped)

4. **`synchronizer`** - Temporal synchronization of detections
   - **Status**: ✅ Complete
   - **Language**: Rust (using rclrs)
   - **Subscribes**: ArUco and board detections
   - **Publishes**: Synchronized detection pairs

5. **`pointcloud_image_overlay`** - 3D visualization and overlay
   - **Status**: ✅ Complete
   - **Language**: Rust (using rclrs)
   - **Features**: Rerun-based 3D visualization
   - **Subscribes**: Point clouds, images, camera info, transforms

### Data Management and Playback ✅

6. **`rosbag_deck_node`** - Advanced ROS bag playback system
   - **Status**: ✅ Complete
   - **Language**: C++ (using rclcpp)
   - **Features**: Seeking, metadata queries, playback control
   - **Services**: `GetBagInfo`, `SeekToTime`
   - **Publishes**: `PlaybackStatus`

### Supporting Infrastructure ✅

7. **`rosbag_deck_interface`** - Custom message/service definitions
   - **Status**: ✅ Complete
   - **Messages**: `PlaybackStatus`
   - **Services**: `GetBagInfo`, `SeekToTime`

8. **`calib_launch`** - Launch file package for calibration pipeline
   - **Status**: ✅ Complete
   - **Features**: Complete calibration pipeline orchestration

9. **Python Integration**
   - **`rosbag_deck_python`**: ✅ Python bindings
   - **`rosbag_deck_tui`**: ✅ Terminal UI

## Ongoing Work

### Multi-LiDAR Calibration
- **`multi_wayside`** - Multi-wayside calibration tool
  - **Status**: 🚧 Has package.xml but needs ROS 2 node conversion
  - **Priority**: High
  - **Scope**: Convert CLI tool to ROS 2 service-based architecture

### ArUco Generation
- **`aruco_generator_node`** - ArUco pattern generation
  - **Status**: 🚧 Has package.xml but remains CLI tool
  - **Priority**: Low
  - **Scope**: Consider ROS 2 service interface for dynamic generation

## Current Work in Progress

### 1. Enhanced Board Detection with small_gicp Integration 🎯

**Status**: ✅ **small_gicp_rust submodule added** - Ready for integration

**Implementation Progress**:
- ✅ `small_gicp_rust` submodule integrated at `src/lib/small_gicp_rust/`
- ✅ `board-fitter-config` crate with advanced board shape definitions
- 🚧 `board-fitter` crate (placeholder implementation)
- 🚧 Integration of small_gicp for enhanced point cloud registration

**Next Steps**:
- Implement full board-fitter using small_gicp for robust plane fitting
- Create ROS 2 node: `advanced_board_locator_node`
  - **Subscribes**: Point clouds for advanced board detection
  - **Publishes**: Enhanced board detections with better accuracy
  - **Features**: Support for complex board shapes (rectangles, circles, polygons)

**Benefits**:
- Significantly faster and more accurate board fitting algorithms
- Support for multiple board geometries beyond simple rectangles
- Enhanced calibration accuracy through better point cloud alignment
- Real-time performance with parallel processing capabilities

## Future Work

### 1. Enhanced Calibration Pipeline

**Real-time Calibration**:
- Automatic calibration quality assessment
- Dynamic parameter adjustment based on detection confidence
- Calibration convergence monitoring

**Multi-sensor Support**:
- IMU integration for initial pose estimation
- GPS integration for global coordinate systems
- Thermal camera support for additional calibration targets

### 2. Distributed Computing Support

**Multi-node Calibration**:
- Distributed processing across multiple compute nodes
- Load balancing for compute-intensive operations
- Fault tolerance and recovery mechanisms

## Refactored Rust Crates

### Core Libraries (Non-ROS)
- **`aruco-config`** - ArUco pattern configuration types
- **`aruco-detector`** - ArUco marker detection algorithms
- **`hollow-board-config`** - Hollow board pattern configuration
- **`hollow-board-detector`** - Hollow board detection in point clouds
- **`board-fitter-config`** ✅ - Advanced board shape configurations (rectangles, circles, polygons)
- **`board-fitter`** 🚧 - Advanced board detection using small_gicp
- **`plane-estimator`** - Point cloud plane fitting
- **`pnp-solver`** - OpenCV PnP solving wrapper
- **`multi-stream-synchronizer`** - Temporal synchronization utilities
- **`small_gicp_rust`** ✅ - High-performance point cloud registration library

### ROS 2 Integration Status

**Fully Integrated** ✅:
- All core calibration nodes use modern rclrs patterns
- Proper ROS 2 parameter handling
- Standard message type compliance
- Launch file integration

**Architecture Patterns**:
- Arc-based state management for thread safety
- Async/await patterns for non-blocking operations
- Modular design for easy testing and maintenance

## Development Notes

### Build System
- Three-pass build process: ROS 2 Rust → Interface types → LCTK binaries
- Colcon integration for seamless ROS 2 workspace management
- Cargo workspace for Rust dependency management

### Code Quality
- Named parameters in format strings (`println!("{e}")`)
- Proper error handling (no silent exception catching)
- Modern Rust patterns with Arc and move closures
- Comprehensive documentation and examples

### Testing Strategy
- Unit tests for core algorithms
- Integration tests for ROS 2 node functionality
- Calibration accuracy validation
- Performance benchmarking (especially for CUDA integration)

## Timeline

- **Q1 2025**: 
  - ✅ Integrate small_gicp_rust submodule
  - ✅ Create board-fitter-config crate
  - 🚧 Complete board-fitter implementation
  - 🚧 Complete multi_wayside ROS 2 conversion
- **Q2 2025**: Create advanced_board_locator_node with small_gicp integration
- **Q3 2025**: Enhanced real-time calibration features
- **Q4 2025**: Distributed computing and multi-sensor support

## Success Metrics

1. **Performance**: >10x speedup in registration with CUDA acceleration
2. **Usability**: Complete calibration pipeline runnable with single launch command
3. **Accuracy**: Improved calibration precision through better point cloud alignment
4. **Scalability**: Support for 4+ LiDAR sensors in real-time
5. **Reliability**: 99%+ uptime in production calibration workflows
