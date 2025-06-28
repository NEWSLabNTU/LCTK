# Multi-Wayside Node Design

## Overview

Multi-wayside performs LiDAR-to-LiDAR calibration by detecting a common calibration board (hollow board with ArUco markers) visible from both sensors and computing the transformation between their coordinate frames.

## Purpose

The multi_wayside_node operates as a real-time ROS 2 node with ROI-based board detection for automatic LiDAR-to-LiDAR calibration.

## Enhanced ROS 2 Architecture with Hybrid ROI Selection

### Multi-Node Design
A hybrid system combining Rust processing node with Python interactive interface:

```
┌─────────────────────────────────────────────────────────────────┐
│                    LCTK Multi-Wayside System                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  multi_wayside_node (Rust)           roi_interactive_node      │
│  ├── Core Processing                 (Python)                  │
│  ├── Subscribers/Publishers          ├── Interactive Markers   │
│  ├── Services (ROI Control)          ├── RViz2 Integration     │
│  │   ├── /set_roi_bounds             └── Service Clients       │
│  │   ├── /get_roi_bounds                  │                    │
│  │   ├── /reset_roi                       │                    │
│  │   └── /save_roi_config                 │                    │
│  └── Parameters                           │                    │
│      ├── ROS 2 parameter system          │                    │
│      └── ROI defaults (size, position)    │                    │
│                                           │                    │
│  Communication: Services ←───────────────┘                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Service-Based ROI Control
```
Python Interactive Node          Rust Processing Node
┌─────────────────────┐          ┌──────────────────────┐
│ Interactive Markers │   ROS2   │ ROI State            │
│ - Drag & Drop ROI   │ ────────▶│ - Update ROI Bounds  │
│ - Real-time Feedback│ Services │ - Republish Markers  │
│ - RViz2 Integration │          │ - Apply to Detection │
└─────────────────────┘          └──────────────────────┘
```

## Workflow

### 1. Initialization
- Load configuration files (board model, detector config, ArUco pattern)
- Validate parameters (queue sizes, sync tolerance, file paths, ROI settings)
- Create publishers and subscribers for all topics
- Initialize detection buffers and shared state

### 2. Real-time Point Cloud Processing
For each incoming PointCloud2 message from `/lidar1/points` or `/lidar2/points`:
1. **Parse Point Cloud**: Convert ROS PointCloud2 to internal LidarPoint format
2. **Apply ROI Cropping**: Use ROI box filter with current ROI parameters
3. **Convert to nalgebra**: Transform cropped points to nalgebra::Point3<f64> for detector
4. **Board Detection**: Run hollow board detector on cropped point cloud
5. **Store Detection**: If successful, store timestamped detection in buffer
6. **Publish Results**: Detection messages, cropped/filtered point clouds, visualization markers

### 3. ROI Configuration
- ROI bounds managed via ROS parameters and services
- Interactive ROI adjustment through Python companion node (roi_interactive_node)
- Real-time ROI updates applied to point cloud processing pipeline

### 4. Data Flow
```
Input Topics:
├── /lidar1/points (sensor_msgs/PointCloud2)
├── /lidar2/points (sensor_msgs/PointCloud2)
├── /lidar1/board_pose_adjustment (geometry_msgs/PoseStamped)
└── /lidar2/board_pose_adjustment (geometry_msgs/PoseStamped)

Processing:
├── ROI-based point cloud cropping
├── Point cloud parsing and filtering
├── Hollow board detection on cropped point clouds
├── Pose adjustment with constraints
└── Marker generation for visualization

Output Topics:
├── /lidar1/board_detection (vision_msgs/Detection3DArray)
├── /lidar2/board_detection (vision_msgs/Detection3DArray)
├── /lidar1/points_cropped (sensor_msgs/PointCloud2)
├── /lidar2/points_cropped (sensor_msgs/PointCloud2)
├── /lidar1/points_filtered (sensor_msgs/PointCloud2)
├── /lidar2/points_filtered (sensor_msgs/PointCloud2)
├── /calibration_markers (visualization_msgs/MarkerArray)
├── /adjustment_markers (visualization_msgs/MarkerArray)
└── /calibration_transform (geometry_msgs/TransformStamped)
```

### 5. Parameter Configuration
Key parameters loaded at startup:
- `board_config_file`: Path to hollow board configuration (YAML)
- `detector_config_file`: Path to detector parameters (YAML)
- `aruco_pattern_file`: Path to ArUco pattern definition (JSON5)
- `max_queue_size`: Maximum detections to buffer (default: 100)
- `sync_tolerance_ms`: Synchronization tolerance (default: 100ms)
- `same_face_mode`: Whether LiDARs see same board face (default: true)
- `apply_bug_fix`: Apply VLP16 coordinate correction (default: false)
- **ROI Parameters**:
  - `roi_box_size_x/y/z`: Default ROI box dimensions (default: 4.0m x 4.0m x 2.0m)
  - `roi_box_position_x/y/z`: Default ROI box center position (default: 2.0m, 0.0m, 0.0m)
  - `min_range/max_range`: Point cloud filtering range (default: 0.5m to 50.0m)

## Module Architecture
```
multi_wayside_node/
├── src/
│   ├── main.rs              # Application entry point
│   ├── node/                # ROS 2 node interface
│   │   ├── publishers.rs    # Publisher management
│   │   ├── subscribers.rs   # Subscriber management
│   │   └── services.rs      # Service handlers
│   ├── detection/           # Board detection logic
│   │   ├── detector.rs      # Detection algorithm wrapper
│   │   ├── processor.rs     # Point cloud processing pipeline
│   │   └── synchronizer.rs  # Detection synchronization
│   ├── pointcloud/          # Point cloud utilities
│   │   ├── parser.rs        # PointCloud2 parsing
│   │   ├── filter.rs        # Point cloud filtering
│   │   └── converter.rs     # Format conversions
│   ├── roi/                 # ROI management
│   │   ├── manager.rs       # ROI state management
│   │   ├── cropper.rs       # ROI-based cropping
│   │   └── service.rs       # ROI service handlers
│   ├── visualization/       # Visualization markers
│   │   ├── board_markers.rs # Board visualization
│   │   ├── roi_markers.rs   # ROI box markers
│   │   └── text_markers.rs  # Status text display
│   ├── calibration/         # Calibration logic
│   │   ├── transform.rs     # Transform computation
│   │   └── validator.rs     # Calibration validation
│   ├── config/              # Configuration management
│   │   ├── parameters.rs    # ROS parameter handling
│   │   └── validator.rs     # Config validation
│   ├── types/               # Shared types
│   └── utils/               # Utility functions
│       └── time.rs          # Time utilities
```

## Design Principles

### Modular Architecture
- **Trait-based design** for dependency injection and testability
- **Clear module separation** with single responsibilities
- **Error propagation** consistent across all modules
- **Test coverage** with comprehensive unit tests

### Service Layer
- **ROS publishers**: Extract to module (node/publishers.rs)
- **ROS subscribers**: Extract to module (node/subscribers.rs)
- **Service handlers**: Extract to module (node/services.rs)
- **Dependency injection**: Pass dependencies via traits

### Detection Pipeline
- **Detection processor**: Create trait interface (detection/processor.rs)
- **Detection logic**: Extract to module with modular design
- **Synchronizer**: Extract sync logic (detection/synchronizer.rs)
- **Pipeline configuration**: Configurable stages with trait-based pipeline