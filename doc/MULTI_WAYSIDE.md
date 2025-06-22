# Multi-Wayside ROS 2 Node Refactoring Plan

## Overview

This document outlines the plan to refactor `multi_wayside` from a standalone Rust application with kiss3d GUI to a ROS 2 node with RViz2-based visualization. The goal is to maintain all core functionality while integrating seamlessly into ROS 2 workflows.

## Current Architecture Analysis

### Purpose
Multi-wayside performs LiDAR-to-LiDAR calibration by detecting a common calibration board (hollow board with ArUco markers) visible from both sensors and computing the transformation between their coordinate frames.

### multi_wayside_node Workflow

The multi_wayside_node operates as a real-time ROS 2 node with ROI-based board detection:

#### 1. Initialization
- Load configuration files (board model, detector config, ArUco pattern)
- Validate parameters (queue sizes, sync tolerance, file paths, ROI settings)
- Create publishers and subscribers for all topics
- Initialize detection buffers and shared state

#### 2. Real-time Point Cloud Processing
For each incoming PointCloud2 message from `/lidar1/points` or `/lidar2/points`:
1. **Parse Point Cloud**: Convert ROS PointCloud2 to internal LidarPoint format
2. **Apply ROI Cropping**: Use ROI box filter with current ROI parameters
3. **Convert to nalgebra**: Transform cropped points to nalgebra::Point3<f64> for detector
4. **Board Detection**: Run hollow board detector on cropped point cloud
5. **Store Detection**: If successful, store timestamped detection in buffer
6. **Publish Results**: Detection messages, cropped/filtered point clouds, visualization markers

#### 3. ROI Configuration
- ROI bounds managed via ROS parameters and services
- Interactive ROI adjustment through Python companion node (roi_interactive_node)
- Real-time ROI updates applied to point cloud processing pipeline

#### 4. Data Flow Summary
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

#### 5. Parameter Configuration
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

## Implementation Progress

### Overall Status
| Phase | Description | Status | Progress |
|-------|-------------|--------|----------|
| Phase 1 | Core Functionality Migration | ✅ Complete | 100% |
| Phase 2 | RViz2 Visualization | ✅ Complete | 100% |
| Phase 3 | User Interaction (Pose Adjustment) | ✅ Complete | 100% |
| Phase 4 | Configuration & Parameters | ✅ Complete | 100% |
| Phase 5 | ROI Interactive Selection | ✅ Complete | 100% |
| Phase 6 | Modular Refactoring | ✅ Complete | 100% |
| Phase 7 | Automatic Calibration & Sync | ✅ Complete | 100% |
| Phase 8 | Testing & Documentation | ⏳ Planned | 0% |

## Current Phase: Testing & Documentation

### Overview
Phase 7 (Automatic Calibration & Synchronization) has been successfully completed. The focus now shifts to comprehensive testing and documentation to ensure the system is production-ready.

### Module Architecture
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

### Modular Refactoring Progress

#### Core Module Extraction
| Task                | Description                 | Status      | Notes                    |
|---------------------|-----------------------------|-------------|--------------------------|
| Module structure    | Directory organization      | ✅ Complete | All modules created      |
| Point cloud parsing | Extract to module           | ✅ Complete | `pointcloud/parser.rs`   |
| ROI management      | Extract to module           | ✅ Complete | `roi/manager.rs`         |
| Visualization       | Extract to module           | ✅ Complete | `visualization/` modules |
| Core traits         | Define dependency injection | ✅ Complete | Trait-based design       |

#### Service Layer Refactoring
| Task                 | Description                  | Status      | Notes                 |
|----------------------|------------------------------|-------------|-----------------------|
| ROS publishers       | Extract to module            | ✅ Complete | `node/publishers.rs`  |
| ROS subscribers      | Extract to module            | ✅ Complete | `node/subscribers.rs` |
| Service handlers     | Extract to module            | ✅ Complete | `node/services.rs`    |
| Dependency injection | Pass dependencies via traits | ✅ Complete | Implemented           |

#### Detection Pipeline
| Task                   | Description            | Status      | Notes                       |
|------------------------|------------------------|-------------|-----------------------------|
| Detection processor    | Create trait interface | ✅ Complete | `detection/processor.rs`    |
| Detection logic        | Extract to module      | ✅ Complete | Modular design              |
| Synchronizer           | Extract sync logic     | ✅ Complete | `detection/synchronizer.rs` |
| Pipeline configuration | Configurable stages    | ✅ Complete | Trait-based pipeline        |

#### Testing Infrastructure
| Task                    | Description               | Status      | Notes                 |
|-------------------------|---------------------------|-------------|-----------------------|
| Unit tests              | Test modules in isolation | ✅ Complete | 59/59 tests passing   |
| Mock implementations    | Mock traits for testing   | ✅ Complete | Test framework ready  |
| Test compilation        | All tests compile and run | ✅ Complete | Verified              |
| Parameter loading tests | ROS parameter system      | ✅ Complete | Added parameter tests |

### Recent Achievements

#### ✅ **Phase 7: Automatic Calibration & Synchronization Complete**
- **Detection Synchronization**: Implemented time-matched detection pair finding with configurable tolerance
- **Transform Computation**: Automatic LiDAR-to-LiDAR transform calculation using board detections
- **TF2 Broadcasting**: Real-time calibration transform publishing on `/calibration_transform` topic
- **Quality Metrics**: Comprehensive calibration confidence assessment with validation thresholds
- **State Management**: Automatic and manual calibration triggering with persistent state tracking
- **Calibration Validation**: Transform reasonableness checking with configurable limits

#### ✅ **Modular Architecture**
- **Trait-based design** for dependency injection and testability
- **Clear module separation** with single responsibilities
- **Error propagation** consistent across all modules
- **Test coverage** with all 68 tests passing (updated from 59)

## Next Steps

### Phase 8: Testing & Documentation

#### High Priority Tasks
| Task                      | Description                        | Status     | Timeline |
|---------------------------|------------------------------------|------------|----------|
| Integration tests         | Full pipeline testing              | ⏳ Planned | Week 8   |
| Test rosbag2 files        | Create test scenarios              | ⏳ Planned | Week 8   |
| User documentation        | README updates with Phase 7       | ⏳ Planned | Week 8   |
| API documentation         | Topic specifications               | ⏳ Planned | Week 8   |

#### Medium Priority Tasks
| Task                   | Description                   | Status     | Timeline |
|------------------------|-------------------------------|------------|----------|
| State persistence      | Save/load calibration results | ⏳ Planned | Week 8   |
| Performance profiling  | Optimize critical paths       | ⏳ Planned | Week 8   |


## Success Criteria

### ✅ Completed
- [x] All board detection functionality preserved
- [x] Calibration accuracy unchanged
- [x] Real-time processing capability (>10 Hz)
- [x] Pose adjustment functionality via topics
- [x] ROS 2 parameter system implemented
- [x] Modular architecture with dependency injection
- [x] Comprehensive test suite (68/68 tests passing)
- [x] Automatic calibration and synchronization
- [x] TF2 transform broadcasting
- [x] Calibration quality assessment
- [x] Detection synchronization with configurable tolerance

### 🎯 Remaining Goals
- [ ] CPU usage <50% on target hardware
- [ ] Memory usage <500MB
- [ ] Latency <100ms for detection
- [ ] RViz2 visualization smooth (>30 FPS)
- [ ] User guide complete
- [ ] API fully documented
- [ ] Migration guide for existing users

## Conclusion

The multi_wayside_node refactoring has successfully completed Phase 7 (Automatic Calibration & Synchronization), achieving a fully functional automatic calibration system with real-time transform broadcasting. The implementation includes:

- **Complete modular architecture** with trait-based dependency injection
- **Automatic detection synchronization** for time-matched LiDAR pairs  
- **Real-time calibration computation** with quality assessment
- **TF2 transform broadcasting** on `/calibration_transform` topic
- **Comprehensive validation** with configurable quality thresholds
- **68/68 tests passing** ensuring robust implementation

The project demonstrates effective use of Rust for high-performance ROS 2 node development, with a clean separation between performance-critical processing (Rust) and interactive user interfaces (Python). The trait-based design ensures maintainability and testability while providing automatic calibration capabilities that exceed the original functionality.
