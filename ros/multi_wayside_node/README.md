# Multi Wayside Node

ROS 2 node for automatic LiDAR-to-LiDAR calibration using hollow board detection with comprehensive visualization and adjustment capabilities.

## Features

- **Automatic Calibration** (Phase 7): Real-time detection and calibration without manual intervention
- **Quality Assessment**: Confidence scoring and validation of calibration results
- **Time Synchronization**: Automatic pairing of detections across sensors
- **Interactive ROI**: Service-based region of interest adjustment
- **Manual Refinement**: Pose adjustment for challenging scenarios
- **Transform Broadcasting**: Automatic TF2 integration for calibration results
- **Rich Visualization**: RViz2 integration with color-coded markers and point clouds
- **Production Ready**: Comprehensive error handling and recovery

## Topics

### Subscribed Topics
- `/lidar1/points` (sensor_msgs/PointCloud2) - LiDAR 1 point cloud data
- `/lidar2/points` (sensor_msgs/PointCloud2) - LiDAR 2 point cloud data
- `/lidar1/board_pose_adjustment` (geometry_msgs/PoseStamped) - Manual pose adjustment for LiDAR 1
- `/lidar2/board_pose_adjustment` (geometry_msgs/PoseStamped) - Manual pose adjustment for LiDAR 2

### Published Topics
- `/lidar1/board_detection` (vision_msgs/Detection3DArray) - Board detection results from LiDAR 1
- `/lidar2/board_detection` (vision_msgs/Detection3DArray) - Board detection results from LiDAR 2
- `/lidar1/points_filtered` (sensor_msgs/PointCloud2) - Filtered point cloud from LiDAR 1
- `/lidar2/points_filtered` (sensor_msgs/PointCloud2) - Filtered point cloud from LiDAR 2
- `/calibration_markers` (visualization_msgs/MarkerArray) - Detection visualization markers
- `/adjustment_markers` (visualization_msgs/MarkerArray) - Manual adjustment visualization markers
- `/calibration_transform` (geometry_msgs/TransformStamped) - Computed calibration transform

### Services
- `/trigger_calibration` (std_srvs/Trigger) - Manually compute calibration from synchronized detections
- `/reset_calibration` (std_srvs/Trigger) - Reset calibration state and clear detection buffers
- `/set_roi_bounds` (multi_wayside_node/SetROIBounds) - Set detection region of interest
- `/get_roi_bounds` (multi_wayside_node/GetROIBounds) - Query current ROI settings
- `/reset_roi` (std_srvs/Trigger) - Reset ROI to default values
- `/save_roi_config` (std_srvs/Trigger) - Save current ROI configuration
- `/reset_adjustments` (std_srvs/Trigger) - Clear all manual pose adjustments
- `/save_adjustments` (std_srvs/Trigger) - Save current pose adjustments to config/pose_adjustments.json
- `/load_adjustments` (std_srvs/Trigger) - Load pose adjustments from config/pose_adjustments.json
- `/save_config` (std_srvs/Trigger) - Save current node configuration to timestamped YAML file
- `/load_config` (std_srvs/Trigger) - Load configuration (requires node restart, use launch parameters instead)

## Automatic Calibration (Phase 7)

The node now features automatic calibration that continuously monitors detections and computes calibration when sufficient synchronized data is available.

### How It Works
1. **Detection**: Processes point clouds from both LiDARs to find calibration boards
2. **Synchronization**: Automatically pairs detections within time tolerance
3. **Quality Check**: Validates detection quality before attempting calibration
4. **Transform Computation**: Calculates relative pose between sensors
5. **Validation**: Assesses calibration quality with confidence scoring
6. **Broadcasting**: Publishes transform on `/calibration_transform` topic

### Configuration
```bash
# Enable automatic calibration (default: true)
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p auto_calibrate:=true \
  -p min_detections_for_calibration:=5 \
  -p calibration_timeout_seconds:=30 \
  -p quality_threshold:=0.7
```

### Monitoring
```bash
# Watch calibration progress
ros2 topic echo /calibration_transform

# Check detection synchronization
ros2 topic hz /lidar1/board_detection
ros2 topic hz /lidar2/board_detection
```

## Manual Pose Adjustment

The node supports manual refinement of board pose detections through pose adjustment messages. This allows users to correct detection errors or fine-tune calibration results.

### Using the Python Script

A helper script is provided for sending pose adjustments:

```bash
# Adjust LiDAR 1 board pose by 10cm in X direction
python3 scripts/adjust_pose.py --lidar 1 --x 0.1

# Adjust LiDAR 2 board pose with rotation
python3 scripts/adjust_pose.py --lidar 2 --yaw-deg 5.0

# Combined translation and rotation adjustment
python3 scripts/adjust_pose.py --lidar 1 --x 0.05 --y -0.02 --yaw-deg 2.5
```

### Using ROS 2 CLI

```bash
# Send pose adjustment using ros2 topic pub
ros2 topic pub --once /lidar1/board_pose_adjustment geometry_msgs/PoseStamped "
header:
  frame_id: 'lidar1'
pose:
  position: {x: 0.1, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

### Manual Adjustment Workflow

1. **Run the node**: Start multi_wayside_node and ensure it's receiving point cloud data
2. **Monitor detections**: Check that boards are being detected in both LiDARs via RViz2
3. **Apply adjustments**: Use the script or manual commands to refine poses
4. **Visualize results**: Green markers in RViz2 show the adjusted poses
5. **Compute calibration**: Call the `/trigger_calibration` service to compute the final transform
6. **Save adjustments**: Use `/save_adjustments` service to persist manual adjustments for later use
7. **Load adjustments**: Use `/load_adjustments` service to restore previously saved adjustments
8. **Reset if needed**: Use `/reset_adjustments` service to clear manual changes

## RViz2 Visualization

The included RViz2 configuration (`config/multi_wayside.rviz`) provides comprehensive visualization:

- **Point Clouds**: Original data (white/rainbow) and filtered data (red/blue)
- **Detection Markers**: Red arrows for LiDAR 1, blue arrows for LiDAR 2
- **Adjustment Markers**: Green arrows showing manual pose adjustments
- **TF Frames**: Coordinate system visualization
- **Board Outlines**: Semi-transparent cubes showing detected board geometry

## Configuration Files

- `config/hollow_board.json5` - Board geometry configuration
- `config/detector.json5` - Detection algorithm parameters
- `config/aruco_pattern.json5` - ArUco marker pattern definition
- `config/multi_wayside.rviz` - RViz2 display configuration

## Launch

```bash
# Build the node
source install/setup.bash
cargo build --release --manifest-path src/bin/multi_wayside_node/Cargo.toml

# Run the node directly
cargo run --release --manifest-path src/bin/multi_wayside_node/Cargo.toml

# Using ROS 2 launch files (after colcon build)

# Option 1: Python launch file
ros2 launch multi_wayside_node multi_wayside.launch.py

# Option 2: XML launch file (recommended)
ros2 launch multi_wayside_node multi_wayside.launch.xml

# Option 3: Minimal XML launch file (quick testing)
ros2 launch multi_wayside_node multi_wayside_minimal.launch.xml

# Option 4: Advanced XML launch file with environment configurations
ros2 launch multi_wayside_node multi_wayside_advanced.launch.xml environment:=outdoor

# Launch with custom parameters
ros2 launch multi_wayside_node multi_wayside.launch.xml \
    lidar1_topic:=/velodyne1/points \
    lidar2_topic:=/velodyne2/points \
    use_rviz:=true \
    log_level:=debug

# Launch with rosbag playback
ros2 launch multi_wayside_node multi_wayside_advanced.launch.xml \
    use_rosbag:=true \
    bag_file:=/path/to/your/bag \
    use_rviz:=true

# Launch with data recording
ros2 launch multi_wayside_node multi_wayside_advanced.launch.xml \
    record_data:=true \
    bag_name:=calibration_session_1
```

### Available Launch Files

1. **multi_wayside.launch.py** - Python launch file with full configurability
2. **multi_wayside.launch.xml** - Standard XML launch file (recommended)
3. **multi_wayside_minimal.launch.xml** - Minimal configuration for quick testing
4. **multi_wayside_advanced.launch.xml** - Advanced features including:
   - Environment-specific configurations (sim, indoor, outdoor)
   - Rosbag playback and recording
   - Debug mode with verbose logging
   - Automatic respawn for outdoor environments
   - Diagnostic aggregator integration

## Parameters

### Core Configuration
| Parameter | Type | Default | Description | Range/Constraints |
|-----------|------|---------|-------------|-------------------|
| `board_config_file` | string | `config/hollow_board.yaml` | Path to board geometry configuration | Must exist and be readable |
| `detector_config_file` | string | `config/detector.yaml` | Path to detection algorithm parameters | Must exist and be readable |
| `aruco_pattern_file` | string | `config/aruco_pattern.json5` | Path to ArUco marker pattern definition | Must exist and be readable |

### Calibration Mode
| Parameter | Type | Default | Description | Range |
|-----------|------|---------|-------------|-------|
| `same_face_mode` | bool | `true` | Whether both LiDARs see the same face of the board | - |
| `apply_bug_fix` | bool | `false` | Apply VLP16 coordinate system correction | - |
| `auto_calibrate` | bool | `true` | Enable automatic calibration when detections are synchronized | - |
| `min_detections_for_calibration` | int | 5 | Minimum synchronized detection pairs before attempting calibration | 1-20 |
| `calibration_timeout_seconds` | int | 30 | Maximum age of detections to consider for calibration | 5-300 |
| `quality_threshold` | float | 0.7 | Minimum quality score to accept calibration result | 0.1-1.0 |

### Synchronization
| Parameter | Type | Default | Description | Range |
|-----------|------|---------|-------------|-------|
| `max_queue_size` | int | 100 | Maximum detection buffer size | 1-10000 |
| `sync_tolerance_ms` | int | 100 | Time tolerance for detection synchronization (ms) | 1-10000 |

### Frame IDs
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar1_frame` | string | `lidar1` | Frame ID for first LiDAR |
| `lidar2_frame` | string | `lidar2` | Frame ID for second LiDAR |
| `base_frame` | string | `base_link` | Base frame for visualization |

### ROI Configuration
| Parameter | Type | Default | Description | Range |
|-----------|------|---------|-------------|-------|
| `roi_box_position_x` | float | 2.0 | ROI center X coordinate (meters) | -10.0 to 10.0 |
| `roi_box_position_y` | float | 0.0 | ROI center Y coordinate (meters) | -10.0 to 10.0 |
| `roi_box_position_z` | float | 0.0 | ROI center Z coordinate (meters) | -5.0 to 5.0 |
| `roi_box_size_x` | float | 4.0 | ROI size along X axis (meters) | 0.5 to 10.0 |
| `roi_box_size_y` | float | 4.0 | ROI size along Y axis (meters) | 0.5 to 10.0 |
| `roi_box_size_z` | float | 2.0 | ROI size along Z axis (meters) | 0.5 to 5.0 |

### Point Cloud Filtering
| Parameter | Type | Default | Description | Range |
|-----------|------|---------|-------------|-------|
| `min_range` | float | 0.5 | Minimum range for point cloud filtering (meters) | 0.1 to 5.0 |
| `max_range` | float | 50.0 | Maximum range for point cloud filtering (meters) | 5.0 to 200.0 |

### Visualization
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `visualization.publish_filtered_clouds` | bool | `true` | Whether to publish filtered point clouds |
| `visualization.marker_lifetime_sec` | float | 5.0 | Lifetime of visualization markers in seconds |
| `visualization.adjustment_marker_scale` | float | 0.4 | Scale factor for manual adjustment markers |
| `visualization.detection_marker_scale` | float | 0.3 | Scale factor for detection markers |

### Adjustment Constraints
| Parameter | Type | Default | Description | Units |
|-----------|------|---------|-------------|-------|
| `adjustment_constraints.max_translation` | float | 2.0 | Maximum allowed manual translation | meters |
| `adjustment_constraints.max_rotation_deg` | float | 30.0 | Maximum allowed manual rotation | degrees |

### File Paths
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `files.adjustment_save_path` | string | `config/pose_adjustments.json` | Path for saving/loading manual adjustments |

### Topic Configuration
All topic names are configurable via parameters under the `topics` namespace:
- `topics.lidar1_input`: Input point cloud from LiDAR 1
- `topics.lidar2_input`: Input point cloud from LiDAR 2
- `topics.lidar1_detection`: Detection output for LiDAR 1
- `topics.lidar2_detection`: Detection output for LiDAR 2
- `topics.calibration_markers`: Visualization markers for detections
- `topics.adjustment_markers`: Visualization markers for manual adjustments
- And more...

### Service Configuration
Service names are configurable via parameters under the `services` namespace:
- `services.trigger_calibration`: Service to compute calibration
- `services.reset_adjustments`: Service to clear adjustments
- `services.save_adjustments`: Service to save adjustments
- `services.load_adjustments`: Service to load adjustments

## Integration Testing

The node includes comprehensive integration tests with synthetic data generation and automated validation.

### Quick Test
```bash
# Run all integration tests
./scripts/run_integration_tests.sh

# Run specific scenario
./scripts/run_integration_tests.sh --scenario scenario_1_perfect_boards

# Run with visualization
./scripts/run_integration_tests.sh --visual
```

### Test Scenarios
1. **Perfect Boards**: Ideal conditions for baseline validation
2. **Noisy Data**: Sensor noise and environmental artifacts
3. **Partial Occlusion**: Board partially blocked from view
4. **Multi-Board**: Multiple boards in scene

### Generating Test Data
```bash
# Generate all test scenarios
python3 scripts/generate_test_data.py --output_dir test_data --scenarios perfect noisy occlusion multi

# Generate specific scenario
python3 scripts/generate_test_data.py --output_dir test_data --scenarios perfect
```

### Launch Files for Testing
```bash
# Basic integration test
ros2 launch launch/test_basic_calibration.launch.py

# Multi-scenario test
ros2 launch launch/test_multi_scenario.launch.py

# With custom parameters
ros2 launch launch/test_basic_calibration.launch.py \
  test_bag:=scenario_1_perfect_boards.bag \
  use_rviz:=true \
  auto_validate:=true
```