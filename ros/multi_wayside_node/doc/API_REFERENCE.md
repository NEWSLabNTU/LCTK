# Multi-Wayside Node API Reference

## Overview

The `multi_wayside_node` is a ROS 2 node that performs automatic LiDAR-to-LiDAR calibration by detecting common calibration boards visible from multiple sensors and computing the transformation between their coordinate frames.

## Topics

### Subscribed Topics

#### Point Cloud Input
- **`/lidar1/points`** (`sensor_msgs/PointCloud2`)
  - Point cloud data from the first LiDAR sensor
  - Expected frame_id: Configurable via `lidar1_frame` parameter
  - Frequency: Typically 10-20 Hz

- **`/lidar2/points`** (`sensor_msgs/PointCloud2`)
  - Point cloud data from the second LiDAR sensor
  - Expected frame_id: Configurable via `lidar2_frame` parameter
  - Frequency: Typically 10-20 Hz

#### Pose Adjustment (Optional)
- **`/lidar1/board_pose_adjustment`** (`geometry_msgs/PoseStamped`)
  - Manual adjustment for detected board pose from LiDAR 1
  - Used for fine-tuning detections in challenging scenarios

- **`/lidar2/board_pose_adjustment`** (`geometry_msgs/PoseStamped`)
  - Manual adjustment for detected board pose from LiDAR 2
  - Used for fine-tuning detections in challenging scenarios

### Published Topics

#### Detection Results
- **`/lidar1/board_detection`** (`vision_msgs/Detection3DArray`)
  - Detected calibration boards from LiDAR 1
  - Contains pose, confidence, and bounding box information
  - Published on successful detection

- **`/lidar2/board_detection`** (`vision_msgs/Detection3DArray`)
  - Detected calibration boards from LiDAR 2
  - Contains pose, confidence, and bounding box information
  - Published on successful detection

#### Processed Point Clouds
- **`/lidar1/points_cropped`** (`sensor_msgs/PointCloud2`)
  - ROI-cropped point cloud from LiDAR 1
  - Only points within the ROI bounds are included
  - Same frame_id as input

- **`/lidar2/points_cropped`** (`sensor_msgs/PointCloud2`)
  - ROI-cropped point cloud from LiDAR 2
  - Only points within the ROI bounds are included
  - Same frame_id as input

- **`/lidar1/points_filtered`** (`sensor_msgs/PointCloud2`)
  - Filtered point cloud from LiDAR 1 (range filtering applied)
  - Points outside min/max range removed
  - Same frame_id as input

- **`/lidar2/points_filtered`** (`sensor_msgs/PointCloud2`)
  - Filtered point cloud from LiDAR 2 (range filtering applied)
  - Points outside min/max range removed
  - Same frame_id as input

#### Visualization
- **`/calibration_markers`** (`visualization_msgs/MarkerArray`)
  - Visualization markers for detected boards
  - Includes:
    - Board coordinate frames (arrows)
    - Board outlines (cubes)
    - Text labels
  - Frame_id: Respective LiDAR frames

- **`/adjustment_markers`** (`visualization_msgs/MarkerArray`)
  - Visualization of pose adjustments
  - Shows the effect of manual pose corrections
  - Frame_id: Respective LiDAR frames

- **`/roi_markers`** (`visualization_msgs/MarkerArray`)
  - ROI bounding box visualization
  - Wire-frame boxes showing detection regions
  - Frame_id: Respective LiDAR frames

#### Calibration Output
- **`/calibration_transform`** (`geometry_msgs/TransformStamped`)
  - The computed transformation from LiDAR 1 to LiDAR 2
  - header.frame_id: `lidar1_frame` (parent)
  - child_frame_id: `lidar2_frame`
  - Published when calibration succeeds
  - Can be used with tf2_ros static_transform_publisher

## Services

### ROI Management
- **`/set_roi_bounds`** (`multi_wayside_node/SetROIBounds`)
  - Set the Region of Interest bounds for a specific LiDAR
  - Request:
    ```
    uint8 lidar_id          # 1 or 2
    float64 center_x        # ROI center X coordinate
    float64 center_y        # ROI center Y coordinate
    float64 center_z        # ROI center Z coordinate
    float64 size_x          # ROI size along X axis
    float64 size_y          # ROI size along Y axis
    float64 size_z          # ROI size along Z axis
    ```
  - Response:
    ```
    bool success            # True if ROI was updated
    string message          # Status message
    ```

- **`/get_roi_bounds`** (`multi_wayside_node/GetROIBounds`)
  - Get the current ROI bounds for a specific LiDAR
  - Request:
    ```
    uint8 lidar_id          # 1 or 2
    ```
  - Response:
    ```
    bool success
    float64 center_x
    float64 center_y
    float64 center_z
    float64 size_x
    float64 size_y
    float64 size_z
    ```

- **`/reset_roi`** (`std_srvs/Trigger`)
  - Reset all ROI bounds to default values
  - Response:
    ```
    bool success
    string message
    ```

- **`/save_roi_config`** (`std_srvs/Trigger`)
  - Save current ROI configuration to file
  - Response:
    ```
    bool success
    string message          # File path if successful
    ```

### Calibration Control
- **`/trigger_calibration`** (`std_srvs/Trigger`)
  - Manually trigger calibration computation
  - Useful when auto_calibrate is false
  - Response:
    ```
    bool success
    string message          # Calibration result or error
    ```

- **`/reset_calibration`** (`std_srvs/Trigger`)
  - Reset calibration state and clear detection buffers
  - Response:
    ```
    bool success
    string message
    ```

## Parameters

### File Paths
- **`board_config_file`** (string, required)
  - Path to hollow board configuration YAML file
  - Defines board geometry and hole patterns

- **`detector_config_file`** (string, required)
  - Path to detector parameters YAML file
  - Contains detection algorithm settings

- **`aruco_pattern_file`** (string, required)
  - Path to ArUco pattern JSON5 file
  - Defines ArUco marker layout on the board

### Detection Parameters
- **`max_queue_size`** (int, default: 100)
  - Maximum number of detections to buffer per LiDAR
  - Range: [10, 1000]

- **`sync_tolerance_ms`** (int, default: 100)
  - Time tolerance for synchronizing detections (milliseconds)
  - Range: [10, 500]

- **`same_face_mode`** (bool, default: true)
  - True: Both LiDARs see the same face of the board
  - False: LiDARs see opposite faces

- **`apply_bug_fix`** (bool, default: false)
  - Apply VLP16 coordinate system correction
  - Enable only for specific VLP16 setups

### Calibration Parameters
- **`auto_calibrate`** (bool, default: true)
  - Enable automatic calibration when synchronized detections are found

- **`min_detections_for_calibration`** (int, default: 5)
  - Minimum number of detection pairs before attempting calibration
  - Range: [1, 20]

- **`calibration_timeout_seconds`** (int, default: 30)
  - Maximum age of detections to consider for calibration
  - Range: [5, 300]

- **`quality_threshold`** (float, default: 0.7)
  - Minimum quality score to accept calibration
  - Range: [0.1, 1.0]

### ROI Parameters
- **`roi_box_size_x`** (float, default: 4.0)
  - ROI box size along X axis (meters)
  - Range: [0.5, 10.0]

- **`roi_box_size_y`** (float, default: 4.0)
  - ROI box size along Y axis (meters)
  - Range: [0.5, 10.0]

- **`roi_box_size_z`** (float, default: 2.0)
  - ROI box size along Z axis (meters)
  - Range: [0.5, 5.0]

- **`roi_box_position_x`** (float, default: 2.0)
  - ROI box center X coordinate (meters)
  - Range: [-10.0, 10.0]

- **`roi_box_position_y`** (float, default: 0.0)
  - ROI box center Y coordinate (meters)
  - Range: [-10.0, 10.0]

- **`roi_box_position_z`** (float, default: 0.0)
  - ROI box center Z coordinate (meters)
  - Range: [-5.0, 5.0]

### Point Cloud Filtering
- **`min_range`** (float, default: 0.5)
  - Minimum range for point cloud filtering (meters)
  - Range: [0.1, 5.0]

- **`max_range`** (float, default: 50.0)
  - Maximum range for point cloud filtering (meters)
  - Range: [5.0, 200.0]

### Frame IDs
- **`lidar1_frame`** (string, default: "lidar1")
  - TF frame ID for the first LiDAR

- **`lidar2_frame`** (string, default: "lidar2")
  - TF frame ID for the second LiDAR

- **`base_frame`** (string, default: "base_link")
  - Base frame for visualization (optional)

## Usage Examples

### Basic Launch
```bash
ros2 run multi_wayside_node multi_wayside_node \
  --ros-args \
  -p board_config_file:=/path/to/board.yaml \
  -p detector_config_file:=/path/to/detector.yaml \
  -p aruco_pattern_file:=/path/to/aruco.json5
```

### Launch with Custom Parameters
```bash
ros2 run multi_wayside_node multi_wayside_node \
  --ros-args \
  -p board_config_file:=/path/to/board.yaml \
  -p detector_config_file:=/path/to/detector.yaml \
  -p aruco_pattern_file:=/path/to/aruco.json5 \
  -p max_queue_size:=200 \
  -p sync_tolerance_ms:=200 \
  -p roi_box_size_x:=5.0 \
  -p roi_box_size_y:=5.0 \
  -p quality_threshold:=0.8
```

### Using Services
```bash
# Set ROI bounds for LiDAR 1
ros2 service call /set_roi_bounds multi_wayside_node/srv/SetROIBounds \
  "{lidar_id: 1, center_x: 3.0, center_y: 0.0, center_z: 0.0, size_x: 4.0, size_y: 4.0, size_z: 2.0}"

# Trigger manual calibration
ros2 service call /trigger_calibration std_srvs/srv/Trigger

# Reset calibration
ros2 service call /reset_calibration std_srvs/srv/Trigger
```

### Monitoring Topics
```bash
# View calibration transform
ros2 topic echo /calibration_transform

# Monitor detection rate
ros2 topic hz /lidar1/board_detection

# Visualize in RViz2
rviz2 -d /path/to/multi_wayside_config.rviz
```

## Calibration Quality Metrics

The node provides quality assessment for calibration results:

- **Translation Magnitude**: Distance between LiDAR origins
- **Rotation Angle**: Relative rotation between LiDAR frames
- **Confidence Score**: Overall quality metric (0.0 - 1.0)
- **Warnings**: List of potential issues detected

Quality thresholds:
- Excellent: confidence > 0.9
- Good: confidence > 0.7
- Acceptable: confidence > 0.5
- Poor: confidence < 0.5

## Troubleshooting

### No Detections
1. Check ROI bounds - ensure board is within ROI
2. Verify point cloud topics are publishing
3. Increase ROI size or adjust position
4. Check board configuration matches physical board

### Poor Calibration Quality
1. Ensure sufficient lighting for intensity-based detection
2. Minimize vibrations during calibration
3. Use same_face_mode appropriate for your setup
4. Increase min_detections_for_calibration

### Synchronization Issues
1. Increase sync_tolerance_ms for sensors with timing drift
2. Ensure both LiDARs have similar publishing rates
3. Check system time synchronization

### Memory/Performance Issues
1. Reduce max_queue_size
2. Decrease ROI size to process fewer points
3. Increase min_range to filter near points
4. Use filtered point cloud topics for visualization