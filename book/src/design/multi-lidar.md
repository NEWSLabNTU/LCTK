# Multi-LiDAR Calibration

The multi-LiDAR calibration pipeline determines the relative transformations between multiple LiDAR sensors, enabling fusion of point clouds from different viewpoints.

## Pipeline Overview

This pipeline uses the `multi_wayside_node` for real-time calibration of two or more LiDAR sensors by detecting calibration boards in each sensor's point cloud data.

## Node Configuration

### calibration_board_locator (per LiDAR)
Each LiDAR requires its own board locator instance.

**LiDAR 1:**
- Input: `/lidar1/pointcloud` (sensor_msgs/PointCloud2)
- Output: `/lidar1/board_detections` (vision_msgs/Detection3DArray)

**LiDAR 2:**
- Input: `/lidar2/pointcloud` (sensor_msgs/PointCloud2)
- Output: `/lidar2/board_detections` (vision_msgs/Detection3DArray)

### multi_wayside_node
Handles multi-LiDAR calibration with real-time processing.

**Inputs:**
- `/lidar1/board_detections`
- `/lidar2/board_detections`
- `/lidar1/board_pose_adjustment` (manual refinement)
- `/lidar2/board_pose_adjustment` (manual refinement)

**Outputs:**
- `/calibration_transform` (geometry_msgs/TransformStamped)
- `/calibration_markers` (visualization_msgs/MarkerArray)

**Features:**
- Automatic detection synchronization
- Real-time transform computation
- TF2 broadcasting
- RViz visualization markers
- Manual pose adjustment interface

## Calibration Process

1. **Board Detection**: Each LiDAR independently detects calibration boards
2. **Synchronization**: Detections are temporally aligned based on timestamps
3. **Correspondence**: Boards detected in multiple LiDARs are matched
4. **Registration**: Point cloud registration using detected board poses
5. **Refinement**: ICP-based refinement for sub-millimeter accuracy
6. **Broadcasting**: Transform published to TF tree

## Advanced Features

### ROI Configuration
Define regions of interest for each LiDAR to focus detection:
```yaml
lidar1_roi:
  min_x: -10.0
  max_x: 10.0
  min_y: -5.0
  max_y: 5.0
  min_z: 0.0
  max_z: 3.0
```

### Detection Parameters
Configurable thresholds for board detection:
- Minimum plane size
- Maximum distance to plane
- Inlier ratio thresholds
- Geometric constraints

### Manual Adjustment
Interactive pose adjustment through RViz:
- Click-and-drag interfaces
- Numeric input fields
- Real-time preview of adjustments

## Launch Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `lidar1_pcap_file` | First LiDAR data file | - |
| `lidar2_pcap_file` | Second LiDAR data file | - |
| `board_config_file` | Board configuration | `board_pattern.json5` |
| `sync_tolerance_ms` | Synchronization tolerance | 100 |
| `enable_manual_adjustment` | Enable pose adjustment UI | true |
| `publish_markers` | Publish visualization markers | true |

## Usage

```bash
# Launch two-LiDAR calibration
ros2 launch calib_launch two_lidar_calibration.launch.xml \
    lidar1_pcap_file:=/path/to/lidar1.pcap \
    lidar2_pcap_file:=/path/to/lidar2.pcap

# Monitor calibration in RViz
rviz2 -d config/multi_lidar_calibration.rviz
```

## Validation

### Geometric Validation
- Board co-planarity checks
- Distance consistency between sensors
- Angular accuracy verification

### Statistical Metrics
- Registration error statistics
- Temporal consistency of transforms
- Detection success rates

### Visual Verification
- Overlaid point clouds in RViz
- Color-coded alignment visualization
- Before/after comparison views