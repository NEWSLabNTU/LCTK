# Multi-Wayside Node User Guide

## Table of Contents
1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Quick Start](#quick-start)
4. [Automatic Calibration](#automatic-calibration)
5. [Manual Calibration](#manual-calibration)
6. [ROI Configuration](#roi-configuration)
7. [Visualization with RViz2](#visualization-with-rviz2)
8. [Advanced Configuration](#advanced-configuration)
9. [Troubleshooting](#troubleshooting)
10. [Best Practices](#best-practices)

## Introduction

The `multi_wayside_node` is a ROS 2 node designed for automatic calibration between multiple LiDAR sensors. It detects calibration boards (hollow boards with circular holes) visible from multiple viewpoints and computes the transformation between sensor coordinate frames.

### Key Features
- **Automatic Calibration**: Real-time detection and calibration without manual intervention
- **Quality Assessment**: Confidence scoring and validation of calibration results
- **Interactive ROI**: Adjustable regions of interest for focused detection
- **Real-time Visualization**: RViz2 integration for monitoring calibration progress
- **Flexible Configuration**: Extensive parameter tuning for various scenarios

## Installation

### Prerequisites
- ROS 2 Humble or later
- OpenCV 4.6.0+
- Point Cloud Library (PCL)
- Python 3.8+

### Building from Source
```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/your-org/lctk.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select multi_wayside_node
source install/setup.bash
```

## Quick Start

### 1. Prepare Configuration Files

Create a board configuration file (`board_config.yaml`):
```yaml
board:
  type: "hollow_square"
  size: 1.0  # meters
  holes:
    - position: [-0.3, -0.3, 0.0]
      radius: 0.05
    - position: [0.3, -0.3, 0.0]
      radius: 0.05
    - position: [0.0, 0.3, 0.0]
      radius: 0.05
```

### 2. Launch the Node
```bash
ros2 launch multi_wayside_node multi_wayside.launch.py \
  board_config:=/path/to/board_config.yaml \
  use_rviz:=true
```

### 3. Play Data or Connect Sensors
```bash
# Using rosbag data
ros2 bag play /path/to/calibration_data.bag

# Or ensure live sensors are publishing to:
# - /lidar1/points
# - /lidar2/points
```

### 4. Monitor Calibration
The node will automatically detect boards and compute calibration when sufficient synchronized detections are available. Watch the terminal output for:
```
[INFO] Found synchronized detection pair, attempting calibration
[INFO] Calibration successful! Quality score: 0.912, Translation: 1.523m, Rotation: 5.7°
```

## Automatic Calibration

### How It Works
1. **Detection**: Continuously processes point clouds to find calibration boards
2. **Synchronization**: Matches detections from both LiDARs within time tolerance
3. **Transform Computation**: Calculates relative pose between sensors
4. **Quality Validation**: Assesses calibration quality before accepting
5. **Transform Broadcasting**: Publishes result on `/calibration_transform`

### Configuration for Automatic Mode
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p auto_calibrate:=true \
  -p min_detections_for_calibration:=5 \
  -p sync_tolerance_ms:=100 \
  -p quality_threshold:=0.7
```

### Monitoring Progress
```bash
# Watch calibration state
ros2 topic echo /calibration_transform

# Check detection rate
ros2 topic hz /lidar1/board_detection
ros2 topic hz /lidar2/board_detection
```

## Manual Calibration

### Disabling Automatic Mode
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p auto_calibrate:=false
```

### Triggering Calibration Manually
```bash
# Wait for sufficient detections, then:
ros2 service call /trigger_calibration std_srvs/srv/Trigger
```

### Fine-tuning with Pose Adjustment
For challenging scenarios, manually adjust detected poses:
```bash
# Publish adjusted pose for LiDAR 1
ros2 topic pub /lidar1/board_pose_adjustment geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'lidar1'}, pose: {position: {x: 3.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"
```

## ROI Configuration

### Setting ROI via Parameters
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p roi_box_position_x:=3.0 \
  -p roi_box_position_y:=0.0 \
  -p roi_box_position_z:=0.5 \
  -p roi_box_size_x:=4.0 \
  -p roi_box_size_y:=4.0 \
  -p roi_box_size_z:=2.0
```

### Dynamic ROI Adjustment
```bash
# Set ROI for LiDAR 1
ros2 service call /set_roi_bounds multi_wayside_node/srv/SetROIBounds \
  "{lidar_id: 1, center_x: 3.0, center_y: 0.0, center_z: 0.0, size_x: 5.0, size_y: 5.0, size_z: 2.0}"

# Get current ROI
ros2 service call /get_roi_bounds multi_wayside_node/srv/GetROIBounds "{lidar_id: 1}"

# Reset to defaults
ros2 service call /reset_roi std_srvs/srv/Trigger
```

### Interactive ROI with Python
Launch the interactive ROI adjustment tool:
```bash
ros2 run multi_wayside_node roi_interactive_node.py
```

## Visualization with RViz2

### Launch with RViz
```bash
ros2 launch multi_wayside_node multi_wayside.launch.py use_rviz:=true
```

### Manual RViz Setup
1. Add displays:
   - PointCloud2: `/lidar1/points`, `/lidar2/points`
   - PointCloud2: `/lidar1/points_cropped`, `/lidar2/points_cropped` (ROI visualization)
   - MarkerArray: `/calibration_markers` (board detections)
   - MarkerArray: `/roi_markers` (ROI boxes)
   - TF: Show transform relationships

2. Set Fixed Frame to `base_link` or `lidar1`

3. Adjust point cloud settings:
   - Size: 0.01 for dense clouds
   - Color Transformer: Intensity or FlatColor
   - Alpha: 0.8 for overlay visualization

### Understanding Visualizations
- **Red markers**: LiDAR 1 detections
- **Blue markers**: LiDAR 2 detections
- **Wire-frame boxes**: ROI boundaries
- **Arrows**: Board coordinate frames
- **Text labels**: Detection confidence and LiDAR ID

## Advanced Configuration

### Multi-Board Scenarios
When multiple boards are present:
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p min_detections_for_calibration:=10 \
  -p quality_threshold:=0.8
```

### Noisy Environment Settings
For challenging conditions:
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p sync_tolerance_ms:=200 \
  -p quality_threshold:=0.5 \
  -p calibration_timeout_seconds:=60
```

### VLP-16 Specific Configuration
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p apply_bug_fix:=true
```

### Opposite Face Mode
When LiDARs see different sides of the board:
```bash
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p same_face_mode:=false
```

## Troubleshooting

### Problem: No Board Detections

**Symptoms**: No messages on `/lidar1/board_detection` or `/lidar2/board_detection`

**Solutions**:
1. Check point cloud data:
   ```bash
   ros2 topic hz /lidar1/points
   ros2 topic echo /lidar1/points | head -20
   ```

2. Verify ROI contains the board:
   ```bash
   ros2 service call /get_roi_bounds multi_wayside_node/srv/GetROIBounds "{lidar_id: 1}"
   ```

3. Increase ROI size:
   ```bash
   ros2 service call /set_roi_bounds multi_wayside_node/srv/SetROIBounds \
     "{lidar_id: 1, center_x: 3.0, center_y: 0.0, center_z: 0.0, size_x: 8.0, size_y: 8.0, size_z: 4.0}"
   ```

4. Check board configuration matches physical board

### Problem: Poor Calibration Quality

**Symptoms**: Low confidence scores, calibration rejected

**Solutions**:
1. Ensure stable mounting (no vibrations)
2. Improve lighting for intensity-based detection
3. Increase detection count:
   ```bash
   -p min_detections_for_calibration:=10
   ```
4. Adjust quality threshold:
   ```bash
   -p quality_threshold:=0.5
   ```

### Problem: Synchronization Failures

**Symptoms**: Detections from both LiDARs but no calibration

**Solutions**:
1. Increase sync tolerance:
   ```bash
   -p sync_tolerance_ms:=200
   ```
2. Check sensor timestamps:
   ```bash
   ros2 topic echo /lidar1/points --field header.stamp
   ros2 topic echo /lidar2/points --field header.stamp
   ```
3. Ensure consistent publishing rates

### Problem: High CPU/Memory Usage

**Solutions**:
1. Reduce queue size:
   ```bash
   -p max_queue_size:=50
   ```
2. Decrease ROI size to process fewer points
3. Increase minimum range filter:
   ```bash
   -p min_range:=1.0
   ```

## Best Practices

### 1. **Board Placement**
- Place board 2-5 meters from sensors
- Ensure clear line of sight from both LiDARs
- Orient board at ~45° angle for optimal detection
- Avoid reflective surfaces nearby

### 2. **Environmental Conditions**
- Minimize vibrations during calibration
- Avoid direct sunlight on board
- Ensure consistent lighting
- Remove moving objects from scene

### 3. **Parameter Tuning**
- Start with default parameters
- Adjust ROI first, then detection parameters
- Monitor confidence scores to set appropriate thresholds
- Save successful configurations for reuse

### 4. **Validation**
- Always verify calibration visually in RViz2
- Check transform magnitude is reasonable
- Test with known ground truth if available
- Re-calibrate periodically in production

### 5. **Production Deployment**
```bash
# Recommended production parameters
ros2 run multi_wayside_node multi_wayside_node --ros-args \
  -p auto_calibrate:=true \
  -p min_detections_for_calibration:=10 \
  -p quality_threshold:=0.8 \
  -p calibration_timeout_seconds:=30 \
  -p max_queue_size:=100
```

## Integration with TF2

The calibration result can be integrated into your TF tree:

### Static Transform Publisher
```bash
# After successful calibration
ros2 run tf2_ros static_transform_publisher \
  --x 1.523 --y 0.234 --z 0.145 \
  --qx 0.0 --qy 0.0 --qz 0.0523 --qw 0.9986 \
  --frame-id lidar1 --child-frame-id lidar2
```

### Programmatic Integration
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class CalibrationIntegrator(Node):
    def __init__(self):
        super().__init__('calibration_integrator')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(
            TransformStamped,
            '/calibration_transform',
            self.calibration_callback,
            10)

    def calibration_callback(self, msg):
        # Broadcast as static transform
        self.tf_broadcaster.sendTransform(msg)
        self.get_logger().info(f'Broadcasting calibration: {msg.child_frame_id} relative to {msg.header.frame_id}')
```

## Conclusion

The multi_wayside_node provides a robust solution for automatic LiDAR-to-LiDAR calibration. With proper configuration and environmental setup, it can achieve sub-centimeter accuracy in translation and sub-degree accuracy in rotation. The automatic calibration features introduced in Phase 7 significantly reduce manual effort while maintaining high quality results.

For additional support, refer to the [API Reference](API_REFERENCE.md) or [Troubleshooting Guide](TROUBLESHOOTING.md).