# Calibration Evaluator

The Calibration Evaluator is a ROS 2 node that measures extrinsic calibration quality in real-time by computing Intersection over Union (IoU) metrics between detected ArUco board regions and projected LiDAR points.

## Overview

**Purpose**: Provide quantitative feedback on the accuracy of LiDAR-to-camera extrinsic calibration during and after the calibration process.

**Key Features**:
- Real-time IoU computation
- ArUco-based ground truth detection
- Live extrinsic calibration subscription
- Message synchronization for temporal alignment
- Comprehensive quality metrics
- Visual overlay for debugging

## Architecture

### Input Topics

The evaluator subscribes to the following topics:

| Topic                 | Message Type                     | Description                                                  |
|-----------------------|----------------------------------|--------------------------------------------------------------|
| `image`               | `sensor_msgs/Image`              | Camera images for ground truth detection                     |
| `camera_info`         | `sensor_msgs/CameraInfo`         | Camera intrinsic calibration (auto-derived from image topic) |
| `pointcloud`          | `sensor_msgs/PointCloud2`        | LiDAR point cloud data                                       |
| `aruco_detections`    | `vision_msgs/Detection2DArray`   | ArUco marker detections for ground truth board regions       |
| `extrinsic_transform` | `geometry_msgs/TransformStamped` | Live extrinsic calibration from solver                       |

### Output Topics

| Topic             | Message Type                         | Description                                                      |
|-------------------|--------------------------------------|------------------------------------------------------------------|
| `~/iou_score`     | `std_msgs/Float64`                   | Single IoU score value                                           |
| `~/metrics`       | `lctk_interfaces/CalibrationMetrics` | Detailed calibration metrics                                     |
| `~/overlay_image` | `sensor_msgs/Image`                  | Visualization overlay showing ground truth and projected regions |
| `~/status`        | `std_msgs/String`                    | Human-readable status message                                    |

**Note**: The `~` prefix indicates topics relative to the node's namespace (typically `/calibration/calibration_evaluator`).

## Algorithm

The calibration evaluation follows these steps:

### 1. Ground Truth Detection

**ArUco-Based Detection** (Primary Method):
- Extracts bounding boxes from all detected ArUco markers
- Computes convex hull of all marker corners to define the board region
- More reliable than contour-based detection

**Benefits**:
- Robust to lighting variations
- Precise corner localization
- No manual tuning required

### 2. Point Cloud Projection

1. Extract XYZ coordinates from LiDAR point cloud
2. Transform points from LiDAR frame to camera frame using extrinsic calibration:
   ```
   X_cam = R * X_lidar + t
   ```
3. Project 3D points to 2D image coordinates using camera intrinsics:
   ```
   u = K * X_cam
   [u/z, v/z] = image coordinates
   ```
4. Filter points:
   - Remove points behind camera (z ≤ 0)
   - Keep only points within image bounds
5. Create projected region mask from convex hull of projected points

### 3. IoU Computation

Compute Intersection over Union between ground truth and projected regions:

```
IoU = |Ground Truth ∩ Projected| / |Ground Truth ∪ Projected|
```

Additional metrics:
- **Coverage**: Fraction of ground truth region covered by projection
  ```
  Coverage = |Intersection| / |Ground Truth|
  ```
- **Precision**: Fraction of projected region within ground truth
  ```
  Precision = |Intersection| / |Projected|
  ```
- **Point Density**: Number of projected points per pixel in ground truth region
- **Inlier Count**: Number of projected points within ground truth region

### 4. Visualization

Generates an overlay image showing:
- **Green region**: Ground truth board (from ArUco detections)
- **Red region**: Projected LiDAR points
- **Yellow region** (overlap): Correct calibration
- **Metrics text**: IoU, Coverage, Precision values
- **Status messages**: Error conditions (no board, no points, etc.)

## Message Synchronization

The evaluator uses **ApproximateTimeSynchronizer** to align messages from multiple sensors:

```python
sync = ApproximateTimeSynchronizer(
    [image, pointcloud, aruco_detections],
    queue_size=10,
    slop=0.1  # 100ms tolerance
)
```

**Parameters**:
- `queue_size`: Number of messages buffered per topic
- `slop`: Maximum time difference (seconds) between synchronized messages

## Configuration

### Launch Parameters

| Parameter                   | Type   | Default                                             | Description                                            |
|-----------------------------|--------|-----------------------------------------------------|--------------------------------------------------------|
| `camera_topic`              | string | `/sensing/camera/front_center/image_raw`            | Input camera image topic                               |
| `pointcloud_topic`          | string | `/sensing/lidar/top/pointcloud_raw`                 | Input point cloud topic                                |
| `aruco_detections_topic`    | string | `/calibration/aruco_locator/aruco_detections`       | Input ArUco detections topic                           |
| `extrinsic_transform_topic` | string | `/calibration/extrinsic_solver/extrinsic_transform` | Live extrinsic transform topic                         |
| `extrinsic_json`            | string | `""` (empty)                                        | Path to static extrinsic JSON file (optional fallback) |
| `namespace`                 | string | `calibration`                                       | Node namespace                                         |
| `use_best_effort_qos`       | bool   | `true`                                              | Use best effort QoS for sensor topics                  |
| `sync_queue_size`           | int    | `10`                                                | Message synchronization queue size                     |
| `sync_slop`                 | float  | `0.1`                                               | Synchronization time tolerance (seconds)               |
| `log_level`                 | string | `info`                                              | ROS log level                                          |

### CalibrationMetrics Message

```
std_msgs/Header header

float64 iou                      # Intersection over Union (0.0 to 1.0)
float64 coverage                 # Fraction of ground truth covered
float64 precision                # Fraction of projection within ground truth

uint32 projected_point_count     # Points projected within image
uint32 inlier_point_count        # Points within ground truth region

float64 ground_truth_area        # Ground truth region area (pixels)
float64 projected_area           # Projected region area (pixels)
float64 intersection_area        # Intersection area (pixels)
float64 union_area               # Union area (pixels)

string status                    # Status message
```

## Usage

### Basic Usage

Launch the evaluator as part of the calibration pipeline:

```bash
ros2 launch lctk_launch lidar_camera_calibration.launch.xml
```

The evaluator is automatically included and will publish metrics in real-time.

### Standalone Usage

Launch the evaluator separately:

```bash
ros2 launch calibration_evaluator calibration_evaluator.launch.xml \
    camera_topic:=/sensing/camera/front_center/image_raw \
    pointcloud_topic:=/sensing/lidar/top/pointcloud_raw
```

### Monitoring Metrics

View IoU score in real-time:

```bash
ros2 topic echo /calibration/calibration_evaluator/iou_score
```

View detailed metrics:

```bash
ros2 topic echo /calibration/calibration_evaluator/metrics
```

View overlay visualization in RViz or image viewer:

```bash
ros2 run rqt_image_view rqt_image_view /calibration/calibration_evaluator/overlay_image
```

## Interpreting Results

### IoU Score Interpretation

| IoU Range | Calibration Quality | Interpretation |
|-----------|-------------------|----------------|
| 0.9 - 1.0 | Excellent | Near-perfect alignment |
| 0.7 - 0.9 | Good | Acceptable for most applications |
| 0.5 - 0.7 | Fair | May require refinement |
| 0.3 - 0.5 | Poor | Significant misalignment |
| 0.0 - 0.3 | Very Poor | Unusable calibration |

### Coverage vs Precision

- **High Coverage, Low Precision**: Projected region is larger than ground truth (over-projection)
- **Low Coverage, High Precision**: Projected region is smaller than ground truth (under-projection)
- **High Coverage and Precision**: Good alignment with minor scale differences
- **High Coverage, High Precision, High IoU**: Excellent calibration

### Common Issues

**"No ArUco board detected"**:
- ArUco markers not visible in camera image
- Board moved out of camera field of view
- Lighting conditions prevent detection

**"No valid points"**:
- LiDAR point cloud is empty
- All points filtered out (behind camera or out of range)
- Check LiDAR sensor connection

**"Insufficient projected points"**:
- Very few points project onto image
- Board may be too far from LiDAR
- Extrinsic calibration severely incorrect

**Low IoU with high point count**:
- Extrinsic calibration is misaligned
- Board detection may be incorrect
- Temporal misalignment (adjust sync_slop)

## Integration with Calibration Pipeline

The evaluator integrates seamlessly with the calibration pipeline:

```
┌─────────────────┐
│  Camera Sensor  │────► image ───────────┐
└─────────────────┘                        │
                                           │
┌─────────────────┐                        │
│  LiDAR Sensor   │────► pointcloud ──────┤
└─────────────────┘                        │
                                           ▼
┌─────────────────────┐         ┌──────────────────────┐
│ ArUco Locator Node  │────────►│ Calibration          │
└─────────────────────┘         │ Evaluator Node       │
                                │                      │
┌─────────────────────┐         │ (Synchronizes &      │
│ Extrinsic Solver    │────────►│  Computes IoU)       │
└─────────────────────┘         └──────────────────────┘
                                           │
                                           ├──► iou_score
                                           ├──► metrics
                                           ├──► overlay_image
                                           └──► status
```

## Performance Considerations

**Computational Cost**:
- Point cloud processing: O(N) where N is number of points
- Convex hull computation: O(N log N)
- Mask operations: O(W×H) where W×H is image resolution

**Typical Performance**:
- Processing time: ~50-100ms per frame (720p image, 100k points)
- Recommended frame rate: 10-30 Hz

**Optimization Tips**:
- Downsample point clouds for faster processing
- Reduce synchronization queue size if memory is constrained
- Increase sync_slop if experiencing frequent sync failures

## Troubleshooting

### No Metrics Published

1. Check all input topics are publishing:
   ```bash
   ros2 topic list | grep -E "(image|pointcloud|aruco|extrinsic)"
   ```

2. Verify message synchronization:
   ```bash
   ros2 param get /calibration/calibration_evaluator sync_slop
   # Try increasing if too small
   ```

3. Check for errors in node log:
   ```bash
   ros2 node info /calibration/calibration_evaluator
   ```

### Incorrect IoU Values

1. Verify ArUco detections are correct:
   ```bash
   ros2 topic echo /calibration/aruco_locator/aruco_detections
   ```

2. Check extrinsic calibration is reasonable:
   ```bash
   ros2 topic echo /calibration/extrinsic_solver/extrinsic_transform
   ```

3. Inspect overlay image for visual verification:
   ```bash
   ros2 run rqt_image_view rqt_image_view /calibration/calibration_evaluator/overlay_image
   ```

## Future Enhancements

Potential improvements for future versions:

1. **Geometric Error Metrics**: Compute point-to-plane distances
2. **Temporal Stability**: Track IoU over time to detect calibration drift
3. **Multi-Board Support**: Evaluate calibration using multiple calibration boards
4. **Automated Thresholds**: Adaptive quality thresholds based on scene characteristics
5. **RViz Plugin**: Interactive visualization with metric overlays

## References

- [IoU Metric](https://en.wikipedia.org/wiki/Jaccard_index)
- [ApproximateTimeSynchronizer](http://wiki.ros.org/message_filters/ApproximateTime)
- [LiDAR-Camera Calibration](./lidar-camera.md)
