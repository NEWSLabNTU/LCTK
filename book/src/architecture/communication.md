# Communication

ROS 2 nodes communicate using standardized topics and services, enabling flexible system composition.

## LiDAR-Camera Calibration Topics

### Input Topics
- `/sensing/camera/front_center/image_raw`: Raw camera images
- `/sensing/camera/front_center/camera_info`: Camera calibration parameters
- `/sensing/lidar/top/pointcloud_raw`: Raw LiDAR point clouds

### Detection Topics
- `/calibration/aruco_locator/aruco_detections`: Detected ArUco markers (vision_msgs/Detection2DArray)
- `/calibration/calibration_board_locator/board_detections`: Detected calibration boards (vision_msgs/Detection3DArray)

### Synchronization Topics
- `/calibration/synchronizer/synchronized_detections`: Time-aligned detections from multiple sources

### Output Topics
- `/calibration/extrinsic_solver/extrinsic_transform`: Final calibration transform (geometry_msgs/TransformStamped)
- `/calibration/visualization/image_with_detections`: Annotated images for monitoring

## Multi-LiDAR Calibration Topics

### Input Topics
- `/lidar1/points`, `/lidar2/points`: Point clouds from multiple LiDAR sensors
- `/lidar1/board_pose_adjustment`: Manual pose refinement inputs

### Detection Topics
- `/lidar1/board_detection`, `/lidar2/board_detection`: Board detections from each LiDAR

### Output Topics
- `/calibration_transform`: Real-time LiDAR-to-LiDAR transform
- `/calibration_markers`: RViz visualization markers

## Services

### Calibration Control
- `/calibration/trigger`: Start calibration process
- `/calibration/reset`: Reset calibration state
- `/calibration/save`: Save calibration results

### Configuration
- `/detection/set_roi`: Configure region of interest
- `/detection/set_threshold`: Adjust detection thresholds

## TF2 Integration

The system publishes transforms to the TF tree:
- `base_link` → `lidar_frame`
- `base_link` → `camera_frame`
- `lidar1_frame` → `lidar2_frame`

## Parameter Server

Dynamic parameters for runtime configuration:
- Detection thresholds
- Synchronization tolerances
- Calibration algorithm selection
- Visualization options

## Quality of Service

Topics use appropriate QoS profiles:
- **Sensor data**: Best effort, volatile
- **Detections**: Reliable, transient local
- **Transforms**: Reliable, transient local