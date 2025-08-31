# LiDAR-Camera Calibration

The LiDAR-camera calibration pipeline determines the extrinsic transformation between a LiDAR sensor and a camera, enabling accurate projection of 3D point clouds onto 2D images.

## Pipeline Overview

The calibration process uses a hybrid approach combining ArUco markers (detected in camera images) and calibration boards (detected in LiDAR point clouds) placed on the same rigid structure.

## Node Configuration

### aruco_locator_node
Detects ArUco markers in camera image streams.

**Inputs:**
- `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
- `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)

**Outputs:**
- `/calibration/aruco_locator/aruco_detections` (vision_msgs/Detection2DArray)

**Configuration:**
- ArUco dictionary (DICT_5X5_1000)
- Marker IDs: [696, 64, 306, 195]
- Refinement method: CORNER_REFINE_SUBPIX

### calibration_board_locator
Detects calibration boards in LiDAR point clouds.

**Inputs:**
- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)

**Outputs:**
- `/calibration/calibration_board_locator/board_detections` (vision_msgs/Detection3DArray)

**Features:**
- Plane fitting using RANSAC
- Geometric pattern matching
- Noise filtering and outlier rejection

### synchronizer
Synchronizes detections from camera and LiDAR sensors.

**Inputs:**
- `/calibration/aruco_locator/aruco_detections`
- `/calibration/calibration_board_locator/board_detections`

**Outputs:**
- `/calibration/synchronizer/synchronized_detections`
- `/calibration/synchronizer/synchronized_pointcloud`
- `/calibration/synchronizer/synchronized_image`

### extrinsic_solver
Computes the extrinsic transformation using PnP algorithms.

**Inputs:**
- Synchronized detections
- Camera calibration parameters

**Outputs:**
- `/calibration/extrinsic_solver/extrinsic_transform` (geometry_msgs/TransformStamped)

**Algorithms:**
- SQPNP (default)
- IPPE
- Iterative refinement

## Calibration Target

The calibration setup uses a combined target featuring:
- ArUco markers for camera detection
- Hollow board pattern for LiDAR detection
- Known geometric relationship between markers and board

## Launch Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `aruco_config_file` | ArUco pattern configuration | `aruco_pattern.json5` |
| `board_config_file` | Board pattern configuration | `board_pattern.json5` |
| `camera_topic` | Camera image topic | `/sensing/camera/front_center/image_raw` |
| `camera_info_topic` | Camera info topic | `/sensing/camera/front_center/camera_info` |
| `pointcloud_topic` | LiDAR topic | `/sensing/lidar/top/pointcloud_raw` |
| `sync_window_ms` | Sync tolerance | 50 |
| `debug_mode` | Enable debugging | false |

## Usage

```bash
# Launch complete calibration pipeline
ros2 launch calib_launch lidar_camera_calibration.launch.xml \
    pcap_file:=/path/to/lidar.pcap \
    video_file:=/path/to/camera.avi

# View results in RViz
rviz2 -d config/aruco_detection.rviz
```

## Validation

The pipeline includes visualization nodes for validation:
- **pointcloud_image_overlay**: Projects calibrated point clouds onto images
- **aruco_detection_overlay**: Shows ArUco detections with bounding boxes

Quality metrics include:
- Reprojection error statistics
- Detection consistency across frames
- Geometric validation of calibration target