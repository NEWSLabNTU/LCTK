# Simple Extrinsic Solver Python Package

A simplified Python ROS2 node for demonstrating solvePnP with ArUco and board detections. This package provides a basic implementation of extrinsic calibration between LiDAR and camera sensors.

## Features

- **ArUco Detection Processing**: Subscribes to ArUco marker detections and processes them for calibration
- **Board Detection Processing**: Handles calibration board detections from LiDAR data
- **PnP Solving**: Uses OpenCV's solvePnP to compute extrinsic transformations
- **Quality Assessment**: Provides basic calibration quality metrics
- **Debug Publishing**: Publishes debug information for visualization

## Dependencies

- ROS2 Humble
- Python 3.10+
- OpenCV (python3-opencv)
- NumPy (python3-numpy)
- rclpy

## Topics

### Subscribed Topics
- `/aruco_detections` (vision_msgs/Detection2DArray): ArUco marker detections
- `/calibration_board_detections` (vision_msgs/Detection3DArray): Calibration board detections
- `/camera_info` (sensor_msgs/CameraInfo): Camera intrinsic parameters

### Published Topics
- `/extrinsic_transform` (geometry_msgs/TransformStamped): Computed extrinsic transformation
- `/calibration_quality` (std_msgs/String): Calibration quality metrics (JSON format)
- `/debug/recent_aruco_detections` (vision_msgs/Detection2DArray): Debug ArUco detections
- `/debug/recent_board_detections` (vision_msgs/Detection3DArray): Debug board detections

## Parameters

- `parent_frame` (string, default: "lidar"): Parent frame for the extrinsic transform
- `child_frame` (string, default: "camera"): Child frame for the extrinsic transform
- `aruco_pattern_file` (string, default: ""): Path to ArUco pattern configuration file
- `enable_quality_assessment` (bool, default: true): Enable calibration quality assessment

## Usage

### Running the Node

```bash
# Source the workspace
source install/setup.sh

# Run the node directly
ros2 run lidar_to_camera_solver lidar_to_camera_solver

# Or run with launch file
ros2 launch lidar_to_camera_solver lidar_to_camera_solver.launch.py
```

### Example Launch with Parameters

```bash
ros2 launch lidar_to_camera_solver lidar_to_camera_solver.launch.py \
    parent_frame:=lidar \
    child_frame:=camera \
    enable_quality_assessment:=true
```

## Configuration

The package includes a default ArUco pattern configuration in `config/aruco_pattern.yaml`:

```yaml
markers:
  - id: 0
    size: 0.1  # 10cm markers
  - id: 1
    size: 0.1
  - id: 2
    size: 0.1
  - id: 3
    size: 0.1

board_size: [1.0, 1.0]  # 1m x 1m board
marker_spacing: 0.2  # 20cm spacing between markers
```

## Implementation Details

NOTE: this section is stale (it predates SQPnP, real ArUco corners and covariance weighting) and
is slated for deletion. It described an early Python port of a Rust solver:

1. **Simplified Point Correspondence**: Uses basic bounding box corners for ArUco markers
2. **Basic PnP Solving**: Uses OpenCV's SOLVEPNP_ITERATIVE method
3. **Simple Quality Metrics**: Provides basic reprojection error and inlier ratio
4. **Python Implementation**: Easier to understand and modify for educational purposes

## Limitations

- Simplified ArUco corner detection (uses bounding box instead of actual corners)
- Basic board pose transformation (simplified coordinate system handling)
- Limited quality assessment compared to the full Rust implementation
- No dynamic parameter adjustment

## Building

The package is built as part of the main LCTK workspace:

```bash
make build
```

Or build individually:

```bash
cd src/ros2
colcon build --packages-select lidar_to_camera_solver --symlink-install
```

## Testing

The node can be tested by running it and checking that it initializes correctly:

```bash
source install/setup.sh
timeout 5s ros2 run lidar_to_camera_solver lidar_to_camera_solver
```

You should see initialization messages indicating the node is ready to receive detections.

