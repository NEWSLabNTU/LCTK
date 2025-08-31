# API Documentation

This section provides comprehensive API documentation for LCTK's core libraries and ROS 2 nodes.

## Core Libraries

### ArUco Detection

#### aruco-config
Configuration types for ArUco marker patterns.

```rust
pub struct ArucoConfig {
    pub dictionary: ArucoDictionary,
    pub marker_size: f64,
    pub markers: Vec<MarkerDefinition>,
}

pub struct MarkerDefinition {
    pub id: i32,
    pub position: Point2<f64>,
}
```

#### aruco-detector  
ArUco marker detection algorithms.

```rust
pub struct ArucoDetector {
    dictionary: ArucoDictionary,
    parameters: DetectorParameters,
}

impl ArucoDetector {
    pub fn new(config: &ArucoConfig) -> Result<Self>;
    pub fn detect(&self, image: &Image) -> Result<Vec<DetectionResult>>;
}
```

### Point Cloud Processing

#### hollow-board-detector
Calibration board detection in point clouds.

```rust
pub struct HollowBoardDetector {
    config: HollowBoardConfig,
    plane_estimator: PlaneEstimator,
}

impl HollowBoardDetector {
    pub fn new(config: HollowBoardConfig) -> Self;
    pub fn detect(&self, cloud: &PointCloud) -> Result<Vec<BoardDetection>>;
}
```

### Calibration Algorithms

#### pnp-solver
Perspective-n-Point problem solving.

```rust
pub enum PnPMethod {
    SQPNP,
    IPPE,
    Iterative,
}

pub struct PnPSolver {
    method: PnPMethod,
    refinement: bool,
}

impl PnPSolver {
    pub fn new(method: PnPMethod) -> Self;
    pub fn solve(&self, points_2d: &[Point2<f64>], 
                 points_3d: &[Point3<f64>], 
                 camera_matrix: &Matrix3<f64>) -> Result<Transform>;
}
```

## ROS 2 Node APIs

### aruco_locator_node

#### Topics
**Subscriptions:**
- `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
- `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)

**Publications:**
- `/calibration/aruco_locator/aruco_detections` (vision_msgs/Detection2DArray)

#### Parameters
- `aruco_config_file` (string): Path to ArUco configuration file
- `debug_mode` (bool): Enable debug visualization
- `camera_namespace` (string): Camera topic namespace

### calibration_board_locator

#### Topics  
**Subscriptions:**
- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)

**Publications:**
- `/calibration/calibration_board_locator/board_detections` (vision_msgs/Detection3DArray)

#### Parameters
- `board_config_file` (string): Path to board configuration file
- `min_points` (int): Minimum points for plane detection
- `max_distance` (double): Maximum distance to plane

### extrinsic_solver

#### Topics
**Subscriptions:**
- `/calibration/synchronizer/synchronized_detections` (custom message)
- `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)

**Publications:**
- `/calibration/extrinsic_solver/extrinsic_transform` (geometry_msgs/TransformStamped)

#### Parameters
- `method` (string): PnP solving method (SQPNP, IPPE, Iterative)
- `refinement` (bool): Enable iterative refinement
- `min_correspondences` (int): Minimum point correspondences required

## Message Types

### Custom Messages

#### CalibrationQuality
```yaml
# Calibration quality metrics
float64 reprojection_error
float64 detection_consistency  
float64 geometric_validation
float64 temporal_stability
float64 convergence_score
```

#### SynchronizedDetections
```yaml
# Time-synchronized detection results
std_msgs/Header header
vision_msgs/Detection2DArray aruco_detections
vision_msgs/Detection3DArray board_detections
float64 synchronization_error
```

### Service Types

#### GenerateAruco
```yaml
# ArUco generation service
---
# Request
string dictionary        # ArUco dictionary type
float64 marker_size      # Marker size in meters
int32[] marker_ids       # List of marker IDs
string output_format     # Output format (pdf, png, svg)

---
# Response  
bool success
string message
string output_path       # Path to generated file
```

#### TriggerCalibration
```yaml
# Calibration trigger service
---
# Request
bool reset_previous      # Reset previous calibration data
string output_path       # Path for calibration results

---
# Response
bool success
string message
geometry_msgs/Transform result
```

## Configuration Formats

### ArUco Configuration (JSON5)
```json5
{
  "dictionary": "DICT_5X5_1000",
  "marker_size": 0.05,
  "markers": [
    {"id": 696, "position": [0.0, 0.0]},
    {"id": 64, "position": [0.1, 0.0]},
    {"id": 306, "position": [0.0, 0.1]},
    {"id": 195, "position": [0.1, 0.1]}
  ]
}
```

### Board Configuration (JSON5)  
```json5
{
  "board_size": [0.6, 0.4],
  "hole_diameter": 0.05,
  "hole_positions": [
    [0.1, 0.1], [0.5, 0.1],
    [0.1, 0.3], [0.5, 0.3]
  ],
  "detection_params": {
    "min_plane_points": 100,
    "max_plane_distance": 0.01,
    "hole_detection_tolerance": 0.005
  }
}
```

For detailed implementation examples and usage patterns, see the source code documentation and example programs in the `examples/` directory.