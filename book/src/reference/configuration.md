# Configuration

LCTK uses multiple configuration mechanisms to provide flexibility while maintaining ease of use.

## Configuration Hierarchy

1. **Default values**: Built into the code
2. **Configuration files**: JSON5 files for complex structures
3. **Launch parameters**: ROS 2 launch file arguments
4. **Runtime parameters**: ROS 2 parameter server
5. **Environment variables**: System-level configuration

## Launch File Configuration

### Basic Parameters
```xml
<launch>
  <arg name="pcap_file" default="/path/to/lidar.pcap"/>
  <arg name="video_file" default="/path/to/camera.avi"/>
  <arg name="loop" default="true"/>
  <arg name="debug_mode" default="false"/>
</launch>
```

### Advanced Parameters
```xml
<arg name="aruco_config_file"
     default="$(find-pkg-share calib_launch)/config/aruco/aruco_pattern.json5"/>
<arg name="board_config_file"
     default="$(find-pkg-share calib_launch)/config/board/board_pattern.json5"/>
<arg name="sync_window_ms" default="50"/>
<arg name="max_distance" default="10.0"/>
```

## Configuration Files

### ArUco Pattern Configuration
Location: `config/aruco/aruco_pattern.json5`

```json5
{
  // ArUco dictionary selection
  "dictionary": "DICT_5X5_1000",

  // Physical marker size in meters
  "marker_size": 0.05,

  // Marker definitions
  "markers": [
    {"id": 696, "position": [0.0, 0.0]},    // Bottom-left
    {"id": 64,  "position": [0.1, 0.0]},    // Bottom-right
    {"id": 306, "position": [0.0, 0.1]},    // Top-left
    {"id": 195, "position": [0.1, 0.1]}     // Top-right
  ],

  // Detection parameters
  "detection": {
    "corner_refinement": "CORNER_REFINE_SUBPIX",
    "adaptive_threshold_window_size": 23,
    "adaptive_threshold_constant": 7,
    "min_marker_perimeter_rate": 0.03,
    "max_marker_perimeter_rate": 4.0
  }
}
```

### Board Pattern Configuration
Location: `config/board/board_pattern.json5`

```json5
{
  // Physical board dimensions in meters
  "board_size": [0.6, 0.4],  // [width, height]

  // Hole specifications
  "hole_diameter": 0.05,
  "hole_positions": [
    [0.1, 0.1], [0.5, 0.1],  // Bottom row
    [0.1, 0.3], [0.5, 0.3]   // Top row
  ],

  // Detection parameters
  "detection_params": {
    "min_plane_points": 100,
    "max_plane_distance": 0.01,
    "plane_detection_iterations": 1000,
    "hole_detection_tolerance": 0.005,
    "min_hole_points": 50
  },

  // Filtering parameters
  "filtering": {
    "voxel_size": 0.01,
    "noise_filter_k": 50,
    "noise_filter_std_dev": 1.0
  }
}
```

### Camera Calibration
Location: `config/camera/front_center_camera_info.yaml`

```yaml
image_width: 1920
image_height: 1080
camera_name: front_center

camera_matrix:
  rows: 3
  cols: 3
  data: [1500.0, 0.0, 960.0,
         0.0, 1500.0, 540.0,
         0.0, 0.0, 1.0]

distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.0]

rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]

projection_matrix:
  rows: 3
  cols: 4
  data: [1500.0, 0.0, 960.0, 0.0,
         0.0, 1500.0, 540.0, 0.0,
         0.0, 0.0, 1.0, 0.0]
```

## ROS 2 Parameters

### Node Parameters
Each node supports runtime parameter configuration:

```bash
# Set parameters via command line
ros2 param set /calibration/aruco_locator debug_mode true

# Load parameters from file
ros2 param load /calibration/aruco_locator params.yaml

# Get current parameter values
ros2 param get /calibration/aruco_locator aruco_config_file
```

### Parameter Files
```yaml
# params.yaml
/calibration/aruco_locator:
  ros__parameters:
    debug_mode: true
    camera_namespace: "/sensing/camera/front_center"
    detection_frequency: 30.0

/calibration/extrinsic_solver:
  ros__parameters:
    pnp_method: "SQPNP"
    refinement_enabled: true
    min_correspondences: 4
    convergence_threshold: 0.001
```

## Environment Variables

### Build Configuration
```bash
# OpenCV configuration
export OPENCV_PKGCONFIG_NAME=opencv4
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4

# CUDA support (optional)
export CUDA_PATH=/usr/local/cuda
export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Build optimization
export CARGO_BUILD_JOBS=8
export CMAKE_BUILD_TYPE=Release
```

### Runtime Configuration
```bash
# ROS 2 configuration
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# LCTK-specific
export LCTK_CONFIG_DIR=/path/to/config
export LCTK_LOG_LEVEL=INFO
export LCTK_VISUALIZATION_BACKEND=rviz
```

## Advanced Configuration

### Multi-Sensor Setup
```json5
{
  "sensors": {
    "lidar": {
      "front": {
        "topic": "/sensing/lidar/front/pointcloud_raw",
        "frame_id": "lidar_front",
        "roi": {
          "min_x": -10.0, "max_x": 50.0,
          "min_y": -10.0, "max_y": 10.0,
          "min_z": -2.0,  "max_z": 5.0
        }
      },
      "rear": {
        "topic": "/sensing/lidar/rear/pointcloud_raw",
        "frame_id": "lidar_rear"
      }
    },
    "cameras": {
      "front_center": {
        "image_topic": "/sensing/camera/front_center/image_raw",
        "info_topic": "/sensing/camera/front_center/camera_info",
        "frame_id": "camera_front"
      }
    }
  }
}
```

### Calibration Quality Thresholds
```json5
{
  "quality_thresholds": {
    "reprojection_error_max": 2.0,        // pixels
    "detection_consistency_min": 0.8,      // ratio
    "geometric_validation_min": 0.9,       // ratio
    "temporal_stability_min": 0.95,        // ratio
    "convergence_threshold": 0.001,        // parameter change
    "min_detection_count": 50              // frames
  }
}
```

### Performance Tuning
```json5
{
  "performance": {
    "detection_threads": 4,
    "processing_queue_size": 100,
    "gpu_acceleration": true,
    "memory_pool_size": "1GB",
    "cache_size": 1000,
    "parallel_processing": {
      "aruco_detection": true,
      "board_detection": true,
      "synchronization": false
    }
  }
}
```

## Configuration Validation

### Automatic Validation
LCTK automatically validates configurations on startup:
- File existence and permissions
- Parameter value ranges
- Geometric consistency
- Hardware capability checks

### Manual Validation
```bash
# Validate configuration files
ros2 run lctk_tools validate_config --aruco config/aruco_pattern.json5
ros2 run lctk_tools validate_config --board config/board_pattern.json5

# Test sensor connectivity
ros2 run lctk_tools test_sensors --config config/sensors.json5
```

## Configuration Management

### Version Control
- Store configuration files in version control
- Use environment-specific configuration branches
- Document configuration changes in commit messages

### Deployment
- Use configuration templates for different environments
- Validate configurations in CI/CD pipelines
- Provide configuration migration tools for updates

### Backup and Recovery
- Regular backups of working configurations
- Configuration rollback capabilities
- Disaster recovery procedures