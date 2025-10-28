# Reference

Quick reference for configuration schemas, API documentation, and command-line tools.

## API Documentation

**Generate rustdoc:**
```bash
cargo doc --open --no-deps
```

**Browse documentation:**
- Core libraries: `target/doc/aruco_detector/index.html`
- All crates: `target/doc/index.html`

**Online documentation** (if published): `https://docs.rs/lctk`

## Configuration File Schemas

### Board Detector Configuration

**File:** `config/board/board_detector.json5`

```json5
{
  // RANSAC plane detection
  "plane_ransac_max_iterations": 2000,        // int, iterations
  "plane_ransac_inlier_threshold": 0.05,      // float, meters
  "plane_ransac_min_inlier_ratio": 0.5,       // float, 0.0-1.0

  // ICP pose refinement
  "max_icp_iterations": 10,                   // int, iterations
  "icp_convergence_threshold": 1e-6,          // float, meters
  "icp_rejection_threshold": 0.030,           // float, meters
  "icp_pose_weight_threshold": 1e-10,         // float

  // Bounding box filter
  "bbox_center": [2.0, 0.0, 0.0],            // [x, y, z] meters
  "bbox_size": [4.0, 4.0, 2.0],              // [width, depth, height] meters

  // Board geometry
  "board_width": 1.0,                         // float, meters
  "board_height": 1.0,                        // float, meters
  "board_thickness": 0.02,                    // float, meters
  "hole_radius": 0.15,                        // float, meters
  "hole_center_shift": 0.2                    // float, meters from corner
}
```

### ArUco Pattern Configuration

**File:** `config/aruco/aruco_pattern.json5`

```json5
{
  "dictionary": "DICT_5X5_1000",              // string, ArUco dictionary
  "marker_size": 0.05,                        // float, meters

  "markers": [
    {"id": 696, "position": [-0.2, -0.2]},    // bottom-left
    {"id": 64,  "position": [ 0.2, -0.2]},    // bottom-right
    {"id": 306, "position": [-0.2,  0.2]},    // top-left
    {"id": 195, "position": [ 0.2,  0.2]}     // top-right
  ],

  // Detection parameters (optional)
  "detection": {
    "corner_refinement": "CORNER_REFINE_SUBPIX",  // string
    "adaptive_threshold_window_size": 23,         // int, pixels
    "adaptive_threshold_constant": 7,             // int
    "min_marker_perimeter_rate": 0.03,            // float, 0.0-1.0
    "max_marker_perimeter_rate": 4.0              // float
  }
}
```

### Bounding Box Configuration

**File:** `config/board/bbox.json5`

```json5
{
  "center": [2.0, 0.0, 0.0],  // [x, y, z] meters from sensor
  "size": [4.0, 4.0, 2.0]     // [width, depth, height] meters
}
```

### Multi-Wayside Configuration

**File:** `config/multi_wayside.yaml`

```yaml
# Calibration mode
same_face_mode: true           # bool, both LiDARs see same board side
apply_bug_fix: false           # bool, VLP16 coordinate correction

# Synchronization
sync_tolerance_ms: 100         # int, milliseconds
max_queue_size: 100            # int, message buffer size
min_detections_for_calibration: 5  # int, minimum pairs

# ROI filtering
min_range: 0.5                 # float, meters
max_range: 50.0                # float, meters
roi_box_size: [4.0, 4.0, 2.0]  # [x, y, z] meters
roi_box_center: [2.0, 0.0, 0.0]  # [x, y, z] meters
```

### Camera Calibration

**File:** `config/camera/<camera_name>_camera_info.yaml`

```yaml
image_width: 1920
image_height: 1080
camera_name: front_center

camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx,
         0, fy, cy,
         0, 0, 1]

distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]

rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0,
         0, 1, 0,
         0, 0, 1]

projection_matrix:
  rows: 3
  cols: 4
  data: [fx, 0, cx, 0,
         0, fy, cy, 0,
         0, 0, 1, 0]
```

## ROS 2 Message Types

### Detection Messages

**ArUco Detection:**
```
vision_msgs/Detection2DArray
├── header (std_msgs/Header)
└── detections[] (vision_msgs/Detection2D)
    ├── id (string) - marker ID
    ├── bbox (vision_msgs/BoundingBox2D)
    └── results[] (vision_msgs/ObjectHypothesisWithPose)
```

**Board Detection:**
```
vision_msgs/Detection3DArray
├── header (std_msgs/Header)
└── detections[] (vision_msgs/Detection3D)
    ├── id (string)
    ├── bbox (vision_msgs/BoundingBox3D)
    └── results[] (vision_msgs/ObjectHypothesisWithPose)
        └── pose (geometry_msgs/PoseWithCovariance)
```

**Calibration Transform:**
```
geometry_msgs/TransformStamped
├── header (std_msgs/Header)
├── child_frame_id (string)
└── transform (geometry_msgs/Transform)
    ├── translation (geometry_msgs/Vector3)
    └── rotation (geometry_msgs/Quaternion)
```

## Command-Line Tools

### Calibration Launchers

```bash
# LiDAR-Camera calibration
make launch_lidar_camera_calibration

# Multi-LiDAR calibration
make launch_two_lidar_calibration

# Sample data playback
make launch_lidar_camera_sample_data
```

### ROS 2 Services

**Multi-Wayside Node:**
```bash
# Trigger calibration
ros2 service call /trigger_calibration std_srvs/srv/Trigger

# Reset calibration
ros2 service call /reset_calibration std_srvs/srv/Trigger

# Set ROI bounds
ros2 service call /set_roi_bounds lctk_interfaces/srv/SetROI \
    "min_x: -5.0, max_x: 5.0, min_y: -5.0, max_y: 5.0, min_z: 0.0, max_z: 3.0"
```

**Detection Services:**
```bash
# Update bounding box (lidar_board_detector)
ros2 service call /update_bbox lctk_interfaces/srv/SetBBox \
    "center: [3.0, 0.0, 0.5], size: [2.0, 2.0, 1.0]"
```

### Debugging Commands

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /calibration_transform

# Echo topic (first message)
ros2 topic echo /calibration_transform --once

# Check topic rate
ros2 topic hz /aruco_detections

# Check topic bandwidth
ros2 topic bw /sensing/lidar/top/pointcloud_raw

# List nodes
ros2 node list

# Node info
ros2 node info /calibration/aruco_locator

# View TF tree
ros2 run tf2_tools view_frames
```

### Performance Monitoring

```bash
# CPU and memory usage
htop

# GPU usage (if CUDA enabled)
nvidia-smi

# ROS 2 daemon status
ros2 daemon status

# Kill hung daemon
pkill -9 -f ros2-daemon

# Check disk space
df -h
```

## Environment Variables

### Build Configuration

```bash
export OPENCV_PKGCONFIG_NAME=opencv4
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
export CUDA_PATH=/usr/local/cuda
export CARGO_BUILD_JOBS=$(nproc)
```

### Runtime Configuration

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONE_DDS_URI=file://$PWD/config/dds/cyclone_local.xml
export RUST_LOG=debug
export RCUTILS_LOGGING_LEVEL=DEBUG
```

## DDS Configuration

**File:** `config/dds/cyclone_local.xml`

Restricts DDS to localhost for security:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
```

## File Locations

| Type | Location | Description |
|------|----------|-------------|
| Core libraries | `src/lib/` | Pure Rust, no ROS |
| ROS nodes | `src/bin/`, `src/ros2/` | ROS 2 executables |
| Interfaces | `src/interface/` | Custom messages |
| Launch files | `src/ros2/lctk_launch/launch/` | Workflow definitions |
| Config files | `config/` | Configuration files |
| Sample data | `data/sampledata/` | Test datasets |
| Build output | `install/` | Installed packages |

## Known Issues

See `CLAUDE.md` for detailed list. Key issues:

- **45° tilt in pointcloud overlay**: Corner ordering mismatch (under investigation)
- **DDS discovery timing**: Services may timeout 5-10s after launch (use `ros2 service wait`)
- **empy version**: ROS 2 Humble requires empy 3.3.4 (system package, not pip)

## Further Reading

- **Rust API**: `cargo doc --open`
- **ROS 2 Docs**: https://docs.ros.org/en/humble/
- **OpenCV Rust**: https://docs.rs/opencv/
- **rclrs**: https://github.com/ros2-rust/ros2_rust

## Next Steps

- [Architecture](./architecture.md) - System design
- [Advanced Topics](./advanced-topics.md) - Performance tuning
- [Contributing](./contributing.md) - Submit improvements
