# Configuration

LCTK uses configuration files in the `config/` directory. Most users only need to modify a few key settings.

## Essential Configuration Files

### 1. Camera Intrinsics (Required for LiDAR-Camera)

**Location:** `config/camera/front_center_camera_info.yaml`

Get this file from calibrating your camera (using `camera_calibration` or similar tool):

```yaml
image_width: 1920
image_height: 1080
camera_matrix:
  data: [fx, 0, cx,
         0, fy, cy,
         0, 0, 1]
distortion_coefficients:
  data: [k1, k2, p1, p2, k3]
```

**Key parameters:**
- `fx, fy`: Focal length (pixels)
- `cx, cy`: Principal point (usually image center)
- `k1-k3, p1-p2`: Distortion coefficients

### 2. Board Detector Configuration

**Location:** `config/board/board_detector.json5`

Adjust these if detection fails:

```json5
{
  // RANSAC plane fitting
  "plane_ransac_max_iterations": 2000,    // Increase if board not detected
  "plane_ransac_inlier_threshold": 0.05,  // Meters (5cm tolerance)

  // ICP pose refinement
  "max_icp_iterations": 10,               // Usually sufficient
  "icp_rejection_threshold": 0.030,       // Meters (3cm outlier threshold)

  // Bounding box (ROI filter)
  "bbox_center": [2.0, 0.0, 0.0],        // Meters from sensor
  "bbox_size": [4.0, 4.0, 2.0]           // Width, depth, height
}
```

**When to adjust:**
- **Board too far/close:** Change `bbox_center` and `bbox_size`
- **Noisy point clouds:** Increase `plane_ransac_max_iterations`
- **False detections:** Decrease `plane_ransac_inlier_threshold`

### 3. ArUco Pattern Configuration

**Location:** `config/aruco/aruco_pattern.json5`

Defines the markers on your calibration board:

```json5
{
  "dictionary": "DICT_5X5_1000",    // ArUco dictionary used
  "marker_size": 0.05,              // Physical marker size (meters)
  "markers": [
    {"id": 696, "position": [-0.2, -0.2]},  // Bottom-left corner
    {"id": 64,  "position": [ 0.2, -0.2]},  // Bottom-right corner
    {"id": 306, "position": [-0.2,  0.2]},  // Top-left corner
    {"id": 195, "position": [ 0.2,  0.2]}   // Top-right corner
  ]
}
```

**Must match your physical board** (marker IDs and positions).

### 4. Bounding Box (ROI) Configuration

**Location:** `config/board/bbox.json5`

Defines where to look for the calibration board:

```json5
{
  "center": [2.0, 0.0, 0.0],  // [x, y, z] in meters from sensor
  "size": [4.0, 4.0, 2.0]     // [width, depth, height] in meters
}
```

Visualize the bounding box in RViz (debug mode) to ensure it covers the board.

## Common Configuration Tasks

### Change Detection ROI

If the board is in a different location:

1. Edit `config/board/bbox.json5`
2. Set `center` to board's approximate position
3. Set `size` large enough to cover board movement
4. Restart calibration

### Enable Debug Visualization

```bash
ros2 launch lctk_launch lidar_camera_calibration.launch.xml debug_mode:=true
```

Debug topics show intermediate detection steps in RViz.

### Adjust Detection Sensitivity

**Board not detected:**
- Increase `plane_ransac_max_iterations` (e.g., 5000)
- Increase `bbox_size` to search wider area
- Check that board is in sensor range (3-8 meters works best)

**Too many false detections:**
- Decrease `plane_ransac_inlier_threshold` (e.g., 0.03)
- Tighten `bbox_size` to focus on expected area

### Use Custom Data Files

```bash
ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
    pcap_file:=/path/to/your/lidar.pcap \
    video_file:=/path/to/your/camera.mp4
```

## Configuration File Locations

| File | Purpose | Required For |
|------|---------|-------------|
| `camera_info.yaml` | Camera intrinsics | LiDAR-Camera |
| `board_detector.json5` | Board detection params | All calibrations |
| `aruco_pattern.json5` | ArUco marker layout | LiDAR-Camera |
| `bbox.json5` | Detection region | All calibrations |
| `multi_wayside.yaml` | Multi-LiDAR settings | Multi-LiDAR |

## Advanced: Multi-LiDAR Configuration

**Location:** `config/multi_wayside.yaml`

Key parameter:
```yaml
same_face_mode: true  # Both LiDARs see same side of board
```

Set to `false` if LiDARs see opposite sides (applies 180° correction).

## Next Steps

- Test configuration with [Quick Start](./quickstart.md)
- Troubleshoot issues: [Troubleshooting](./troubleshooting.md)
- For developers: [Architecture](../developer-guide/architecture.md)
