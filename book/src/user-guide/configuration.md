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
  // Board geometry — must match the physical plate
  "board_width": "1000mm",        // edge length of the square plate
  "hole_radius": "150mm",
  "hole_center_shift": "200mm",   // hole offset from the centre, along the plate's EDGES

  // RANSAC plane fitting
  "plane_ransac_max_iterations": 2000,    // Increase if board not detected
  "plane_ransac_inlier_threshold": 0.05,  // Meters (5cm tolerance)

  // Sensor convention (seeds the initial pose before ICP)
  "sensor_up_axis": "z",                  // "x" | "y" | "z" — which sensor axis is up
  "initial_inplane_rotation_deg": 0.0,    // leave at 0.0; see below

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

`hole_center_shift` is measured along the plate's **edges**, as it is
stamped on the plate. Because the board model's axes run corner to corner,
the hole centres come out at `hole_center_shift × √2` from the plate
centre — 283 mm for the shipped 200 mm — which is the number you will see
in RViz.

#### `sensor_up_axis` and `initial_inplane_rotation_deg`

These two seed the ICP; they do not tune it.

- **`sensor_up_axis`** names which of the sensor's own axes points up, so
  the detector can work out which way is up in the board plane. Velodyne
  and other Z-up spinning LiDARs use `"z"`; the Seyond Falcon is X-up, so
  its preset uses `"x"`. Getting this wrong gives a 90°-off seed.
- **`initial_inplane_rotation_deg`** is the board's roll *within its own
  plane*, relative to corner-up (diamond) mounting. **`0.0` is correct for
  every rig in this repository — it is not a tuning dial, and sweeping it
  is wasted time.** All three shipped configs (the template, `velodyne`,
  and `seyond`) agree on `0.0`.

  A non-zero value is justified only by a board that is genuinely *not*
  hung corner-up; then set it to that board's roll in degrees. Nothing
  else. Historically the presets carried `45.0` to bridge a mismatch
  between the board model's axes and the physical plate; that mismatch has
  been removed from the model, so the correction is no longer needed.

  Note that ICP cannot rescue a wrong value here. 45° sits exactly halfway
  between two of the square's four 90°-symmetric orientations, and points
  landing on the plate's interior carry no in-plane information at all, so
  there is no gradient to follow — the detector simply publishes nothing.

#### Crop-box-free detection (optional)

By default the detector crops to a bounding box (`detection_mode: "bbox"`).
To detect the board **without** a bounding box, set `detection_mode:
"bbox_free"` and add a `bbox_free` block to the same file:

```json5
{
  // ... RANSAC/ICP keys above ...
  "detection_mode": "bbox_free",   // "bbox" (default) | "bbox_free"
  "bbox_free": {
    // "background_subtraction" (fast, needs warmup) | "plane_strip" (slower, no warmup)
    "foreground_method": "background_subtraction",
    "voxel": 0.05,                 // internal downsample edge (m)
    "board": {                     // board shape/size gates (production operating point)
      "side_m": 1.0,
      "up_axis": [0.0, 0.0, 1.0],
      "cluster_min_points": 30,
      "flatness_rms_max": 0.045,
      "stance_floor": 0.9,
      "isolation": true
      // ... plus side_tol, cell_m, vertical_gap_deg, square_icp_residual_max, isolation_max_density
    },
    "background": {
      "dilation_radius": 1,
      "warmup_frames": 20          // board-FREE frames to observe before detecting
    }
  }
}
```

**`background_subtraction` warmup:** start the node with the scene **empty**
(no board). It observes `warmup_frames` clouds to learn the static
background, then begins detecting. Walk the board in afterward. To
re-learn the background at runtime (e.g. after moving the rig):

```bash
ros2 service call /lidar_board_detector/reset_background std_srvs/srv/Empty
```

> **Note:** the `bbox_free.board` values must be the production operating
> point spelled out explicitly (`flatness_rms_max: 0.045`, `stance_floor:
> 0.9`, `isolation: true`) — the library's own defaults are looser and are
> not the tuned values.

### 3. ArUco Pattern Configuration

**Location:** `config/aruco/aruco_pattern.json5`

Defines the markers on your calibration board:

```json5
{
  "marker_ids": [696, 64, 306, 195],  // x-major order by (x, y) on the sheet
  "dictionary": "DICT_5X5_1000",      // ArUco dictionary used
  "board_size": "500mm",              // printed sheet, including its white margin
  "board_border_size": "10mm",        // white border around the marker grid
  "num_squares_per_side": 2,          // 2x2 grid (the only supported layout)
  "marker_square_size_ratio": 0.8,    // marker size as a fraction of its square
  "border_bits": 1,

  // Where the sheet is glued on the plate: the offset of the PAPER's centre
  // from the PLATE's centre, resolved along the plate's two diagonals.
  "paper_placement": {
    "toward_left_corner": "0mm",
    "toward_top_corner": "-353.5533905932738mm"
  }
}
```

**Must match your physical board** (marker IDs, sizes, and where the sheet
sits on the plate).

This file describes the *printed sheet*; `aruco_detector.json5` describes
how the detector finds it (corner refinement, adaptive thresholding). Only
this one is read by `aruco_generator_node` when printing the pattern.

`paper_placement` is optional; when absent, the code falls back to the
sheet's origin corner sitting on the plate's **bottom** corner. The shipped
values **are** the measured placement of the board this repository
calibrates against — confirmed against the physical hardware: the 500 mm
sheet sits in the plate's lower quarter, with its top corner exactly at the
plate centre. That is why the up-diagonal offset equals
`(paper_size - board_width) / sqrt(2)`; the arithmetic reproduces the
measurement rather than substituting for it. Do not "correct" these toward a
centred sheet — that moves every marker corner and breaks the camera solve.

If your board is built differently, measure yours and state it here.

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
