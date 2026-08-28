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

### 2. Calibration Target Configuration

Calibration configuration is split across two files with different jobs — do not mix them:

- **Target Definition** (`config/targets/<target>.json5`) — the physical truth: plate geometry,
  cutout layout, fiducial (ArUco) marker IDs and placement. This is what you edit if you build a
  *different physical board*.
- **Detector Tuning** (`config/board/<target>/<sensor>.json5`) — sensor-specific, geometry-free
  parameters: RANSAC/ICP knobs, the sensor's up-axis convention, and (for `bbox` mode only) which
  crop box to use. This is what you edit if detection fails on a given sensor but the physical
  board hasn't changed.

#### Target Definition

**Location:** `config/targets/hollow_1000_aruco_4_v1.json5` (the shipped 1000 mm perforated
target; `config/targets/solid_600_aruco_1_v1.json5` is the shipped 600 mm solid target)

```json5
{
  schema_version: 1,
  target_id: "hollow_1000_aruco_4",
  revision: 1,
  board_frame_convention: "corner_aligned_plate_center_v1",

  plate: {
    side: "1000mm",               // edge length of the square plate
    surface: {
      kind: "perforated",
      circular_cutouts: [
        // cutout centre (x, y from the plate centre) and radius
        { center: { x: "282.842712mm", y: "0mm" }, radius: "150mm" },
        { center: { x: "0mm", y: "282.842712mm" }, radius: "150mm" },
        { center: { x: "-282.842712mm", y: "0mm" }, radius: "150mm" },
      ],
    },
  },

  fiducial: {
    kind: "square_aruco_grid",
    dictionary: "DICT_5X5_1000",
    marker_ids: [696, 64, 306, 195],
    paper_side: "500mm",
    paper_center: {
      toward_left_corner: "0mm",
      toward_top_corner: "-353.553391mm",
    },
    outer_border: "10mm",
    cells_per_side: 2,
    marker_fill_ratio: 0.8,
    border_bits: 1,
  },

  lidar_orientation_reference: {
    kind: "asymmetric_cutouts",
  },
}
```

**Must match your physical board.** The cutout centres are measured along the plate's
**diagonals** here (unlike the older convention this superseded, which stated a single symmetric
shift along the plate's edges); each `circular_cutouts` entry is independent, so an asymmetric
board is representable too. `fiducial.paper_center` is the offset of the printed sheet's centre
from the plate's centre, resolved the same way. This file is read both by the detector (via
`target_config`) and by `aruco_generator_node` when printing the pattern.

#### Detector Tuning

**Location:** `config/board/hollow_1000/velodyne.json5` (per-target, per-sensor; e.g.
`config/board/hollow_1000/seyond.json5`, `config/board/solid_600/velodyne.json5`)

Adjust these if detection fails on a given sensor. Geometry keys (`board_width`, `hole_radius`,
`hole_center_shift`, `side_m`, …) do **not** belong here any more — they were removed from
Detector Tuning entirely and now live only in the Target Definition above:

```json5
{
  // RANSAC plane fitting
  "plane_ransac_max_iterations": 2000,    // Increase if board not detected
  "plane_ransac_inlier_threshold": 0.05,  // Meters (5cm tolerance)

  // Sensor convention (seeds the initial pose before ICP)
  "sensor_up_axis": "z",                  // "x" | "y" | "z" — which sensor axis is up
  "initial_inplane_rotation_deg": 0.0,    // leave at 0.0; see below

  // ICP pose refinement (values as shipped in hollow_1000/velodyne.json5)
  "max_icp_iterations": 100,              // Seyond preset ships 50
  "icp_rejection_threshold": 0.005,       // Max accepted per-iteration ICP loss

  "detection_mode": "bbox_free"           // "bbox" (Rust-level default) | "bbox_free" (shipped default)
}
```

**When to adjust:**
- **Board too far/close (bbox mode only):** Change the referenced `bbox_config`'s `pose` and
  `size_xyz` (see Bounding Box below)
- **Noisy point clouds:** Increase `plane_ransac_max_iterations`
- **False detections:** Decrease `plane_ransac_inlier_threshold`

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

At the Rust type level, an omitted `detection_mode` defaults to `"bbox"`; in practice every
shipped Detector Tuning preset selects `"bbox_free"` except
`config/board/hollow_1000/velodyne_bbox.json5`, the one bbox-mode preset. Crop-box-free keys are
**flat, top-level fields in the same tuning file** (not a nested `bbox_free` block) — geometry
keys like `side_m` are no longer accepted here at all; board shape comes from the Target
Definition instead. This mirrors the shipped
`config/board/hollow_1000/velodyne.json5`:

```json5
{
  // ... RANSAC/ICP keys above ...
  "detection_mode": "bbox_free",        // "bbox" (type-level default) | "bbox_free" (shipped default)

  // "background_subtraction" (fast, needs warmup) | "plane_strip" (slower, no warmup)
  "foreground_method": "background_subtraction",
  "bbf_voxel": 0.05,                    // internal downsample edge (m)
  "bg_dilation_radius": 1,
  "bg_warmup_frames": 20,               // board-FREE frames to observe before detecting

  // Board shape/size gates (production operating point)
  "up_axis": [0.0, 0.0, 1.0],
  "cluster_min_points": 20,
  "flatness_rms_max": 0.045,
  "stance_floor": 0.9,
  "isolation": true
  // ... plus side_tol, cell_m, vertical_gap_deg, square_icp_residual_max, isolation_max_density
}
```

**`background_subtraction` warmup:** start the node with the scene **empty**
(no board). It observes `bg_warmup_frames` clouds to learn the static
background, then begins detecting. Walk the board in afterward. To
re-learn the background at runtime (e.g. after moving the rig):

```bash
ros2 service call /lidar_board_detector/reset_background std_srvs/srv/Empty
```

> **Note:** these board-shape-gate values must be the production operating
> point spelled out explicitly (`flatness_rms_max: 0.045`, `stance_floor:
> 0.9`, `isolation: true`) — the library's own defaults are looser and are
> not the tuned values.

### 3. ArUco Detector Tuning

**Location:** `config/aruco/aruco_detector.json5`

The printed sheet's marker IDs, dictionary, size, and placement on the plate
(`fiducial.*` and `fiducial.paper_center`) are part of the **Target
Definition** — see the `fiducial` block under Target Definition above, not
this file. This file is purely about how the *detector* finds markers that
are already printed: corner refinement and adaptive thresholding. It has no
business describing a piece of paper, so it carries no geometry.

```json5
{
    "corner_refinement": {
        // NONE | SUBPIX | CONTOUR | APRILTAG
        "method": "SUBPIX",
        "win_size": 5,          // half-width of the SUBPIX search window, in pixels
        "max_iterations": 30,
        "min_accuracy": 0.01,
    },

    // Adaptive-threshold sweep used to find marker candidates.
    "adaptive_thresh": {
        "win_size_min": 13,
        "win_size_max": 33,
        "win_size_step": 10,
    },
}
```

Optional per-marker in the calibration config's `aruco_detector_config` key; when omitted it
defaults to this same file.

**Must match your physical board:** the marker IDs, sheet size, and where the sheet sits on the
plate are stated in the Target Definition, not here. If your board is built differently, measure
yours and state it in `config/targets/<your-target>.json5`.

### 4. Bounding Box (ROI) Configuration

**Location:** `config/board/bbox.json5` — only used when a Detector Tuning preset selects
`detection_mode: "bbox"` (referenced via that preset's `bbox_config`)

Defines where to look for the calibration board:

```json5
{
  "pose": {
    "translation": [2.0, 0.0, 0.0],        // [x, y, z] in meters from sensor
    "rotation": [0.0, 0.0, 0.0, 1.0]       // quaternion [x, y, z, w]; identity = no tilt
  },
  "size_xyz": [4.0, 4.0, 2.0]              // [width, depth, height] in meters
}
```

Visualize the bounding box in RViz (debug mode) to ensure it covers the board.

## Common Configuration Tasks

### Change Detection ROI

If the board is in a different location, when using a `bbox`-mode Detector Tuning preset:

1. Edit the referenced `bbox_config` file (e.g. `config/board/bbox.json5`)
2. Set `pose.translation` to the board's approximate position, and `pose.rotation` if the box
   needs to be tilted
3. Set `size_xyz` large enough to cover board movement
4. Restart calibration

### Enable Debug Visualization

```bash
just debug_mode=true calibrate /path/to/your_config.yaml
```

Debug topics show intermediate detection steps in RViz.

### Adjust Detection Sensitivity

**Board not detected:**
- Increase `plane_ransac_max_iterations` (e.g., 5000)
- Increase `size_xyz` in the `bbox_config` file to search a wider area (bbox mode only)
- Check that board is in sensor range (3-8 meters works best)

**Too many false detections:**
- Decrease `plane_ransac_inlier_threshold` (e.g., 0.03)
- Tighten `size_xyz` in the `bbox_config` file to focus on the expected area (bbox mode only)

### Use Custom Data Files

Calibration is config-driven: point a YAML config's device topics at your own data, then run it
against your recording.

```bash
# Terminal 1: play your recording
ros2 bag play /path/to/your_data.bag

# Terminal 2: run the config-driven pipeline
just calibrate /path/to/your_config.yaml
```

## Configuration File Locations

| File | Purpose | Required For |
|------|---------|-------------|
| `camera_info.yaml` | Camera intrinsics | LiDAR-Camera |
| `config/targets/<target>.json5` | Target Definition: physical plate/cutout/fiducial geometry | All calibrations |
| `config/board/<target>/<sensor>.json5` | Detector Tuning: sensor-specific ICP/RANSAC params | All calibrations |
| `config/aruco/aruco_detector.json5` | ArUco detector tuning (corner refinement, threshold) | LiDAR-Camera |
| `bbox.json5` | Detection region (only when `detection_mode: "bbox"`) | bbox-mode calibrations |

## Advanced: Multi-LiDAR Configuration

> **Note:** `config/multi_wayside.yaml` (referenced here previously) does not exist; the
> multi-LiDAR / LiDAR-to-LiDAR configuration surface is under active revision in a parallel
> work item as of this writing (see `docs/issues/` for the current M-1x/M-2x tracker entries on
> `lidar_to_lidar_solver`). Not corrected further here to avoid documenting a moving target —
> check `ros/lidar_to_lidar_solver/README.md` and `ros/lctk_launch/launch/calibrate.launch.py`
> for the current state.

`same_face_mode` (both LiDARs see the same side of the board vs. opposite sides, applying a 180°
correction) is a `lidar_to_lidar_solver` ROS parameter, not a standalone config file.

## Next Steps

- Test configuration with [Quick Start](./quickstart.md)
- Troubleshoot issues: [Troubleshooting](./troubleshooting.md)
- For developers: [Architecture](../developer-guide/architecture.md)
