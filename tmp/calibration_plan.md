# Calibration Plan — LiDAR × Camera (Seyond + Left Camera)

**Date:** 2026-06-05  
**Rosbag:** currently playing

---

## Detected Topics

| Topic | Type | Frame ID | Rate |
|-------|------|----------|------|
| `/iv_points` | `sensor_msgs/PointCloud2` | `seyond` | ~10 Hz |
| `/camera/left/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_left` | ~30 Hz |
| `/camera/left/camera_info` | `sensor_msgs/CameraInfo` | `camera_left` | ~30 Hz |

**Camera intrinsics:** already calibrated (rational_polynomial, 1920×1280, fx=986, fy=1007)

---

## Files Created / Modified

| File | Status | Notes |
|------|--------|-------|
| `ros/lctk_launch/config/examples/seyond_left.yaml` | **Created** | Calibration config for this rosbag |
| `ros/lctk_launch/config/rviz/calibration.rviz` | **Modified** | Updated topics + fixed frame for seyond setup |
| `ros/lctk_launch/config/board/bbox.json5` | **Modified** | Adjusted to board position in seyond frame |
| `ros/lctk_launch/launch/calibrate.launch.py` | **Modified** | Added `plane_inliers` + `extrinsic_transform` remappings to overlay node (supports multi-lidar) |
| `ros/pointcloud_image_overlay/pointcloud_image_overlay/overlay_node.py` | **Modified** | Changed two hardcoded absolute topics to relative (`plane_inliers`, `extrinsic_transform`) |
| `rust/hollow-board-detector/src/config.rs` | **Modified** | Added `SensorUpAxis` enum + `sensor_up_axis` field to Config |
| `ros/lidar_board_detector/src/main.rs` | **Modified** | Rewrote `compute_initial_pose_from_plane` to use `sensor_up_axis` for correct initial board pose |
| `ros/lctk_launch/config/board/board_detector.json5` | **Modified** | Added `"sensor_up_axis": "x"` for Seyond (X=up, Z=forward); `initial_inplane_rotation_deg: 45.0` |
| `ros/aruco_locator_node/src/main.rs` | **Modified** | Fixed double undistortion: pass original image to detect_markers (undistorts internally); undistort separately for display overlay |

---

## Config File

`ros/lctk_launch/config/examples/seyond_left.yaml`:

```yaml
devices:
  lidars:
    seyond_lidar:
      pointcloud_topic: /iv_points
      frame_id: seyond

  cameras:
    left_camera:
      image_topic: /camera/left/image_raw        # requires decompressed topic
      frame_id: camera_left

markers:
  calibration_board:
    type: hollow_board
    board_config: $(find-pkg-share lctk_launch)/config/board/board_detector.json5
    aruco_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_pattern.json5
    bbox_config: $(find-pkg-share lctk_launch)/config/board/bbox.json5
    pairs:
      - [seyond_lidar, left_camera]
```

Generated topic namespaces (from config_parser.py naming rules):
- Board detector: `/calibration/seyond_lidar_calibration_board/debug/*`
- ArUco locator: `/calibration/left_camera/image_with_detections`
- Solver output: `/calibration/seyond_lidar_left_camera/extrinsic_transform`

---

## Step-by-Step Procedure

### 1. Decompress image (required — aruco_locator_node needs raw Image)

Run in a dedicated terminal before launching calibration:
```bash
ros2 run image_transport republish compressed raw \
    --ros-args \
    -r in/compressed:=/camera/left/image_raw/compressed \
    -r out:=/camera/left/image_raw
```

### 2. Ensure rosbag is looping
```bash
ros2 bag play <bagfile> --loop
```

### 3. Build (if not already built)
```bash
cd ~/LCTK && just build
```

### 4. Launch calibration pipeline
```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=$(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/seyond_left.yaml
```
Use `mode=offline` (default) — RELIABLE QoS, infinite sync window, no dropping.

### 5. Verify detections
```bash
ros2 topic list | grep -E "aruco|board|detection"
ros2 topic hz /calibration/left_camera/aruco_detections
ros2 topic hz /calibration/seyond_lidar_calibration_board/calibration_board_detections
```
Both must publish. Solver produces no output until both streams arrive.

### 6. Launch RViz to visually verify
```bash
just rviz
```
Check: board outline aligns with LiDAR points, ArUco detection image shows markers.

### 7. Collect calibration poses
The `extrinsic_solver_node` auto-publishes transforms per detection pair.
Move board to **5–10 different poses** (vary distance 1–4 m, tilt, rotation).
```bash
ros2 topic echo /calibration/seyond_lidar_left_camera/extrinsic_transform
```

### 8. (Optional) Advanced solver for multi-pose averaging
```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=.../seyond_left.yaml use_advanced_solver:=true
ros2 run interactive_solver_controller interactive_solver_controller
# Space = add pose, p = save to ~/detections.json
```

### 9. Save result
```bash
ros2 topic echo /calibration/seyond_lidar_left_camera/extrinsic_transform --once
# Advanced solver: press `p` in TUI → saves ~/detections.json
```

---

## Bbox Configuration

**Current** (`config/board/bbox.json5`): center `[0,0,0]`, size `20×20×10 m` — intentionally large to cover board regardless of sensor offset.

**After confirming detections work**, tighten to reduce clutter:
```json5
{
    "pose": {
        "translation": [x, y, z],   // observed board position in seyond frame
        "rotation": [1.0, 0.0, 0.0, 0.0]
    },
    "size_xyz": [4.0, 4.0, 3.0]
}
```

---

## RViz Topic Mapping

| Display | Topic |
|---------|-------|
| ArUco detection image | `/calibration/left_camera/image_with_detections` |
| Bounding box marker | `/calibration/seyond_lidar_calibration_board/debug/bbox_marker` |
| Filtered points | `/calibration/seyond_lidar_calibration_board/debug/filtered_points` |
| RANSAC inliers | `/calibration/seyond_lidar_calibration_board/debug/plane_inliers` |
| Final board pose | `/calibration/seyond_lidar_calibration_board/debug/final_board_pose` |
| Point cloud overlay | `/calibration/pointcloud_overlay` (hardcoded) |
| Fixed Frame | `seyond` |

---

## Root Cause Analysis — ICP Failure (Seyond Axis Convention)

**Symptom:** `calibration_board_detections` always empty; ICP loss plateaus at ~0.043 (threshold 0.012).

**Diagnosis chain:**
1. `/calibration/icp_debug/stats` showed loss starting at 0.25 → converging to 0.043 across 50 iters — classic local minimum
2. `final_board_pose` markers were empty → `detect_board()` returning None
3. `filtered_points` had 11,897 pts — sufficient; `plane_inliers` publishing at 2Hz — RANSAC finding a plane
4. Root cause: `compute_initial_pose_from_plane` used hardcoded XY projection for in-plane rotation:
   ```rust
   // OLD — breaks when plane normal has near-zero XY components
   let planar_plane_normal = na::Vector3::new(plane_normal.x, plane_normal.y, 0.0);
   ```
   Seyond has **X=up, Z=forward**. Board at `[0, -0.7, 3.5]` → plane normal ≈ `[0, 0, -1]` → XY projection ≈ `[0, 0, 0]` → degenerate → wrong initial pose → ICP local minimum.

**Fix applied:**
- Added `SensorUpAxis` enum to `rust/hollow-board-detector/src/config.rs`
- Rewrote initial pose computation to use configurable up-axis:
  - board Z → plane_normal (toward sensor)
  - board Y → world "up" projected onto board plane
  - board X → cross(Y, Z)
- Added `"sensor_up_axis": "x"` to `board_detector.json5`
- Works for any sensor convention; default `"z"` preserves old behavior for Z-up sensors

**Other issues found and fixed:**
- `pointcloud_overlay` subscribed to hardcoded stale topics (`/calibration/lidar_board_detector/...`, `/calibration/extrinsic_solver/...`) → fixed to relative topics + launch remappings (supports multi-lidar)
- Image topic is compressed → requires `image_transport republish` before launching

---

## Root Cause Analysis — ICP Board 45° Rotation

**Symptom:** Board detection publishes but board marker is 45° rotated relative to physical board.

**Root cause:** Old code had `-FRAC_PI_4` baked into the lifting rotation. New code (sensor_up_axis fix) removed that implicit offset. Without it, the geometric initial pose is off by 45°.

**Fix applied:** Added `"initial_inplane_rotation_deg": 45.0` to `board_detector.json5` and added code in `lidar_board_detector/src/main.rs` `compute_initial_pose_from_plane` to apply this extra in-plane rotation around the board normal before ICP. Built 2026-06-11 16:37.

**Pending:** Relaunch + visually confirm in RViz that board marker aligns with LiDAR points. If still off, try `-45.0` or `90.0`.

---

## Root Cause Analysis — ArUco Detection Displacement (Fixed)

**Symptom:** Detected corner markers visibly displaced from actual ArUco marker squares in `image_with_detections` RViz panel.

**Root cause: Double undistortion.**

Call chain before fix:
1. `process_image()` in `aruco_locator_node/src/main.rs`:
   - `undistort_image(&mat)` → `processed_mat` (1× undistorted)
   - `detector.detect_markers(&processed_mat)` → calls `aruco_locator::ArucoDetector::detect_markers()`
     → calls `multi_aruco::MultiArucoDetector::detect_markers()` at `rust/aruco-detector/src/multi_aruco.rs:366`
     → **undistorts internally again** → canvas (2× undistorted) → corners in 2× undistorted pixel space
2. `create_overlay_image()` draws those corners on `processed_mat` (1× undistorted) → **DISPLACEMENT**

**Fix applied** (`ros/aruco_locator_node/src/main.rs`):
- Pass ORIGINAL distorted `mat` (not pre-undistorted) to `detector.detect_markers()`
- Undistort separately once for display overlay
- Both paths now apply exactly one undistortion → corners align with display image

```rust
// detect_markers() undistorts internally — pass original distorted image to avoid
// double undistortion
let detection_result = detector.detect_markers(&mat)?;
let undistorted_for_display = Self::undistort_image(&mat, calibration)?;
Ok((detection_result, undistorted_for_display))
```

**Files changed:** `ros/aruco_locator_node/src/main.rs` (process_image function)

---

## Known Issues / Watchpoints

| Issue | Likely Cause | Fix |
|-------|-------------|-----|
| `aruco_locator_node` no detections | Node receives no `/camera/left/image_raw` | Ensure `image_transport republish` is running |
| Board detections empty, ICP loss ~0.043 | Wrong initial pose from incorrect up-axis | Fixed: `sensor_up_axis: "x"` in board_detector.json5 |
| Board marker 45° rotated in RViz | Missing in-plane offset after axis fix | Fixed: `initial_inplane_rotation_deg: 45.0`; if wrong try -45.0 or 90.0 |
| Solver not publishing | Only one of the two detection streams arriving | Check both topics with `ros2 topic hz` |
| ICP loss too high after fix | Board geometry mismatch or threshold too tight | Verify board_width/hole_radius match physical board; adjust icp_good_fit_threshold |
| Overlay shows camera but no LiDAR points | Stale hardcoded topic names | Fixed: relative topics + launch remappings |
| ArUco corners displaced in RViz overlay | Double undistortion: pre-undistort in process_image + internal undistort in detect_markers | Fixed: pass original mat to detect_markers, undistort separately for display |

---

## Quick Reference

```bash
# Check detections
ros2 topic hz /calibration/left_camera/aruco_detections
ros2 topic hz /calibration/seyond_lidar_calibration_board/calibration_board_detections

# Kill stale ROS daemon
pkill -9 -f ros2-daemon

# Raw topic rates
ros2 topic hz /iv_points
ros2 topic hz /camera/left/image_raw/compressed
```
