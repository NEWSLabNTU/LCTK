# Calibration Plan — LiDAR × LiDAR (VLP32 + Seyond Falcon)

**Date:** 2026-06-12  
**Rosbag:** currently playing

---

## Detected Topics

| Topic | Type | Frame ID | Rate |
|-------|------|----------|------|
| `/lidar/vlp32/velodyne_points` | `sensor_msgs/PointCloud2` | `velodyne` | ~10 Hz |
| `/lidar/falcon/iv_points` | `sensor_msgs/PointCloud2` | `seyond` | ~10 Hz |

---

## Files Created / Modified

| File | Status | Notes |
|------|--------|-------|
| `ros/lctk_launch/config/examples/two_lidar.yaml` | **Exists** | Config for this calibration (created previously) |
| `ros/lctk_launch/config/board/board_detector_vlp32.json5` | **Fixed** | `sensor_up_axis` changed `"x"` → `"z"`; `icp_min_inlier_points` 1000 → 100 (VLP32 sparse at range) |
| `ros/lctk_launch/config/board/bbox_2_lidar_vlp32.json5` | **Exists** | Bbox for VLP32 frame: center `[9.6, 1.1, -0.5]`, size `[1.0, 1.6, 1.6]` |
| `ros/lctk_launch/config/board/bbox_2_lidar_seyond.json5` | **Exists** | Bbox for Seyond frame: center `[-0.7, -0.7, 7.6]`, size `[1.8, 2.0, 1.0]` |
| `ros/lctk_launch/config/rviz/two_lidar_calibration.rviz` | **Exists** | RViz config; fixed frame = `velodyne` |
| `ros/lidar_board_detector/src/main.rs` | **Fixed** | ICP debug topics changed from absolute `/calibration/icp_debug/*` to relative `debug/icp/*` (namespace-scoped); `debug/icp_stats` now also publishes on ICP failure |
| `ros/lidar_to_lidar_solver/lidar_to_lidar_solver/main.py` | **Fixed** | (1) Stale check guarded by `max_message_age_ms > 0`; (2) `_handle_sync_group` uses `group.get()` + substring fallback — resolved Conflux KeyError where SyncGroup keys don't match literal topic strings |
| `ros/lctk_launch/launch/calibrate.launch.py` | **Temp workaround** | Lidar-lidar solver hardcoded to `sync_tolerance_ms=0` (infinite), `sync_queue_size=100`, `max_message_age_ms=0` — bypasses rosbag QoS+timestamp issues. Needs proper solution (rosbag RELIABLE override or mode-aware solver sync params) |

---

## Config File

`ros/lctk_launch/config/examples/two_lidar.yaml`:

```yaml
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /lidar/vlp32/velodyne_points
      frame_id: velodyne
      board_config: $(find-pkg-share lctk_launch)/config/board/board_detector_vlp32.json5
      bbox_config: $(find-pkg-share lctk_launch)/config/board/bbox_2_lidar_vlp32.json5
    front_lidar:
      pointcloud_topic: /lidar/falcon/iv_points
      frame_id: seyond
      bbox_config: $(find-pkg-share lctk_launch)/config/board/bbox_2_lidar_seyond.json5

markers:
  calibration_board:
    type: hollow_board
    board_config: $(find-pkg-share lctk_launch)/config/board/board_detector.json5
    aruco_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_pattern.json5
    bbox_config: $(find-pkg-share lctk_launch)/config/board/bbox.json5
    pairs:
      - [top_lidar, front_lidar]
```

Generated topic namespaces (from config_parser.py naming rules):
- Board detector (VLP32): `/calibration/top_lidar_calibration_board/debug/*`
- Board detector (Seyond): `/calibration/front_lidar_calibration_board/debug/*`
- Solver output: `/calibration/top_lidar_front_lidar/lidar_to_lidar_transform`

---

## Step-by-Step Procedure

### 1. Fix `board_detector_vlp32.json5` (required before launch)

VLP32 is a standard ROS sensor (Z=up). Current config has wrong `sensor_up_axis: "x"` (Seyond convention).

Edit `ros/lctk_launch/config/board/board_detector_vlp32.json5`:
```json5
"sensor_up_axis": "z",        // was "x" — VLP32 is Z-up (standard ROS)
"initial_inplane_rotation_deg": 0.0   // reset; tune after first detection
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
just two-lidar
```
Equivalent to:
```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=$(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/two_lidar.yaml \
    rviz_config:=$(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/rviz/two_lidar_calibration.rviz
```
Use `mode=offline` (default) — RELIABLE QoS, infinite sync window, no dropping.

### 5. Verify both board detectors receive data
```bash
# Confirm nodes are alive
ros2 node list | grep board_detector

# Check raw input is arriving
ros2 topic hz /lidar/vlp32/velodyne_points
ros2 topic hz /lidar/falcon/iv_points

# Check board detections
ros2 topic hz /calibration/top_lidar_calibration_board/calibration_board_detections
ros2 topic hz /calibration/front_lidar_calibration_board/calibration_board_detections
```
Both detection topics must publish. Solver produces no output until both streams arrive.

### 6. Verify in RViz

RViz launches automatically via `just two-lidar`. Fixed frame = `velodyne`.

Check per-lidar groups:
- **LiDAR 1 (top_lidar)**: "Input points" = raw VLP32 cloud, "Filtered points" = bbox-filtered, "Inlier points" = RANSAC plane, "Final board" = ICP result
- **LiDAR 2 (front_lidar)**: same structure for Seyond

Both "Final board" markers must appear and look geometrically plausible (board outline matching point cluster).

### 7. Check solver output
```bash
ros2 topic echo /calibration/top_lidar_front_lidar/lidar_to_lidar_transform
```
The lidar_to_lidar_solver publishes the transform from `velodyne` → `seyond`.

**Measured transform (2026-06-12, this rosbag):**

| | x | y | z |
|-|---|---|---|
| translation (m) | ~2.051 | ~0.274 | ~-1.318 |

| | x | y | z | w |
|-|---|---|---|---|
| rotation (quat) | ~0.641 | ~-0.004 | ~0.768 | ~-0.008 |

RPY from quaternion: roll≈~79.7°, pitch≈~0°, yaw≈~100°  
(Large roll indicates the two lidars are mounted at very different orientations — VLP32 Z-up vs Seyond X-up.)

Across 3 consecutive frames: translation stable to ±3mm, rotation stable to ±0.001 quat — good convergence.

### 8. Tune `initial_inplane_rotation_deg` for VLP32 if board is rotated

If "Final board" marker is visibly rotated in RViz relative to point cluster:
```json5
// Try these values in board_detector_vlp32.json5:
"initial_inplane_rotation_deg": 0.0    // default
"initial_inplane_rotation_deg": 45.0
"initial_inplane_rotation_deg": -45.0
"initial_inplane_rotation_deg": 90.0
"initial_inplane_rotation_deg": 135.0  // original value (Seyond-tuned)
```
No rebuild needed — config file is read at node startup. Restart `just two-lidar` after each change.

---

## Bbox Configuration

**VLP32 bbox** (`bbox_2_lidar_vlp32.json5`): center `[9.6, 1.1, -0.5]` m, size `[1.0, 1.6, 1.6]` m  
VLP32 coordinate: X=forward, Y=left, Z=up. Board must be ~9.6 m forward from sensor.

**Seyond bbox** (`bbox_2_lidar_seyond.json5`): center `[-0.7, -0.7, 7.6]` m, size `[1.8, 2.0, 1.0]` m  
Seyond coordinate: X=up, Y=right(?), Z=forward. Board must be ~7.6 m forward from sensor.

If no filtered points appear for either lidar, the board is outside the bbox. Temporarily widen:
```json5
{
    "pose": { "translation": [cx, cy, cz], "rotation": [1.0, 0.0, 0.0, 0.0] },
    "size_xyz": [4.0, 4.0, 4.0]
}
```
Find board position from "Input points" in RViz (read coordinates with Publish Point tool), then tighten bbox around it.

---

## RViz Topic Mapping

| Display | Topic |
|---------|-------|
| VLP32 input points | `/calibration/top_lidar_calibration_board/debug/all_points` |
| VLP32 bbox marker | `/calibration/top_lidar_calibration_board/debug/bbox_marker` |
| VLP32 filtered points | `/calibration/top_lidar_calibration_board/debug/filtered_points` |
| VLP32 RANSAC inliers | `/calibration/top_lidar_calibration_board/debug/plane_inliers` |
| VLP32 final board pose | `/calibration/top_lidar_calibration_board/debug/final_board_pose` |
| Seyond RANSAC inliers | `/calibration/front_lidar_calibration_board/debug/plane_inliers` |
| Seyond final board pose | `/calibration/front_lidar_calibration_board/debug/final_board_pose` |
| Fixed Frame | `velodyne` |

---

## Root Cause Analysis — Solver KeyError (SyncGroup topic key mismatch)

**Symptom:** `lidar_to_lidar_solver` crashes immediately on first sync group with `KeyError: '/calibration/top_lidar_calibration_board/calibration_board_detections'`.

**Diagnosis:**
- Conflux stats confirm group formed: `received=2, groups=1`
- Stats keys show full `/calibration/...` paths — same as `self.lidar1_topic`
- But `group[self.lidar1_topic]` fails with KeyError
- Root cause unclear: possible topic key normalization in Rust FFI `conflux_poll` or node-namespace resolution between `add_subscription` and group construction

**Workaround applied:** `_handle_sync_group` now uses `group.get()` with substring fallback and logs `group.topics()` on every sync. After rebuild, inspect the log to see actual keys returned by Conflux, then fix the lookup permanently.

**Resolution:** `group.get()` + substring fallback in `_handle_sync_group` resolved the crash. Transform topic now publishing. Root cause of key mismatch (Conflux topic name normalization or namespace resolution) not fully determined — workaround is stable.

---

## Root Cause Analysis — Conflux Groups Freeze (Shared Buffer Starvation)

**Symptom:** Solver publishes 6-10 transforms at startup, then `groups` counter freezes forever. Rejection rate climbs to >90% and keeps growing. Both streams still actively receiving.

**Diagnosis from [STATS] log:**
- `node_rx` (our own counter subscription) confirms both streams receive continuously: VLP32 ~6 Hz, Seyond ~1 Hz
- After the initial burst of groups, Conflux in-buffer math shows: `front_lidar in-buffer = rx - rej - consumed = 0` while `top_lidar in-buffer ≈ 27`
- New Seyond messages get "rejected" even though the Seyond buffer should be empty

**Root cause:** Conflux `buffer_size=100` is a **shared total** across ALL streams, not 100 per stream. VLP32 at 6 Hz fills the shared 100 slots in ~16s. Once full:
- `reject_new`: new Seyond messages rejected (no room in shared buffer)
- `drop_oldest`: new Seyond messages cause VLP32 eviction — Seyond briefly enters then immediately gets evicted by the next VLP32 flood
- Either way: Seyond never accumulates in buffer → no groups form

**What Conflux is:** A multi-stream timestamp synchronizer. Waits until all subscribed topics have a message within `window_size_ms`, fires a callback with the matched group. Works well when stream rates are similar. Fails when one stream (VLP32 6 Hz) is 6× faster than the other (Seyond 1 Hz) and the buffer is shared.

**Fix (pending):** Replace Conflux in `lidar_to_lidar_solver` with manual nearest-timestamp matching:
- `deque(maxlen=N)` for lidar1 (VLP32) messages
- On each lidar2 (Seyond) arrival: find nearest lidar1 by timestamp, compute transform
- Eliminates shared-buffer issue entirely; always pairs latest Seyond with nearest VLP32

**What to log (new [STATS] format after fix):**
```
[STATS] node_rx: top_lidar_calibration_board=123(buf=20) front_lidar_calibration_board=22(buf=0)
        | pairs=22 no_match=0 | last_transform=0.9s ago
```

---

## Root Cause Analysis — Solver Not Publishing (Rosbag QoS + Stale Check)

**Symptom:** `lidar_to_lidar_transform` topic exists but silent.

**Cause 1 — QoS mismatch:** Rosbag publishes VLP32 with `BEST_EFFORT`. Solver in `mode=offline` subscribes `RELIABLE` → all messages silently dropped.  
Fix: use `mode=realtime` (BEST_EFFORT QoS).

**Cause 2 — Sync tolerance too tight:** `mode=realtime` sets `sync_tolerance_ms=50ms, queue=2`. Seyond detects ~1Hz vs VLP32 ~6Hz → timestamps rarely within 50ms window.  
Fix: solver launch params hardcoded to `sync_tolerance_ms=0` (infinite), `queue=100`.

**Cause 3 — Stale message check:** `max_message_age_ms=500` (default). Rosbag messages checked against wall clock — always fresh when bag recorded recently. But `max_message_age_ms=0` now disables check explicitly.

---

## Root Cause Analysis — VLP32 No Detections (Wrong `sensor_up_axis`)

**Symptom:** VLP32 board detection never publishes; Seyond works fine.

**Root cause:** `board_detector_vlp32.json5` has `"sensor_up_axis": "x"` (Seyond convention).  
VLP32 is a standard ROS sensor (Z=up). With wrong up-axis, `compute_initial_pose_from_plane` projects the plane normal incorrectly → degenerate initial pose → ICP local minimum → detection fails.

Fix: change to `"sensor_up_axis": "z"` in `board_detector_vlp32.json5`.

This is the same class of bug that was diagnosed and fixed for the Seyond during lidar-camera calibration (see `calibration_plan.md` — "Root Cause Analysis — ICP Failure"). The fix there added `sensor_up_axis: "x"` for Seyond; the VLP32 config was mistakenly copied with that value.

---

## Known Issues / Watchpoints

| Issue | Likely Cause | Fix |
|-------|-------------|-----|
| VLP32 input points absent in RViz | `board_detector_top_lidar` node not running | `ros2 node list`; check launch logs for crash |
| VLP32 filtered points empty | Board outside bbox `[9.6, 1.1, -0.5]±[0.5, 0.8, 0.8]` | Use RViz Publish Point on input cloud to find board position; adjust bbox |
| VLP32 detections empty, ICP fails | `sensor_up_axis: "x"` wrong for VLP32 | **Fix**: change to `"z"` in `board_detector_vlp32.json5` |
| VLP32 board marker rotated | `initial_inplane_rotation_deg: 135.0` tuned for Seyond | Reset to `0.0`, tune in 45° steps |
| Seyond detections empty | Bbox at `[-0.7, -0.7, 7.6]` doesn't cover board | Widen bbox; read actual board position from input points |
| VLP32 ICP stops at iteration 1 | `icp_min_inlier_points: 1000` too high; VLP32 at ~10m yields ~270-300 pts | **Fixed**: lowered to 100 in `board_detector_vlp32.json5` |
| `debug/all_points` silent (node alive) | QoS mismatch: rosbag BEST_EFFORT vs node RELIABLE | Use `mode=realtime` |
| Solver node disappears from node list | KeyError crash in `_handle_sync_group` on first sync group | **Fixed**: `group.get()` + substring fallback in `_handle_sync_group` |
| Solver not publishing (no crash) | Sync tolerance 50ms too tight for mismatched detection rates | **Fixed**: launch hardcoded to infinite window |
| Transform looks wrong | Board detected but ICP poor fit | Check `icp_good_fit_threshold` in vlp32 config; inspect RViz board vs points alignment |

---

## Quick Reference

```bash
# Check detection rates
ros2 topic hz /calibration/top_lidar_calibration_board/calibration_board_detections
ros2 topic hz /calibration/front_lidar_calibration_board/calibration_board_detections

# Monitor ICP quality (fires every frame, success and failure)
ros2 topic echo /calibration/top_lidar_calibration_board/debug/icp_stats
ros2 topic echo /calibration/front_lidar_calibration_board/debug/icp_stats

# Read solver output
ros2 topic echo /calibration/top_lidar_front_lidar/lidar_to_lidar_transform

# Check QoS of raw lidar (must be BEST_EFFORT for rosbag playback)
ros2 topic info /lidar/vlp32/velodyne_points --verbose | grep Reliability

# Raw lidar rates
ros2 topic hz /lidar/vlp32/velodyne_points
ros2 topic hz /lidar/falcon/iv_points

# Kill stale ROS daemon
pkill -9 -f ros2-daemon
```
