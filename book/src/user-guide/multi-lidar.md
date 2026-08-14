# Multi-LiDAR Calibration

This guide shows how to calibrate two or more LiDAR sensors, computing the transformation between their coordinate frames for point cloud fusion.

## Workflow Overview

```mermaid
graph LR
    A[(LiDAR 1)] --> C[Board Detector 1]
    B[(LiDAR 2)] --> D[Board Detector 2]
    C -->|3D pose| E[Multi-Wayside Node]
    D -->|3D pose| E
    E --> F>Transform]

    classDef sensor fill:#e0e0e0,stroke:#333,color:#000
    classDef node fill:#4a90d9,stroke:#333,color:#fff
    classDef output fill:#2d6a4f,stroke:#333,color:#fff

    class A,B sensor
    class C,D,E node
    class F output
```

**What happens:**
1. Both LiDARs see the **same calibration board** from different angles
2. Each **Board Detector** finds the board in its point cloud (3D pose)
3. **Multi-Wayside Node** computes the LiDAR-to-LiDAR transformation

**Key difference from LiDAR-camera:** No ArUco markers needed, only the hollow board pattern.

## Calibration Target

You need the same **1m x 1m hollow board**:
- **3** circular holes (150mm radius) — see
  [the board model](../developer-guide/architecture.md#the-board-model-and-its-frame)
- Hung as a **diamond**, standing on one corner
- Board must be visible to **both LiDARs simultaneously**

## Step-by-Step Process

### 1. Prepare Your Data

Use the included sample data:
```bash
cd ~/repos/LCTK
just two-lidar
```

This plays back data from two LiDARs that both see the calibration board.

Or record your own:
- Place board where both LiDARs can see it clearly
- Record PCAP files from both sensors simultaneously
- Keep board stationary for 30-60 seconds per position

### 2. Launch Calibration

The `just two-lidar` command starts:
- Two Velodyne driver nodes (playing PCAP data)
- Two board detector nodes (one per LiDAR)
- Multi-wayside node (computes transformation)

### 3. Monitor Progress

Open `http://localhost:8000` to see the web UI.

Check that both LiDARs are detecting the board:
```bash
source install/setup.bash

# Should both show >1 Hz
ros2 topic hz /sensing/lidar/top/board_detections
ros2 topic hz /sensing/lidar/front/board_detections
```

View the calibration result:
```bash
ros2 topic echo /calibration_transform
```

### 4. Validate Results

If you have a display, visualize in RViz:
```bash
just rviz
```

Add both point cloud topics:
- `/sensing/lidar/top/pointcloud_raw`
- `/sensing/lidar/front/pointcloud_raw`

Set Fixed Frame to `velodyne_top`. If calibrated correctly, point clouds from both sensors should align perfectly (e.g., walls, floors appear as single surfaces).

## Configuration

Key parameters in `ros/lctk_launch/config/multi_wayside.yaml`:
- `same_face_mode: true` - Both LiDARs see the **same side** of the board
- `sync_tolerance_ms: 100` - Maximum time difference between detections
- `min_detections_for_calibration: 5` - Minimum synchronized pairs needed

If LiDARs see **opposite sides** of the board, set `same_face_mode: false` to apply 180 degree correction.

## Tips for Good Calibration

- **Placement**: Position board 3-8 meters from both LiDARs
- **Overlap**: Ensure both LiDARs have good view of the board
- **Multiple positions**: Move board to 3-5 different positions
- **Stability**: Keep board stationary during capture
- **Distance variation**: Include near (3m) and far (8m) positions

## Troubleshooting

**Only one LiDAR detecting:** Check bounding box configuration covers board location for both sensors

**Detections not synchronizing:** Increase `sync_tolerance_ms` or check that data streams have valid timestamps

**Transform seems flipped:** Toggle `same_face_mode` in configuration
