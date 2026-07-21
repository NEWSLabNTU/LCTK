# Recorded bags (not tracked in git)

This directory is gitignored: the bags are ~2.4 GB of `.db3` plus ~1.9 GB of
`.zip`, which must not enter git history. Obtain them from the project's data
share and unpack them here so the layout is:

```
ros/lctk_sample_data/bags/
  TWO_LIDAR_1/
    metadata.yaml
    TWO_LIDAR_1_0.db3
  TWO_LIDAR_2/ ...
  TWO_LIDAR_3/ ...
  TWO_LIDAR_4/ ...
```

Each bag is a ~20 s, ~199-frame-per-sensor static capture of the calibration
board from a two-LiDAR rig:

| topic | sensor | frame_id | points/frame |
|---|---|---|---|
| `/lidar/vlp32/velodyne_points` | Velodyne VLP-32C (spinning) | `velodyne` | ~51,400 |
| `/lidar/falcon/iv_points` | Innovusion/Seyond Falcon (solid-state) | `seyond` | ~92,300 |

The board is held static within each bag.

To use them in the `boarddet` experiment, export to its `.npz` cache first —
see `experiments/board-detection-2d/README.md`.

Verify a bag with:

```bash
source /opt/ros/humble/setup.bash
ros2 bag info ros/lctk_sample_data/bags/TWO_LIDAR_1
```
