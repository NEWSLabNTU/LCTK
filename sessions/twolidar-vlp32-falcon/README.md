# twolidar-vlp32-falcon

Two LiDARs and no camera: a spinning VLP-32C and a solid-state Seyond Falcon
observing the same hollow 1000 mm board. Solving this pair produces a LiDAR-to-LiDAR
extrinsic.

**The recording does not ship.** The `TWO_LIDAR_*` bags are gitignored — see
[`ros/lctk_sample_data/bags/README.md`](../../ros/lctk_sample_data/bags/README.md)
to obtain one. Then place or symlink it at `bag/` inside this directory:

```bash
ln -s ../../ros/lctk_sample_data/bags/TWO_LIDAR_1 sessions/twolidar-vlp32-falcon/bag
```

or point `data.path` somewhere else entirely. The symlink is gitignored, so the
recording never enters git.

| | |
|---|---|
| Data | `bag`, `$(session-dir)/bag` |
| LiDAR A | device `top_lidar`, frame `velodyne`, `/lidar/vlp32/velodyne_points` |
| LiDAR B | device `front_lidar`, frame `seyond`, `/lidar/falcon/iv_points` |
| Target | hollow 1000 mm, four ArUco markers |
| Detector | `hollow_1000/velodyne.json5`, overridden per-device for the Falcon |

## Two things worth knowing

**The topics are the ones the bag actually records.** The config this session
replaces named `/velodyne_points` and `/iv_points`, which no `TWO_LIDAR_*` bag
publishes; the pipeline launched cleanly and sat silent forever (M-26). Startup now
checks the stated set against `metadata.yaml` and refuses a name the bag lacks,
listing the names it has.

**`front_lidar` carries its own `detector_config`.** A per-device preset overrides
the marker-level one, which is how two differently-sampled LiDARs share a single
target: the Falcon gets the Seyond-tuned preset while `top_lidar` takes the
marker-level Velodyne one. That override is the feature this session exists to
demonstrate.

## Session-local files

`bbox_vlp32.json5` and `bbox_falcon.json5` are the true-board reference boxes for
this recording, one per sensor. The shipped detector presets here are `bbox_free`
and do not read them; they are the `--bbox` arguments of the
`experiments/board-detection-2d` benchmark, which labels frames against this rig.
