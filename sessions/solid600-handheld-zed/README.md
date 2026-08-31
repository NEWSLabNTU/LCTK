# solid600-handheld-zed

The solid 600 mm calibration target, **held by hand and moved slowly through the
scene**, seen by a Velodyne and a ZED. This is the one session selecting a target
other than the hollow 1000 mm board.

**No recording ships for this rig**, and none exists in the repo yet. `data.kind`
is `live`.

| | |
|---|---|
| Data | `live` |
| LiDAR | device `top_lidar`, frame `velodyne`, `/velodyne_points` |
| Camera | device `front_center`, frame `zed_left_camera_frame_optical`, `/sensing/camera/zed/rgb/color/rect/image` |
| Target | solid 600 mm, one ArUco marker |
| Detector | `solid_600/velodyne.json5` — **EXPERIMENTAL** |

## Why `sync.tolerance_ms` is 50, not 100

Every hollow session uses 100 ms. With a board that moves, a mis-paired camera
frame and LiDAR sweep do not merely add noise — they produce a detection pair that
is *wrong*, because the board is not where the other sensor saw it. 50 ms is the
stated intent for a slow hand-held sweep, and it is **unconfirmed**: check it on a
first replay against the `pair skew last=/max=` figures in the periodic `sync: ...`
line `lctk_sync.DetectionPairSource` logs.

## Before recording

`solid_600/velodyne.json5` is a sensor-specific starting point, not a
field-validated operating point. It does not block launch, but its numbers await a
real-bag evidence report before they can be trusted the way the hollow presets are.

The preset uses background subtraction with `bg_warmup_frames: 20`, so any
recording made for this session must begin with at least 20 consecutive
board-absent frames — roughly 2 s at 10 Hz — before the board enters the scene.
Otherwise the background never finalizes and nothing is detected.
