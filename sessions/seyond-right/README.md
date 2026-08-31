# seyond-right

A Seyond Falcon solid-state LiDAR paired with the **right** camera of the
two-camera rig, against the hollow 1000 mm board. The mirror of `seyond-left`.

**No recording ships for this rig.** `data.kind` is `live`.

| | |
|---|---|
| Data | `live` |
| LiDAR | device `seyond_lidar`, frame `seyond`, `/lidar/falcon/iv_points` |
| Camera | device `right_camera`, frame `camera_right`, `/camera/right/image_raw` |
| Target | hollow 1000 mm, four ArUco markers |
| Detector | `hollow_1000/seyond.json5` (bbox-free) |

The device is named `right_camera`. The config this session replaces
(`config/examples/seyond_right.yaml`) called it `left_camera` — a copy-paste from
the left example that survived because the topic and frame underneath it were
already correct. The device name reaches generated node names and namespaces, so
the wrong one made a right-camera calibration report itself as left.

Same `CompressedImage` caveat as `seyond-left`; republish
`/camera/right/image_raw/compressed` to `/camera/right/image_raw` first.
