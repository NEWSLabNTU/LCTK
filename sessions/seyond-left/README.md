# seyond-left

A Seyond Falcon solid-state LiDAR paired with the **left** camera of the two-camera
rig, against the hollow 1000 mm board.

**No recording ships for this rig.** `data.kind` is `live`: the sensors are expected
to be publishing already, so the topics below are stated rather than derived.

| | |
|---|---|
| Data | `live` |
| LiDAR | device `seyond_lidar`, frame `seyond`, `/lidar/falcon/iv_points` |
| Camera | device `left_camera`, frame `camera_left`, `/camera/left/image_raw` |
| Target | hollow 1000 mm, four ArUco markers |
| Detector | `hollow_1000/seyond.json5` (bbox-free) |

The camera publishes `sensor_msgs/CompressedImage` on
`/camera/left/image_raw/compressed`. `aruco_locator_node` wants a raw `Image`, so
republish before running this session:

```bash
ros2 run image_transport republish compressed raw \
    --ros-args -r in/compressed:=/camera/left/image_raw/compressed \
               -r out:=/camera/left/image_raw
```

To calibrate against a recording of this rig instead, copy the session
(`just new my-rig seyond-left`) and change `data:` to `kind: bag` with the bag's
path; the stated topics are then checked against the bag's `metadata.yaml`.
