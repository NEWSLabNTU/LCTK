# Multi-LiDAR Calibration

LCTK estimates the transform between two LiDAR frames by observing the same
Calibration Target from both sensors at synchronized times.

## Prepare the session

The shipped example is `twolidar-vlp32-falcon`. It pairs a Velodyne VLP-32C and a
Seyond Falcon, but its rosbag is not included in the repository. Follow
`ros/lctk_sample_data/bags/README.md` to obtain the recording and place or link it at
the path declared in the session.

Before running, confirm that both sensors can see the same target, that both detector
presets use the same Target Definition, and that the session's topics and frame IDs
match the recording. Then validate it:

```bash
just check twolidar-vlp32-falcon
```

For another pair of LiDARs, create a session based on the example and update the
devices, pair, target, and detector settings. See [Sessions](./sessions.md) and
[Configuration](./configuration.md).

## Run the calibration

With the recording in place, run the session:

```bash
just run twolidar-vlp32-falcon
```

The session starts the bag and calibration graph. The `just` launch status page is at
<http://localhost:8000>. If the data is already being published or played, run only the
calibration graph:

```bash
just calibrate /path/to/session/session.yaml
```

The solver updates its published transform for each synchronized detection pair; it
does not accumulate a multi-pose capture set. Move the target through several distinct
positions and tilts while both LiDARs can see it, and compare the reported transform
across those views as a consistency check. When the values are consistent, save a
selected output message with the session and field record.

## Check the output

The solver output is named from the LiDAR device names in the session:

```text
/calibration/<lidar1>_<lidar2>/lidar_to_lidar_transform
```

For the shipped session:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /calibration/top_lidar_front_lidar/lidar_to_lidar_transform
```

To capture one message while the session is running, use `--once` and copy the
result into the field record:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --once /calibration/top_lidar_front_lidar/lidar_to_lidar_transform
```

If no transform appears, check the concrete detection topics and the bag metadata
against the manifest. Do not assume a fixed detection rate; it depends on the sensors,
target, and scene. Use [Troubleshooting](./troubleshooting.md) for the next checks.
