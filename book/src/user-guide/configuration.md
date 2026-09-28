# Configuration

LCTK reads a calibration configuration from YAML. A [session](./sessions.md) is the
preferred form: it combines the calibration configuration with the data source and
session-local files. A plain YAML is also accepted by `calibrate.launch.py` when the
data is already being published or played separately.

## Configuration layout

A session normally looks like this:

```
my-session/
  session.yaml
  bbox.json5          # only for a bbox-mode detector preset
  camera_info.yaml    # optional camera intrinsics file
  rviz.rviz           # optional RViz layout
  out/                # run outputs
```

Use `$(session-dir)` for files that belong beside the loaded YAML, and
`$(find-pkg-share lctk_launch)` for files installed with LCTK. For example:

```yaml
data:
  kind: live

devices:
  lidars:
    top:
      frame_id: velodyne
      pointcloud_topic: /velodyne_points
  cameras:
    front_center:
      frame_id: camera_front_center
      image_topic: /camera/image_raw

markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
    aruco_detector_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_detector.json5
    pairs:
      - [top, front_center]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
```

The exact device topics and frame IDs belong to the data source. For a bag, LCTK checks
the stated topics against the bag metadata. For a live source, the topics and camera-info
publisher are supplied by the running graph. For pcap/AVI playback, a session may set
`data.camera.info_url` to a session-local camera-info file.

## Target Definition and Detector Tuning

Calibration configuration is split across files with different responsibilities:

- **Target Definition** describes the physical Calibration Target: plate geometry,
  cutout layout, fiducial marker IDs, and the target identity. Shipped examples are
  under `lctk_launch/config/targets/`.
- **Detector Tuning** describes how one sensor detects that Target Definition:
  foreground extraction, crop mode, ICP settings, and sensor orientation. Shipped
  presets are under `lctk_launch/config/board/<target>/`.
- **ArUco detector tuning** controls image-side marker detection and is separate from
  the marker geometry in the Target Definition.

Change the Target Definition only when the physical target or its fiducial layout
changes. Change Detector Tuning when the same target needs different sensor-specific
detection settings.

The shipped hollow-target Velodyne preset is:

```
$(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
```

It currently selects `bbox_free` detection, uses background subtraction, sets
`bg_warmup_frames` to 20, skips the RANSAC stage, uses sensor up-axis `z`, and
allows 100 ICP iterations. These are values of that preset, not universal detector
defaults. The bbox-mode sample preset has different values and requires a crop-box
file.

Useful tuning keys are:

- `detection_mode`: `bbox` or `bbox_free`.
- `sensor_up_axis`: the sensor axis used as the LiDAR orientation reference.
- `initial_inplane_rotation_deg`: the physical in-plane roll of a target that is
  not mounted corner-up.
- `max_icp_iterations` and the other ICP termination settings.
- Foreground, cluster, and structural acceptance settings appropriate to the selected
  target and sensor.

Keep geometry such as plate dimensions, cutout positions, and marker placement in the
Target Definition rather than copying it into a detector preset.

## Bounding boxes

A crop box is session-local because it describes where a Calibration Target appears in
one recording. It is used only when the selected Detector Tuning preset has
`detection_mode: "bbox"`:

```yaml
markers:
  calibration_board:
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne_bbox.json5
    bbox_config: $(session-dir)/bbox.json5
```

Example `bbox.json5`:

```json5
{
  pose: {
    translation: [2.0, 0.0, 0.0],
    rotation: [0.0, 0.0, 0.0, 1.0],
  },
  size_xyz: [4.0, 4.0, 2.0],
}
```

Do not add `bbox_config` to a bbox-free preset.

## Camera intrinsics and ArUco detection

A camera-info file is not universally required by the manifest. The camera may receive
its `camera_info` topic from the live graph or bag, or a session can provide
`data.camera.info_url` for the playback path.

The marker IDs, dictionary, printed-sheet size, and its placement on the plate belong
to the Target Definition. The file under `config/aruco/` contains detector settings
such as corner refinement and adaptive thresholding:

```yaml
markers:
  calibration_board:
    aruco_detector_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_detector.json5
```

## Synchronization and transport

Every calibration configuration requires:

```yaml
sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new  # or drop_oldest
```

Sensor transport reliability is configured per device with `qos: reliable` or
`qos: best_effort`, or is resolved from a bag's offered QoS when no value is stated.
There is no graph-wide `mode` setting for transport.

## Common tasks

Validate a session without starting nodes:

```bash
source install/setup.bash
ros2 run lctk_launch lctk_session check /path/to/session
```

Enable debug output for a config-driven calibration run. Justfile variables go before
the recipe name:

```bash
just debug_mode=true calibrate /path/to/session.yaml
```

For an end-to-end session, prefer:

```bash
just run /path/to/session
```

This starts the session's data source and calibration graph. Use `just calibrate` when
the data source is already running or is being played separately.

For a plain bag/config workflow:

```bash
ros2 bag play /path/to/bag --clock
just calibrate /path/to/config.yaml
```

The config must state the topics and frames that the player publishes.

## Multi-LiDAR configurations

Multi-LiDAR calibration is configured through the same session manifest. The shipped
example is:

```
sessions/twolidar-vlp32-falcon/session.yaml
```

It defines two LiDAR devices, their Target Definition and Detector Tuning, a shared
synchronization section, and a LiDAR-to-LiDAR pair. See
[Multi-LiDAR Calibration](./multi-lidar.md) for the runnable workflow.

`same_face_mode` is a parameter of `lidar_to_lidar_solver`; it is not a key in the
session YAML schema generated by the current launch planner.

## Configuration files

| File | Purpose |
|---|---|
| `session.yaml` | Data source, devices, pairs, synchronization, and optional assisted settings |
| `config/targets/<target>.json5` | Target Definition and Target Identity |
| `config/board/<target>/<sensor>.json5` | Sensor-specific Detector Tuning |
| `config/aruco/aruco_detector.json5` | Camera-side ArUco detector tuning |
| `bbox.json5` | Session-local crop box for bbox-mode detection |
| `camera_info.yaml` | Optional session-local camera calibration data |

See [Calibration Sessions](./sessions.md) for the complete manifest schema.
