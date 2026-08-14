# Exporting to Autoware

This guide shows how to move a solved LiDAR-camera extrinsic from LCTK into an Autoware
workspace, using the `lctk_autoware_export` tool.

## How Autoware stores extrinsics

Autoware's whole static TF tree comes from two YAML files that
`robot_state_publisher` loads through xacro at launch time:

| File | Parent frame | Children |
|------|--------------|----------|
| `sensors_calibration.yaml` | `base_link` | `sensor_kit_base_link` + vehicle-mounted sensors |
| `sensor_kit_calibration.yaml` | `sensor_kit_base_link` | every kit sensor (`velodyne_top_base_link`, `camera0/camera_link`, …) |

Both use the same schema — meters and radians, URDF fixed-axis RPY:

```yaml
sensor_kit_base_link:
  camera0/camera_link:
    x: 0.10731
    y: 0.56343
    z: -0.27697
    roll: -0.025
    pitch: 0.315
    yaw: 1.035
```

A LiDAR-camera calibration edits **`sensor_kit_calibration.yaml`** — normally just the
camera entry, since the lidar is the kit's reference sensor.

**Where the file lives depends on the Autoware version:**

- **≤ 2024.11**: per-vehicle values go to
  `autoware_individual_params/individual_params/config/$VEHICLE_ID/<kit>/sensor_kit_calibration.yaml`.
- **≥ 0.45.1 (incl. 1.5.0 and current main)**: `autoware_individual_params` is gone; the file
  lives at `autoware_launch/sensor_kit/<kit>_launch/<kit>_description/config/sensor_kit_calibration.yaml`,
  and a per-vehicle copy can be pointed at with the `config_dir:=` launch argument.

The exporter doesn't care which era you're on — point it at the right file.

## Step 1: Save the calibration

Run the pipeline with the advanced solver and dump the result (via the
[interactive controller](./lidar-camera.md)'s `p` key, or the service directly):

```bash
ros2 service call /calibration/<pair>/lidar_to_camera_solver/dump_detections \
    lctk_interfaces/srv/DumpDetections "{file_path: '$HOME/detections.json'}"
```

The JSON contains the raw solver `rvec`/`tvec`. This is the exporter's only input format —
deliberately, because the values on the TF topic carry inverted frame labels (issue M-01)
while the dump is unambiguous.

## Step 2: Preview the export

```bash
ros2 run lctk_autoware_export export \
  --detections ~/detections.json \
  --target ~/autoware/src/launcher/autoware_launch/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link \
  --dry-run
```

- `--lidar-frame` names the **existing** lidar entry in the target file; it anchors the
  chain from `sensor_kit_base_link` to your camera.
- `--camera-frame` is the entry to write (created if missing).
- `--dry-run` prints the six values and touches nothing.

## Step 3: Write it

Drop `--dry-run`. The exporter:

- patches **only** the target entry — comments, ordering, and every other entry in the
  file survive byte-for-byte;
- saves a `sensor_kit_calibration.yaml.bak` next to the target the first time it
  modifies it;
- refuses to guess: a missing kit key or lidar entry aborts with the list of available
  frames instead of writing something wrong.

## What the exporter computes

The solver's output relates the **camera optical frame** (z forward) to the LiDAR frame.
Autoware's entry wants the pose of **`camera_link`** (x forward, REP-103) in
**`sensor_kit_base_link`**. The exporter composes:

```text
T(kit → camera_link) = T(kit → lidar)          # read from the target YAML
                     · T(lidar → optical)      # solver result, inverted
                     · T(optical → camera_link) # fixed REP-103 rotation
```

and decomposes to fixed-axis RPY. All three conversions are covered by tests, including
an end-to-end test that pushes an exported file through the real `xacro` pipeline and
checks the emitted URDF joint reproduces the solved transform.

## Verify in Autoware

Launch Autoware with your vehicle and sensor model, then check the TF:

```bash
ros2 run tf2_ros tf2_echo sensor_kit_base_link camera0/camera_link
```

The translation/rotation should match the exporter's printed values.
