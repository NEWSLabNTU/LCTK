# Exporting to Autoware

This guide shows how to export a solved LiDAR-camera extrinsic into an
Autoware-style `sensor_kit_calibration.yaml` with `lctk_autoware_export`.

LCTK accepts the target YAML path supplied by the operator. The repository does not
select an Autoware release or install layout for you.

## 1. Save a detection archive

Run a LiDAR-camera session in `manual` or `assisted` mode. Both modes provide the
solver services. The interactive controller can save the archive with its save command,
or call the dump service directly:

```bash
ros2 service call /calibration/<pair>/lidar_to_camera_solver/dump_detections \
    lctk_interfaces/srv/DumpDetections "{file_path: '$HOME/detections.json'}"
```

The exporter reads the solved `rvec` and `tvec` from this JSON archive. It accepts
archive versions 4 and 5. The archive must use the
`corner_aligned_plate_center_v1` board-frame convention; version 5 also requires a
structurally valid Target Identity.

Use the raw archive as the export input. The published ROS transform has correct frame
labels, but its numeric value is the inverse representation used by the raw solver
coordinates.

## 2. Preview the export

Supply the target YAML and the existing LiDAR entry that anchors the sensor-kit chain:

```bash
ros2 run lctk_autoware_export export \
  --detections ~/detections.json \
  --target /path/to/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link \
  --dry-run
```

- `--lidar-frame` names an existing child entry under the kit frame.
- `--camera-frame` names the entry to create or update.
- `--dry-run` prints the six exported values and writes nothing.

The exporter refuses a missing kit frame, missing LiDAR anchor, unsupported archive, or
incompatible board-frame convention instead of guessing.

## 3. Write the entry

Remove `--dry-run` only after reviewing the printed values:

```bash
ros2 run lctk_autoware_export export \
  --detections ~/detections.json \
  --target /path/to/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link
```

The exporter updates only the selected camera entry. It creates a
`sensor_kit_calibration.yaml.bak`-style backup beside the target the first time it
writes. The YAML round-trip preserves comments, key order, and unrelated entries, but
the serializer may change formatting.

## Transform composition

The archive's `rvec` and `tvec` map LiDAR coordinates into the camera optical frame,
`T_optical_from_lidar`. The exporter inverts that matrix and composes it into the
camera-link entry under the sensor-kit frame:

```
T_kit_camera_link = T_kit_lidar
                  · inv(T_optical_from_lidar)
                  · inv(OPTICAL_IN_CAMERA_LINK)
```

It then writes translation in metres and fixed-axis roll, pitch, and yaw in radians.

## Verify the result

Use the target system's normal launch and TF tools to inspect the exported entry:

```bash
ros2 run tf2_ros tf2_echo sensor_kit_base_link camera0/camera_link
```

The exporter test suite also checks the frame conversions and a minimal xacro
round-trip when the ROS `xacro` package is available. That test is not a full test of an
installed Autoware workspace.
