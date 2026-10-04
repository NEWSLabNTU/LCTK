# Exporting to Autoware

Use `lctk_autoware_export` to add a reviewed LiDAR-camera calibration to an existing
Autoware `sensor_kit_calibration.yaml`. The tool updates one camera entry; it does not
choose or install an Autoware workspace for you.

Run the ROS commands below from a terminal with the ROS and LCTK workspace sourced:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Save a Detection Archive

In assisted mode, set `assisted.review_archive_path` in the session before launch, then
choose **Export archive**. In manual mode, call the running solver's dump service.
This example uses the shipped sample pair; replace the namespace with your session's
LiDAR and camera names:

```bash
ros2 service call \
  /calibration/top_front_center/lidar_to_camera_solver/dump_detections \
  lctk_interfaces/srv/DumpDetections \
  "{file_path: '/home/you/calib/detections.json'}"
```

The exporter accepts only version-6 Detection Archives with the supported board-frame
convention, valid Target Identity and camera-projection metadata. It checks that every
camera detection header matches the saved optical frame and that all LiDAR detection
headers use one consistent, distinct frame. Older archive versions are rejected. Use
the raw archive; do not manually invert or edit its transform.

## Preview the entry

Provide the archive, the existing sensor-kit YAML, the camera entry to create or
update, and the existing LiDAR entry used as the transform anchor:

```bash
ros2 run lctk_autoware_export export \
  --detections ~/calib/detections.json \
  --target /path/to/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link \
  --dry-run
```

The preview prints translation and rotation values and writes nothing. The archive's
camera header names the optical frame used for the solved transform; `--camera-frame`
names the Autoware camera-link entry and may differ because the exporter applies the
existing optical-to-camera-link convention. `--lidar-frame` names the existing YAML
entry that anchors the chain and must represent the LiDAR coordinates in the archive.
The LiDAR entry must already exist under the kit frame.

The exporter uses `sensor_kit_base_link` as its default kit frame. If your YAML uses a
different root key, pass `--kit-frame <frame>` and use that frame as the parent in the
TF verification command below.

## Write and verify

After checking the preview, run the same command without `--dry-run`:

```bash
ros2 run lctk_autoware_export export \
  --detections ~/calib/detections.json \
  --target /path/to/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link
```

The exporter updates only the selected camera entry and creates a `.bak` backup next
to the YAML on the first write. Then inspect the resulting transform with the target
system's normal TF tools, substituting your actual frame names:

```bash
ros2 run tf2_ros tf2_echo sensor_kit_base_link camera0/camera_link
```

For capture review and field checks before export, see the
[Field Validation Runbook](./field-validation.md).
For a held-out reprojection check against separately captured observations, see
[Extrinsic Validation](./extrinsic-validation.md).
