# Configuration

Calibration settings are written in YAML. A [session](./sessions.md) adds a data
source and keeps run-specific files with it. If the data is already being published,
`calibrate.launch.py` also accepts a plain calibration YAML without a `data:` section.

## Required calibration settings

The example below shows a live LiDAR-camera configuration. Replace the topics, target,
and detector preset with values that match your rig. Sensor frame labels come from the
message headers at runtime; the manifest has no frame override.

```yaml
data:
  kind: live

devices:
  lidars:
    top:
      pointcloud_topic: /velodyne_points
  cameras:
    front:
      image_topic: /camera/image_raw

markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
    pairs:
      - [top, front]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
```

Every calibration configuration needs a `sync:` section with positive
`tolerance_ms`, positive `queue_size`, and `drop_policy` set to `reject_new` or
`drop_oldest`. These settings determine which observations are paired; tune them for
the timing and motion of your sensors and Calibration Target.

For `bag` and `live` sources, each LiDAR needs `pointcloud_topic` and each camera needs
`image_topic`. At runtime LCTK takes frame labels from the synchronized sensor message
headers. For `pcap_avi`, topics are derived from the device names and must not be
specified; generated frame labels use the LiDAR device name and the camera device name
with `_optical_frame` appended. See [Sessions](./sessions.md) for data-source settings
and validation.

## Choose the right target and detector files

Each marker entry names two different files:

- **Target Definition** (`target_config`) describes the physical Calibration Target:
  its plate, cutouts, fiducial IDs, layout, and Target Identity.
- **Detector Tuning** (`detector_config`) configures detection of that target for a
  particular LiDAR or sensor model.

Use the Target Definition that matches the physical target and its marker sheet. Use
the Detector Tuning preset intended for the LiDAR. A per-LiDAR `detector_config` can
override the marker's preset when a session pairs multiple LiDARs with one target.

The optional `aruco_detector_config` controls camera-side marker detection; it does
not define the target's marker IDs or placement. Shipped files are under:

```text
$(find-pkg-share lctk_launch)/config/targets/
$(find-pkg-share lctk_launch)/config/board/<target>/
$(find-pkg-share lctk_launch)/config/aruco/
```

Only add `bbox_config` when the selected Detector Tuning uses `detection_mode: "bbox"`.
The box describes the target's location in a particular recording, so keep it in the
session directory:

```yaml
markers:
  calibration_board:
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne_bbox.json5
    bbox_config: $(session-dir)/bbox.json5
```

Do not add a crop box for a `bbox_free` preset. Do not copy target geometry into a
detector preset.

## Camera calibration data

Camera images must be `sensor_msgs/Image`. When a camera or recording provides only
compressed images, configure a `data.republish` bridge in the session. Camera
intrinsics must also be available to the graph. The solver derives the CameraInfo
topic from the image topic by replacing its final path component with `camera_info`;
for example, `/camera/image_raw` uses `/camera/camera_info`. Make the camera or bag
publish CameraInfo on that topic. The manifest has no separate CameraInfo topic key;
if a driver or recording uses a different name, remap its publisher or playback to the
derived topic. For pcap/AVI playback, provide a calibration file with
`data.camera.info_url`.

## Transport reliability

Set `qos: reliable` or `qos: best_effort` on a sensor device when needed. A top-level
`qos:` supplies a default for devices without an override. For a bag, LCTK can infer
reliability from the recording's offered QoS when it is not stated. A reliability
setting that cannot receive from the bag's publisher is rejected during validation.

## Assisted capture settings

Assisted mode works with defaults when there is no `assisted:` section. The current
defaults and their roles are:

| Setting | Default | Purpose |
|---|---:|---|
| `stability_window_s` | `1.0` | Duration used to assess whether the target is still. |
| `stability_max_translation_m` | `0.005` | Maximum translation span over that window. |
| `stability_max_rotation_deg` | `0.5` | Maximum rotation span over that window. |
| `stability_cooldown_s` | `1.0` | Minimum interval between automatic captures. |
| `novelty_position_tol_m` | `0.05` | Position tolerance for treating a placement as new. |
| `novelty_orientation_tol_deg` | `5.0` | Orientation tolerance for treating a placement as new. |
| `review_bind_host` | `127.0.0.1` | Address for the unauthenticated review page. |
| `review_port` | `8080` | Base review port; additional LiDAR-camera pairs use the next ports. |

The review page can change the three `stability_*` values while it is running. Tune
them against your sensor's observed pose noise and review the resulting Captures;
looser settings may admit a moving target. See [Assisted Capture](./assisted-capture.md)
for the workflow and network-access warning.

## Validate and run

Validate a session before starting the graph:

```bash
just check /path/to/session
```

Run a session and its data source together:

```bash
just run /path/to/session
```

When data is already running, start only the calibration graph:

```bash
just calibrate /path/to/session/session.yaml
```

The session CLI also supports listing and scaffolding sessions; see
[Calibration Sessions](./sessions.md). For pair definitions and the multi-LiDAR
workflow, see [Multi-LiDAR Calibration](./multi-lidar.md).
