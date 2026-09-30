# Calibration Sessions

A session describes one calibration run: its data source, sensors, Calibration Target,
and the files specific to that run. Start from the closest shipped session and adapt it
to your rig.

The `just` recipes source the workspace for you. For direct `ros2` commands in a new
terminal, source ROS and the installed workspace from the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Session directory

```text
my-rig/
  session.yaml       # manifest
  out/               # optional run outputs
  data/              # optional recording files
  bbox.json5         # optional crop box for this recording
  camera_info.yaml   # optional camera calibration file
  rviz.rviz          # optional visualization layout
```

Keep files tied to one recording in its session directory. Shared physical target
geometry and sensor-specific detector presets are supplied separately; see
[Configuration](./configuration.md).

## Create a session

List the available examples, then copy the closest one:

```bash
just sessions
ros2 run lctk_launch lctk_session new ~/calib/my-rig \
    --from "$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne"
```

Edit `~/calib/my-rig/session.yaml` to match your data, sensors, target, and detector
settings. Validate it before running:

```bash
just check ~/calib/my-rig
```

For a bag-backed session, the check confirms the bag exists and contains the configured
sensor topics. It also reports the topics and frames that the session will use.

## Manifest example: live sensors

```yaml
data:
  kind: live

devices:
  lidars:
    top:
      frame_id: velodyne
      pointcloud_topic: /velodyne_points
  cameras:
    front:
      frame_id: camera_link
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

For `live` and `bag` data, state the topic and frame for each sensor. For
`pcap_avi`, LCTK derives the point-cloud and image topics from the device names, so do
not add `pointcloud_topic` or `image_topic` to those devices.

## Choose a data source

| `data.kind` | Required field | What it describes |
|---|---|---|
| `live` | none | Sensors already publishing in ROS 2. |
| `bag` | `path` | A rosbag2 directory containing `metadata.yaml`. |
| `pcap_avi` | `dir` | A directory containing `lidar.pcap` and `video.avi`. |

Paths belonging to the session can use `$(session-dir)`, for example
`$(session-dir)/bag` or `$(session-dir)/camera_info.yaml`. Shared installed files can
use `$(find-pkg-share lctk_launch)`. Ordinary relative paths are relative to the
process working directory.

For pcap/AVI playback, set `data.camera.info_url` when the recording needs a supplied
camera calibration file. If a camera or recording provides compressed images, add a
bridge under `data.republish` so LCTK receives raw images:

```yaml
data:
  kind: live
  republish:
    - from: /camera/image_raw/compressed
      to: /camera/image_raw
```

The same `republish` form can be used with bags. For a bag, the stated sensor topics
are checked against its metadata; republished output topics are also accepted.

## Topics, frames, and reliability

Sensor topic names and `frame_id` values must match what the live rig publishes or the
recording contains. For a bag, inspect `metadata.yaml` or run `just check` to catch
topic mismatches before starting the graph.

Transport reliability can be stated per sensor with `qos: reliable` or
`qos: best_effort`; a top-level `qos:` supplies a session-wide default. For bag data,
LCTK can use the reliability offered by the recording when no value is stated. A
contradictory setting is rejected because it would prevent messages from arriving.

## Run a session

```bash
just run ~/calib/my-rig
```

This starts the session's data source and calibration graph. For a shipped sample,
`just demo` runs `sample3-hollow-velodyne`. To run only calibration while sensors or a
recording are already publishing:

```bash
just calibrate ~/calib/my-rig/session.yaml
```

`just run` accepts a session path or a shipped session name; `just calibrate` takes a
configuration file path. If you launch with `ros2 launch` directly, give
`session.launch.py` an explicit path:

```bash
ros2 launch lctk_launch session.launch.py \
    session:="$HOME/calib/my-rig"
```

Direct `session.launch.py` defaults to `solver_mode=continuous`. To get assisted
capture and the review page, include `solver_mode:=assisted` in that launch command.

## Run outputs

For an assisted-capture archive, set `assisted.review_archive_path` to a path such as
`$(session-dir)/out/detections.json`. That setting defaults to an empty path; the
review page's archive action does not prompt for one. Keep the session manifest and
the exact target/detector files together with the resulting archive so the calibration
can be reviewed later.

For the complete set of manifest fields and detector settings, see
[Configuration](./configuration.md). For field capture and review, see the
[Field Validation Runbook](./field-validation.md).
