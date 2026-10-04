# LiDAR-Camera Calibration Workshop

This workshop walks through one LiDAR-camera calibration using a solid board, a
rosbag recorded during the lab, assisted capture, and the Autoware exporter. It is
not a full introduction to LCTK or ROS 2.

## 1. Install LCTK

Before the lab, complete [Installation](./installation.md), including the workspace
build. The `just` commands below run from the LCTK repository root. For direct ROS 2
commands in a new terminal, source ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Bring up the camera and LiDAR drivers and confirm that both are publishing before
recording. Find their topic names and types with:

```bash
ros2 topic list -t
```

Record each sensor's `header.frame_id` to check the publishers. LCTK binds frame labels
from message headers at runtime; they are not session settings.

Read the frame IDs while the drivers are publishing:

```bash
ros2 topic echo /velodyne_points --once --field header.frame_id
ros2 topic echo /camera/image_raw/compressed --once --field header.frame_id
```

## 2. Record the workshop bag

This example assumes the camera publishes compressed images. Record the LiDAR point
cloud, compressed camera image, and CameraInfo topic. Replace the example topics with
the names published by your rig. LCTK derives the CameraInfo topic from the raw image
topic: `/camera/image_raw` uses `/camera/camera_info`.

Use a solid 600 mm Calibration Target whose marker layout matches the
`solid_600_aruco_1_v1.json5` Target Definition used below. A solid board with different
dimensions or marker placement needs a matching Target Definition.

```bash
mkdir -p ~/calib/lab1
ros2 bag record -o ~/calib/lab1/bag \
    /velodyne_points \
    /camera/image_raw/compressed \
    /camera/camera_info
```

Start with the board out of the LiDAR's view for at least two seconds so the detector
can build its empty-scene background. The shipped solid-600 detector presets currently
use 20 LiDAR frames for this warm-up; if your sensor has not supplied 20 frames after
two seconds, keep the scene clear until it has.

Then move the solid Calibration Target into the shared view of both sensors. Hold it
still for four seconds at each distinct placement. Aim for at least ten placements,
spreading them across the field of view and range, and varying the board's yaw and
pitch. Keep the marker sheet visible to the camera and the board unobstructed for the
LiDAR. Assisted mode decides whether each observation is still and novel, so a
four-second hold does not guarantee exactly one Capture. Watch the review page during
the calibration to confirm observations are being collected.

Stop the recorder with Ctrl+C after the last placement. Check that the bag contains
the expected topics and messages:

```bash
ros2 bag info ~/calib/lab1/bag
```

After the bag is finalized, stop the live camera and LiDAR drivers before replaying
it. Otherwise the calibration graph can receive live and recorded messages on the
same topics, mixing the empty-scene warm-up with a second data source.

## 3. Configure a session for the recording

Create a session directory and save a `session.yaml` like this. Replace the sample
topics with the values from your bag; select the detector preset for your LiDAR.

```bash
mkdir -p ~/calib/lab1/session
```

```yaml
name: lab1
description: LiDAR-camera workshop recording

# Playback is started separately in terminal 2, so this session only starts
# the calibration graph. Do not add data.republish; terminal 3 starts that bridge.
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
    target_config: $(find-pkg-share lctk_launch)/config/targets/solid_600_aruco_1_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/solid_600/velodyne.json5
    pairs:
      - [top, front]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new

assisted:
  review_archive_path: $(session-dir)/out/detections.json
```

After saving the manifest, validate it:

```bash
just check ~/calib/lab1/session
```

For a Seyond Falcon, use its topic and the `solid_600/seyond.json5`
detector preset instead. The shipped solid-600 detector presets are experimental
starting points, not field-validated settings; validate the result independently
before deployment.

The raw image topic in the session is the output of the bridge in terminal 3. The bag
must contain the compressed input topic and the corresponding CameraInfo topic. This
session uses `data.kind: live` because the bag is supplied externally; `just run` with
`data.kind: bag` would start playback itself. Since this workflow supplies playback
manually, `just check` can check the session configuration but cannot compare its
topics or QoS with the bag. Use `ros2 bag info` to check the session topics against the
recording, and inspect message headers while playing to confirm the frame labels. Leave
per-device `qos` unset unless you need an explicit
override; this `live` session uses the default BEST_EFFORT subscription and cannot
infer reliability from bag metadata. If you set `qos`, compare it with the recorded
`offered_qos_profiles` in the bag's `metadata.yaml`.

This manifest names topics but has no frame overrides. While the bag is playing, inspect
the message headers to confirm the LiDAR and camera labels:

```bash
ros2 topic echo /velodyne_points --once --field header.frame_id
ros2 topic echo /camera/image_raw --once --field header.frame_id
```

`ros2 bag info` lists topic names and message types/counts, but not the frame IDs inside
those messages.

## 4. Launch assisted calibration

Use three terminals. Start the graph first, then the image bridge, and start bag
playback only after the graph and bridge are ready. This avoids losing the opening
background frames before the subscribers have connected.

Terminal 1 — from the LCTK repository root:

```bash
just solver_mode=assisted run ~/calib/lab1/session
```

Terminal 3 — source ROS and the workspace, then bridge the compressed camera stream
to the raw topic named in the session:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run image_transport republish compressed raw \
    --ros-args \
    -r in/compressed:=/camera/image_raw/compressed \
    -r out:=/camera/image_raw
```

Wait for the calibration nodes and republisher to start. In terminal 2, before
playing the bag, use the verbose endpoint list to confirm that the LiDAR detector
subscribes to the point cloud, the republisher subscribes to the compressed image,
the ArUco locator subscribes to the raw image, and the solver subscribes to
CameraInfo. A subscriber count alone is not enough: RViz can also subscribe to sensor
topics.

```bash
ros2 topic info /velodyne_points --verbose
ros2 topic info /camera/image_raw/compressed --verbose
ros2 topic info /camera/image_raw --verbose
ros2 topic info /camera/camera_info --verbose
```

Terminal 2 — source ROS and play the recording:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/calib/lab1/bag --clock
```

Open the assisted review page at <http://localhost:8080>. Review the Captures and
Quality Verdict. The collection targets are guidance, not a guarantee of a valid
calibration: aim for at least ten distinct placements, 1.5 m of depth range, 1 m of
lateral spread, and 20 degrees of board-normal variation. If the estimate will be
deployed, check it against independent observations and the real rig. Do not deploy a
result whose verdict reports insufficient geometry.

When the review is complete, choose **Export archive**. The session saves it at
`~/calib/lab1/session/out/detections.json` as configured above.

## 5. Export to Autoware

Use a working copy of the vehicle's existing `sensor_kit_calibration.yaml`. The
exporter needs the existing LiDAR entry and the Autoware camera-link frame to add or
update; that camera-link frame may differ from the optical frame in the image messages.
The LiDAR entry must represent the same coordinate frame as the point cloud's
`header.frame_id`; the exporter does not use TF to convert between different LiDAR
frames. In this example both are `velodyne`. Preview the proposed change first, and
write only after checking the values and confirming the estimate is suitable for the
vehicle:

```bash
ros2 run lctk_autoware_export export \
    --detections ~/calib/lab1/session/out/detections.json \
    --target /path/to/sensor_kit_calibration.yaml \
    --camera-frame camera_link \
    --lidar-frame velodyne \
    --dry-run
```

Check the frames and values in the preview. Then run the same command without
`--dry-run` to write the entry. The exporter creates a `.bak` file next to the YAML on
its first write. The CLI exports the solved transform stored in the Detection Archive;
it does not apply a separately adjusted transform from the review page. See
[Exporting to Autoware](./autoware-export.md) for kit-frame options and verification.
For an offline check against separately captured observations, see
[Extrinsic Validation](./extrinsic-validation.md).
