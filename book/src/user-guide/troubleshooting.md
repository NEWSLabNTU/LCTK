# Troubleshooting

Start by validating the session and checking the actual topics in the running graph.
Names depend on the devices and marker names in your manifest.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
just check sample3-hollow-velodyne
ros2 topic list
ros2 node list
```

Replace the sample session name with your own session name or path.

## The session will not start

- Run `just check` and fix the reported missing file, invalid setting, or topic
  mismatch before launching again.
- For a bag, confirm that `data.path` points to a rosbag2 directory containing
  `metadata.yaml`, and that the configured LiDAR and camera topics are recorded.
- For `pcap_avi`, confirm that the session's data directory contains both
  `lidar.pcap` and `video.avi`.
- For a live rig, check the sensors are publishing the exact topic names and frame IDs
  in `session.yaml`.

## No camera detections

- Confirm the camera image topic has messages and the target's fiducial markers are
  visible in the image.
- LCTK's detector expects raw `sensor_msgs/Image`. If the camera or bag publishes
  compressed images, configure `data.republish` in the session.
- Confirm CameraInfo is available on the topic derived from the image topic, or set
  `data.camera.info_url` for pcap/AVI playback. See [Configuration](./configuration.md)
  for the topic naming rule.
- Make sure the target's marker IDs and layout match its Target Definition.

## No LiDAR detections

- Confirm that the LiDAR topic is active and that the target is in the sensor's view.
- Check that `target_config` matches the physical target and `detector_config` is
  intended for that target and LiDAR.
- For a preset using `detection_mode: "bbox"`, verify `bbox_config` is measured for
  this recording and covers the target. Do not add a box for `bbox_free` presets.
- If using background subtraction, allow the configured warmup to observe the scene
  without the target before capture.

## Sensors publish, but there are no synchronized pairs

- Confirm both sensors use compatible timestamps and that both see the target at the
  same time.
- Check `sync.tolerance_ms` against the actual camera/LiDAR timing and target motion.
- For a bag, inspect its offered reliability in `metadata.yaml`. A `qos: reliable`
  subscription cannot receive from a BEST_EFFORT publisher; use the offered QoS or
  configure `best_effort` for that device.

## The estimate is unstable or the Quality Verdict is degenerate

- Capture the target at several distinct positions and tilts. Repeating one placement
  adds observations but little geometric variety.
- Review captures and remove blurred images, occlusions, or poor LiDAR detections.
- Check the physical target, Target Definition, and Target Identity agree across the
  sensors.
- Treat residuals and the Quality Verdict as evidence about the captured data, not as
  independent proof that the transform is correct. See
  [Field Validation](./field-validation.md).

## Assisted review does not open

Assisted capture requires `solver_mode=assisted`. With the `just` launch recipes, the
review page normally uses <http://localhost:8080>. Check the solver logs for a bind
error or an already-used port. The page binds to localhost by default; see
[Assisted Capture](./assisted-capture.md) before exposing it to a network.

## A command is missing after installation

If `just` is missing after setup, open a new login shell (or log out and back in) to
load the updated command path. From the repository root, source ROS and the built
workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If a build or dependency installation is failing, see
[Installation](./installation.md). For build-system or test failures, use the
[Developer Guide](../developer-guide/build-system.md).
