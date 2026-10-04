# LiDAR-Camera Calibration

LCTK estimates the transform between a LiDAR frame and a camera frame by observing
the same physical Calibration Target in both sensors. Use the
[Quick Start](./quickstart.md) for the included sample; this guide covers a rig or
recording of your own.

## Before you run

- Mount or hold a Calibration Target that matches the selected Target Definition,
  including its marker layout and orientation.
- Ensure the camera and LiDAR have overlapping views of the target and usable
  timestamps.
- Check that the session names the correct sensor topics, target, and sensor-specific
  Detector Tuning. Frame labels come from sensor message headers at runtime. See
  [Sessions](./sessions.md) and
  [Configuration](./configuration.md).
- For a bag, make sure the recording is available where the manifest expects it.

Validate the session before starting the graph:

```bash
just check /path/to/session
```

## Run and capture

For a live rig, start the sensors first. For a bag or pcap/AVI recording, the session
launch starts playback as well. Run assisted capture and disable the optional quality
judge unless the session has matching ground-truth data:

```bash
just solver_mode=assisted enable_judge=false run /path/to/session
```

The launch status page is at <http://localhost:8000>; assisted review is at
<http://localhost:8080>. In the review page, move the target through several distinct
positions and tilts, pausing at each one. The system captures synchronized observations
when it considers the target still and in a new placement. Review the evidence, remove
captures that are blurred, occluded, or otherwise unsuitable, and assess the current
estimate and Quality Verdict before using it. The verdict describes how strongly the
collected geometry constrains the estimate; it is not a physical validation of the rig.

If you prefer to choose each capture yourself, use manual mode and the interactive
controller:

```bash
just solver_mode=manual enable_judge=false run /path/to/session
```

In another terminal:

```bash
just extrinsic-solver-controller
```

The controller exposes the capture-buffer operations for the running solver. The
assisted and manual workflows are described in more detail in
[Assisted Capture](./assisted-capture.md) and the [Field Validation Runbook](./field-validation.md).
For a held-out reprojection check, see [Extrinsic Validation](./extrinsic-validation.md).

## Inspect the transform and visualize the fit

The output topic is derived from the sensor names in the session:

```text
/calibration/<lidar>_<camera>/extrinsic_transform
```

For example, the sample session publishes
`/calibration/top_front_center/extrinsic_transform`. Inspect your session's concrete
name with `ros2 topic list`, then read the transform (replace the sample topic if
needed):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /calibration/top_front_center/extrinsic_transform
```

The `just` run commands enable the point-cloud overlay and RViz by default. The default
RViz layout displays the overlay image on `/calibration/pointcloud_overlay`; a
session-specific `rviz.rviz` may use a different layout. For direct `ros2 launch`,
enable the overlay explicitly with `enable_overlay:=true`.
The overlay helps check the estimated alignment, but a plausible image alone does not
prove the transform is correct. Check frame IDs, Target Identity, repeatability, and
independent field measurements as part of validation.

## Choosing settings

Target geometry belongs in the Target Definition. Detector Tuning is selected for the
target and LiDAR model; do not reuse a preset just because its filename looks similar.
For a `bbox` preset, the session also needs a crop box measured for that recording. A
`bbox_free` preset does not use one.

Camera images must be available as raw `sensor_msgs/Image` messages. If a camera or bag
provides compressed images, configure a republish bridge in the session. The full
session schema and the available target/detector files are documented in
[Configuration](./configuration.md).
