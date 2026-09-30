# Field Validation Runbook

Use this checklist for a real rig. A successful numerical solve is not, by itself,
proof that the transform is correct.

This runbook assumes LCTK is installed and the workspace is built. Before using direct
`ros2` commands in a new terminal, source ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 1. Prepare and validate the session

Start from the closest session, then edit it for the actual sensors and target:

```bash
ros2 run lctk_launch lctk_session new ~/calib/my-rig \
    --from "$(ros2 pkg prefix lctk_launch --share)/sessions/seyond-left"
```

Confirm the data source, topic and frame IDs, camera image type, Target Definition,
Detector Tuning, and synchronization settings. If the selected Detector Tuning uses
`detection_mode: "bbox"`, provide a crop box measured for this recording; `bbox_free`
presets do not use one. See
[Sessions](./sessions.md) and [Configuration](./configuration.md).

```bash
just check ~/calib/my-rig
```

Do not continue until the check succeeds and the required bag or live sensor topics are
available.

## 2. Start the rig and verify inputs

For a live rig, start the sensors first. For a bag-backed or pcap/AVI session, LCTK
starts the data source with the graph:

```bash
just solver_mode=assisted enable_judge=false run ~/calib/my-rig
```

The launch status page is at <http://localhost:8000>; assisted review is at
<http://localhost:8080>. Confirm that the camera image and LiDAR detections are
arriving, and that both sensors see the same physical Calibration Target. Use
`just check` to confirm the configured sensor topics and frames; use the manifest and
`ros2 topic list` for the corresponding detector and solver topics.

If your workflow requires operator-selected captures, use `solver_mode=manual` and
the interactive controller instead. See the manual workflow in
[LiDAR-Camera Calibration](./lidar-camera.md).

## 3. Collect and review captures

Move the target through distinct positions and tilts in the sensors' shared view. For
assisted capture, pause at each placement and watch the page's stillness and placement
coverage feedback. For manual capture, add only synchronized observations you intend
to keep.

Review the image evidence and remove poor captures. Check the Quality Verdict as an
assessment of the current Capture set, not as a pass/fail claim about the physical
rig. Preserve the Detection Archive with the session and configuration used to create
it.

## 4. Validate independently

Before deployment, review the transform's frame IDs and direction, compare the
projected point cloud with the camera image where available, and check the result on
observations not used to estimate it. Repeat the capture if the estimate changes
substantially when poor observations are removed or when the target is measured at
different placements.

The repository does not define universal placement counts, distances, residual limits,
or acceptable repeatability for every rig. Establish those acceptance limits for the
actual sensors and application.

## 5. Export only after review

If the downstream system is Autoware, use the raw Detection Archive with
[Exporting to Autoware](./autoware-export.md). Preview the proposed entry, check the
frame names and values, then write and verify it on the target system.

## Keep a field record

Retain the session manifest, exact Target Definition and Detector Tuning, data source
and bag metadata (if applicable), the Detection Archive, review decisions, validation
observations, and any export preview/write result. These records make the calibration
reproducible and allow later review of which physical setup it describes.
