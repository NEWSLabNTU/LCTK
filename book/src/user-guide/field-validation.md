# Field Validation Runbook

This runbook covers a current field run using a session manifest, real sensor data,
and either manual or assisted capture. It does not treat a successful numerical solve
as proof that the extrinsic is physically correct.

## 1. Prepare a session

Start from the closest shipped session rather than maintaining a detached YAML file:

```bash
source install/setup.bash
ros2 run lctk_launch lctk_session new ~/calib/my-rig \
    --from $(ros2 pkg prefix lctk_launch --share)/sessions/seyond-left
```

Edit `~/calib/my-rig/session.yaml` so that it describes the actual rig:

- use the correct `data.kind` (`live`, `bag`, or `pcap_avi`);
- state each device's topic and frame;
- select the physical Calibration Target with `target_config`;
- select sensor-specific Detector Tuning with `detector_config`;
- add `bbox_config` only when the selected preset uses `detection_mode: "bbox"`;
- set the required `sync:` section;
- add `data.republish` when a compressed camera topic must become a raw
  `sensor_msgs/Image` topic.

For a bag-backed session, make the bag available at the manifest's `data.path`.
For a live Seyond session, the camera may publish `CompressedImage`; use the
republish command documented in that session's README or declare the bridge in the
manifest.

Validate before starting any nodes:

```bash
ros2 run lctk_launch lctk_session check ~/calib/my-rig
```

The check resolves paths, confirms required files, and verifies bag topics against
the bag metadata.

## 2. Start the graph

For a bag-backed or pcap/AVI session, run the complete session:

```bash
just solver_mode=manual enable_judge=false run ~/calib/my-rig
```

This starts the session data source and calibration graph. The `just` recipe uses
`play_launch` and exposes its launch status page at <http://localhost:8000>.

For a live rig, the same command starts the calibration nodes and any configured
`data.republish` bridges; the sensors must already be publishing. If data is played
independently, start only the calibration graph:

```bash
just solver_mode=manual enable_judge=false calibrate ~/calib/my-rig/session.yaml
```

Direct launch is also supported:

```bash
ros2 launch lctk_launch session.launch.py \
    session:="$HOME/calib/my-rig" \
    solver_mode:=manual \
    enable_judge:=false
```

The direct launch defaults differ from the justfile. The justfile defaults to assisted
mode and enables debug, overlay, and judge; direct session launch defaults to continuous
mode with those optional features disabled. State the desired values explicitly for a
field run.

Disable the judge unless a matching ground-truth file is intentionally configured for
the rig. A judge result is not a substitute for field validation.

## 3. Observe the running system

Topic namespaces come from the manifest:

```
/calibration/<lidar>_<marker>/calibration_board_detections
/calibration/<camera>/aruco_detections
/calibration/<lidar>_<camera>/extrinsic_transform
```

For a multi-LiDAR pair, the output is instead:

```
/calibration/<lidar1>_<lidar2>/lidar_to_lidar_transform
```

Use `ros2 topic list`, `ros2 topic hz`, and `ros2 topic echo` with the concrete
names printed by `lctk_session check` and the session manifest. Do not assume a
universal detection frequency.

If debug output is enabled, inspect the namespaced debug topics created for the
detector. If the overlay is enabled, inspect the pointcloud overlay and confirm the
projected points agree with the camera image.

## 4. Collect multiple poses

For manual capture, run:

```bash
just extrinsic-solver-controller
```

The controller discovers the solver services. Services are available in manual and
assisted modes, but not continuous mode.

For assisted capture, use `solver_mode=assisted` and review the page on port 8080.
The node queues synchronized observations when its stillness and novelty gates pass.
The browser can remove captures and re-solve the current Detection Buffer.

A Capture is one synchronized camera/LiDAR observation retained in the buffer. Several
Captures can describe the same Board Placement. Move the Calibration Target through
distinct positions and orientations rather than relying only on repeated observations
of one placement.

Record the conditions of the run and retain the session manifest, Detector Tuning
files, bag metadata, and output archive together. The exact number of captures and
the useful placement geometry depend on the rig and are not established by a
universal threshold in the repository.

## 5. Review the result

Separate these outcomes:

- a numerical solve produces a **Solved Estimate**;
- the quality code produces a **Quality Verdict** about the geometry in the current
  Detection Buffer;
- a manually edited result is an **Adjusted Transform**;
- an archive records a buffer revision, verdict, and optional adjustment.

A low residual by itself does not establish that the physical transform is correct.
Check the overlay, frame conventions, target identity, placement coverage, and
repeatability on independently selected data.

When an archive is saved, keep the version and Target Identity with the field record.
The current LiDAR-camera archive format is version 5. The migration utility supports
the explicit transitions v3 to v4 and v4 to v5; v1 and v2 are not supported by those
commands.

## 6. Export only after review

Use the raw detection archive with
[Exporting to Autoware](./autoware-export.md). The exporter accepts version 4 and
version 5 archives with the supported board-frame convention and refuses archives it
cannot validate.

The published ROS transform has the correct frame labels. The numeric value used by
the exporter comes from the raw archive transform, whose coordinate representation is
composed into the Autoware camera-link entry by the exporter.

## Field record

For each sensor-target campaign, keep:

- the session manifest and the exact Target Definition and Detector Tuning files;
- the data source path and, for bags, `metadata.yaml`;
- the synchronization settings and observed data conditions;
- the captures kept and removed during review;
- the Solved Estimate, Quality Verdict, and any Adjusted Transform;
- overlay or TF verification results;
- the archive and any export preview/write result.

This record is the evidence for deciding whether a rig-specific calibration is ready
for use. Repository examples and detection smoke tests do not replace that field
validation.
