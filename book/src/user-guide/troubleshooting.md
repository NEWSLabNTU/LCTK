# Troubleshooting

Use the current session manifest and the running graph as the source of truth for
topics, frames, Target Definitions, and Detector Tuning.

## Start with validation

Before launching a session:

```bash
source install/setup.bash
just check <session-name-or-path>
```

For a bag, this checks that the manifest's device topics occur in `metadata.yaml`.
It also catches missing data, missing session-local files, and invalid path
substitutions before the graph starts.

If the graph is already running:

```bash
ros2 topic list
ros2 node list
```

Use the concrete topic and frame names from the manifest. Generic names such as
`/aruco_detections` or `/calibration_transform` are not the names published by
the current nodes.

## Setup and build failures

### Commands are missing

Source the ROS and workspace environments:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If the setup script has not completed, rerun it from the repository root. Use flags,
not positional step names:

```bash
./setup.sh --status
./setup.sh --verify
./setup.sh --dry-run
```

The default setup plan installs the required system, ROS, Rust, Python, and build
tools. Optional steps can be selected with `--only` or omitted with `--skip`.

### Missing C/C++ or geometric libraries

Use the setup steps so the repository's pinned dependencies remain consistent:

```bash
./setup.sh --only build-tools
./setup.sh --only geometric-libs
```

### Python packages shadow ROS packages

Do not install ROS/OpenCV dependencies with `pip3 install --user`. User-site
versions of setuptools, NumPy, SciPy, or pytest plugins can shadow the apt versions
used by ROS 2 Humble and break builds, imports, or tests.

## No detections

### Check the data source

For a pcap/AVI or bag session, confirm that the data source is part of the launch:

```bash
just run <session-name-or-path>
```

For a live session, confirm the sensors are publishing before starting the calibration
graph. For a plain calibration YAML, play the recording separately and then run
`just calibrate`.

If the camera publishes `CompressedImage` but the camera detector expects raw
`Image`, add a `data.republish` bridge to the session or run the corresponding
`image_transport republish compressed raw` command.

### Check the actual topics

Use the session's device names and marker names:

```bash
ros2 topic hz /calibration/<camera>/aruco_detections
ros2 topic hz /calibration/<lidar>_<marker>/calibration_board_detections
```

For a bag, a missing topic should be fixed in the manifest after inspecting
`metadata.yaml`. A topic that exists in the bag under a different name is not
automatically remapped.

### Check the target and detector files

The marker entry must name both:

```yaml
target_config: $(find-pkg-share lctk_launch)/config/targets/<target>.json5
detector_config: $(find-pkg-share lctk_launch)/config/board/<target>/<sensor>.json5
```

If the detector preset selects `detection_mode: "bbox"`, its marker entry must also
name the session-local crop box:

```yaml
bbox_config: $(session-dir)/bbox.json5
```

Do not copy plate geometry into Detector Tuning. The physical target and its
Target Definition must match.

### Check transport reliability

For bag sessions, inspect the offered QoS profiles in `metadata.yaml`. A manifest
value of `qos: reliable` is incompatible with a BEST_EFFORT publisher; use the
recording's offered reliability or state `best_effort` for that device.

### Check Target Identity

The LiDAR detector, camera detector, and solver must use matching Target Identity
values. If the Target Definition changed, restart the complete session so all
observers and the solver load the same identity.

## Poor or surprising results

- Confirm the camera intrinsics and camera-info topic/file.
- Confirm the LiDAR and camera observe the same Calibration Target at the same time.
- Check that the selected Detector Tuning belongs to the actual sensor.
- Use distinct Board Placements for manual or assisted capture.
- Inspect the overlay when `enable_overlay:=true`.
- Treat a Quality Verdict as a geometric assessment, not proof that the physical
  transform is correct.

The repository does not define universal distances, detection rates, capture counts,
or performance thresholds. Record rig-specific operating limits from field data.

## Debugging and logs

Enable debug output with a justfile variable before the recipe:

```bash
just debug_mode=true calibrate /path/to/session.yaml
```

Debug topics are namespaced by the device and marker, for example:

```
/calibration/<lidar>_<marker>/debug/plane_inliers
/calibration/<lidar>_<marker>/debug/final_board_pose
/calibration/<camera>/image_with_detections
```

Use `ros2 topic list` to discover the exact topics emitted by the selected nodes.
The justfile's launch recipes use `play_launch`; its recorded run data is under
`play_log/`. ROS node logs are under the system ROS log directory unless
`ROS_LOG_DIR` is set.

## Runtime issues

### ROS 2 daemon is unresponsive

```bash
pkill -9 -f ros2-daemon
```

### Text file busy during a rebuild

Stop running LCTK nodes, remove only the affected package's build/install entries,
and rebuild from the repository root:

```bash
pkill -9 -f "<node_name>"
rm -rf build/<package> install/<package>
just build
```

If interface bindings were changed or a binding path is missing, follow the binding
cleanup instructions in the repository's build documentation before rebuilding.

### RViz shows no data

Check:

```bash
echo $ROS_DOMAIN_ID
ros2 topic list
ros2 node list
```

Use the frame IDs and topics from the session manifest. A session may provide its own
`rviz.rviz`; otherwise pass an explicit `rviz_config:=...` or run `just rviz`.

## Before reporting a problem

Include:

1. the session manifest and the selected Target Definition/Detector Tuning paths;
2. the output of `just check <session>`;
3. `ros2 topic list` and `ros2 node list`;
4. the relevant error or refusal message;
5. whether the data source is live, pcap/AVI, or bag, including bag metadata when
   applicable.

Run commands from your repository checkout, not from a machine-specific path. See
[Installation](./installation.md), [Calibration Sessions](./sessions.md), and
[Configuration](./configuration.md) for the supported setup and launch forms.
