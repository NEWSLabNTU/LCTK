# Calibration sessions — design

- **Date:** 2026-08-31
- **Status:** Approved, not yet implemented
- **Area:** `lctk_launch`, `lctk_sample_data`, justfile, `sessions/`
- **Breaking:** yes — `config/examples/*.yaml` is replaced by `sessions/`

## The problem

Switching datasets means hand-editing the same facts in several places, and nothing
checks that the edits agree.

A run today is described by two files that must match but never reference each other:

- `ros/lctk_sample_data/launch/lidar_camera.launch.xml` — 13 arguments naming the pcap,
  the video, the topics, the frame ids and the camera intrinsics.
- `ros/lctk_launch/config/examples/<name>.yaml` — a `devices:` section restating those
  topics and frame ids, plus the target, detector tuning, crop box and sync window.

`just demo` passes arguments to neither: it is pinned to dataset 3 and `sample_data.yaml`
by the body of `demo.launch.py`.

The cost is not hypothetical. Three findings in the tracker are this one bug wearing
different clothes:

- **[M-26](../../issues/M-26-two-lidar-example-topics-unreachable.md)** — `two_lidar.yaml`
  names topics no in-repo data source publishes.
- **[M-27](../../issues/M-27-solid-600-handheld-topics-alias-sample-data.md)** —
  `solid_600_handheld.yaml`'s placeholder topics collide with the sample-data playback,
  so it silently calibrates against the wrong recording.
- **[M-29](../../issues/M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md)** —
  `config/board/bbox.json5` was retuned for a Seyond rosbag, which killed the shipped
  demo. One crop box, two recordings, different rig geometry.

`config/board/` already shows where this ends: seven crop boxes — `bbox.json5`,
`bbox_v1.json5`, `bbox-seyond.json5`, `bbox-vlp.json5`, `bbox_2_lidar_seyond.json5`,
`bbox_2_lidar_vlp32.json5`, `sample_data_bbox.json5` — with no way to tell which
recording any of them describes.

Every one of these failures is silent. A wrong topic produces a healthy-looking graph and
no data; a wrong crop box produces a detector that publishes empty results forever.

## The idea

**A session is one directory describing one run: where the data comes from, and everything
needed to calibrate against it.** It is self-contained and relocatable, and it can live
anywhere — inside this repo or in the operator's own tree.

The split is between what belongs to a *recording* and what belongs to the *world*:

| session-local | shared library |
|---|---|
| data source (pcap/avi dir, bag path, or live) | target definitions — a physical board |
| topics and frame ids | detector tuning per (target, sensor model) |
| crop box — where the board sat in *this* recording | ArUco detector tuning |
| camera intrinsics | |
| sync window — how fast the board moved *here* | |
| judge ground truth — this rig | |
| RViz layout | |

A crop box is the clearest case: it describes where a board was during one recording, so it
cannot be shared, and sharing it is exactly what broke the demo.

## File structure

```
sessions/                                  # shipped sessions; a normal location, not a special one
  sample3-hollow-velodyne/
    session.yaml
    bbox.json5
    camera_info.yaml
    rviz.rviz                              # optional
    README.md                              # what this recording is, how it was taken
    out/                                   # gitignored: detections.json, exports, logs
  solid600-handheld-zed/
  twolidar-vlp32-falcon/
  ...

ros/lctk_launch/config/                    # shared library, unchanged
  targets/  hollow_1000_aruco_4_v1.json5
  board/    hollow_1000/velodyne.json5
  aruco/    aruco_detector.json5
```

An operator's own session is the same shape, anywhere on disk:

```
~/calib/rig-a/
  session.yaml
  bbox.json5
  camera_info.yaml
  my_board.json5                           # optional: their own target definition
  data/                                    # optional: or point elsewhere
  out/
```

`sessions/` is installed into `share/lctk_launch/sessions/` the way `config/` already is,
so shipped sessions are runnable from any working directory after a build.

## The manifest

One file, replacing the split between the playback arguments and the calibration config.

```yaml
name: sample3-hollow-velodyne
description: >
  Dataset 3 from lctk_sample_data: VLP-32C pcap plus camera avi, hollow 1000 board,
  a single board placement. The shipped demo.

data:
  kind: pcap_avi                                    # pcap_avi | bag | live
  dir: $(find-pkg-share lctk_sample_data)/data/3
  lidar:
    model: vlp32c
    rpm: 600
  camera:
    info_url: $(session-dir)/camera_info.yaml

devices:
  lidars:
    top_lidar:
      frame_id: velodyne_top                        # topic derived; see below
  cameras:
    front_center:
      frame_id: camera_front_center

markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne_bbox.json5
    bbox_config: $(session-dir)/bbox.json5
    pairs:
      - [top_lidar, front_center]

sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new

assisted:                                           # optional, solver_mode=assisted only
  review_archive_path: $(session-dir)/out/detections.json
```

### `$(session-dir)`

A new substitution alongside the existing `$(find-pkg-share …)`, resolving to the directory
of the manifest that used it. It is what makes a session directory relocatable: nothing
inside points into LCTK by absolute path, so the directory can be copied to another machine
and still run.

`resolve_package_path(path)` becomes `resolve_config_path(path, session_dir)`. Every existing
call site passes the manifest's directory. A `$(session-dir)` in a file loaded from somewhere
without a directory context is an error naming the offending key, not a silent empty string.

### Topics: derived, verified, or stated

The `data.kind` decides, because what is knowable differs:

| kind | topics | rationale |
|---|---|---|
| `pcap_avi` | **derived** — absent from the manifest | LCTK drives this playback, so one source feeds both the player and the calibration graph. A mismatch becomes unrepresentable rather than merely unlikely. |
| `bag` | **stated**, then verified against the bag's `metadata.yaml` | The recording fixes them. Launch refuses with the topic list the bag actually has, turning M-26 from a silent hang into a startup error. |
| `live` | **stated** | Nothing to check against before the sensor is up. |

Derived names follow one documented convention:

```
/sensing/lidar/<device_name>/pointcloud_raw
/sensing/lidar/<device_name>/velodyne_packets
/sensing/camera/<device_name>/image_raw
/sensing/camera/<device_name>/camera_info
```

These are exactly the defaults `lidar_camera.launch.xml` already carries, so dataset 3
keeps its current topic names and the convention is a description of existing behaviour
rather than a new one.

Under `bag` and `live`, `topic:` is required on each device; under `pcap_avi` it is
**refused**, because accepting it would reintroduce the two-sources-of-truth bug the
manifest exists to remove.

## Running it

The interface is plain ROS 2. `just` is an alias layer and nothing more.

```bash
# End to end: data playback plus the calibration pipeline
ros2 launch lctk_launch session.launch.py session:=~/calib/rig-a

# A shipped session, using the idiom the justfile already uses for config_file
ros2 launch lctk_launch session.launch.py \
    session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne

# Any existing launch argument still applies
ros2 launch lctk_launch session.launch.py \
    session:=~/calib/rig-a solver_mode:=assisted enable_rviz:=false

ros2 launch lctk_launch session.launch.py --show-args
```

`session:=` is **always an explicit path** — absolute, or relative to the working directory.
It accepts either the session directory (it then looks for `session.yaml` inside) or the
manifest file itself. There is no search path and no implicit `./sessions`: an implicit
location would assume both where sessions live and where the user is standing, and the whole
point is that a session can live in the operator's own tree.

The two halves run separately, which is what a live rig or an externally-played bag needs:

```bash
ros2 launch lctk_launch session_data.launch.py session:=~/calib/rig-a    # terminal 1
ros2 launch lctk_launch calibrate.launch.py config_file:=~/calib/rig-a/session.yaml
```

`calibrate.launch.py` keeps its current `config_file:=` interface unchanged. A session
manifest is a valid calibration config; the `data:` section is simply not read by it.

### Session management

```bash
ros2 run lctk_launch lctk_session list                    # ./sessions and the installed share
ros2 run lctk_launch lctk_session list ~/calib            # an explicit collection
ros2 run lctk_launch lctk_session check ~/calib/rig-a     # validate, launch nothing
ros2 run lctk_launch lctk_session new  ~/calib/rig-b --from sample3-hollow-velodyne
```

One more `console_scripts` entry beside the existing `tf_tree_broadcaster`.

`check` is the piece worth having: it resolves every path in the manifest, confirms the data
exists, verifies bag topics against `metadata.yaml`, and reports what it found — without
starting a graph. It is the answer to "why is nothing being detected", asked before the run
rather than after.

### The justfile

| recipe | expands to |
|---|---|
| `just run <path-or-name>` | `ros2 launch lctk_launch session.launch.py session:=…` |
| `just check <path-or-name>` | `ros2 run lctk_launch lctk_session check …` |
| `just sessions` | `ros2 run lctk_launch lctk_session list` |
| `just new <path>` | `ros2 run lctk_launch lctk_session new …` |
| `just demo` | alias for `just run sample3-hollow-velodyne` |

Name resolution — try `./sessions/<name>`, then the installed share — lives **here**, in the
alias layer, where being opinionated is free. The launch interface stays honest.

## Preparing a new experiment

1. `ros2 run lctk_launch lctk_session new ~/calib/rig-b --from sample3-hollow-velodyne`
2. Edit `session.yaml`: point `data:` at the bag, pcap directory or live topics; name the
   devices and their frames; pick the target and detector preset.
3. Tune `bbox.json5` with `just filter-box-tuner` if the preset needs a crop box.
4. `ros2 run lctk_launch lctk_session check ~/calib/rig-b`
5. `ros2 launch lctk_launch session.launch.py session:=~/calib/rig-b`

## Components

| unit | responsibility | depends on |
|---|---|---|
| `session.py` (new, in `lctk_launch`) | resolve a session path to a manifest and its directory; validate `data:`; derive or verify topics | `yaml`, bag `metadata.yaml` |
| `config_parser.py` | gains the `data:` section and `$(session-dir)`; unchanged otherwise | `session.py` |
| `session_data.launch.py` (new) | start the data source for a manifest. For `pcap_avi`, include the existing `lidar_camera.launch.xml` with derived arguments — reuse, not reimplementation. For `bag`, `ros2 bag play`. For `live`, nothing. | `session.py` |
| `session.launch.py` (new) | include `session_data.launch.py`, then `calibrate.launch.py` with the same manifest | both of the above |
| `lctk_session` (new console script) | `list` / `check` / `new` | `session.py` |
| `demo.launch.py` | deleted — `session.launch.py` generalizes it | |

`session.py` holds no ROS types and no launch objects, so its validation, derivation and bag
verification are unit-testable without a graph. That is where the logic goes; the launch
files stay thin.

## Error handling

| situation | behaviour |
|---|---|
| `session:=` path does not exist | refuse, naming the path that was tried |
| directory given but no `session.yaml` inside | refuse, naming the directory and what was expected |
| `data.dir` / `data.path` missing on disk | refuse, naming the resolved absolute path — not the unresolved substitution |
| `topic:` present under `kind: pcap_avi` | refuse, explaining that the topic is derived and why stating it would reintroduce two sources of truth |
| `topic:` absent under `bag` or `live` | refuse, naming the device |
| bag lacks a named topic | refuse, listing the topics the bag does contain |
| `$(session-dir)` used without a directory context | refuse, naming the key |
| unknown key in `data:` | refuse, listing the known keys — the same rule the `assisted:` section already uses |

Every one of these is a startup refusal rather than a warning. The failures this design
exists to prevent are all silent; converting them into loud ones at launch is the point.

## Migration

All six `config/examples/*.yaml` become sessions:

| example | session | note |
|---|---|---|
| `sample_data.yaml` | `sample3-hollow-velodyne` | absorbs `config/board/sample_data_bbox.json5` |
| `seyond_left.yaml` | `seyond-left` | `kind: live` |
| `seyond_right.yaml` | `seyond-right` | `kind: live` |
| `solid_600_handheld.yaml` | `solid600-handheld-zed` | `kind: live`; closes M-27 by declaring a real source |
| `two_lidar.yaml` | `twolidar-vlp32-falcon` | `kind: bag`, `TWO_LIDAR_1`; closes M-26 by verifying topics |
| `vehicle.yaml` | `vehicle-multisensor` | `kind: live`; a schema example, marked as such |

`config/examples/` is then deleted. The crop boxes under `config/board/` move into the
sessions that own them; any left with no owner are deleted rather than kept as mystery files.

This is a breaking change: local scripts or notes naming `config/examples/…` will break. The
replacement path is mechanical, and `lctk_session check` names the manifest it read, so a
stale path fails loudly.

## Testing

| unit | test |
|---|---|
| path resolution | `$(session-dir)` resolves against the manifest's directory; a bare path stays relative to it; a missing directory context is an error |
| topic derivation | `pcap_avi` yields the documented names for a given device name; stating `topic:` is refused |
| bag verification | a manifest naming a topic the bag lacks is refused, and the message lists the bag's actual topics; a matching one passes |
| `check` | reports missing data, missing files and bad topics without starting a graph |
| launch graph | `session.launch.py` on a `pcap_avi` manifest generates both the playback and the calibration nodes, and the topics on both sides are equal — the property the whole design exists to guarantee |
| migration | each converted session parses, and its resolved device topics match what the old example named |

The launch-graph test is the important one: it asserts the *agreement* between the two
halves, which is the thing no current test covers and the thing that keeps breaking.

## What this does not do

- It does not verify that a live sensor is publishing before launching. A startup probe with
  a timeout would make a slow sensor look like a config error.
- It does not move or copy recordings. Data is referenced, so one recording can back several
  sessions, and a live session has no data files at all.
- It does not change the calibration pipeline itself. Detectors, synchronizer, solvers and
  the export path are untouched; only how a run is described and started.
