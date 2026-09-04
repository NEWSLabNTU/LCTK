# Calibration Sessions

A **session** is one directory that describes one calibration run: where the data comes
from, and everything needed to calibrate against it.

Everything below is plain `ros2 launch` and `ros2 run`. The `just` recipes shown at the end
are aliases over exactly these commands — nothing in LCTK requires `just`.

## Why a session

Before sessions, a run was described by two files that had to agree but never referenced
each other: a playback launch file naming a recording and its topics, and a calibration
config restating those topics by hand. When they disagreed the pipeline still launched
cleanly, every node came up healthy, and nothing was ever detected. Three tracker findings
were that one bug in different clothes — a config naming topics no recording published, a
config whose placeholder topics collided with a different recording's, and a crop box shared
between two rigs with different geometry.

A session closes the gap by making both halves read one file, and by refusing at startup
what used to fail silently at run time.

## What is in a session directory

```
<name>/
  session.yaml       # the manifest
  README.md          # what the recording is, and whether the data ships
  data/              # optional: the recording itself, when it is small enough to ship
  bbox.json5         # optional: the crop box for THIS recording
  camera_info.yaml   # optional: intrinsics for THIS camera
  rviz.rviz          # optional: the RViz layout for THIS experiment
  out/               # run outputs — gitignored
```

A session that ships its recording keeps it in its own `data/`, reached as
`$(session-dir)/data`. The five pcap/avi sample recordings used to live in
`lctk_sample_data` and be reached across packages; each now sits inside the session that
describes it, so copying the directory copies the run. A large recording is still
referenced rather than moved — the two-LiDAR bags are gitignored and symlinked in.

The rule for what belongs here is whether the file describes a *recording* or the *world*.
A crop box says where a board sat during one recording, so it is session-local. A target
definition describes a physical board that exists independently of any recording, so it
stays in the shared library under `lctk_launch/config/`:

| session-local | shared library (`lctk_launch/config/`) |
|---|---|
| data source, the recording, topics, frame ids | `targets/` — a physical board |
| crop box, camera intrinsics | `board/` — detector tuning per (target, sensor) |
| sync window, RViz layout | `aruco/` — ArUco detector tuning |

The shipped sessions live at `sessions/` in the repo and are installed to
`share/lctk_launch/sessions/`. That is a normal location, not a special one — your own
sessions can live anywhere.

## The manifest

`session.yaml` is a normal calibration config plus a `data:` section, so everything
[Configuration](./configuration.md) documents still applies.

```yaml
name: sample3-hollow-velodyne
description: >
  A VLP-32C pcap and a camera avi, hollow 1000 board, one board placement.
  The recording ships in git, in this session's own data/.

data:
  kind: pcap_avi                                   # pcap_avi | bag | live
  dir: $(session-dir)/data
  lidar: { model: vlp32c, rpm: 600 }               # optional
  camera: { info_url: $(session-dir)/camera_info.yaml }   # optional

devices:
  lidars:
    top: { frame_id: velodyne_top }
  cameras:
    front_center: { frame_id: camera_front_center }

markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne_bbox.json5
    bbox_config: $(session-dir)/bbox.json5
    pairs:
      - [top, front_center]

sync: { tolerance_ms: 100, queue_size: 100, drop_policy: reject_new }

assisted:                                          # optional, solver_mode=assisted only
  review_archive_path: $(session-dir)/out/detections.json
```

### `$(session-dir)`

`$(session-dir)` expands to the directory the manifest was loaded from. Use it for every
session-local file. A manifest that names no absolute path can be copied to another machine,
or into your own tree, and still run — which is the whole reason a session is a directory
rather than a file.

`$(find-pkg-share <pkg>)` works as before, for the shared presets. Using `$(session-dir)` in
a file that was not loaded from a session directory is refused rather than silently expanded
to an empty string.

### The `data:` section

| `kind` | Required keys | What LCTK starts |
|---|---|---|
| `pcap_avi` | `dir` — a directory holding `lidar.pcap` and `video.avi` | the `lctk_sample_data` playback |
| `bag` | `path` — a rosbag2 directory with a `metadata.yaml` | `ros2 bag play --clock` |
| `live` | none | nothing; the sensors are already publishing |

Optional under any kind: `lidar: {model, rpm}` and `camera: {info_url}`.

A manifest with no `data:` section at all still parses. That is what `calibrate.launch.py`
accepts today, and it keeps working.

### Topics: derived, verified, or stated

Which side owns a topic name is not a style choice — it follows from what is knowable before
the run:

| kind | topics | why |
|---|---|---|
| `pcap_avi` | **derived** — and stating one is refused | LCTK drives this playback, so a single source feeds both the player and the calibration graph. A mismatch stops being unlikely and becomes unrepresentable. |
| `bag` | **stated**, then verified against `metadata.yaml` | The recording fixes the names. Startup refuses a name the bag lacks, and prints the names it has. |
| `live` | **stated** | There is nothing to check against until the sensor is up. |

Derived names follow one convention, which is exactly what the sample-data playback already
defaulted to:

```
/sensing/lidar/<device_name>/pointcloud_raw
/sensing/lidar/<device_name>/velodyne_packets
/sensing/camera/<device_name>/image_raw
/sensing/camera/<device_name>/camera_info
```

So under `pcap_avi`, naming the lidar device `top` is what produces
`/sensing/lidar/top/pointcloud_raw`.

## Running a session

```bash
source /opt/ros/humble/setup.bash
source /path/to/LCTK/install/setup.bash

# End to end: the data source, then the calibration graph
ros2 launch lctk_launch session.launch.py session:=~/calib/rig-a

# A shipped session, from any working directory
ros2 launch lctk_launch session.launch.py \
    session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne

# Every calibrate argument still applies
ros2 launch lctk_launch session.launch.py \
    session:=~/calib/rig-a solver_mode:=assisted enable_rviz:=false

ros2 launch lctk_launch session.launch.py --show-args
```

### `session:=` is always an explicit path

It is a path — absolute, or relative to your working directory — pointing at either the
session directory or the `session.yaml` inside it. There is no search path, no implicit
`./sessions`, and no `LCTK_SESSION_PATH`.

That is deliberate. An implicit location would have to assume two things at once: where
sessions live, and where you are standing when you type the command. Both assumptions are
wrong as soon as a session lives in your own tree rather than in this repo — which is the
case sessions exist to support. Name lookup is genuinely convenient, so it lives one layer
up in the justfile, where being opinionated about a repo layout costs nothing.

### Running the two halves separately

A live rig, or a bag you are playing yourself, needs only the calibration half:

```bash
# terminal 1 — data only
ros2 launch lctk_launch session_data.launch.py session:=~/calib/rig-a

# terminal 2 — calibration only
ros2 launch lctk_launch calibrate.launch.py config_file:=~/calib/rig-a/session.yaml
```

`calibrate.launch.py` keeps its `config_file:=` interface unchanged. A session manifest is a
valid calibration config; the `data:` section is simply not read by it.

### RViz layout

If a session directory contains `rviz.rviz`, `session.launch.py` passes it as `rviz_config`.
Otherwise `calibrate.launch.py`'s own default (`config/rviz/calibration.rviz`) stands. An
explicit `rviz_config:=` on the command line wins over both — if you typed it, you meant it.

An RViz layout names the debug topics of specific devices, so it is session-local wherever it
is specific: `sessions/seyond-left/rviz.rviz` and
`sessions/solid600-handheld-vlp/rviz.rviz` ship that way. `calibration.rviz` and
`two_lidar_calibration.rviz` stay shared under `config/rviz/`.

The judge's ground truth is the same shape of thing — it describes one rig — but it is not
session-local yet: `calibrate.launch.py` declares no argument to forward one through, so
`calibration_judge.launch.xml`'s own `ground_truth_file` default under
`config/judge/` is what `enable_judge:=true` uses.

## Inspecting, checking and creating sessions

```bash
ros2 run lctk_launch lctk_session list                 # ./sessions plus the installed share
ros2 run lctk_launch lctk_session list ~/calib         # an explicit collection
ros2 run lctk_launch lctk_session check ~/calib/rig-a  # validate; launch nothing
ros2 run lctk_launch lctk_session new ~/calib/rig-b --from ~/calib/rig-a
```

`check` is the one worth running before every recording session. It resolves the manifest,
confirms the data exists, verifies bag topics against `metadata.yaml`, and prints the topics
and frames each device will actually use — without starting a graph:

```
$ ros2 run lctk_launch lctk_session check sessions/sample3-hollow-velodyne
session:  /home/you/LCTK/sessions/sample3-hollow-velodyne
manifest: /home/you/LCTK/sessions/sample3-hollow-velodyne/session.yaml
data:     pcap_avi /home/you/LCTK/sessions/sample3-hollow-velodyne/data
  lidar  top: /sensing/lidar/top/pointcloud_raw  frame=velodyne_top
  camera front_center: /sensing/camera/front_center/image_raw  frame=camera_front_center
OK
```

It answers "why is nothing being detected" before the run rather than after. Every failure
it reports names the **resolved** absolute path, never the `$(session-dir)` string that
produced it — the path that was actually tried is what you need to fix it.

`new` copies an existing session, skips its `out/`, and refuses to overwrite an existing
directory.

## Preparing a new experiment

1. Scaffold from the closest existing session:
   ```bash
   ros2 run lctk_launch lctk_session new ~/calib/rig-b \
       --from $(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne
   ```
2. Edit `session.yaml`: point `data:` at your bag, pcap directory or live topics; name the
   devices and their frames; pick the target definition and detector preset.
3. If the detector preset uses `detection_mode: "bbox"`, tune the session's `bbox.json5`
   with `ros2 run filter_box_tuner filter_box_tuner`.
4. Validate before you launch anything:
   ```bash
   ros2 run lctk_launch lctk_session check ~/calib/rig-b
   ```
5. Run it:
   ```bash
   ros2 launch lctk_launch session.launch.py session:=~/calib/rig-b
   ```

## Where outputs land

Anything a run produces belongs in the session's own `out/`, named through
`$(session-dir)/out/…` — detection archives, Autoware exports, assisted-capture review
archives. `sessions/*/out/` is gitignored, so a shipped session stays clean while its runs
accumulate beside it.

## Startup refusals

Every one of these is a refusal at startup, not a warning. The failures sessions exist to
prevent are all silent at run time, so converting them into loud ones before the graph starts
is the point.

| situation | behaviour |
|---|---|
| `session:=` path does not exist | refused, naming the path that was tried |
| a directory with no `session.yaml` | refused, naming the directory and what was expected |
| `data.dir` / `data.path` missing on disk | refused, naming the resolved absolute path |
| `topic:` stated under `kind: pcap_avi` | refused — the topic is derived, and stating it would restore two sources of truth |
| `topic:` absent under `bag` or `live` | refused, naming the device |
| a bag that lacks a named topic | refused, listing the topics the bag does contain |
| `$(session-dir)` with no session directory | refused, naming the offending value |
| an unknown key under `data:` | refused, listing the known keys |

## The shipped sessions

| session | data | notes |
|---|---|---|
| `sample1` | `pcap_avi`, own `data/` | ships in git; **never run** — target and preset are assumptions |
| `sample2` | `pcap_avi`, own `data/` | ships in git; **never run** — target and preset are assumptions |
| `sample3-hollow-velodyne` | `pcap_avi`, own `data/` | ships in git; verified end to end; what `just demo` runs |
| `sample4` | `pcap_avi`, own `data/` | ships in git; **never run**; its pcap is the second LiDAR of the two-LiDAR captures |
| `sample5` | `pcap_avi`, own `data/` | ships in git; **never run** — target and preset are assumptions |
| `seyond-left` | `live` | Seyond Falcon + left camera; no recording ships |
| `seyond-right` | `live` | Seyond Falcon + right camera; no recording ships |
| `solid600-handheld-vlp` | `live` | solid 600 mm target, hand-held, ZED; 50 ms sync window |
| `twolidar-vlp32-falcon` | `bag`, `TWO_LIDAR_1` | the bag is gitignored — see `ros/lctk_sample_data/bags/README.md` |
| `vehicle-multisensor` | `live` | a schema demonstration; no rig behind it |

Each has its own `README.md` saying what the recording is and whether the data ships. Only
`sample3-hollow-velodyne` has been run end to end; the other four `sampleN` sessions ship a
playable recording whose board, detector preset and rig geometry nobody has verified. Their
manifests are bbox-free on purpose: a crop box is per-recording geometry, and a borrowed one
is what silenced the shipped demo (M-29). Read those READMEs before trusting their values.

## The `just` shorthand

The recipes are aliases over the commands above, plus one convenience the launch interface
deliberately does not have: they resolve a bare session *name* against `./sessions/` and then
the installed share directory.

| recipe | expands to |
|---|---|
| `just sessions` | `ros2 run lctk_launch lctk_session list` |
| `just check <name-or-path>` | `ros2 run lctk_launch lctk_session check …` |
| `just new <path> [<template>]` | `ros2 run lctk_launch lctk_session new … --from …` |
| `just run <name-or-path>` | `ros2 launch lctk_launch session.launch.py session:=…` |
| `just demo` | `just run sample3-hollow-velodyne` |
| `just sample-data [<name>]` | `ros2 launch lctk_launch session_data.launch.py session:=…` — playback only |
| `just lidar-camera [<name>]` | `just run`, defaulting to `seyond-left` |
| `just solid [<name>]` | `just run` with the solid-board bench settings |
| `just assisted [<name>]` | `just run` with `solver_mode:=assisted` |
| `just calibrate <config-path>` | `ros2 launch lctk_launch calibrate.launch.py config_file:=…` |

`just new <path> <template>` takes its template as a second **positional** argument, not as
`FROM=…`; `FROM=x` would be passed through as the literal string.

The run-shaping variables (`mode`, `solver_mode`, `debug_mode`, `log_level`,
`rviz_enabled`, `enable_overlay`, `enable_judge`) are just-variables, so they go before the
recipe name:

```bash
just solver_mode=manual run seyond-left
just mode=realtime demo
```
