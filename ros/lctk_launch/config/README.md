# LCTK Configuration Files

This is the **shared library**: files describing things that exist independently of any one
recording — a physical target, a sensor model's detector tuning, the ArUco detector's own
knobs.

Anything specific to one recording lives in that recording's **session** directory instead:
its data source, topics and frame ids, crop box, camera intrinsics, sync window and RViz
layout. See `sessions/README.md` and `book/src/user-guide/sessions.md`. That split is not
tidiness — a crop box says where a board sat during one recording, and sharing one between two
rigs is what silently killed the shipped demo (M-29).

## Directory Structure

```
config/
├── targets/          # Target Definitions: physical plate geometry, cutouts,
│                     #  fiducial layout (the geometric truth)
├── board/            # Detector Tuning presets (sensor-specific, geometry-free)
│   ├── hollow_1000/  #   per-sensor presets for the hollow_1000 target
│   └── solid_600/    #   per-sensor presets for the solid_600 target
├── aruco/            # ArUco detector tuning (corner refinement, adaptive threshold)
├── judge/            # calibration_judge (quality metric) configs
├── rviz/             # RViz layouts general enough to be shared
├── extrinsic.json5   # A 4x4 lidar-to-camera transform matrix, in the format
│                     #  pointcloud_image_overlay's `extrinsic_json5` param reads
└── pointcloud_overlay_filter.json5
```

Crop boxes (`bbox*.json5`) and camera intrinsics used to live here. They are session-local
now: see `sessions/sample3-hollow-velodyne/bbox.json5` and `camera_info.yaml`.

## Configuration Files

### Target Definitions (`targets/`)
The physical truth for a calibration target: plate geometry, cutout layout, fiducial (ArUco)
marker IDs and placement. Selected per-marker via a calibration config's `target_config` key.
- `hollow_1000_aruco_4_v1.json5` - shipped 1000 mm perforated target
- `solid_600_aruco_1_v1.json5` - shipped 600 mm solid target

### Detector Tuning (`board/`)
Sensor-specific, geometry-free detection parameters (RANSAC/ICP knobs, sensor up-axis
convention, crop-box selection). Selected per-marker (or per-lidar, to override the marker-level
one) via `detector_config`. No board geometry belongs here — that is the Target Definition's job.
- `hollow_1000/velodyne.json5`, `hollow_1000/seyond.json5` - `bbox_free` presets
- `hollow_1000/velodyne_bbox.json5` - the one shipped `bbox`-mode preset. It needs a
  `bbox_config`, which comes from the session using it (`$(session-dir)/bbox.json5`), because
  a crop box describes one recording and cannot be shared
- `solid_600/velodyne.json5`, `solid_600/seyond.json5` - `bbox_free` presets (EXPERIMENTAL)

### ArUco Detector Tuning (`aruco/`)
- `aruco_detector.json5` - corner refinement and adaptive-threshold parameters for the ArUco
  detector itself (not the printed pattern, which lives in the Target Definition)

### RViz Layouts (`rviz/`)
- `calibration.rviz` - the default `calibrate.launch.py` opens
- `two_lidar_calibration.rviz` - the two-lidar layout

A layout that names one rig's device-specific debug topics belongs to that rig's session
instead, as `rviz.rviz` — `sessions/seyond-left/` and `sessions/solid600-handheld-vlp/` ship
theirs that way, and `session.launch.py` forwards them automatically.

### Calibration Configs

Full calibration configs (devices, markers, sync, pairs) are **session manifests** now, in
`sessions/<name>/session.yaml`. See the repo root `CLAUDE.md`'s "Calibration Sessions"
section for the schema and a description of each shipped session, or run
`ros2 run lctk_launch lctk_session list`.

A config may also carry an optional `assisted:` section, read only by
`solver_mode=assisted`. It is optional -- unlike `sync:` -- because `continuous` and
`manual` read none of it, so refusing a config that omits it would break both modes over a
setting neither uses. An unknown key inside the section *is* refused, since silently
ignoring a misspelling would leave you tuning a value that never reaches the node. Keys and
defaults are documented in `book/src/user-guide/assisted-capture.md`.

## Usage in Launch Files

Calibration is config-driven: a single YAML config file — a session manifest — is passed to
`calibrate.launch.py`, which reads it and generates the required nodes with the right
`target_config` / `detector_config` / `bbox_config` / `aruco_detector_config` parameters. There
is no per-file XML launch argument any more (the old `aruco_config_file:=` and
`board_config_file:=` arguments do not exist on any maintained launch path).

```bash
# The whole session: data source plus calibration graph
ros2 launch lctk_launch session.launch.py \
    session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne

# The calibration half alone, against any manifest
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne/session.yaml
```

Or accessed via command line:
```bash
ros2 pkg prefix lctk_launch
# Returns: /path/to/install/lctk_launch
# Config files at: /path/to/install/lctk_launch/share/lctk_launch/config/
```

## File Formats

- **JSON5** (`.json5`): Used for Target Definitions, Detector Tuning presets, and ArUco detector
  tuning (complex structured configurations with comments)
- **YAML** (`.yaml`/`.yml`): Used for session manifests (`sessions/<name>/session.yaml`),
  judge configs and camera parameters

## Adding New Configurations

1. Place configuration files in the appropriate subdirectory
2. Update `CMakeLists.txt` if adding new file extensions
3. Reference the file from a calibration config's `target_config`/`detector_config`/
   `bbox_config`/`aruco_detector_config` key, using `$(find-pkg-share lctk_launch)/config/<subdir>/<file>`

## Notes

- A path to a *shared* file uses `$(find-pkg-share lctk_launch)/config/...`; a path to a
  *session-local* file uses `$(session-dir)/...`, which keeps the session directory
  relocatable. Absolute paths work but pin the manifest to one machine
- Configuration files are installed during the build process (`just build`)
- Changes to config files require rebuilding to update the installed versions
