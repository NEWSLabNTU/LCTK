# LCTK Configuration Files

This directory contains centralized configuration files for all LCTK calibration packages.

## Directory Structure

```
config/
├── targets/         # Target Definitions: physical plate geometry, cutouts,
│                     #  fiducial layout (the geometric truth)
├── board/            # Detector Tuning presets (sensor-specific, geometry-free)
│   ├── hollow_1000/  #   per-sensor presets for the hollow_1000 target
│   ├── solid_600/    #   per-sensor presets for the solid_600 target
│   └── bbox*.json5   #   crop-box configs, used when a preset selects detection_mode: "bbox"
├── aruco/            # ArUco detector tuning (corner refinement, adaptive threshold)
├── camera/           # Camera intrinsics and settings
├── examples/         # Example calibration configs (see below)
├── judge/            # calibration_judge (quality metric) configs
├── rviz/             # RViz configs
├── extrinsic.json5   # A 4x4 lidar-to-camera transform matrix, in the format
│                     #  pointcloud_image_overlay's `extrinsic_json5` param reads
└── pointcloud_overlay_filter.json5
```

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
- `hollow_1000/velodyne_bbox.json5` - the one shipped `bbox`-mode preset (needs a `bbox_config`)
- `solid_600/velodyne.json5`, `solid_600/seyond.json5` - `bbox_free` presets (EXPERIMENTAL)
- `bbox.json5` and friends - crop-box configs referenced by a `bbox`-mode preset's `bbox_config`

### ArUco Detector Tuning (`aruco/`)
- `aruco_detector.json5` - corner refinement and adaptive-threshold parameters for the ArUco
  detector itself (not the printed pattern, which lives in the Target Definition)

### Camera Configurations (`camera/`)
- `front_center_camera_info.yaml` - Camera intrinsic parameters (focal length, distortion, etc.)

### Example Calibration Configs (`examples/`)
Full calibration configs (devices, markers, sync, pairs) for `ros2 launch lctk_launch
calibrate.launch.py config_file:=...` / `just calibrate`. See the repo root `CLAUDE.md`'s
"Config-Driven Calibration" section for the schema and a description of each shipped example.

A config may also carry an optional `assisted:` section, read only by
`solver_mode=assisted`. It is optional -- unlike `sync:` -- because `continuous` and
`manual` read none of it, so refusing a config that omits it would break both modes over a
setting neither uses. An unknown key inside the section *is* refused, since silently
ignoring a misspelling would leave you tuning a value that never reaches the node. Keys and
defaults are documented in `book/src/user-guide/assisted-capture.md`.

## Usage in Launch Files

Calibration is config-driven: a single YAML config file (see `examples/`) is passed to
`calibrate.launch.py`, which reads it and generates the required nodes with the right
`target_config` / `detector_config` / `bbox_config` / `aruco_detector_config` parameters. There
is no per-file XML launch argument any more (the old `aruco_config_file:=` and
`board_config_file:=` arguments do not exist on any maintained launch path).

```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=$(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/sample_data.yaml
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
- **YAML** (`.yaml`/`.yml`): Used for calibration configs (`examples/`) and camera parameters

## Adding New Configurations

1. Place configuration files in the appropriate subdirectory
2. Update `CMakeLists.txt` if adding new file extensions
3. Reference the file from a calibration config's `target_config`/`detector_config`/
   `bbox_config`/`aruco_detector_config` key, using `$(find-pkg-share lctk_launch)/config/<subdir>/<file>`

## Notes

- All paths in calibration configs should use the installed share directory location
  (`$(find-pkg-share lctk_launch)/config/...`) or an absolute path
- Configuration files are installed during the build process (`just build`)
- Changes to config files require rebuilding to update the installed versions
