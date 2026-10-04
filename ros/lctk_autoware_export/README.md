# lctk_autoware_export

Patches one entry of an Autoware `sensor_kit_calibration.yaml` with an LCTK-solved
LiDAR-camera extrinsic. See
[docs/superpowers/specs/2026-07-16-autoware-export-design.md](../../docs/superpowers/specs/2026-07-16-autoware-export-design.md)
for the format survey and frame algebra, and
[docs/roadmap/phase-6-autoware-export.md](../../docs/roadmap/phase-6-autoware-export.md)
for the plan.

## Usage

```bash
ros2 run lctk_autoware_export export \
  --detections ~/detections.json \
  --target /path/to/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link \
  --dry-run          # print the entry first; drop the flag to write
```

- `--detections`: a version-6 Detection Archive created by the current writer (assisted
  **Export archive** action or the manual dump service). The exporter checks the Target
  Identity structure, camera projection metadata, and consistent sensor-header frames
  before reading the saved raw solver `rvec`/`tvec`. It does not resolve the identity
  against a Target Definition. Older versions are rejected. The re-labeled TF topic is
  deliberately not accepted as input (M-01).
- Camera detections must use the frame saved in `camera_projection.frame_id`; all archived
  LiDAR detections must consistently use one distinct, nonempty frame. The
  `--camera-frame` value remains the Autoware camera-link YAML key and may differ from
  the image optical frame.
- `--lidar-frame`: existing entry in the target YAML, used as the
  `sensor_kit_base_link -> lidar` anchor of the chain.
- Writes are comment-preserving (`ruamel.yaml` round-trip) and create
  `<target>.yaml.bak` on first modification.
- Works for both Autoware layouts: point `--target` at
  `individual_params/config/$VEHICLE_ID/<kit>/sensor_kit_calibration.yaml` (≤ 2024.11)
  or `autoware_launch/sensor_kit/<kit>_launch/<kit>_description/config/sensor_kit_calibration.yaml`
  (≥ 0.45.1), or any dir you pass to Autoware as `config_dir:=`.

## Tests

```bash
cd ros/lctk_autoware_export && python3 -m pytest test/ -q
```
