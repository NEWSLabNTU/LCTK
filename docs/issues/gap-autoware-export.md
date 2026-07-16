# GAP · No export path from a calibration result to Autoware

- **Severity:** High (headline gap)
- **Area:** output / integration
- **Status:** Open
- **Verified:** Static review (whole-repo grep for "autoware", "sensor_kit", "static_transform" — zero hits)
- **Related:** [M-01](./M-01-transform-direction-inverted.md), [M-02](./archive/M-02-radians-degrees-mix.md), [L-09](./L-09-setup-fragility-export-labeling.md)

## Problem

LCTK produces a calibration result only as:
1. a `geometry_msgs/TransformStamped` on a topic (and optionally `/tf_static` via `tf_tree_broadcaster`), and
2. a formatted **log line** with translation (m) + RPY (deg) + quaternion.

Nothing is written to disk automatically. The `dump_detections` / interactive-controller "Save" produces a solver **re-load** JSON (raw rvec/tvec correspondences), not a usable extrinsic file. The word "autoware" appears nowhere in the repo, and there is no export tool, template, or documentation for the last mile.

## What a user must do manually today

1. Run the pipeline and watch the terminal / `ros2 topic echo` the transform.
2. Hand-copy translation and quaternion (or the degrees-RPY line) out of the console.
3. **Invert the transform direction** — the published frame labels are backwards vs ROS TF (see M-01).
4. **Convert quaternion → roll/pitch/yaw (radians)** because Autoware `sensor_kit_calibration.yaml` uses `x,y,z,roll,pitch,yaw`.
5. Hand-write the values into the Autoware sensor-kit YAML. No template, no generator, no example.

## Suggested fix

Ship an exporter (service or CLI) that writes an Autoware-shaped `sensor_kit_calibration.yaml` snippet: correct transform direction (parent = base/sensor-kit frame), meters, radians RPY, with the parent/child frames taken from the config. Document it as an "Exporting Results" page in the book.

## Update (2026-07-16) — Autoware target format verified

Explored `~/repos/autoware` (worktrees: `main`/1.5.0, `newslab/0.45.1-ws`, `newslab/rosdebian/2024.11`).
Findings recorded in full in the design doc; summary:

**Runtime mechanism (unchanged across versions).** `tier4_vehicle_launch/launch/vehicle.launch.xml`
runs `robot_state_publisher` on `xacro vehicle.xacro sensor_model:=<X> config_dir:=<Y>`;
`sensors.xacro` / `sensor_kit.xacro` call `xacro.load_yaml()` on two YAML files in `config_dir`.
The whole static TF tree comes from those YAMLs.

**Two YAML files, one schema** — `parent_frame: { child_frame: {x, y, z, roll, pitch, yaw} }`,
meters + radians, URDF fixed-axis (extrinsic XYZ) RPY:

| File | Parent | Children |
|------|--------|----------|
| `sensors_calibration.yaml` | `base_link` | `sensor_kit_base_link` + vehicle-mounted sensors |
| `sensor_kit_calibration.yaml` | `sensor_kit_base_link` | every kit sensor (`velodyne_top_base_link`, `camera0/camera_link`, …) |

LCTK's result feeds `sensor_kit_calibration.yaml`.

**Two destination eras, same schema:**
- **≤ 2024.11**: separate repos `sensor_kit/sample_sensor_kit_launch` + `param/autoware_individual_params`;
  calibrated values go to `individual_params/config/$VEHICLE_ID/<kit>/sensor_kit_calibration.yaml`.
- **≥ 0.45.1 / 1.5.0 / main**: both repos folded into `autoware_launch`
  (`sensor_kit/<kit>_launch/<kit>_description/config/*.yaml`); `autoware_individual_params` deleted.
  Per-vehicle override survives only via the `config_dir:=` launch arg (`autoware.launch.xml:74`).

One writer + a destination flag covers both eras.

**Frame traps the exporter must own:**
1. Transform direction (M-01) — Autoware entry is parent(kit) → child(sensor); LCTK publishes the
   opposite labeling.
2. Autoware camera child is `camera*/camera_link` (ROS body frame, x-forward); PnP solves the
   **optical** frame (z-forward). Fixed rotation between them must be composed in.
3. Chain composition: `T(kit→camera_link) = T(kit→lidar) · T(lidar→camera_optical) · T(optical→camera_link)`,
   with `T(kit→lidar)` read from the existing YAML (often identity+yaw for the reference lidar).

**Design:** [2026-07-16-autoware-export-design.md](../superpowers/specs/2026-07-16-autoware-export-design.md)
**Roadmap:** [phase-6-autoware-export.md](../roadmap/phase-6-autoware-export.md)
