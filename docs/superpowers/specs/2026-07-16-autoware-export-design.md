# Design: Autoware Calibration Export

- **Date:** 2026-07-16
- **Issue:** [gap-autoware-export](../../issues/archive/gap-autoware-export.md)
- **Roadmap:** [Phase 6](../../roadmap/phase-6-autoware-export.md)
- **Status:** Proposed

## 1. What Autoware actually consumes (verified 2026-07-16)

Explored `~/repos/autoware` at three versions: `main` (1.5.0-42-gdeed5ae), worktree
`1.5.0-ws`, and branches `newslab/0.45.1-ws`, `newslab/rosdebian/2024.11`.

### 1.1 Runtime mechanism (identical in every version checked)

```
autoware.launch.xml
  └─ tier4_vehicle_launch/launch/vehicle.launch.xml
       └─ robot_state_publisher
            robot_description = xacro vehicle.xacro
              vehicle_model:=<model> sensor_model:=<kit> config_dir:=<dir>
```

`<kit>_description/urdf/sensors.xacro` loads `$(config_dir)/sensors_calibration.yaml`;
its `sensor_kit.xacro` macro loads `$(config_dir)/sensor_kit_calibration.yaml` via
`xacro.load_yaml()` and emits one fixed URDF joint per entry. **The static TF tree is
exactly these two YAML files.** Nothing else stores extrinsics.

`config_dir` defaults to `$(find-pkg-share <sensor_model>_description)/config` and is
overridable from `autoware.launch.xml:74` — this is the only surviving per-vehicle hook
in new Autoware.

### 1.2 File schema (both files, all versions)

```yaml
<parent_frame>:
  <child_frame>:
    x: 0.0        # meters
    y: 0.0
    z: 0.0
    roll: 0.0     # radians, URDF fixed-axis (extrinsic XYZ == intrinsic ZYX)
    pitch: 0.0
    yaw: 0.0
```

| File | Parent | Children (sample kit) |
|------|--------|------------------------|
| `sensors_calibration.yaml` | `base_link` | `sensor_kit_base_link`, `velodyne_rear_base_link` |
| `sensor_kit_calibration.yaml` | `sensor_kit_base_link` | `velodyne_top_base_link`, `velodyne_left_base_link`, `camera0/camera_link` … `camera5/camera_link`, `gnss_link`, `tamagawa/imu_link` |

LiDAR-camera calibration edits **`sensor_kit_calibration.yaml`** (and only the camera
entries; the lidar entry is normally the kit reference and stays put).

### 1.3 Where the file lives — two eras

| Era | Layout | Calibrated values go to |
|-----|--------|------------------------|
| ≤ 2024.11 | separate repos `sensor_kit/sample_sensor_kit_launch`, `param/autoware_individual_params` | `individual_params/config/$VEHICLE_ID/<kit>/sensor_kit_calibration.yaml` |
| ≥ 0.45.1 (incl. 1.5.0, main) | sensor kits + vehicles folded into the `autoware_launch` repo; `autoware_individual_params` **deleted** | `autoware_launch/sensor_kit/<kit>_launch/<kit>_description/config/sensor_kit_calibration.yaml`, or any dir passed as `config_dir:=` |

Same schema either way → one writer, destination is just a path argument.

## 2. What LCTK produces

- `advanced_extrinsic_solver` / `extrinsic_solver_node`: `cv2.solvePnP` extrinsic as
  rvec/tvec — the pose of the **camera optical frame** relative to the LiDAR frame
  (object points are in LiDAR frame, image points in camera pixels, so rvec/tvec map
  LiDAR-frame points into the optical frame: `T(optical ← lidar)`).
- Published as `TransformStamped` with **inverted frame labels** (M-01, still open).
- `dump_detections` JSON (version 2) persists `transform: {rvec, tvec}` — this is the
  stable machine-readable artifact the exporter should read.

## 3. Frame algebra the exporter owns

Three traps, all silent if missed:

1. **Direction (M-01).** Autoware entry means `T(parent → child)` = pose of child in
   parent frame. The solver's rvec/tvec is `T(optical ← lidar)`; the exporter inverts it
   once, explicitly, in one audited place.
2. **Optical vs camera_link.** Autoware children are `camera*/camera_link` — ROS body
   convention (x forward, z up). PnP lives in the optical frame (z forward, x right).
   Fixed rotation: `T(camera_link → optical)` has RPY `(-π/2, 0, -π/2)`.
3. **Chain to the kit frame.** The solve relates camera to *lidar*, not to
   `sensor_kit_base_link`. Compose with the existing lidar entry:

```
T(kit → camera_link) = T(kit → lidar)                # read from existing YAML
                     · T(lidar → optical)            # = inv(solver rvec/tvec)
                     · T(optical → camera_link)      # = inv(fixed RPY above)
```

Then decompose to xyz + fixed-axis RPY (radians). Round-trip check: recompose the RPY
and assert max abs error < 1e-9 against the rotation matrix before writing.

LiDAR-to-LiDAR results export the same way minus trap 2.

## 4. Tool design

### 4.1 Shape

`lctk_autoware_export` — Python CLI (ament_python package under `ros/`, also runnable as
a plain script). No ROS graph dependency for the core path: input is the dump JSON, output
is a YAML file. Optional `--from-topic` convenience wraps a one-shot subscription.

```
ros2 run lctk_autoware_export export \
  --detections ~/detections.json \          # solver dump (version 2), source of rvec/tvec
  --target /path/to/sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \      # child key to write
  --lidar-frame velodyne_top_base_link \    # existing entry used for T(kit→lidar)
  [--kit-frame sensor_kit_base_link]        # parent key (default shown)
  [--optical-to-link default|identity|"r,p,y"]
  [--dry-run]                               # print the entry, write nothing
```

### 4.2 Behavior

- **Patch, not regenerate.** Load target YAML, replace only
  `[kit_frame][camera_frame]`, keep every other entry byte-stable. Use `ruamel.yaml`
  round-trip mode to preserve comments and key order (plain PyYAML rewrite would churn
  the whole file in review).
- **Refuse to guess.** Missing lidar entry in the target YAML → hard error naming the
  available children. Missing `transform` in the JSON → hard error pointing at
  `dump_detections`.
- **Self-check before write.** Recompose `T(kit→camera_link)` from the values as they
  will be serialized (post-rounding), compare to the composed matrix; print translation
  and rotation residual. Also print the entry in `--dry-run` format so the operator can
  eyeball it.
- **Backup.** Write `<target>.bak` beside the target before the first modification.
- **Era-agnostic.** `--target` is just a path; works for `individual_params/config/$VEHICLE_ID/...`
  (old era) and `<kit>_description/config/...` or custom `config_dir` (new era) alike.

### 4.3 Out of scope (deliberate)

- Editing `sensors_calibration.yaml` (base_link → kit): LCTK doesn't measure it.
- Generating a new sensor kit package from scratch.
- xacro parsing — the exporter treats YAML as the interface, per §1.1.

## 5. Dependencies and ordering

- **M-01 must be settled first** (or simultaneously): the exporter encodes the transform
  direction; if the publisher's labels get fixed later, the exporter's inversion must be
  updated in lockstep. Single source of truth: exporter reads the dump JSON rvec/tvec
  (raw solver output, direction unambiguous), *not* the re-labeled TF topic.
- `ruamel.yaml` — new Python dep; apt `python3-ruamel.yaml` exists on Ubuntu 22.04
  (no pip needed; see CLAUDE.md pip-shadowing hazard).

## 6. Test plan (headless)

1. Unit: composition algebra vs hand-computed fixture (identity lidar, 90° yaw lidar,
   synthetic camera pose); RPY round-trip property test over random rotations.
2. Golden-file: patch a copy of the real `sample_sensor_kit` YAML (vendored as fixture);
   assert only the target entry changed, comments preserved.
3. End-to-end: dump from the sample-data pipeline → export → feed the YAML to
   `xacro sensor_kit.xacro` (vendored from sample kit) → compare emitted joint origin
   against the solver transform.

## 7. Documentation

New book page "Exporting to Autoware": the two eras, the command, the frame conventions
diagram (kit / lidar / optical / camera_link), and a worked example from `just demo` to a
patched sample-kit YAML.
