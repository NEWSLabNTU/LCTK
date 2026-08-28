# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Project Overview

LCTK (LiDAR and Camera Toolkit) is a set of libraries and tools for calibrating LiDAR and camera systems. Implemented in Rust with ROS 2 integration.

## Quick Start

```bash
# Set up development environment
./setup.sh

# Build the project
just build

# Launch calibration
just lidar-camera

# See all commands
just
```

## Project Structure

- **`rust/`**: Pure Rust libraries (aruco-config, aruco-detector, hollow-board-detector, board-cluster-detector, plane-estimator, etc.)
- **`ros/`**: ROS 2 packages
  - `lctk_launch/` - Unified launch system with config-driven calibration pipeline
  - `lctk_interfaces/` - Shared msg/srv definitions (solver services, quality report)
  - `aruco_locator_node/` - ArUco marker detection from camera images
  - `aruco_generator_node/` - Prints the ArUco board pattern from `aruco_pattern.json5`
  - `lidar_board_detector/` - Calibration board detection from point clouds
  - `extrinsic_solver_node/` - Superseded LiDAR-camera solver (unreachable from config-driven launch; pending deletion)
  - `lidar_to_camera_solver/` - LiDAR-camera solver with continuous and manual modes
  - `interactive_solver_controller/` - Rich TUI driving `lidar_to_camera_solver`
  - `lidar_to_lidar_solver/` - LiDAR-to-LiDAR calibration solver
  - `lctk_quality/` + `calibration_judge/` - Extrinsic quality metric (H-09)
  - `pointcloud_image_overlay/` - Projects the cloud onto the image for visual verification
  - `filter_box_tuner/` - Interactive crop-box tuning for the board detector
  - `lctk_autoware_export/` - Exports a solved extrinsic into Autoware `sensor_kit_calibration.yaml`
  - `lctk_sample_data/` - Sample data playback (pcap + avi), plus gitignored recorded
    `bags/TWO_LIDAR_*` (two-LiDAR: VLP-32C + solid-state Falcon; see `bags/README.md`)
  - `conflux/` - Git submodule (jerry73204/conflux): message synchronizer used by all solvers
    (`conflux_cpp` builds `libconflux_ffi.so`, `conflux_py` wraps it via ctypes)
- **`setup/`**: Development environment setup scripts
- **`book/`**: Documentation (mdbook with mermaid diagrams)
- **`docs/`**: Engineering docs — `issues/` (tracker: one file per finding; closed ones move to
  `issues/archive/`), `roadmap/` (phase docs), `superpowers/specs/` (design docs)

## Build System

- Uses `colcon-cargo-ros2` for Rust ROS 2 integration
- ROS interface bindings are auto-generated at `build/<pkg>/rosidl_cargo/`
- Uses `rclrs` v0.7 from crates.io (requires `ros-humble-test-msgs`)
- Launch commands use `play_launch` for foreground execution
- **Always use `just build`** - never run raw `colcon build` commands. The justfile uses specific flags:
  ```bash
  colcon build \
      --base-paths ros \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      --cargo-args --profile=test-release
  ```
- To build a single package, use: `just build` with `--packages-select <pkg>` appended manually if needed, but prefer building all packages
- `just build` depends on `just build-conflux` (builds `conflux_cpp` + `conflux_py` first; the rest
  of the conflux submodule is excluded because its git rclrs conflicts with our crates.io rclrs)
- Binding generation runs once per `build/` tree, guarded by `build/.colcon/bindgen.lock` — see
  Known Issue 7 before deleting anything under `build/`
- **Dependency updates must run inside the sourced build env** (`source /opt/ros/humble/setup.bash
  && source install/setup.bash`): a bare `cargo update` re-resolves the wildcard ROS message
  crates against crates.io and aborts on the yanked `sensor_msgs`. Procedure + vuln log:
  `docs/roadmap/phase-4-dependency-updates-and-vulns.md`. Tracked advisory exceptions live in
  `.cargo/audit.toml`
- Setup installers are version-pinned (L-09) with env overrides: `ROS_APT_VERSION`,
  `CARGO_AMENT_BUILD_VERSION`, `CARGO_NEXTEST_VERSION`, `CUDA_KEYRING_VERSION`

## Key Commands

```bash
just build      # Build all packages
just clean      # Clean build artifacts
just test       # Run tests (cargo nextest + pytest suites)
just lint       # Full lint (rustfmt + clippy + ruff; clippy takes minutes)
just lint-py    # Fast ruff-only lint
just audit      # cargo-audit for RUSTSEC advisories (runs in the sourced build env)

just lidar-camera   # Launch calibration (legacy XML launch)
just demo           # Launch demo (sample data + calibration pipeline)
just sample-data    # Launch sample data playback
just rviz           # Launch RViz

# Config-driven calibration (preferred)
just calibrate /path/to/config.yaml

# Justfile variables (override with just var=value command)
just demo mode=realtime              # Use realtime mode (BEST_EFFORT QoS, no buffering)
just demo mode=offline               # Use offline mode (RELIABLE QoS, default)
just demo solver_mode=manual         # Use service-driven multi-pose buffering
just demo debug_mode=false           # Disable debug output

# Documentation (run from book/ directory)
just build          # Build docs
just serve          # Serve with live reload
just serve-public   # Serve on 0.0.0.0
```

## Known Issues

1. **Old .cargo/config.toml conflicts**: If build fails with `Unable to update .../install/.../rust`:
   ```bash
   mv .cargo/config.toml .cargo/config.toml.bak
   ```
   Since M-18 the root `.cargo/config.toml` is **generated** by
   `setup/scripts/sync-root-cargo-config.sh`, which `just build` and `just test` run every time.
   It is synthesised from a per-package `ros/*/.cargo/config.toml` with the patch paths rewritten
   root-relative, and it is what lets cargo (`nextest`, `clippy`, `audit`) work from the repo root
   instead of dying on the yanked `sensor_msgs`. Never hand-edit it; deleting it is always safe.

2. **Colcon-cargo conflicts**: Remove old packages before installing colcon-cargo-ros2:
   ```bash
   pip3 uninstall colcon-cargo colcon-ros-cargo
   ```

3. **pip packages shadowing apt ones** (this has bitten three times — `just build` now guards all three):

   A pip `--user` install lands in `~/.local/lib/python3.10/site-packages`, which **precedes**
   `/usr/lib/python3/dist-packages` on `sys.path` and silently shadows the apt package that ROS 2
   Humble and apt's OpenCV were built against. All known cases fail far from the cause:

   | symptom | when | fix |
   |---------|------|-----|
   | `error: option --editable not recognized` (kills `conflux_py` and every ament_python package) | **build** time | `pip3 uninstall -y setuptools` |
   | `ImportError: numpy.core.multiarray failed to import` (kills every solver node at startup, after a clean build) | **run** time | `pip3 uninstall -y numpy` |
   | `TypeError: 'numpy._DTypeMeta' object is not subscriptable` inside scipy (kills any test/node importing `scipy.optimize`) | **test/run** time | `pip3 uninstall -y scipy` |

   setuptools >= 80 removed the `setup.py develop --editable` step colcon uses for
   `--symlink-install`; numpy >= 2 breaks the ABI apt's `cv2` was compiled against;
   scipy >= 1.15 requires numpy >= 1.23 while apt ships 1.21.
   **Never `pip3 install --user` setuptools, numpy, or scipy on this machine** — and note that
   installing *anything else* with pip can drag them in as dependencies.

4. **ROS2 daemon issues**: Kill unresponsive daemon:
   ```bash
   pkill -9 -f ros2-daemon
   ```

5. **Text file busy during build**: If build fails with "Text file busy (os error 26)", kill running nodes and clean:
   ```bash
   pkill -9 -f "<node_name>"
   rm -rf build/<package> install/<package>
   rm -f build/.colcon/bindgen.lock   # see Known Issue 7
   just build
   ```

6. **Killing play_launch leaves orphan processes**: When killing play_launch with `pkill`, child processes become orphans. Kill the entire process group instead:
   ```bash
   # Find the play_launch process group ID (PGID)
   ps -o pid,pgid,cmd | grep play_launch

   # Kill the entire process group (note the negative sign before PGID)
   kill -9 -<PGID>
   ```
   Alternatively, run play_launch in its own process group and use Ctrl+C for clean shutdown.

7. **Stale rosidl bindings after deleting a build dir** (bit us 2026-07-16): the Rust binding
   generation for interface packages runs **once** per `build/` tree and then marks itself done
   with `build/.colcon/bindgen.lock`. If you `rm -rf build/lctk_interfaces` (e.g. after changing
   or removing a `.msg`), the next `just build` will **skip regeneration** because the lock still
   exists, and every Rust package fails with:
   ```
   failed to read `.../build/lctk_interfaces/rosidl_cargo/lctk_interfaces/Cargo.toml`
   ```
   Relatedly, changing/removing a message without cleaning leaves stale generated C sources that
   fail at link time (`undefined reference to lctk_interfaces__msg__...__create`). Fix for both:
   ```bash
   rm -rf build/lctk_interfaces install/lctk_interfaces
   rm -f build/.colcon/bindgen.lock
   just build
   ```
   The partial-clean case is now auto-guarded: `just build` deletes the lock itself when any
   binding path from `.cargo/config.toml` is missing (L-16). The manual clean is still needed
   after changing/removing a `.msg`/`.srv`.

## Coding Guidelines

- **Temporary files**: Create temporary files and scripts in `$project/tmp/` directory, not `/tmp/`
- Use named parameters in format strings: `println!("{e}")` not `println!("{}", e)`
- Clone Arc variables in local scope before moving to closures
- Use `just build` to rebuild ROS2 packages (not `cargo build` directly)
- Always run build commands from project root directory
- Don't use Pokemon exception handling (`try: except Exception: pass`)
- Prefer functional struct initialization in Rust
- When running sudo commands, show command to user instead of executing

## Docs & Issue Tracking Practices

- Findings/bugs are filed as one markdown file per issue under `docs/issues/` and indexed in its
  `README.md` status table (🔴 open · 🟡 in progress · 🟢 fixed · ⚪ won't fix)
- When an issue closes, **move the file to `docs/issues/archive/`** and repair every relative
  link that crosses the move (both directions). Verify with a link check: every `](...*.md)`
  target under `docs/` must exist
- Larger remediations get a phase doc in `docs/roadmap/` and, when designed up front, a design
  doc in `docs/superpowers/specs/`
- Fixes land on a `fix/...` or `feat/...` branch, then `git checkout main && git merge --ff-only`
- Multiple agents may work this repo concurrently: before starting an issue, check the tracker
  for 🟡 (in-progress) markers, and always `git fetch` + rebase before pushing

## ROS 2 Conventions

- Camera info topics auto-derived from image topics (image_pipeline convention)
- All nodes require explicit config file parameters (no hardcoded defaults)
- Workspace dependencies defined in root Cargo.toml

## rclrs Patterns

### Dynamic Parameters
Use `MandatoryParameter<T>` wrapped in `Arc` for runtime-configurable parameters:
```rust
let param: Arc<MandatoryParameter<f64>> = Arc::new(
    node.declare_parameter::<f64>("param_name")
        .default(1.0)
        .mandatory()?
);
// Read current value (reflects runtime changes via `ros2 param set`)
let value = param.get();
```

### High-Frequency Sensor Data with Slow Processing
The rclrs executor queues ALL messages internally, regardless of QoS KEEP_LAST settings. For slow processing (e.g., ICP taking 600ms+ with 10Hz input), use `ArcSwap` to decouple reception from processing:
```rust
use arc_swap::ArcSwap;

// Store latest message
let latest_msg: Arc<ArcSwap<Option<Arc<SensorMsg>>>> = Arc::new(ArcSwap::new(Arc::new(None)));

// Subscription callback - lightweight, just stores latest
let msg_for_callback = Arc::clone(&latest_msg);
node.create_subscription(opts, move |msg| {
    msg_for_callback.store(Arc::new(Some(Arc::new(msg))));
})?;

// Processing thread - takes latest, skips stale
let msg_for_processing = Arc::clone(&latest_msg);
std::thread::spawn(move || loop {
    let msg_opt = msg_for_processing.swap(Arc::new(None));
    if let Some(msg) = msg_opt.as_ref() {
        process(msg);  // Slow processing here
    } else {
        std::thread::sleep(Duration::from_millis(5));
    }
});
```
This ensures always processing the latest data, not stale queued messages.

## Calibration Workflow

### Processing Modes

The calibration pipeline supports two processing modes controlled by the `mode` parameter. `mode`
controls **only** transport QoS — live-versus-recorded data is a genuine transport property. The
synchronizer window/buffer/drop-policy are a physical judgement about the scene (how far the
calibration target can move between a camera frame and a LiDAR sweep) and are **not** derivable
from `mode`; they come from the calibration config file's required `sync:` section instead — see
`sync:` under Configuration Format below.

| Mode | QoS | Use Case |
|------|-----|----------|
| `offline` (default) | RELIABLE | Recorded data (rosbags or the pcap/avi sample playback). |
| `realtime` | BEST_EFFORT | Live sensor data. Low latency. |

Note: `lctk_sample_data` ships pcap + avi in git (datasets 1–5; dataset 3 is the lidar-camera
default, dataset 4 the second lidar). Recorded two-LiDAR bags live in
`ros/lctk_sample_data/bags/TWO_LIDAR_*` but are **gitignored** — see that directory's README to
obtain them. To record more: `ros2 bag record -a` alongside `just sample-data`.

**Setting derived from mode:**
- **QoS Reliability**: RELIABLE (offline) vs BEST_EFFORT (realtime)

**Usage:**
```bash
just demo mode=offline    # For recorded/sample-data playback (default)
just demo mode=realtime   # For live sensors
```

### Performance Profiling Results

Profiling conducted on sample data (2026-01-18):

**Throughput Comparison:**
| Metric | Offline | Realtime | Notes |
|--------|---------|----------|-------|
| Board detections | 190 | 127 | RELIABLE QoS captures all messages |
| Transform messages | 378 | 274 | Higher throughput in offline mode |
| QoS warnings | 0 | 0 | Both modes have compatible QoS |

**Latency Comparison (median):**
| Component | Offline | Realtime | Notes |
|-----------|---------|----------|-------|
| Board detection | 99.9 ms | 103.5 ms | ICP is the bottleneck |
| Solver | 38.2 ms | 50.1 ms | PnP computation |

**Key Insights:**
- Board detection (ICP) is the processing bottleneck at ~100ms per frame
- Offline mode achieves ~50% higher throughput due to RELIABLE QoS
- ICP quality is consistent across modes (loss: 0.026-0.029). **This is the noise floor, not a bad
  fit** — a VLP-32C is spec'd at ~±3 cm range accuracy, so a ~2.6 cm mean point-to-model residual is
  as good as the sensor gets. `icp_good_fit_threshold` must sit *above* this; it was once set to
  0.012 and the detector then silently accepted nothing (see `docs/issues/archive/C-04-board-detector-gate-unreachable.md`).
- Realtime mode has higher latency variance due to message skipping

### Config-Driven Calibration (Preferred)

The unified calibration interface uses YAML configuration files to define sensors and calibration pairs. This automatically generates the required nodes.

**Usage:**
```bash
# With sample data
just sample-data                    # Terminal 1: Start data playback
just calibrate $(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/sample_data.yaml  # Terminal 2

# Or with ros2 launch directly
ros2 launch lctk_launch calibrate.launch.py config_file:=/path/to/config.yaml
```

**Configuration Format:**
```yaml
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /sensing/lidar/top/pointcloud_raw
      frame_id: velodyne_top
  cameras:
    front_center:
      image_topic: /sensing/camera/front_center/image_raw
      frame_id: camera_front_center

markers:
  calibration_board:
    # target_config: the physical target -- plate, cutouts, fiducial
    # layout, identity. detector_config: sensor-specific tuning only; it
    # must contain no geometry.
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
    # bbox_config is omitted here because this preset is bbox_free. It is
    # required only when the chosen detector_config selects
    # detection_mode: "bbox" -- of the shipped presets, only
    # config/board/hollow_1000/velodyne_bbox.json5 does.
    #
    # Optional. ArUco *detector* tuning (corner refinement, adaptive threshold).
    # Defaults to config/aruco/aruco_detector.json5 when omitted.
    aruco_detector_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_detector.json5
    # Calibration pairs are defined inside each marker as a list of
    # [deviceA, deviceB] pairs that observe this marker.
    pairs:
      - [top_lidar, front_center]

# Required. Conflux synchronizer window/buffer/drop-policy -- a physical
# judgement about the scene (how far the calibration target can move
# between a camera frame and a LiDAR sweep), not derived from `mode`.
sync:
  tolerance_ms: 100    # Must be finite and > 0; 0/inf/nan are refused
  queue_size: 100       # Positive integer buffer size per stream
  drop_policy: reject_new   # "reject_new" or "drop_oldest"
```

A per-lidar `detector_config` under `devices.lidars.<name>` overrides the marker-level one. That is
how two differently-sampled LiDARs (a spinning VLP-32C and a solid-state Falcon, say) share one
target while each keeps its own sensor-specific tuning; `config/examples/two_lidar.yaml` does
exactly this.

The legacy `type`/`board_config`/`aruco_config` marker keys still parse, but are scheduled for
removal and no maintained example uses them any more.

**ArUco config files are split by purpose:**

| File | Describes | Also read by |
|------|-----------|--------------|
| `aruco_pattern.json5` | the *printed board* (marker IDs, dictionary, sizes) | `aruco_generator_node`, to print it |
| `aruco_detector.json5` | how the *detector* finds it (corner refinement, adaptive threshold) | — |

Detection runs on the **raw** camera frame: sub-pixel corner refinement reads image gradients, and
undistorting first resamples them away. The detector maps the refined corners into the rectified
frame with `undistortPoints`, so the corners on the wire pair with `K` and zero distortion. Do not
hand `detect_markers` a rectified image.

**Generated Nodes:**
- `lidar_board_detector` - One per unique (lidar, marker) pair
- `aruco_locator_node` - One per camera
- `lidar_to_camera_solver` (one per LiDAR-camera pair), selected by `solver_mode`:
  - `continuous` (default) - Auto-solves and publishes from each latest detection pair
  - `manual` - Multi-pose buffered solve with service control
- `lidar_to_lidar_solver` - One per lidar-lidar pair

**Synchronizer Parameters (Conflux):**

The maintained `lidar_to_camera_solver` and `lidar_to_lidar_solver` use
`lctk_sync.DetectionPairSource`, which owns Conflux, finite-window validation, replay recovery,
freshness/skew checks and operator diagnosis. The legacy `extrinsic_solver_node` still calls Conflux
directly, is unreachable from config-driven launch, and is scheduled for deletion by the
diamond-frame plan.

These three node parameters are populated by `calibrate.launch.py` from the calibration config's
required `sync:` section (see Configuration Format above) — there is no mode-derived fallback, and
a config that omits `sync:` is refused at parse time.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sync_tolerance_ms` | float | none — required in config | Time window in ms. Must be finite and strictly positive; 0, negative, `inf` and `nan` enable unsafe arrival-order pairing (or worse) and are refused at config parse time. |
| `sync_queue_size` | int | none — required in config | Buffer size per stream. Must be a positive integer. |
| `sync_drop_policy` | string | none — required in config | Buffer overflow policy: "reject_new" or "drop_oldest" |

**Drop Policies:**
- `reject_new`: Preserves existing buffered data. Good for offline processing where you don't want to lose any data.
- `drop_oldest`: Always accepts new data by evicting oldest. Good for realtime where latest data matters most.

**Statistics Logging:**

`DetectionPairSource` periodically logs synchronization status:
```
[INFO] sync: groups=580; pair skew last=12.4ms max=31.8ms; aruco_detections: received=800 rejected=0 dropped=0; calibration_board_detections: received=400 rejected=0 dropped=0
```

Buffer overflow warnings are rate-limited and logged automatically:
```
[WARN] Buffer overflow on '/topic': 15/100 messages rejected (15.0%), policy=REJECT_NEW, buffer_size=64
```

**Example Configs:**
- `config/examples/sample_data.yaml` - Single lidar + camera (matches `just sample-data`); the one
  maintained example still in bbox mode, via `hollow_1000/velodyne_bbox.json5`
- `config/examples/seyond_left.yaml` - Single Seyond lidar + camera, left mount
- `config/examples/seyond_right.yaml` - Single Seyond lidar + camera, right mount
- `config/examples/two_lidar.yaml` - Two lidars, no camera: `top_lidar` takes the marker-level
  Velodyne preset, `front_lidar` overrides it with the Seyond one
- `config/examples/vehicle.yaml` - Multi-sensor vehicle setup
- `config/examples/solid_600_handheld.yaml` - Selects the solid 600 mm target with an EXPERIMENTAL
  preset; no recording for it ships in the repo. Its 50 ms sync window is tighter than the hollow
  examples' because the intended recording is a hand-held, moving board

### LiDAR-to-LiDAR Calibration

The `lidar_to_lidar_solver` Python node replaces the deprecated `multi_wayside_node` for two-LiDAR calibration. It subscribes to Detection3DArray messages from two `lidar_board_detector` nodes and computes the transform between frames. **Note: This pipeline is not yet tested.**

### LiDAR-to-Camera Solver: Continuous Mode (Default)

`lidar_to_camera_solver` with `solver_mode=continuous` automatically replaces its latest detection
pair, solves it with SQPnP plus LM refinement, and publishes to
`/calibration/<lidar>_<camera>/extrinsic_transform`. This single-pose path is useful for quick visual
checks but is under-constrained by construction; low reprojection RMS is not proof of a good
calibration.

Use it for quick calibration verification or real-time transform updates.

### LiDAR-to-Camera Solver: Manual Mode

`lidar_to_camera_solver` with `solver_mode=manual` provides multi-pose calibration with manual
adjustment capabilities. Run `just solver_mode=manual lidar-camera` (or `just solver_mode=manual
demo`), then `just manual-solver-controller`.

**Services** (under `~/calibration/<pair>/lidar_to_camera_solver/`):
- `add_detection` - Add current ArUco + board detection pair to buffer
- `clear_buffer` - Clear all buffered detections
- `get_status` - Get buffer size, correspondences, solve status
- `list_buffer` - List all buffered detection pairs
- `remove_detection` - Remove detection by index
- `dump_detections` - Save detections + transform to JSON file
- `load_detections` - Load detections + transform from JSON file
- `adjust_transform` - Manual x/y/z/roll/pitch/yaw adjustment
- `reset_transform` - Reset manual adjustments (re-solve from buffer)
- `get_pose_info` - Get solved pose, current pose, and adjustment delta

**Detection File Format** (version 4):
```json
{
  "version": 4,
  "board_frame_convention": "corner_aligned_plate_center_v1",
  "num_detections": 5,
  "detections": [...],
  "transform": {
    "rvec": [rx, ry, rz],
    "tvec": [tx, ty, tz]
  }
}
```
Version 4 (H-11) records the board-frame convention that produced the file and keeps the
board pose's 6x6 covariance (v3 dropped it, so a reloaded buffer silently solved with
uniform weight). Version 3 (H-10) persists the real ArUco corner pixels inside each 2D
detection's `results`. `transform` is the raw solver output (`T_optical←lidar`), the input
the Autoware exporter consumes. A saved calibration also carries its own quality record (H-09).

**Versions below 4 are rejected, not migrated on load** — a v3 file cannot say which board
frame produced it, and reinterpreting it would make its meaning depend on the build that
opened it. Convert one you still trust:
```bash
ros2 run lidar_to_camera_solver migrate_detections \
    --input ~/detections.json --output ~/detections-v4.json \
    --assume-convention corner_aligned_plate_center_v1
```
`lctk_autoware_export` enforces the same check, because it writes into a file that reaches
a vehicle.

### Interactive Solver Controller

Rich TUI for controlling `lidar_to_camera_solver`. Run via:
```bash
ros2 run interactive_solver_controller interactive_solver_controller
```

**Key Bindings:**
```
Buffer:     Space (Add)  Backspace (Delete)  c (Clear)
File:       p (Save ~/detections.json)  o (Load)
Transform:  q/a (X)  w/s (Y)  e/d (Z)  r/f (Roll)  t/g (Pitch)  y/b (Yaw)
Step Size:  ] (Increase)  [ (Decrease)
Reset:      0 (Re-solve from buffer)
Exit:       ESC
```

**Display Panels:**
- Buffer Status: Detection count, correspondences, publishing status
- Pose Information: Three columns showing Solved (PnP), Adjustment (delta), Current (final)
- Step Size: Current translation (mm) and rotation (deg) step sizes
- Key Bindings: Quick reference for all controls

### Exporting to Autoware

`lctk_autoware_export` patches one entry of an Autoware `sensor_kit_calibration.yaml` with a
solved extrinsic. Full guide: `book/src/user-guide/autoware-export.md`; design:
`docs/superpowers/specs/2026-07-16-autoware-export-design.md`.

```bash
ros2 run lctk_autoware_export export \
  --detections ~/detections.json \
  --target .../sensor_kit_calibration.yaml \
  --camera-frame camera0/camera_link \
  --lidar-frame velodyne_top_base_link \
  --dry-run    # preview; drop to write (comment-preserving, creates .bak)
```

**Frame pitfalls the exporter owns — do not "fix" these ad hoc elsewhere:**
- Input is the dump JSON's raw `rvec`/`tvec` (`T_optical←lidar`), **never** the TF topic —
  the published frame labels are inverted (issue M-01)
- Autoware's `camera*/camera_link` is the REP-103 body frame (x forward); PnP solves the
  optical frame (z forward). Fixed rotation `T(camera_link→optical)` = RPY `(-π/2, 0, -π/2)`
- The exported entry is `T(kit→camera_link) = T(kit→lidar) · inv(solve) · inv(optical-in-link)`,
  with `T(kit→lidar)` read from the target YAML's existing lidar entry
- Autoware YAML schema is `parent: {child: {x,y,z,roll,pitch,yaw}}`, meters, radians, URDF
  fixed-axis RPY. Same schema in every Autoware version; only the file's location moved
  (`autoware_individual_params` per-`$VEHICLE_ID` dirs ≤ 2024.11; folded into
  `autoware_launch/sensor_kit/<kit>_launch/<kit>_description/config/` since 0.45.1)
