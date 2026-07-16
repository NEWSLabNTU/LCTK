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

- **`rust/`**: Pure Rust libraries (aruco-config, aruco-detector, hollow-board-detector, board-fitter, plane-estimator, etc.)
- **`ros/`**: ROS 2 packages
  - `lctk_launch/` - Unified launch system with config-driven calibration pipeline
  - `lctk_interfaces/` - Shared msg/srv definitions (solver services, quality report)
  - `aruco_locator_node/` - ArUco marker detection from camera images
  - `aruco_generator_node/` - Prints the ArUco board pattern from `aruco_pattern.json5`
  - `lidar_board_detector/` - Calibration board detection from point clouds
  - `extrinsic_solver_node/` - Auto-publishing single-pose LiDAR-camera solver (default)
  - `advanced_extrinsic_solver/` - Multi-pose buffered LiDAR-camera solver with services
  - `interactive_solver_controller/` - Rich TUI driving the advanced solver
  - `lidar_to_lidar_solver/` - LiDAR-to-LiDAR calibration solver
  - `lctk_quality/` + `calibration_judge/` - Extrinsic quality metric (H-09)
  - `pointcloud_image_overlay/` - Projects the cloud onto the image for visual verification
  - `filter_box_tuner/` - Interactive crop-box tuning for the board detector
  - `lctk_autoware_export/` - Exports a solved extrinsic into Autoware `sensor_kit_calibration.yaml`
  - `lctk_sample_data/` - Sample data playback (pcap + avi; there are **no rosbags** in this repo)
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

## Key Commands

```bash
just build      # Build all packages
just clean      # Clean build artifacts
just test       # Run tests
just lint       # Run linting

just lidar-camera   # Launch calibration (legacy XML launch)
just demo           # Launch demo (sample data + calibration pipeline)
just sample-data    # Launch sample data playback
just rviz           # Launch RViz

# Config-driven calibration (preferred)
just calibrate /path/to/config.yaml

# Justfile variables (override with just var=value command)
just demo mode=realtime              # Use realtime mode (BEST_EFFORT QoS, no buffering)
just demo mode=offline               # Use offline mode (RELIABLE QoS, default)
just demo use_advanced_solver=true   # Use multi-pose buffered solver
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

The calibration pipeline supports two processing modes controlled by the `mode` parameter:

| Mode | QoS | Sync Window | Buffer Size | Drop Policy | Use Case |
|------|-----|-------------|-------------|-------------|----------|
| `offline` (default) | RELIABLE | infinite | 100 | reject_new | Recorded data (rosbags or the pcap/avi sample playback). No time-based dropping, preserves all data. |
| `realtime` | BEST_EFFORT | 50ms | 2 | drop_oldest | Live sensor data. Low latency, always processes latest. |

Note: the repo ships no rosbags — the only recorded data is `lctk_sample_data`'s pcap + avi
(datasets 1–5; dataset 3 is the lidar-camera default, dataset 4 the second lidar). To get a real
bag, record one during playback: `ros2 bag record -a` alongside `just sample-data`.

**Settings derived from mode:**
- **QoS Reliability**: RELIABLE (offline) vs BEST_EFFORT (realtime)
- **Sync Window**: Infinite (offline) vs 50ms tolerance (realtime)
  - Infinite window: Messages are matched regardless of timestamp difference
  - Finite window: Messages outside the time window are dropped
- **Buffer Size**: Large buffer (offline) vs minimal buffering (realtime)
- **Drop Policy**:
  - `reject_new`: When buffer is full, reject new messages (preserves older data)
  - `drop_oldest`: When buffer is full, drop oldest message (always accepts new data)

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
    type: hollow_board
    board_config: $(find-pkg-share lctk_launch)/config/board/board_detector.json5
    aruco_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_pattern.json5
    bbox_config: $(find-pkg-share lctk_launch)/config/board/bbox.json5
    # Optional. ArUco *detector* tuning (corner refinement, adaptive threshold).
    # Defaults to config/aruco/aruco_detector.json5 when omitted.
    aruco_detector_config: $(find-pkg-share lctk_launch)/config/aruco/aruco_detector.json5
    # Calibration pairs are defined inside each marker as a list of
    # [deviceA, deviceB] pairs that observe this marker.
    pairs:
      - [top_lidar, front_center]
```

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
- LiDAR-camera solver (one per pair) - selected by `use_advanced_solver` argument:
  - `extrinsic_solver_node` (default) - Auto-publishes transform on each detection pair
  - `advanced_extrinsic_solver` - Multi-pose buffered solver with manual control
- `lidar_to_lidar_solver` - One per lidar-lidar pair

**Synchronizer Parameters (Conflux):**

All solver nodes use the Conflux synchronizer with these parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sync_tolerance_ms` | float | mode-dependent | Time window in ms. 0 = infinite window (no time-based dropping) |
| `sync_queue_size` | int | mode-dependent | Buffer size per stream |
| `sync_drop_policy` | string | "reject_new" | Buffer overflow policy: "reject_new" or "drop_oldest" |

**Drop Policies:**
- `reject_new`: Preserves existing buffered data. Good for offline processing where you don't want to lose any data.
- `drop_oldest`: Always accepts new data by evicting oldest. Good for realtime where latest data matters most.

**Statistics Logging:**

All solver nodes log synchronization statistics on shutdown:
```
[INFO] Final sync statistics: received=1200, rejected=0, groups=580, rejection_rate=0.0%
[INFO]   aruco_detections: received=800, rejected=0, rejection_rate=0.0%
[INFO]   calibration_board_detections: received=400, rejected=0, rejection_rate=0.0%
```

Buffer overflow warnings are rate-limited and logged automatically:
```
[WARN] Buffer overflow on '/topic': 15/100 messages rejected (15.0%), policy=REJECT_NEW, buffer_size=64
```

**Example Configs:**
- `config/examples/sample_data.yaml` - Single lidar + camera (matches `just sample-data`)
- `config/examples/vehicle.yaml` - Multi-sensor vehicle setup

### LiDAR-to-LiDAR Calibration

The `lidar_to_lidar_solver` Python node replaces the deprecated `multi_wayside_node` for two-LiDAR calibration. It subscribes to Detection3DArray messages from two `lidar_board_detector` nodes and computes the transform between frames. **Note: This pipeline is not yet tested.**

### Standard Extrinsic Solver (Default)

The `extrinsic_solver_node` automatically publishes transforms whenever it receives a synchronized ArUco detection and board detection pair. No manual intervention required - transforms are published continuously to `/calibration/<lidar>_<camera>/extrinsic_transform`.

Use this for quick calibration verification or when you want real-time transform updates.

### Advanced Extrinsic Solver

The `advanced_extrinsic_solver` node provides multi-pose calibration with manual adjustment capabilities. Enable with `use_advanced_solver=true`.

**Services** (under `~/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/`):
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

**Detection File Format** (version 3):
```json
{
  "version": 3,
  "num_detections": 5,
  "detections": [...],
  "transform": {
    "rvec": [rx, ry, rz],
    "tvec": [tx, ty, tz]
  }
}
```
Version 3 (H-10) persists the real ArUco corner pixels inside each 2D detection's `results`;
v1/v2 files reload but fall back to the axis-aligned bbox — a biased (C-01) solve — and the
loader warns loudly. `transform` is the raw solver output (`T_optical←lidar`), the input the
Autoware exporter consumes. A saved calibration also carries its own quality record (H-09).

### Interactive Solver Controller

Rich TUI for controlling the advanced_extrinsic_solver. Run via:
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
