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

# Launch a calibration session
just demo

# See all commands
just
```

## Project Structure

- **`rust/`**: Pure Rust libraries (aruco-config, aruco-detector, calibration-target, calibration-target-detector, board-cluster-detector, etc.)
- **`ros/`**: ROS 2 packages
  - `lctk_launch/` - Unified launch system with config-driven calibration pipeline
  - `lctk_interfaces/` - Shared msg/srv definitions (solver services, quality report)
  - `aruco_locator_node/` - ArUco marker detection from camera images
  - `aruco_generator_node/` - Prints the ArUco board pattern from a Target Definition (`--target-config`)
  - `lidar_board_detector/` - Calibration board detection from point clouds
  - `extrinsic_solver_node/` - Superseded LiDAR-camera solver (unreachable from config-driven launch; pending deletion)
  - `lidar_to_camera_solver/` - LiDAR-camera solver with continuous, manual and assisted modes
  - `interactive_solver_controller/` - Rich TUI driving `lidar_to_camera_solver`
  - `lidar_to_lidar_solver/` - LiDAR-to-LiDAR calibration solver
  - `lctk_quality/` + `calibration_judge/` - Extrinsic quality metric (H-09)
  - `pointcloud_image_overlay/` - Projects the cloud onto the image for visual verification
  - `filter_box_tuner/` - Interactive crop-box tuning for the board detector
  - `lctk_autoware_export/` - Exports a solved extrinsic into Autoware `sensor_kit_calibration.yaml`
  - `lctk_sample_data/` - Sample data playback (pcap + avi), plus gitignored recorded
    `bags/TWO_LIDAR_*` (two-LiDAR: VLP-32C + solid-state Falcon; see `bags/README.md`)
  - `lctk_target/` - ROS-free validated Target Definition loader and board-local ArUco
    geometry (`load_target`, `TargetIdentity`), shared by the solvers, `lctk_launch` and
    `aruco_generator_node`
  - `lctk_sync/` - `DetectionPairSource`: owns Conflux, finite-window validation, replay
    recovery, freshness/skew checks and operator diagnosis for the maintained solvers
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
      --packages-ignore conflux conflux_cpp conflux_py \
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

just sessions       # List the shipped sessions
just check <name>   # Validate a session without launching a graph
just run <name>     # Run a session end to end (data source + calibration graph)
just demo           # Alias for `just run sample3-hollow-velodyne`
just new <path>     # Scaffold a session from an existing one
just sample-data    # Launch sample data playback on its own
just rviz           # Launch RViz

# `run` takes a session name (resolved against ./sessions/ then the installed
# share) or an explicit path. The ros2 interface underneath takes only a path.
just run /abs/path/to/my-session

# Calibration graph only, against an explicit config file
just calibrate /path/to/session.yaml

# Justfile variables go BEFORE the recipe name -- `just demo mode=realtime`
# is parsed as a recipe argument and fails.
just mode=realtime demo              # Use realtime mode (BEST_EFFORT QoS, no buffering)
just mode=offline demo               # Use offline mode (RELIABLE QoS, default)
just solver_mode=manual demo         # Use service-driven multi-pose buffering
just assisted                        # Auto-capture still, novel poses; review at :8080
just debug_mode=false demo           # Disable debug output

# Documentation (run from book/ directory)
just build          # Build docs
just serve          # Serve with live reload
just serve-public   # Serve on 0.0.0.0
```

## Testing Practices

**A test recipe that cannot fail is worse than no recipe.** It converts "untested" into
"believed tested", which is the more expensive state. This has bitten repeatedly, in two
distinct shapes.

**Shape 1 — the recipe runs but cannot fail.**

- `just test` ended with a bare `pytest ...`. apt's `python3-pytest` installs the package but
  no `pytest` executable, so the line exited 127 and LCTK's four Python suites — 92 tests
  covering the config parser, the planner, pose weighting, the quality metric and the whole
  Autoware export path — never ran through the documented entry point (L-28). Fixed by
  invoking it as `python3 -m pytest`.
- In the conflux submodule, `just test-cpp` echoed two lines and exited 0 while `conflux_cpp`
  had no tests at all, and `just test-python` ran pytest-style tests through `colcon test`'s
  unittest path, collecting 0 of 19 and exiting 0. See `ros/conflux/CLAUDE.md`.

**Shape 2 — nothing runs the suite.** Harder to notice, because the recipe you *do* run is
perfectly honest about the packages it covers.

- `crates/conflux-ros2` is excluded from the cargo workspace, so no recipe ran its tests. A
  duplicate synchronization algorithm lived there for months, covered only by tests that
  exercised a bare `VecDeque` rather than any of the real code (H-14).
- `calibration_judge` had no test directory at all until 2026-08-16 (M-17).

### The check

**When adding or changing a test recipe, break an assertion deliberately and confirm a
non-zero exit before trusting it.** Seconds long, and it caught every instance above:

```bash
# make one assertion false, then:
just test > /dev/null 2>&1; echo "exit=$?"   # must be non-zero
# restore, then confirm it returns to 0
```

Two traps when checking this by hand:

- **Don't read `$?` through a pipe.** `just test | grep ...; echo $?` reports grep's status, not
  the recipe's. This produced a false "exit=0" during the L-22 work and nearly let a
  can't-fail recipe through a second time.
- **Clear stale results before re-checking colcon suites.** `colcon test-result` reads
  `build/<pkg>/test_results/`, which survives across runs, so a fixed suite can still report
  yesterday's failures. `rm -rf build/<pkg>/test_results` first. (Applies to the conflux
  submodule's `just test-cpp`; LCTK's own suite is cargo + pytest and has no such cache.)

### Adding a new package

New ROS packages are not picked up automatically. Add the test directory to the `pytest`
invocation in the `test` recipe, then run the break-an-assertion check to confirm it is
actually wired in.

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

3. **pip packages shadowing apt ones** (this has bitten four times — `just build` guards the first three):

   A pip `--user` install lands in `~/.local/lib/python3.10/site-packages`, which **precedes**
   `/usr/lib/python3/dist-packages` on `sys.path` and silently shadows the apt package that ROS 2
   Humble and apt's OpenCV were built against. All known cases fail far from the cause:

   | symptom | when | fix |
   |---------|------|-----|
   | `error: option --editable not recognized` (kills `conflux_py` and every ament_python package) | **build** time | `pip3 uninstall -y setuptools` |
   | `ImportError: numpy.core.multiarray failed to import` (kills every solver node at startup, after a clean build) | **run** time | `pip3 uninstall -y numpy` |
   | `TypeError: 'numpy._DTypeMeta' object is not subscriptable` inside scipy (kills any test/node importing `scipy.optimize`) | **test/run** time | `pip3 uninstall -y scipy` |
   | `ModuleNotFoundError: No module named '_pytest.scope'` raised from `anyio/pytest_plugin.py` during pytest **startup** (kills every pytest run in the workspace before a single test is collected) | **test** time | `pip3 uninstall -y anyio` |

   setuptools >= 80 removed the `setup.py develop --editable` step colcon uses for
   `--symlink-install`; numpy >= 2 breaks the ABI apt's `cv2` was compiled against;
   scipy >= 1.15 requires numpy >= 1.23 while apt ships 1.21;
   anyio >= 4.3 ships a pytest plugin that imports `_pytest.scope`, added in pytest 7 — apt ships
   pytest 6.2.5, and plugin autoload pulls it in on *every* pytest invocation.
   **Never `pip3 install --user` setuptools, numpy, scipy, or anyio on this machine** — and note
   that installing *anything else* with pip can drag them in as dependencies.

   **anyio was uninstalled on 2026-08-15 to unblock the conflux test suite.** It was a dependency
   of the pip `--user` `starlette`, which is now broken. If you need starlette/fastapi back,
   reinstall into a venv rather than `--user`; a bare `pip3 install --user anyio` re-breaks pytest
   workspace-wide. Escape hatch without uninstalling: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest ...`.

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

8. **Dangling symlinks after a source file is deleted** (L-29, auto-guarded): `--symlink-install`
   symlinks package data files into `build/` and `install/` rather than copying them. Delete a
   launch file — in a rebase, say — and the symlink is left pointing at nothing, so the next build
   fails:
   ```
   error: can't copy '.../build/lctk_launch/launch/<file>.launch.xml': doesn't exist or not a regular file
   ```
   The path it names still appears in `ls`, because a dangling symlink is a directory entry with no
   target, which makes the message read as nonsense until you know to look for that.

   `just build` now prunes broken symlinks from both trees before invoking colcon and says what it
   removed. A broken symlink is never useful — colcon recreates the ones that should exist. If you
   hit this on an older checkout, the manual form is:
   ```bash
   find build install -xtype l -delete
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
- **Never commit `Cargo.lock` churn from a build.** Building in the sourced ROS environment
  rewrites the workspace lockfiles with this machine's generated message-crate versions and
  `[[patch.unused]]` entries. That is local build state, not a change: `git checkout --` them
  before committing. Real dependency updates are a separate, deliberate procedure — see
  `docs/roadmap/phase-4-dependency-updates-and-vulns.md`

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
- **Tracker state as of 2026-08-16: 71 🟢 fixed, 3 ⚪ won't-fix, 0 open.** Every finding from the
  2026-07-09, 2026-07-12 and 2026-08-15 audits is closed. Read the archived issue before
  re-opening one — several record *why* a fix took the shape it did, which is the part that is
  expensive to rediscover

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
just mode=offline demo    # For recorded/sample-data playback (default)
just mode=realtime demo   # For live sensors
```

### Performance Profiling Results

Profiling conducted on sample data (2026-01-18). These numbers predate the Phase 8 selectable-target
detector path (Target Definition / Detector Tuning split); the new path has not yet been re-profiled
against real data — see "Outstanding items no packet owns" in
`docs/roadmap/phase-8-selectable-calibration-targets.md`. The numbers below were real when measured
and are kept rather than deleted, but treat them as a baseline from the prior implementation, not a
claim about the current one.

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

### Calibration Sessions (Preferred)

A **session** is one directory describing one run: where the data comes from, and everything
needed to calibrate against it. It replaces the old split between a playback launch file that
hard-coded a recording and a `config/examples/*.yaml` that restated its topics by hand — a
split whose disagreements were always silent. Full guide: `book/src/user-guide/sessions.md`;
design: `docs/superpowers/specs/2026-08-31-calibration-sessions-design.md`.

Shipped sessions live in `sessions/` and install to `share/lctk_launch/sessions/`.
`config/examples/` is **deleted**; there is no automatic migration.

**Usage:**
```bash
# End to end: data source + calibration graph
ros2 launch lctk_launch session.launch.py session:=/path/to/sessions/sample3-hollow-velodyne

# Data only, then calibration only (a live rig, or a bag you play yourself)
ros2 launch lctk_launch session_data.launch.py session:=/path/to/<session>
ros2 launch lctk_launch calibrate.launch.py config_file:=/path/to/<session>/session.yaml

# Validate a session without launching a graph -- resolves every path, checks the
# data exists, verifies bag topics, prints the topics and frames each device will use
ros2 run lctk_launch lctk_session check /path/to/<session>
ros2 run lctk_launch lctk_session list
ros2 run lctk_launch lctk_session new /path/to/new --from /path/to/template
```

`session:=` is **always an explicit path** (directory or `session.yaml`). There is no search
path and no `LCTK_SESSION_PATH`: an implicit location would assume both where sessions live
and where the user is standing. Bare-name lookup lives in the justfile instead — `just run
<name-or-path>`, `just check`, `just sessions`, `just new`, `just demo`.

**The `data:` section** decides who owns the topic names, because what is knowable differs:

| `kind` | required | topics | what launches |
|---|---|---|---|
| `pcap_avi` | `dir` (holds `lidar.pcap`, `video.avi`) | **derived** from device names; stating one is refused | the `lctk_sample_data` playback |
| `bag` | `path` (rosbag2 dir with `metadata.yaml`) | **stated**, then verified against the bag | `ros2 bag play --clock` |
| `live` | none | **stated** | nothing |

Derived names are `/sensing/lidar/<device>/pointcloud_raw`,
`/sensing/lidar/<device>/velodyne_packets`, `/sensing/camera/<device>/image_raw`,
`/sensing/camera/<device>/camera_info` — exactly `lidar_camera.launch.xml`'s old defaults.

`$(session-dir)` expands to the manifest's directory. Use it for every session-local file
(`bbox.json5`, `camera_info.yaml`, `rviz.rviz`, `out/detections.json`) so the directory stays
relocatable. Using it outside a session directory is refused, not silently emptied.

If a session ships `rviz.rviz`, `session.launch.py` forwards it as `rviz_config`; an explicit
`rviz_config:=` still wins. The judge's ground truth is *not* session-local yet —
`calibrate.launch.py` declares no argument to forward one through.

A manifest with no `data:` section still parses, so `calibrate.launch.py` keeps working
against plain configs.

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
target while each keeps its own sensor-specific tuning; `sessions/twolidar-vlp32-falcon/` does
exactly this.

The legacy `type`/`board_config`/`aruco_config` marker keys (and a lidar device's `board_config`
key) are now **retired**: `config_parser.py` raises a `ValueError` naming the offending key rather
than parsing it, pointing at `target_config`/`detector_config` as the replacement. There is no
automatic migration for launch YAML — replace the retired keys by hand, the same reasoning
`detection_format.py` applies to a saved detection archive.

**Physical layout and detector tuning are separate files:**

| File | Describes | Also read by |
|------|-----------|--------------|
| `config/targets/<target>.json5` | the *physical target* (plate, cutouts, marker IDs, dictionary, sizes) | `aruco_generator_node --target-config`, to print it |
| `aruco_detector.json5` | how the *detector* finds it (corner refinement, adaptive threshold) | — |

The standalone `aruco_pattern.json5` that used to hold the printed-board half was deleted in
W5-E1: the Target Definition is now the single source of that geometry.

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
  - `assisted` - Auto-captures still, geometrically-new poses and serves a browser review
    page; also creates the manual services
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

**Shipped Sessions** (`sessions/`, one directory each, every one with its own README):
- `sample3-hollow-velodyne` - `pcap_avi`, dataset 3. The only session still in bbox mode, via
  `hollow_1000/velodyne_bbox.json5` plus its own `bbox.json5`. The data ships in git; this is
  what `just demo` runs. Its lidar device is named `top`, which is what makes the derived
  topics reproduce the old `/sensing/lidar/top/pointcloud_raw`
- `seyond-left` - `live`. Seyond Falcon + left camera; ships `rviz.rviz`
- `seyond-right` - `live`. Seyond Falcon + right camera. The old example named this device
  `left_camera` while giving it the right camera's topic and frame; the session corrects it
- `twolidar-vlp32-falcon` - `bag` (`TWO_LIDAR_1`, gitignored). Two lidars, no camera;
  `front_lidar` overrides the marker-level Velodyne preset with the Seyond one. Its topics are
  the ones the bag actually records, verified at parse time (M-26)
- `vehicle-multisensor` - `live`. Two lidars, four cameras, three markers. A schema
  demonstration; there is no rig behind it
- `solid600-handheld-zed` - `live`. Solid 600 mm target with an EXPERIMENTAL preset; no
  recording ships. Its 50 ms sync window is tighter than the hollow sessions' because the
  board is hand-held and moving. Ships `rviz.rviz` (M-27)

### LiDAR-to-LiDAR Calibration

The `lidar_to_lidar_solver` Python node replaces the deprecated `multi_wayside_node` for two-LiDAR calibration. It subscribes to Detection3DArray messages from two `lidar_board_detector` nodes and computes the transform between frames.

**Verified end-to-end on 2026-08-15** (M-16) against sample datasets 3 + 4: 81 solves with a
0.304 m lateral baseline, repeatable to σ ≈ 1–9 mm in translation and ≈0.2° in rotation, which is
inside the VLP-32C's ±3 cm range noise. Two bugs had to be fixed before it could run at all —
`two_lidar.launch.xml` used an invalid `$(eval not loop)` substitution, and defaulted the second
LiDAR to UDP port 2369 while both shipped pcaps are recorded on 2368, so the second sensor
published nothing and no pair could ever synchronize.

To reproduce:

```bash
# terminal 1
ros2 launch lctk_sample_data two_lidar.launch.xml
# terminal 2
just two-lidar
```

### LiDAR-to-Camera Solver: Continuous Mode (Default)

`lidar_to_camera_solver` with `solver_mode=continuous` automatically replaces its latest detection
pair, solves it with SQPnP plus LM refinement, and publishes to
`/calibration/<lidar>_<camera>/extrinsic_transform`. This single-pose path is useful for quick visual
checks but is under-constrained by construction; low reprojection RMS is not proof of a good
calibration.

Use it for quick calibration verification or real-time transform updates.

### LiDAR-to-Camera Solver: Assisted Mode

`lidar_to_camera_solver` with `solver_mode=assisted` auto-captures a detection pair whenever the
board is held still in a placement it has not seen before, and serves a review page on
`http://localhost:8080`. Run `just assisted`, then open the page.

Two gates decide a capture, and both are load-bearing:

- **Stillness** — the board pose's *span across a sliding window*, not its frame-to-frame delta.
  A board drifting at 1 mm per frame has a negligible per-frame delta and is not still.
- **Novelty** — `lctk_quality.distinct_placements` (5 cm / 5 deg). Without it an auto-capture loop
  manufactures the degenerate capture `lctk_quality.diversity` exists to detect, where reprojection
  RMSE and subset resampling both *invert* and rate one placement filmed nine times as excellent.
  The page therefore leads with the diversity meter, not the residual.

This is the only solver that subscribes to a camera image. The frame is used for the review
preview only, never for the solve, and a missing frame never blocks a capture. Full guide:
`book/src/user-guide/assisted-capture.md`; design:
`docs/superpowers/specs/2026-08-31-assisted-extrinsic-solver-design.md`.

The review server is **unauthenticated**; it binds `127.0.0.1` unless `review_bind_host` says
otherwise, and logs a warning naming the exposure when it does.

### LiDAR-to-Camera Solver: Manual Mode

`lidar_to_camera_solver` with `solver_mode=manual` provides multi-pose calibration with manual
adjustment capabilities. Run `just solver_mode=manual lidar-camera` (or `just solver_mode=manual
demo`), then `just extrinsic-solver-controller`.

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

**Detection File Format** (version 5):
```json
{
  "version": 5,
  "board_frame_convention": "corner_aligned_plate_center_v1",
  "target_identity": {
    "schema_version": 1,
    "target_id": "hollow_1000_aruco_4",
    "revision": 1,
    "semantic_sha256": "<64 lowercase hex chars>",
    "board_frame_convention": "corner_aligned_plate_center_v1"
  },
  "num_detections": 5,
  "detections": [...],
  "transform": {
    "rvec": [rx, ry, rz],
    "tvec": [tx, ty, tz]
  }
}
```
Version 5 adds the full **Target Identity** — `schema_version`, `target_id`, `revision`,
`semantic_sha256` and `board_frame_convention` — binding the archive to the exact Target
Definition it was captured against. A solver restores a version-5 archive only when every
identity field exactly matches its locally selected target; a mismatch is refused, not
silently reinterpreted.

Version 4 (H-11) records the board-frame convention that produced the file and keeps the
board pose's 6x6 covariance (v3 dropped it, so a reloaded buffer silently solved with
uniform weight). It has no Target Identity and **cannot be restored** into a running
solver's buffer — it remains useful only for migration and for `lctk_autoware_export`,
which needs the solved transform's provenance, not a target match. Version 3 (H-10)
persists the real ArUco corner pixels inside each 2D detection's `results`. `transform` is
the raw solver output (`T_optical←lidar`), the input the Autoware exporter consumes. A
saved calibration also carries its own quality record (H-09).

**Versions below the current one are rejected, not migrated on load** — a v3 file cannot
say which board frame produced it, and a v4 file cannot say which target it was captured
against; reinterpreting either would make the file's meaning depend on the build that
opened it. Reaching version 5 from a version-3 file takes two explicit hops, each naming a
different operator claim:
```bash
# 1. version 3 -> 4: name the board-frame convention the file was CAPTURED in
ros2 run lidar_to_camera_solver migrate_detections \
    --input ~/detections-v3.json --output ~/detections-v4.json \
    --assume-convention corner_aligned_plate_center_v1

# 2. version 4 -> 5: bind the Target Definition the file was CAPTURED against
ros2 run lidar_to_camera_solver migrate_detections \
    --input ~/detections-v4.json --output ~/detections-v5.json \
    --target-config /path/to/config/targets/<target>.json5
```
A file already at version 4 needs only the second hop. Migrating straight from version 3 to
5 in one invocation is refused — each hop is a distinct claim the operator must make
explicitly. Step 2 checks that every marker ID the archive actually observed belongs to the
selected target (catching an obviously wrong selection), but it cannot prove which physical
target produced the recording; that remains the operator's assertion.

`lctk_autoware_export` accepts both version 4 and version 5 archives — it needs the solved
transform, not a target match — and enforces the same board-frame-convention gate, because
it writes into a file that reaches a vehicle.

**Two directions coexist deliberately, and mixing them is the M-01 bug:**

| surface | direction | why |
|---------|-----------|-----|
| `extrinsic_transform` topic and `/tf_static` | `T_lidar←camera` (TF semantics) | what every tf2 consumer expects |
| dump JSON `rvec`/`tvec` | `T_optical←lidar` (raw solvePnP) | what the exporter and any `projectPoints` call need |

`pointcloud_image_overlay` consumes the *topic* and inverts it back internally. If you add a
consumer, decide which of the two it needs — do not assume the topic is the PnP output. A saved calibration also carries its own quality record (H-09).

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
- Input is the dump JSON's raw `rvec`/`tvec` (`T_optical←lidar`). This is still the right
  input — but no longer because the topic is broken. M-01 is **fixed**: since 2026-08-15 the
  `extrinsic_transform` topic follows ROS TF semantics, so `frame_id=lidar, child=camera` is
  genuinely the camera's pose in lidar coordinates. Use the dump JSON because the exporter's
  arithmetic is written against the raw solve and is tested that way; if you ever switch it to
  the topic, invert first
- Autoware's `camera*/camera_link` is the REP-103 body frame (x forward); PnP solves the
  optical frame (z forward). Fixed rotation `T(camera_link→optical)` = RPY `(-π/2, 0, -π/2)`
- The exported entry is `T(kit→camera_link) = T(kit→lidar) · inv(solve) · inv(optical-in-link)`,
  with `T(kit→lidar)` read from the target YAML's existing lidar entry
- Autoware YAML schema is `parent: {child: {x,y,z,roll,pitch,yaw}}`, meters, radians, URDF
  fixed-axis RPY. Same schema in every Autoware version; only the file's location moved
  (`autoware_individual_params` per-`$VEHICLE_ID` dirs ≤ 2024.11; folded into
  `autoware_launch/sensor_kit/<kit>_launch/<kit>_description/config/` since 0.45.1)
