# Crop-box-free Board Detection ROS Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire the parity-validated `board-projection-detector` crate into the ROS
`lidar_board_detector` node so a user can locate the calibration board with **no bounding box** by
setting `detection_mode: bbox_free`, feeding the detector's `selected_points` into the node's
existing PCA+ICP pose engine.

**Architecture:** Option C — the new detector replaces only the Stage-1 bbox-crop stage of
`process_pointcloud`; its `selected_points` flow unchanged into the existing Stage-2
(`skip_ransac` PCA plane) → Stage-3 ICP → `icp_good_fit_threshold` gate. All new pure logic
(config parsing, Method-E warmup state machine) lives in a new `bbox_free` submodule with unit
tests; the ROS wiring in `main.rs` stays thin. The bbox path is fully preserved behind the mode
check.

**Tech Stack:** Rust, rclrs 0.7, colcon (`just build`), cargo-nextest (`just test`), nalgebra
0.32.3, json5, serde, arc-swap. The consumed crate `board-projection-detector` is
OpenCV/open3d-free (deps: nalgebra, anyhow, serde, json5, log, rand).

## Global Constraints

- **Build only with `just build`** (colcon, base-paths `ros`, `--cargo-args --profile=test-release`),
  never raw `cargo`/`colcon`. Run from project root. See CLAUDE.md for pip-shadowing + `bindgen.lock`
  known issues.
- **Dependency/lockfile changes must run inside the sourced ROS env:**
  `source /opt/ros/humble/setup.bash && source install/setup.bash`. Plain `cargo update` aborts on
  the yanked wildcard `sensor_msgs`.
- **Tests:** `just test` (cargo-nextest). The node's Rust tests live in `main.rs`'s `#[cfg(test)]`
  module and in the new `bbox_free` submodule; they run under colcon after the fold.
- **MSRV 1.85.0.** Named params in format strings: `println!("{e}")` not `println!("{}", e)`.
- **VLP-32C noise floor ~0.026–0.031 m.** Do not lower any metric gate (`flatness_rms_max`,
  `icp_good_fit_threshold`) below it — a sub-floor gate silently accepts nothing (C-04 class bug).
- **`BoardConfig` serde defaults are the frozen library defaults** (`flatness_rms_max` 0.035,
  `stance_floor` 0.0, `isolation` false), which differ from the production operating point
  (0.045 / 0.9 / true). The shipped `bbox_free.board` block must spell out production values
  explicitly.
- **Point type:** node uses `na::Point3<f64>` = the crate's `nalgebra::Point3<f64>` (same nalgebra
  0.32.3) — pass directly, no conversion.
- **Default behavior unchanged:** `detection_mode` defaults to `"bbox"`; the bbox path must remain
  byte-identical to today.
- Commit after each task with a conventional-commit subject. End every commit message body with:
  `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`
- Current branch is `feat/method-e-background-subtraction`; commit there. `git fetch` + rebase
  before pushing (multiple agents share this repo).

## File Structure

- **Modify** `rust/board-projection-detector/Cargo.toml` — drop `[workspace]`, deps → `{ workspace = true }`.
- **Modify** `Cargo.toml` (root) — remove crate from `exclude`.
- **Modify** `ros/lidar_board_detector/Cargo.toml` — add path dep on the crate.
- **Create** `ros/lidar_board_detector/src/bbox_free.rs` — pure config types + parsing +
  `BackgroundState` warmup state machine + unit tests. One responsibility: everything bbox-free that
  does **not** touch rclrs.
- **Modify** `ros/lidar_board_detector/src/main.rs` — `mod bbox_free;`, thread the parsed config +
  background state through the processing thread, splice the mode branch into `process_pointcloud`,
  reject diagnostics, `reset_background`.
- **Modify** `ros/lctk_launch/config/board/board_detector.json5` — add `detection_mode` +
  `bbox_free` block.

---

### Task 1: Fold `board-projection-detector` into the root workspace

Pays the sub-project-1 "standalone workspace for dev speed" debt: the node (a root member) must
path-depend on the detector, which requires the detector to be a root member too.

**Files:**
- Modify: `rust/board-projection-detector/Cargo.toml`
- Modify: `Cargo.toml` (root)
- Modify: `ros/lidar_board_detector/Cargo.toml`

**Interfaces:**
- Consumes: nothing.
- Produces: crate `board-projection-detector` v0.1.0 resolvable as a root workspace member and as a
  path dependency of `lidar_board_detector`. Public API unchanged:
  `board_projection_detector::{detector::detect, detector::DetectOutcome, detector::RejectReason,
  config::{BoardConfig, ForegroundMethod, load_board_config_json5}, background::BackgroundModel}`.

- [ ] **Step 1: Remove the crate's `[workspace]` table and switch deps to workspace**

In `rust/board-projection-detector/Cargo.toml`, delete the `[workspace]` line (keep the explanatory
header comment but update it — see Step 2). Change `[dependencies]` and `[dev-dependencies]` to:

```toml
[dependencies]
nalgebra = { workspace = true }
anyhow = { workspace = true }
serde = { workspace = true }
json5 = { workspace = true }
log = { workspace = true }
rand = { workspace = true }

[dev-dependencies]
approx = { workspace = true }
serde_json = { workspace = true }
```

(All eight are present in the root `[workspace.dependencies]`. `nalgebra`'s workspace entry already
carries `features = ["serde-serialize"]`; `serde` already carries `features = ["derive"]`.)

- [ ] **Step 2: Update the crate header comment**

Replace the top-of-file comment block in `rust/board-projection-detector/Cargo.toml` with:

```toml
# Root workspace member. ROS-free crate (deps are all pure-Rust: nalgebra,
# anyhow, serde, json5, log, rand). Folded in from a former standalone
# workspace at sub-project 2 so `lidar_board_detector` can path-depend on it.
# NOTE: as a root member it now shares the ROS-poisoned root resolve
# (aruco-detector -> sensor_msgs = "*", yanked), so plain `cargo test` here no
# longer works — build/test only via colcon (`just build` / `just test`).
# Parity fixtures (~51 MB) stay local/gitignored (regenerate via
# experiments/board-detection-2d/tools/export_golden.py).
```

- [ ] **Step 3: Remove the crate from the root `exclude`**

In root `Cargo.toml`, delete these two lines from the `exclude` array:

```toml
    # Standalone workspace: ROS-free crate kept out of the ROS-poisoned root
    # resolve (see the crate's Cargo.toml header).
    "rust/board-projection-detector",
```

The `members = ["rust/*", ...]` glob now picks the crate up automatically.

- [ ] **Step 4: Add the path dependency to the node**

In `ros/lidar_board_detector/Cargo.toml`, under `[dependencies]` (alphabetical, after
`aruco-config`), add:

```toml
board-projection-detector = { version = "0.1.0", path = "../../rust/board-projection-detector" }
```

- [ ] **Step 5: Build to regenerate the lockfile and verify resolution**

Run:

```bash
just build
```

Expected: PASS. The root `Cargo.lock` now contains `board-projection-detector` resolved once for
the whole workspace. If the build fails on stale bindings, apply CLAUDE.md Known Issue 7
(`rm -f build/.colcon/bindgen.lock`) and rebuild — do not delete `build/lctk_interfaces` unless a
`.msg`/`.srv` changed (it did not).

- [ ] **Step 6: Run the crate's parity + unit tests under colcon**

Run:

```bash
just test
```

Expected: the `board-projection-detector` tests pass (they require the local 51 MB fixtures; if
absent, regenerate per Step 2's note). This confirms the crate still tests correctly as a member.

- [ ] **Step 7: Commit**

```bash
git add rust/board-projection-detector/Cargo.toml Cargo.toml Cargo.lock ros/lidar_board_detector/Cargo.toml
git commit -m "build(board-proj): fold detector crate into root workspace"
```

---

### Task 2: `bbox_free` config types + parsing

Parse `detection_mode` and the nested `bbox_free` block from `board_detector.json5` into typed
config. This is pure serde — no rclrs — so it lives in the new `bbox_free` submodule and is unit
tested against the shipped config. The same json5 file is also parsed into the hollow-board
`Config` (which has no `deny_unknown_fields`, so the new keys are ignored there — verified).

**Files:**
- Create: `ros/lidar_board_detector/src/bbox_free.rs`
- Modify: `ros/lidar_board_detector/src/main.rs` (add `mod bbox_free;`)
- Modify: `ros/lctk_launch/config/board/board_detector.json5`
- Test: `ros/lidar_board_detector/src/bbox_free.rs` (`#[cfg(test)]`)

**Interfaces:**
- Consumes: `board_projection_detector::config::{BoardConfig, ForegroundMethod}` (Task 1).
- Produces:
  - `enum DetectionMode { Bbox, BboxFree }` with `DetectionMode::parse(&str) -> anyhow::Result<Self>`
    (`"bbox"` → `Bbox`, `"bbox_free"` → `BboxFree`, else error).
  - `struct DetectionConfig { pub detection_mode: String, pub bbox_free: Option<BboxFreeRaw> }`
    (`#[derive(Deserialize)]`, both `#[serde(default)]`; `detection_mode` defaults to `"bbox"`).
  - `struct BboxFreeRaw { pub foreground_method: String, pub voxel: f64, pub board: BoardConfig,
    pub background: BackgroundParams }` (`#[derive(Deserialize, Clone)]`).
  - `struct BackgroundParams { pub dilation_radius: i64, pub warmup_frames: usize }`
    (`#[derive(Deserialize, Clone)]`).
  - `impl BboxFreeRaw { pub fn method(&self) -> anyhow::Result<ForegroundMethod> }` — parses
    `foreground_method` via `ForegroundMethod::from_str`.
  - `fn parse_detection_config(json5_text: &str) -> anyhow::Result<DetectionConfig>`.

- [ ] **Step 1: Add the nested block to `board_detector.json5`**

Append to `ros/lctk_launch/config/board/board_detector.json5` (before the closing `}`, after the
`hole_center_shift` line — add a comma to that line):

```json5
    "hole_center_shift": "200mm", // mm

    // ========================================
    // Crop-box-free detection (board-projection-detector)
    // ========================================
    // "bbox" (default, existing bounding-box crop) | "bbox_free"
    "detection_mode": "bbox",
    // Read only when detection_mode == "bbox_free".
    "bbox_free": {
        // "background_subtraction" (Method E, ~34-69ms, shipping) |
        // "plane_strip" (Method B, ~157-202ms, over budget)
        "foreground_method": "background_subtraction",
        // Voxel edge (m) for the detector's internal downsample (the detect() `voxel` arg).
        "voxel": 0.05,
        // Deserialized into board-projection-detector's BoardConfig. These MUST be the
        // production operating point spelled out explicitly: BoardConfig's serde defaults
        // are the frozen library defaults (flatness 0.035, stance_floor 0.0, isolation false),
        // NOT these.
        "board": {
            "side_m": 1.0,
            "up_axis": [0.0, 0.0, 1.0],
            "cluster_min_points": 30,
            "side_tol": 0.20,
            "cell_m": 0.02,
            "vertical_gap_deg": 3.0,
            "flatness_rms_max": 0.045,
            "stance_floor": 0.9,
            "square_icp_residual_max": 0.45,
            "isolation": true,
            "isolation_max_density": 0.3
        },
        // Method-E background model + warmup. min_sources is fixed at 1 (single live session).
        "background": {
            "dilation_radius": 1,
            "warmup_frames": 20
        }
    }
}
```

- [ ] **Step 2: Write the failing test**

Create `ros/lidar_board_detector/src/bbox_free.rs` with only the test module first:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use board_projection_detector::config::ForegroundMethod;

    const SHIPPED: &str = include_str!(
        "../../lctk_launch/config/board/board_detector.json5"
    );

    #[test]
    fn shipped_config_defaults_to_bbox() {
        let cfg = parse_detection_config(SHIPPED).unwrap();
        assert_eq!(cfg.detection_mode, "bbox");
        assert_eq!(DetectionMode::parse(&cfg.detection_mode).unwrap(), DetectionMode::Bbox);
    }

    #[test]
    fn shipped_bbox_free_is_production_operating_point() {
        let cfg = parse_detection_config(SHIPPED).unwrap();
        let bf = cfg.bbox_free.expect("bbox_free block present");
        assert_eq!(bf.method().unwrap(), ForegroundMethod::BackgroundSubtraction);
        assert_eq!(bf.voxel, 0.05);
        // Production operating point — NOT the BoardConfig serde defaults.
        assert_eq!(bf.board.flatness_rms_max, 0.045);
        assert_eq!(bf.board.stance_floor, 0.9);
        assert!(bf.board.isolation);
        assert_eq!(bf.board.cluster_min_points, 30);
        assert_eq!(bf.background.warmup_frames, 20);
        assert_eq!(bf.background.dilation_radius, 1);
    }

    #[test]
    fn detection_mode_parse_rejects_unknown() {
        assert!(DetectionMode::parse("nope").is_err());
        assert_eq!(DetectionMode::parse("bbox_free").unwrap(), DetectionMode::BboxFree);
    }

    #[test]
    fn method_rejects_unknown() {
        let raw = BboxFreeRaw {
            foreground_method: "bogus".into(),
            voxel: 0.05,
            board: board_projection_detector::config::production_config(1.0, [0.0, 0.0, 1.0], 30),
            background: BackgroundParams { dilation_radius: 1, warmup_frames: 20 },
        };
        assert!(raw.method().is_err());
    }
}
```

- [ ] **Step 3: Add `mod bbox_free;` to main.rs**

At the top of `ros/lidar_board_detector/src/main.rs`, next to `mod bbox;`, add:

```rust
mod bbox_free;
```

- [ ] **Step 4: Run the test to verify it fails**

Run: `just test` (or, faster, build first — it will fail to compile because the types don't exist
yet).
Expected: compile error — `parse_detection_config` / `DetectionMode` / `BboxFreeRaw` not found.

- [ ] **Step 5: Implement the config types**

Prepend to `ros/lidar_board_detector/src/bbox_free.rs` (above the test module):

```rust
//! Crop-box-free detection config + Method-E warmup state machine.
//!
//! Pure logic (no rclrs) so it is unit-testable. `main.rs` parses the node's
//! board config into these types and threads them through the processing
//! thread; the ROS wiring stays thin.

use anyhow::{bail, Result};
use board_projection_detector::config::{BoardConfig, ForegroundMethod};
use serde::Deserialize;
use std::str::FromStr;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionMode {
    Bbox,
    BboxFree,
}

impl DetectionMode {
    pub fn parse(s: &str) -> Result<Self> {
        match s {
            "bbox" => Ok(Self::Bbox),
            "bbox_free" => Ok(Self::BboxFree),
            other => bail!("unknown detection_mode: {other} (expected \"bbox\" or \"bbox_free\")"),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct DetectionConfig {
    #[serde(default = "default_mode")]
    pub detection_mode: String,
    #[serde(default)]
    pub bbox_free: Option<BboxFreeRaw>,
}

fn default_mode() -> String {
    "bbox".to_string()
}

#[derive(Debug, Clone, Deserialize)]
pub struct BboxFreeRaw {
    pub foreground_method: String,
    pub voxel: f64,
    pub board: BoardConfig,
    pub background: BackgroundParams,
}

impl BboxFreeRaw {
    pub fn method(&self) -> Result<ForegroundMethod> {
        ForegroundMethod::from_str(&self.foreground_method)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct BackgroundParams {
    pub dilation_radius: i64,
    pub warmup_frames: usize,
}

pub fn parse_detection_config(json5_text: &str) -> Result<DetectionConfig> {
    Ok(json5::from_str(json5_text)?)
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `just test`
Expected: the four `bbox_free` tests PASS. (The `include_str!` path is relative to `bbox_free.rs`:
`../../lctk_launch/...` resolves to `ros/lctk_launch/config/board/board_detector.json5`. If the
build environment relocates config, adjust to the actual shared path — confirm the relative path
resolves at compile time; if not, embed a copy of the operating-point block as a test const.)

- [ ] **Step 7: Commit**

```bash
git add ros/lidar_board_detector/src/bbox_free.rs ros/lidar_board_detector/src/main.rs ros/lctk_launch/config/board/board_detector.json5
git commit -m "feat(board-det): bbox_free config types + nested json5 block"
```

---

### Task 3: Method-E warmup state machine (`BackgroundState`)

The pure, ROS-free heart of the warmup lifecycle: accumulate `warmup_frames` board-free clouds into
a `BackgroundModel`, `finalize`, then serve it for detection; `reset` re-enters warmup. Unit-tested
without a ROS context.

**Files:**
- Modify: `ros/lidar_board_detector/src/bbox_free.rs`
- Test: `ros/lidar_board_detector/src/bbox_free.rs` (`#[cfg(test)]`)

**Interfaces:**
- Consumes: `board_projection_detector::background::BackgroundModel` (`::new(voxel, dilation_radius,
  min_sources)`, `.observe(&[Point3<f64>], &str)`, `.finalize()`), `BackgroundParams` (Task 2).
- Produces:
  - `enum WarmupOutcome { Warming { seen: usize, needed: usize }, Ready }` — returned by
    `BackgroundState::observe_frame`, tells the caller whether to publish-empty or detect.
  - `struct BackgroundState` holding an `Option<BackgroundModel>` in one of two phases.
  - `impl BackgroundState`:
    - `fn new(voxel: f64, params: &BackgroundParams) -> Self` — starts in Warming with a fresh model.
    - `fn observe_frame(&mut self, points: &[Point3<f64>]) -> WarmupOutcome` — while warming,
      `observe` + count; at `warmup_frames`, `finalize` and transition to Ready; when already Ready,
      returns `Ready` without observing.
    - `fn model(&self) -> Option<&BackgroundModel>` — `Some` only when Ready.
    - `fn reset(&mut self)` — drop to a fresh Warming model with `seen = 0`.

- [ ] **Step 1: Write the failing test**

Add to the `#[cfg(test)] mod tests` in `bbox_free.rs`:

```rust
    use nalgebra::Point3;

    fn cloud(n: usize, offset: f64) -> Vec<Point3<f64>> {
        (0..n).map(|i| Point3::new(offset + i as f64 * 0.001, 0.0, 0.0)).collect()
    }

    #[test]
    fn warmup_observes_then_becomes_ready() {
        let params = BackgroundParams { dilation_radius: 1, warmup_frames: 3 };
        let mut state = BackgroundState::new(0.05, &params);
        assert!(state.model().is_none());

        // Frames 1 and 2: still warming, no model yet.
        for i in 1..=2 {
            match state.observe_frame(&cloud(50, 999.0)) {
                WarmupOutcome::Warming { seen, needed } => {
                    assert_eq!(seen, i);
                    assert_eq!(needed, 3);
                }
                WarmupOutcome::Ready => panic!("ready too early"),
            }
            assert!(state.model().is_none());
        }

        // Frame 3: reaches the count, finalizes, becomes Ready.
        assert!(matches!(state.observe_frame(&cloud(50, 999.0)), WarmupOutcome::Ready));
        assert!(state.model().is_some());

        // Subsequent frames stay Ready and do NOT observe.
        assert!(matches!(state.observe_frame(&cloud(50, 0.0)), WarmupOutcome::Ready));
    }

    #[test]
    fn reset_reenters_warming() {
        let params = BackgroundParams { dilation_radius: 1, warmup_frames: 1 };
        let mut state = BackgroundState::new(0.05, &params);
        assert!(matches!(state.observe_frame(&cloud(50, 999.0)), WarmupOutcome::Ready));
        assert!(state.model().is_some());

        state.reset();
        assert!(state.model().is_none());
        assert!(matches!(state.observe_frame(&cloud(50, 999.0)), WarmupOutcome::Ready));
    }
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `just test`
Expected: compile error — `BackgroundState` / `WarmupOutcome` not found.

- [ ] **Step 3: Implement the state machine**

Add to `bbox_free.rs` (above the test module), and add the imports
`use board_projection_detector::background::BackgroundModel;` and `use nalgebra::Point3;` to the
file header:

```rust
/// Fixed single live session: one background source.
const MIN_SOURCES: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarmupOutcome {
    Warming { seen: usize, needed: usize },
    Ready,
}

enum Phase {
    Warming { model: BackgroundModel, seen: usize },
    Ready { model: BackgroundModel },
}

pub struct BackgroundState {
    phase: Phase,
    voxel: f64,
    dilation_radius: i64,
    warmup_frames: usize,
}

impl BackgroundState {
    pub fn new(voxel: f64, params: &BackgroundParams) -> Self {
        Self {
            phase: Phase::Warming {
                model: BackgroundModel::new(voxel, params.dilation_radius, MIN_SOURCES),
                seen: 0,
            },
            voxel,
            dilation_radius: params.dilation_radius,
            warmup_frames: params.warmup_frames,
        }
    }

    pub fn observe_frame(&mut self, points: &[Point3<f64>]) -> WarmupOutcome {
        match &mut self.phase {
            Phase::Ready { .. } => WarmupOutcome::Ready,
            Phase::Warming { model, seen } => {
                model.observe(points, "live");
                *seen += 1;
                if *seen >= self.warmup_frames {
                    // Move the model out of Warming into Ready, finalized.
                    let Phase::Warming { mut model, .. } =
                        std::mem::replace(&mut self.phase, Phase::Ready {
                            model: BackgroundModel::new(self.voxel, self.dilation_radius, MIN_SOURCES),
                        })
                    else {
                        unreachable!("just matched Warming");
                    };
                    model.finalize();
                    self.phase = Phase::Ready { model };
                    WarmupOutcome::Ready
                } else {
                    WarmupOutcome::Warming { seen: *seen, needed: self.warmup_frames }
                }
            }
        }
    }

    pub fn model(&self) -> Option<&BackgroundModel> {
        match &self.phase {
            Phase::Ready { model } => Some(model),
            Phase::Warming { .. } => None,
        }
    }

    pub fn reset(&mut self) {
        self.phase = Phase::Warming {
            model: BackgroundModel::new(self.voxel, self.dilation_radius, MIN_SOURCES),
            seen: 0,
        };
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `just test`
Expected: `warmup_observes_then_becomes_ready` and `reset_reenters_warming` PASS, plus the Task-2
tests still green.

- [ ] **Step 5: Commit**

```bash
git add ros/lidar_board_detector/src/bbox_free.rs
git commit -m "feat(board-det): Method-E warmup state machine (BackgroundState)"
```

---

### Task 4: Splice `bbox_free` into `process_pointcloud` + thread state

Wire the parsed config and background state through the node's processing thread and add the mode
branch that replaces the Stage-1 output. This is ROS-integration code; it is verified by build +
the bbox-path regression test (the pure logic it calls is already unit-tested in Tasks 2–3).

**Files:**
- Modify: `ros/lidar_board_detector/src/main.rs`

**Interfaces:**
- Consumes: `bbox_free::{DetectionMode, DetectionConfig, BboxFreeRaw, BackgroundState,
  WarmupOutcome, parse_detection_config}` (Tasks 2–3); `board_projection_detector::detector::detect`;
  `arc_swap::ArcSwap`.
- Produces: a `process_pointcloud` that, when `detection_mode == BboxFree`, computes `active_points`
  via `detect(...).selected_points` instead of `filter_points_by_bbox`, then runs the unchanged
  Stage-2/Stage-3 pipeline. Bbox path unchanged.

- [ ] **Step 1: Parse the detection config in `new()`**

In `CalibrationBoardLocatorNode::new`, right after `board_detector_config` is loaded
(~`main.rs:368`), add a second parse of the same file into the detection config, and build the
optional resolved bbox-free config + shared background state:

```rust
        // Crop-box-free detection config (same file, separate typed view).
        let detection_text = fs::read_to_string(PathBuf::from(&*board_detector_file_param))?;
        let detection_cfg = bbox_free::parse_detection_config(&detection_text)?;
        let detection_mode = bbox_free::DetectionMode::parse(&detection_cfg.detection_mode)?;
        let bbox_free_cfg: Option<Arc<bbox_free::BboxFreeRaw>> = match detection_mode {
            bbox_free::DetectionMode::BboxFree => {
                let bf = detection_cfg
                    .bbox_free
                    .ok_or_else(|| anyhow!("detection_mode=bbox_free but no bbox_free block in config"))?;
                bf.method()?; // validate foreground_method early
                Some(Arc::new(bf))
            }
            bbox_free::DetectionMode::Bbox => None,
        };
        log_info!(LOGGER_NAME, "detection_mode = {}", detection_cfg.detection_mode);

        // Shared Method-E background state. `BackgroundState` is observed per frame by the single
        // processing thread; `reset_background` (Task 6) mutates it from a service/param callback,
        // so an `Arc<Mutex<Option<..>>>` is sufficient (the design's `ArcSwap<BackgroundState>` is
        // simplified to this — there is only one observer thread). `None` when the bbox_free path
        // is off or uses plane_strip (no background).
        let background_state: Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>> =
            Arc::new(std::sync::Mutex::new(match bbox_free_cfg.as_ref() {
                Some(bf) if bf.method()? == board_projection_detector::config::ForegroundMethod::BackgroundSubtraction => {
                    Some(bbox_free::BackgroundState::new(bf.voxel, &bf.background))
                }
                _ => None,
            }));
```

- [ ] **Step 2: Capture the new state into the processing thread**

Before `std::thread::spawn(move || { ... })` (~`main.rs:569`), clone the handles the thread needs:

```rust
        let bbox_free_for_thread = bbox_free_cfg.clone();
        let background_for_thread = Arc::clone(&background_state);
```

Pass them into `pointcloud_callback` inside the thread (extend that call's argument list,
~`main.rs:598`):

```rust
                        Self::pointcloud_callback(
                            msg_clone,
                            &detector,
                            &detection_publisher_shared,
                            &bbox_params_for_callback,
                            &board_debug_shared,
                            &icp_debug_shared,
                            &bbox_free_for_thread,
                            &background_for_thread,
                        );
```

Also store `background_state` (and, if Task 6 uses it, a clone) on `Self` so it outlives `new`; add
a field `_background_state: Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>` to
`CalibrationBoardLocatorNode` and set it in the returned struct.

- [ ] **Step 3: Extend `pointcloud_callback` and `process_pointcloud` signatures**

Add the two parameters to `pointcloud_callback` (~`main.rs:697`) and forward them to
`process_pointcloud` (~`main.rs:730`). Add to `process_pointcloud` (~`main.rs:758`):

```rust
    fn process_pointcloud(
        msg: &PointCloud2,
        detector: &Arc<BoardDetector>,
        bbox_params: &BBoxParameters,
        board_debug_publishers: &Option<BoardDebugPublishers>,
        icp_debug_publishers: &Option<IcpDebugPublishers>,
        bbox_free_cfg: &Option<Arc<bbox_free::BboxFreeRaw>>,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
    ) -> Result<Detection3DArray> {
```

- [ ] **Step 4: Add the mode branch replacing Stage 1**

In `process_pointcloud`, the current Stage-1 block is (~`main.rs:797-810`):

```rust
        // Stage 1: Filter points by bounding box (reads current parameter values)
        let active_points =
            Self::filter_points_by_bbox(&points, bbox_params, &msg.header, board_debug_publishers)?;

        if active_points.is_empty() {
            log_debug!(LOGGER_NAME, "No points within bounding box - continuing with empty detection");
            return Ok(Detection3DArray { header: msg.header.clone(), detections: Vec::new() });
        }
```

Replace it with a mode dispatch (the empty-cloud early return is preserved in both arms):

```rust
        // Stage 1: select the board cluster — bbox crop (default) or crop-box-free detector.
        let active_points = match bbox_free_cfg {
            None => {
                // Existing bounding-box path, unchanged.
                Self::filter_points_by_bbox(&points, bbox_params, &msg.header, board_debug_publishers)?
            }
            Some(bf) => {
                match Self::select_board_cluster(&points, bf, background_state, &msg.header)? {
                    Some(pts) => pts,
                    None => {
                        return Ok(Detection3DArray {
                            header: msg.header.clone(),
                            detections: Vec::new(),
                        });
                    }
                }
            }
        };

        if active_points.is_empty() {
            log_debug!(LOGGER_NAME, "Stage 1 produced no points - continuing with empty detection");
            return Ok(Detection3DArray { header: msg.header.clone(), detections: Vec::new() });
        }
```

- [ ] **Step 5: Implement `select_board_cluster`**

Add this associated function to the `impl CalibrationBoardLocatorNode` block (near
`filter_points_by_bbox`). It owns the warmup lifecycle and the `detect()` call; reject diagnostics
are added in Task 5 (leave the marked TODO comment for now — Task 5 fills it):

```rust
    /// Crop-box-free Stage 1: returns the detector's selected board-cluster points,
    /// or `None` to publish an empty detection (still warming, or nothing selected).
    fn select_board_cluster(
        points: &[na::Point3<f64>],
        bf: &bbox_free::BboxFreeRaw,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
        _header: &Header,
    ) -> Result<Option<Vec<na::Point3<f64>>>> {
        use board_projection_detector::{config::ForegroundMethod, detector::detect};

        let method = bf.method()?;

        // Method E: run the warmup lifecycle to obtain a finalized background.
        let outcome = if method == ForegroundMethod::BackgroundSubtraction {
            let mut guard = background_state.lock().unwrap();
            let state = guard.as_mut().ok_or_else(|| {
                anyhow!("bbox_free background_subtraction selected but no BackgroundState initialized")
            })?;
            match state.observe_frame(points) {
                bbox_free::WarmupOutcome::Warming { seen, needed } => {
                    log_info!(LOGGER_NAME, "background warmup {seen}/{needed}");
                    return Ok(None);
                }
                bbox_free::WarmupOutcome::Ready => {
                    let model = state.model().expect("Ready implies a finalized model");
                    detect(points, &bf.board, method, bf.voxel, Some(model))
                }
            }
        } else {
            // Method B (plane_strip): no background.
            detect(points, &bf.board, method, bf.voxel, None)
        };

        match outcome.selected_points {
            Some(pts) if !pts.is_empty() => Ok(Some(pts)),
            _ => {
                // TODO(Task 5): log outcome.reject diagnostics here.
                Ok(None)
            }
        }
    }
```

- [ ] **Step 6: Build**

Run: `just build`
Expected: PASS. Fix any signature-threading mismatches the compiler flags (the two new params must
reach `process_pointcloud` through `pointcloud_callback`).

- [ ] **Step 7: Bbox-path regression test**

Add to `main.rs`'s `#[cfg(test)] mod tests` a test asserting the default parse keeps the bbox path,
reusing the shipped config:

```rust
    #[test]
    fn shipped_config_is_bbox_mode_by_default() {
        let text = include_str!("../../lctk_launch/config/board/board_detector.json5");
        let cfg = crate::bbox_free::parse_detection_config(text).unwrap();
        assert_eq!(
            crate::bbox_free::DetectionMode::parse(&cfg.detection_mode).unwrap(),
            crate::bbox_free::DetectionMode::Bbox
        );
    }
```

Run: `just test`
Expected: PASS. The bbox path is unchanged (guarded behind `bbox_free_cfg == None`), so all
pre-existing node tests stay green.

- [ ] **Step 8: Commit**

```bash
git add ros/lidar_board_detector/src/main.rs
git commit -m "feat(board-det): bbox_free Stage-1 splice + Method-E warmup wiring"
```

---

### Task 5: Reject diagnostics + None-background error

Turn silent empty publishes into a named killer-gate log, and make a `None` background under
`BackgroundSubtraction` an explicit error rather than a silent `NoClusters`.

**Files:**
- Modify: `ros/lidar_board_detector/src/main.rs`

**Interfaces:**
- Consumes: `board_projection_detector::detector::{DetectOutcome, RejectReason}` (has
  `outcome.reject: Option<RejectReason>`).
- Produces: `select_board_cluster` logs the reject reason on no-selection; the `None`-background
  case is unreachable-by-construction and logged as an error if ever hit.

- [ ] **Step 1: Add a reject-reason description helper**

Add to `bbox_free.rs` (it is pure, and keeps `main.rs` thin):

```rust
use board_projection_detector::detector::RejectReason;

/// Human-readable one-liner for a detector reject reason.
pub fn describe_reject(reason: &RejectReason) -> &'static str {
    match reason {
        RejectReason::NoClusters => "no candidate clusters survived foreground extraction",
        RejectReason::Flatness => "best candidate exceeded flatness_rms_max (not planar enough)",
        RejectReason::Extent => "best candidate failed the board-size extent gate",
        RejectReason::SizeGate => "best candidate failed the coarse square size gate",
        RejectReason::SquareResidual => "square fit residual exceeded square_icp_residual_max",
        RejectReason::Stance => "best candidate failed the 3D diamond-stance gate",
        RejectReason::Isolation => "best candidate failed the isolation-density gate (embedded clutter)",
    }
}
```

(Confirm the `RejectReason` variant names against `detector.rs` — the enum is
`NoClusters | Flatness | Extent | SizeGate | SquareResidual | Stance | Isolation`. If a variant
carries data, adjust the match arms.)

- [ ] **Step 2: Wire the diagnostics into `select_board_cluster`**

Replace the `TODO(Task 5)` block from Task 4 Step 5 with:

```rust
        match outcome.selected_points {
            Some(pts) if !pts.is_empty() => Ok(Some(pts)),
            _ => {
                match &outcome.reject {
                    Some(reason) => log_info!(
                        LOGGER_NAME,
                        "bbox_free: no board selected — {}",
                        bbox_free::describe_reject(reason)
                    ),
                    None => log_info!(LOGGER_NAME, "bbox_free: no board selected (no reject reason)"),
                }
                Ok(None)
            }
        }
```

- [ ] **Step 3: Make the None-background path an explicit error**

The `Method E` arm in `select_board_cluster` already returns an `anyhow` error when
`background_state` holds `None` while `BackgroundSubtraction` is selected (Task 4 Step 5, the
`.ok_or_else(...)`). Confirm that error is surfaced rather than swallowed: `process_pointcloud`
returns `Result`, and `pointcloud_callback` logs the error. Verify the callback's error handling
logs at `error` level; if it currently drops the error, add:

```rust
        if let Err(e) = result {
            log_error!(LOGGER_NAME, "bbox_free Stage 1 failed: {e}");
        }
```

at the `process_pointcloud` call site in `pointcloud_callback` (match the existing error-handling
style there).

- [ ] **Step 4: Build and test**

Run: `just build && just test`
Expected: PASS. No behavioral change to the bbox path or the happy bbox_free path; only the
no-selection and error logs are added.

- [ ] **Step 5: Commit**

```bash
git add ros/lidar_board_detector/src/main.rs ros/lidar_board_detector/src/bbox_free.rs
git commit -m "feat(board-det): bbox_free reject diagnostics + explicit no-background error"
```

---

### Task 6: `reset_background` control

Let an operator re-capture the empty scene (re-enter warmup) at runtime — e.g. after moving the
rig. First confirm the rclrs 0.7 service API; if service wiring proves heavy, fall back to a watched
bool parameter. Either way the mechanism just calls `BackgroundState::reset`.

**Files:**
- Modify: `ros/lidar_board_detector/src/main.rs`

**Interfaces:**
- Consumes: the shared `Arc<Mutex<Option<bbox_free::BackgroundState>>>` (Task 4);
  `BackgroundState::reset` (Task 3).
- Produces: a runtime trigger that resets the background to a fresh Warming state.

- [ ] **Step 1: Confirm the rclrs 0.7 service (or parameter) mechanism**

Check the rclrs 0.7 API available in this workspace:

```bash
grep -rn "create_service\|std_srvs" ros/ --include=*.rs | head
```

No Rust node in this repo creates a service yet. Decide:
- **If `node.create_service::<std_srvs::srv::Empty, _>(...)` is available** and `std_srvs` resolves
  (add `std_srvs = "*"` to the node `Cargo.toml` and `<depend>std_srvs</depend>` to its
  `package.xml`) → use a service named `~/reset_background`.
- **Else (fallback)** → declare a bool parameter `reset_background_request` (default `false`); the
  processing thread checks it each loop, and on `true` resets the state and sets the parameter back
  to `false`.

Record the choice in a comment at the implementation site.

- [ ] **Step 2 (service path): create the service in `new()`**

Add `std_srvs = "*"` to `ros/lidar_board_detector/Cargo.toml` and `<depend>std_srvs</depend>` to
`ros/lidar_board_detector/package.xml`. In `new()`, after `background_state` is built:

```rust
        let reset_bg = Arc::clone(&background_state);
        let _reset_srv = node.create_service::<std_srvs::srv::Empty, _>(
            "~/reset_background",
            move |_req_header, _req: std_srvs::srv::Empty_Request| {
                if let Some(state) = reset_bg.lock().unwrap().as_mut() {
                    state.reset();
                    log_info!(LOGGER_NAME, "background reset — re-entering warmup");
                }
                std_srvs::srv::Empty_Response::default()
            },
        )?;
```

Store `_reset_srv` on `Self` so it is not dropped. (Confirm the exact rclrs 0.7 service callback
signature — argument order and header type — against the installed `rclrs` docs at execution; the
above is the 0.7 shape.)

- [ ] **Step 2 (fallback path): watched parameter**

If services are not viable, declare in `new()`:

```rust
        let reset_param = node
            .declare_parameter("reset_background_request")
            .default(false)
            .mandatory()?;
```

Clone `reset_param` and `Arc::clone(&background_state)` into the processing thread; at the top of
the loop body:

```rust
                if reset_param_for_thread.get() {
                    if let Some(state) = background_for_thread.lock().unwrap().as_mut() {
                        state.reset();
                        log_info!(LOGGER_NAME, "background reset — re-entering warmup");
                    }
                    let _ = reset_param_for_thread.set(false);
                }
```

(Confirm `MandatoryParameter::set` exists in rclrs 0.7; if not, use an `AtomicBool` the operator
toggles via a separate trigger, or accept that reset is set-once until restart and document it.)

- [ ] **Step 3: Build and test**

Run: `just build && just test`
Expected: PASS.

- [ ] **Step 4: Manual smoke (documented, not automated)**

With sample data + a bbox_free config, launch the node, let it warm up, then trigger the reset
(service call `ros2 service call /lidar_board_detector/reset_background std_srvs/srv/Empty` or
`ros2 param set /lidar_board_detector reset_background_request true`) and confirm the log shows
"background reset — re-entering warmup" followed by fresh "background warmup N/M" lines.

- [ ] **Step 5: Commit**

```bash
git add ros/lidar_board_detector/src/main.rs ros/lidar_board_detector/Cargo.toml ros/lidar_board_detector/package.xml
git commit -m "feat(board-det): reset_background runtime control for Method-E warmup"
```

---

## Self-Review

**Spec coverage:**
- Component 1 (workspace fold) → Task 1. ✓
- Component 2 (nested `bbox_free` config, production-values warning) → Task 2 (json5 block + parse
  types + operating-point assertion test). ✓
- Component 3 (`select_board_cluster` splice, Stage-1 replacement) → Task 4. ✓
- Component 4 (Method-E warmup: `ArcSwap`/state machine, `reset_background`) → Task 3 (pure state
  machine) + Task 4 (wiring) + Task 6 (reset). ✓ (Design's `ArcSwap<BackgroundState>` simplified to
  `Arc<Mutex<Option<BackgroundState>>>` since only the single processing thread observes; reset uses
  the same mutex. Noted in Task 4 Step 1.)
- Component 5 (reject diagnostics + None-bg error) → Task 5. ✓
- Deferred Task 6 (Method B perf) → out of scope, noted in spec. ✓
- Testing section (crate parity under colcon, config parse, warmup state machine, bbox regression)
  → Task 1 Step 6, Task 2, Task 3, Task 4 Step 7. ✓

**Placeholder scan:** No "TBD"/"add error handling"/"similar to Task N". Task 4 carries an explicit
`TODO(Task 5)` marker that Task 5 Step 2 removes — this is an intentional cross-task handoff, not a
placeholder, and the removal is a concrete step. Two spots flagged "confirm at execution" carry the
concrete expected shape and a fallback: (a) the `include_str!` relative path (Task 2 Step 6), (b)
the rclrs 0.7 service signature (Task 6) — both are environment facts that cannot be guessed from
source and have written fallbacks.

**Type consistency:** `DetectionMode`, `DetectionConfig`, `BboxFreeRaw`, `BackgroundParams`,
`BackgroundState`, `WarmupOutcome`, `parse_detection_config`, `describe_reject` are named
identically across Tasks 2–6. `detect(points, board, method, voxel, background) -> DetectOutcome`
matches the crate. `BackgroundModel::new(voxel, dilation_radius, min_sources)`,
`.observe(&[Point3<f64>], &str)`, `.finalize()`, and `DetectOutcome.{selected_points, reject}` match
sub-project 1's public API. `background_state` holder type is `Arc<Mutex<Option<BackgroundState>>>`
consistently in Tasks 4–6. Point type is `na::Point3<f64>` (= crate `nalgebra::Point3<f64>`)
throughout.
