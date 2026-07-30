# board-projection-detector Rust Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the validated Python `boarddet` crop-box-free detection pipeline into a new pure-Rust library crate `rust/board-projection-detector`, matching the Python reference on the sample datasets, so a later ROS task can drop it into `lidar_board_detector` behind a `detection_mode` switch.

**Architecture:** The crate reproduces `boarddet.detect()` as a **cluster selector/discriminator**, not a pose engine (design decision "Option C"). It extracts foreground (plane-strip or background-subtraction), clusters it, gates each cluster through the 2D discriminator (minAreaRect seed → fixed-square fit → pose → stance/isolation gates), and returns the **winning cluster's 3D points + plane** for the existing RANSAC+ICP to pose. Correctness is locked by a golden-vector parity harness: a Python exporter dumps `detect()`'s per-stage outputs on curated frames; Rust tests assert stage-by-stage parity.

**Tech Stack:** Rust (MSRV 1.85), nalgebra (`Point3<f64>`, `Vector3<f64>`, SVD), serde + json5 (config), `plane-estimator` + `arrsac` + `sample-consensus` for RANSAC. No OpenCV, no open3d, no imageproc, and **NOT `hollow-board-detector`** (it drags native `sfcgal-sys`). DBSCAN, minAreaRect, and voxel-downsample are all local. Python side: existing `boarddet` uv project (numpy/open3d/opencv) used only to generate golden fixtures.

**Build/workspace note (discovered in execution):** the root LCTK cargo workspace pulls ROS message crates (`aruco-detector` → `sensor_msgs = "*"`, yanked on crates.io) that resolve only under colcon, so ANY root-workspace cargo command fails. This crate is therefore a **standalone cargo workspace**: it carries its own `[workspace]` table and is listed in the root `Cargo.toml`'s `exclude`. Build/test it with `cd rust/board-projection-detector && cargo test` — never plain `cargo test -p …` from the repo root, and never colcon.

## Global Constraints

- **Target only the `production_config` path** (`square_icp=True`, `stance_floor=0.9`, `isolation=True`, `flatness_rms_max=0.045`). The `detect()` non-ICP branch (`stance_weight` blend, `min_score`, `best_rejected`-by-score) is OUT OF SCOPE — do not port it.
- **Scorer is reduced.** Under `square_icp=True`, `score_candidate`'s raster / `morphologyEx` / `fillPoly` / `findContours` / fill-ratio do NOT affect the decision — only its `minAreaRect` quad-center feeds `fit_fixed_square` as a seed, with a `coords.mean(axis=0)` fallback. Port `minAreaRect` + the seed selection only. Do NOT port the raster machinery.
- **Method names:** `enum ForegroundMethod { PlaneStrip, BackgroundSubtraction }`; config/string form `"plane_strip"` / `"background_subtraction"` (renamed from experiment's B / E).
- **VLP-32C noise floor is ~0.026–0.031 m.** Every metric gate in meters sits ABOVE it. `flatness_rms_max` stays 0.045; do not lower it or any ICP/coplanar tolerance under the floor (C-04 bug class: a gate below the floor silently accepts nothing).
- **`board_pose`'s `up` defaults to `(0,0,1)`** but the detector always passes `board.up_axis` (per-rig; Falcon is z-forward `(0,1,0)`).
- **Parity tolerances:** foreground point-set ≥95% membership match (nearest-neighbour within `voxel`); selected-cluster centroid < 0.02 m from Python; per-frame detect/no-detect decision EXACT; aggregate recall/precision within ±1 frame of the Python reference per dataset.
- **nalgebra convention:** points are `nalgebra::Point3<f64>`; internal math may use `nalgebra::Matrix`/SVD. The Python reference stores `float32` after downsample — cast deliberately and document where (`downsample` returns f32 in Python; keep f64 in Rust but expect ~1e-6 drift, well inside tolerances).
- **No `hollow-board-detector` dependency.** `voxel_downsample` is implemented locally in `geometry.rs` (Task 2); RANSAC (`remove_big_planes`, Task 5) uses `arrsac::Arrsac` + `plane_estimator::PlaneEstimator` + `sample_consensus::Consensus::model_inliers` directly — the exact pair `hollow-board-detector`'s own `fit_plane_ransac` is built from. Standalone-workspace `Cargo.toml` deps: `nalgebra` 0.32.3 (feat `serde-serialize`), `anyhow` 1, `serde` 1 (derive), `json5` 0.4.1, `log` 0.4.20, `plane-estimator` (path `../plane-estimator`), `sample-consensus` 1.0.2, `arrsac` 0.10.0, `rand` 0.8; dev: `approx` 0.5.1, `serde_json` 1. Explicit versions (a standalone workspace has no root `[workspace.dependencies]` to inherit).
- **All cargo commands run from `rust/board-projection-detector/`** (its own workspace), e.g. `cargo test --test config`. Never `-p …` from repo root.
- **Reject diagnostics minimal:** a small `enum RejectReason` for logging ("no_clusters", "flatness", "extent", "size_gate", "square_residual", "stance", "isolation") — NOT the experiment's margin-based `furthest()` side-channel.
- Work on the current branch `feat/method-e-background-subtraction`. Commit after each task; conventional-commit subject; end every commit body with:
  `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`
- Build/test the crate in isolation with cargo (it is ROS-free): `cd rust/board-projection-detector && cargo test`. Do NOT route this crate through `just build` / colcon — it has no ROS deps and colcon is only needed for the ROS wiring in sub-project 2.

## Python Reference Map (source of truth — read before each port task)

| Rust module | Ports from | Key funcs |
|---|---|---|
| `geometry.rs` | `src/boarddet/geometry.py` | `fit_plane`, `project_to_plane`, `unproject`, `finite_only`, `plane_rms`, `extent_2d` |
| `dbscan.rs` | `src/boarddet/candidates/cluster_after_ground.py:19-47` (`_anisotropic_scaled`) + open3d `cluster_dbscan` | anisotropic scale + Euclidean DBSCAN |
| `background.rs` | `src/boarddet/background.py` | `BackgroundModel.{observe,finalize,foreground_points}` |
| `candidates.rs` | `src/boarddet/candidates/__init__.py` + `cluster_after_ground.py` + `background_diff.py` | `plausible_board_patch`, `_remove_big_planes`, `_merge_coplanar_clusters`, `_cluster_and_gate`, generators |
| `scorer.rs` | `src/boarddet/scorer.py` (REDUCED) | `min_area_rect` + seed center |
| `square_fit.rs` | `src/boarddet/square_fit.py` | `fit_fixed_square`, `_coverage_residual`, `_fit_at_theta` |
| `pose.rs` | `src/boarddet/pose.py` + `detector.py:68-82` (`_stance`) + `isolation.py` | `board_pose`, `stance_3d`, `isolation_density` |
| `detector.rs` | `src/boarddet/detector.py` (square_icp branch only) | `detect`, `DetectOutcome` |
| `config.rs` | `src/boarddet/board_config.py` + `presets.py` | `BoardConfig`, `production_config`, json5 loader |

---

### Task 0: Crate scaffold + config module

**Files:**
- Create: `rust/board-projection-detector/Cargo.toml`
- Create: `rust/board-projection-detector/src/lib.rs`
- Create: `rust/board-projection-detector/src/config.rs`
- Test: `rust/board-projection-detector/tests/config.rs`

**Interfaces:**
- Consumes: nothing (workspace `rust/*` glob auto-includes the crate).
- Produces:
  - `struct BoardConfig { side_m: f64, side_tol: f64, cell_m: f64, vertical_gap_deg: f64, cluster_min_points: usize, up_axis: [f64;3], flatness_rms_max: f64, stance_floor: f64, square_icp_residual_max: f64, isolation: bool, isolation_max_density: f64 }` (only the fields the square_icp production path reads — omit `min_score`, `stance_weight`, `strict_squareness`, `edge_support_min`, `square_icp` which is always true here).
  - `fn production_config(side_m: f64, up_axis: [f64;3], cluster_min_points: usize) -> BoardConfig`
  - `fn load_board_config_json5(path: &Path) -> anyhow::Result<BoardConfig>`
  - `enum ForegroundMethod { PlaneStrip, BackgroundSubtraction }` with `FromStr` (`"plane_strip"`/`"background_subtraction"`).

- [ ] **Step 1: Write `Cargo.toml` (standalone workspace) + add root exclude**

`rust/board-projection-detector/Cargo.toml` — carries its OWN `[workspace]` table (standalone; the root workspace is ROS-poisoned) and explicit dep versions:
```toml
[workspace]

[package]
name = "board-projection-detector"
version = "0.1.0"
edition = "2021"
rust-version = "1.85.0"

[dependencies]
nalgebra = { version = "0.32.3", features = ["serde-serialize"] }
anyhow = "1.0"
serde = { version = "1.0", features = ["derive"] }
json5 = "0.4.1"
log = "0.4.20"
plane-estimator = { path = "../plane-estimator" }
sample-consensus = "1.0.2"
arrsac = "0.10.0"
rand = "0.8"

[dev-dependencies]
approx = "0.5.1"
serde_json = "1.0"
```

Then add the crate to the root `/home/jetson/LCTK/Cargo.toml` `exclude` list (so the `rust/*` glob does NOT pull it into the ROS-poisoned root workspace):
```toml
exclude = [
    "rust/board-projection-detector",
    # ... existing entries ...
]
```

- [ ] **Step 2: Write the failing test**

```rust
// tests/config.rs
use board_projection_detector::config::{production_config, BoardConfig, ForegroundMethod};
use std::str::FromStr;

#[test]
fn production_config_matches_python_preset() {
    let c = production_config(1.0, [0.0, 0.0, 1.0], 30);
    assert_eq!(c.side_m, 1.0);
    assert_eq!(c.up_axis, [0.0, 0.0, 1.0]);
    assert_eq!(c.cluster_min_points, 30);
    assert_eq!(c.stance_floor, 0.9);
    assert!(c.isolation);
    assert_eq!(c.flatness_rms_max, 0.045);
    assert_eq!(c.square_icp_residual_max, 0.45);
    assert_eq!(c.side_tol, 0.20);
    assert_eq!(c.cell_m, 0.02);
    assert_eq!(c.vertical_gap_deg, 3.0);
    assert_eq!(c.isolation_max_density, 0.3);
}

#[test]
fn foreground_method_from_str() {
    assert!(matches!(ForegroundMethod::from_str("plane_strip"), Ok(ForegroundMethod::PlaneStrip)));
    assert!(matches!(ForegroundMethod::from_str("background_subtraction"), Ok(ForegroundMethod::BackgroundSubtraction)));
    assert!(ForegroundMethod::from_str("bogus").is_err());
}
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd rust/board-projection-detector && cargo test --test config`
Expected: FAIL — crate/module does not compile (unresolved `config`).

- [ ] **Step 4: Implement `config.rs` + `lib.rs`**

`src/lib.rs`:
```rust
pub mod config;
```

`src/config.rs` — port defaults verbatim from `board_config.py` (defaults) and `presets.py` (production overrides). All-const values, copy exactly:
```rust
use serde::Deserialize;
use std::{path::Path, str::FromStr};

#[derive(Debug, Clone, Deserialize)]
pub struct BoardConfig {
    #[serde(default = "d_side_m")] pub side_m: f64,
    #[serde(default = "d_side_tol")] pub side_tol: f64,
    #[serde(default = "d_cell_m")] pub cell_m: f64,
    #[serde(default = "d_vertical_gap_deg")] pub vertical_gap_deg: f64,
    #[serde(default = "d_cluster_min_points")] pub cluster_min_points: usize,
    #[serde(default = "d_up_axis")] pub up_axis: [f64; 3],
    #[serde(default = "d_flatness")] pub flatness_rms_max: f64,
    #[serde(default = "d_stance_floor")] pub stance_floor: f64,
    #[serde(default = "d_square_res")] pub square_icp_residual_max: f64,
    #[serde(default)] pub isolation: bool,
    #[serde(default = "d_iso_density")] pub isolation_max_density: f64,
}

fn d_side_m() -> f64 { 1.0 }
fn d_side_tol() -> f64 { 0.20 }
fn d_cell_m() -> f64 { 0.02 }
fn d_vertical_gap_deg() -> f64 { 3.0 }
fn d_cluster_min_points() -> usize { 30 }
fn d_up_axis() -> [f64; 3] { [0.0, 0.0, 1.0] }
fn d_flatness() -> f64 { 0.035 } // BoardConfig default; production overrides to 0.045
fn d_stance_floor() -> f64 { 0.0 }
fn d_square_res() -> f64 { 0.45 }
fn d_iso_density() -> f64 { 0.3 }

pub fn production_config(side_m: f64, up_axis: [f64; 3], cluster_min_points: usize) -> BoardConfig {
    BoardConfig {
        side_m, up_axis, cluster_min_points,
        side_tol: 0.20, cell_m: 0.02, vertical_gap_deg: 3.0,
        flatness_rms_max: 0.045,   // presets.py
        stance_floor: 0.9,          // presets.py
        square_icp_residual_max: 0.45,
        isolation: true,            // presets.py
        isolation_max_density: 0.3,
    }
}

pub fn load_board_config_json5(path: &Path) -> anyhow::Result<BoardConfig> {
    let text = std::fs::read_to_string(path)?;
    Ok(json5::from_str(&text)?)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForegroundMethod { PlaneStrip, BackgroundSubtraction }

impl FromStr for ForegroundMethod {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "plane_strip" => Ok(Self::PlaneStrip),
            "background_subtraction" => Ok(Self::BackgroundSubtraction),
            other => anyhow::bail!("unknown foreground method: {other}"),
        }
    }
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd rust/board-projection-detector && cargo test --test config`
Expected: PASS (2 tests).

- [ ] **Step 6: Commit**

```bash
git add rust/board-projection-detector/Cargo.toml rust/board-projection-detector/src/lib.rs rust/board-projection-detector/src/config.rs rust/board-projection-detector/tests/config.rs Cargo.toml
git commit -m "feat(board-proj): crate scaffold + BoardConfig/production_config/json5 loader"
```

---

### Task 1: Golden-vector parity harness (Python exporter + Rust fixture loader)

This task ships NO ported algorithm — it builds the test scaffolding every later task depends on. It is a plan failure to skip it: without golden vectors, "parity" is unverifiable.

**Files:**
- Create: `experiments/board-detection-2d/tools/export_golden.py`
- Create: `rust/board-projection-detector/tests/fixtures/README.md` (documents the format + regeneration command)
- Create: `rust/board-projection-detector/tests/fixtures/*.input.f32` + `*.golden.json` (generated, committed — curated subset)
- Create: `rust/board-projection-detector/tests/common/mod.rs` (Rust fixture loader)

**Interfaces:**
- Produces (Python → disk): per `(dataset, frame, generator)` a pair
  - `<name>.input.f32`: raw little-endian `f32`, layout `[x0,y0,z0, x1,y1,z1, ...]` — the **raw** cloud (pre-downsample), so Rust exercises `finite_only`+`downsample` too.
  - `<name>.golden.json`: expected per-stage outputs (schema below).
- Produces (Rust): `common::Fixture { name, input: Vec<Point3<f64>>, golden: Golden }` and `common::load_all() -> Vec<Fixture>`; `common::load_f32(path) -> Vec<Point3<f64>>`.

**Golden JSON schema** (every field an expected output a later task asserts against):
```json
{
  "generator": "background_subtraction",
  "dataset": 3,
  "voxel": 0.03,
  "up_axis": [0.0, 0.0, 1.0],
  "cluster_min_points": 30,
  "background_keys_file": "ds3.bgkeys.i64",
  "background_params": {"voxel": 0.06, "dilation_radius": 1, "min_sources": 3},
  "foreground_xyz": [[x,y,z], ...],
  "n_candidates": 2,
  "selected_centroid": [x,y,z],
  "selected_corners_3d": [[x,y,z], x4],
  "detected": true,
  "true_board": true
}
```
- `background_keys_file` (background_subtraction only): a sibling file of raw little-endian `i64` = the finalized LOO background's sorted voxel keys, dumped separately because a dataset's room is tens of thousands of voxels (too big to inline). `plane_strip` fixtures omit it.
- `true_board`: `box.contains(selected_centroid)` using the pcap rig's `bbox.json5` (`boarddet.bbox_ref.load_bbox`) — the recall/precision truth label (Task 9). `false`/absent when `detected` is false.
- **E background is cross-dataset LOO**, not single-frame warmup — that is what the 88.4/100 numbers measured. For a fixture from held-out dataset `D`, the background is built from the OTHER four datasets (`build_background(sources, held_out=str(D), 0.06, 1, min_sources=3)`).

- [ ] **Step 1: Write the Python exporter**

`tools/export_golden.py` (run via `cd experiments/board-detection-2d && uv run python tools/export_golden.py`). Reuse the real APIs — do NOT re-derive them:
- `boarddet.ingest.load_frames(ds) -> list[Frame]`; `Frame.xyz` is the raw `(N,3) float32` cloud (the detector's input).
- `boarddet.benchmark_e_loo.build_background(sources, held_out, voxel, dilation_radius, min_sources)` — builds the cross-dataset LOO background (one source per dataset, finalized).
- `boarddet.candidates.cluster_after_ground.big_plane_residual(dn, board, vertical_gap_deg)` — the `plane_strip` foreground.
- `boarddet.bbox_ref.load_bbox(path)` → `box`; `box.contains(center)` is the true-board label. Path: `ros/lctk_launch/config/board/bbox.json5` (see `benchmark_e_loo.DEFAULT_BBOX_PATH`).
- `boarddet.geometry.{finite_only, downsample}`; `boarddet.detector.detect`.

Board config = the production operating point (same object for both generators):
```python
from boarddet.board_config import BoardConfig
board = BoardConfig(side_m=1.0, up_axis=(0.0,0.0,1.0), cluster_min_points=30,
                    square_icp=True, stance_floor=0.9, isolation=True,
                    flatness_rms_max=0.045)  # == presets.production_config()
```

Concrete exporter:
```python
import json, numpy as np, pathlib
from boarddet.ingest import load_frames
from boarddet.benchmark_e_loo import build_background, DEFAULT_BBOX_PATH
from boarddet.bbox_ref import load_bbox
from boarddet.geometry import finite_only, downsample
from boarddet.candidates.cluster_after_ground import big_plane_residual
from boarddet.detector import detect
from boarddet.board_config import BoardConfig

OUT = pathlib.Path(__file__).resolve().parents[1] / "../rust/board-projection-detector/tests/fixtures"
OUT = OUT.resolve()
OUT.mkdir(parents=True, exist_ok=True)
VOXEL = 0.03
BOX = load_bbox(DEFAULT_BBOX_PATH)
def board(): return BoardConfig(side_m=1.0, up_axis=(0.0,0.0,1.0),
    cluster_min_points=30, square_icp=True, stance_floor=0.9,
    isolation=True, flatness_rms_max=0.045)

def dump(name, raw, generator, background=None, ds=None):
    raw = np.ascontiguousarray(raw[:, :3], dtype=np.float32)
    (OUT / f"{name}.input.f32").write_bytes(raw.tobytes())
    b = board()
    dn = downsample(finite_only(raw.astype(np.float64)), VOXEL)
    if generator == "background_subtraction":
        fg = background.foreground_points(dn)
        gen = "e"
    else:
        fg = big_plane_residual(dn, b, b.vertical_gap_deg)
        gen = "b"
    out = detect(raw.astype(np.float64), b, generator=gen, background=background)
    det = out.detection
    g = {"generator": generator, "dataset": ds, "voxel": VOXEL,
         "up_axis": list(b.up_axis), "cluster_min_points": b.cluster_min_points,
         "foreground_xyz": np.asarray(fg, float).tolist(),
         "n_candidates": out.n_candidates, "detected": det is not None}
    if det is not None:
        g["selected_centroid"] = det.center.astype(float).tolist()
        g["selected_corners_3d"] = det.corners_3d.astype(float).tolist()
        g["true_board"] = bool(BOX.contains(det.center))
    if background is not None:
        keys = np.asarray(background.keys() if hasattr(background, "keys") else background._keys, dtype="<i8")
        kf = f"{name}.bgkeys.i64"; (OUT / kf).write_bytes(keys.tobytes())
        g["background_keys_file"] = kf
        g["background_params"] = {"voxel": background.voxel,
            "dilation_radius": background.dilation_radius,
            "min_sources": background.min_sources}
    (OUT / f"{name}.golden.json").write_text(json.dumps(g, indent=1))

def pick(frames, gen, background=None, ds=None):
    """Curate ~4 frames: first true-board hit, first non-detection, + a spread."""
    b = board()
    outs = [detect(f.xyz.astype(np.float64), b,
                   generator=("e" if gen=="background_subtraction" else "b"),
                   background=background) for f in frames]
    hits = [i for i,o in enumerate(outs) if o.detection is not None]
    miss = [i for i,o in enumerate(outs) if o.detection is None]
    idxs = ([hits[0]] if hits else []) + ([miss[0]] if miss else [])
    idxs += list(range(0, len(frames), max(1, len(frames)//3)))
    seen = []
    for i in idxs:
        if i not in seen: seen.append(i)
        if len(seen) >= 4: break
    for i in seen:
        dump(f"ds{ds}_f{i:04d}_{gen[:2]}", frames[i].xyz, gen, background, ds)

DATASETS = [1,2,3,4,5]
sources = {str(d): load_frames(d) for d in DATASETS}
for d in DATASETS:
    pick(sources[str(d)], "plane_strip", None, d)
    bg = build_background(sources, held_out=str(d), voxel=0.06,
                          dilation_radius=1, min_sources=3)
    pick(sources[str(d)], "background_subtraction", bg, d)
```
Notes: `detect()` uses the experiment's `"e"`/`"b"` generator strings — the rename to `background_subtraction`/`plane_strip` lives only in the Rust crate. If `BackgroundModel` has no public `keys()` accessor, read `_keys` (the fallback above handles both). This exporter runs the WHOLE `boarddet` pipeline, so it also confirms the `uv`/open3d env is healthy on this box.

- [ ] **Step 2: Generate the fixtures**

Run: `cd experiments/board-detection-2d && uv run python tools/export_golden.py`
Expected: `tests/fixtures/` fills with `*.input.f32` + `*.golden.json`. Sanity-check ≥1 `detected:true` and ≥1 `detected:false` fixture exist per generator.

- [ ] **Step 3: Write the Rust fixture loader + a smoke test**

`tests/common/mod.rs`:
```rust
use nalgebra::Point3;
use serde::Deserialize;
use std::{fs, path::{Path, PathBuf}};

#[derive(Debug, Deserialize)]
pub struct BgParams { pub voxel: f64, pub dilation_radius: i64, pub min_sources: usize }

#[derive(Debug, Deserialize)]
pub struct Golden {
    pub generator: String,
    #[serde(default)] pub dataset: Option<u32>,
    pub voxel: f64,
    pub up_axis: [f64; 3],
    pub cluster_min_points: usize,
    #[serde(default)] pub background_keys_file: Option<String>,
    #[serde(default)] pub background_params: Option<BgParams>,
    #[serde(default)] pub foreground_xyz: Vec<[f64; 3]>,
    pub n_candidates: usize,
    #[serde(default)] pub selected_centroid: Option<[f64; 3]>,
    #[serde(default)] pub selected_corners_3d: Option<Vec<[f64; 3]>>,
    pub detected: bool,
    #[serde(default)] pub true_board: bool,
}

// helper: load a sibling `<name>.bgkeys.i64` (raw LE i64) into a sorted Vec<i64>
pub fn load_i64(path: &Path) -> Vec<i64> {
    fs::read(path).unwrap().chunks_exact(8)
        .map(|c| i64::from_le_bytes(c.try_into().unwrap())).collect()
}

pub struct Fixture { pub name: String, pub input: Vec<Point3<f64>>, pub golden: Golden }

pub fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

pub fn load_f32(path: &Path) -> Vec<Point3<f64>> {
    let bytes = fs::read(path).unwrap();
    bytes.chunks_exact(12).map(|c| {
        let f = |i: usize| f32::from_le_bytes(c[i*4..i*4+4].try_into().unwrap()) as f64;
        Point3::new(f(0), f(1), f(2))
    }).collect()
}

pub fn load_all() -> Vec<Fixture> {
    let dir = fixtures_dir();
    let mut out = vec![];
    for e in fs::read_dir(&dir).unwrap() {
        let p = e.unwrap().path();
        if p.extension().and_then(|x| x.to_str()) != Some("json") { continue; }
        let name = p.file_stem().unwrap().to_str().unwrap().trim_end_matches(".golden").to_string();
        let golden: Golden = serde_json::from_str(&fs::read_to_string(&p).unwrap()).unwrap();
        let input = load_f32(&dir.join(format!("{name}.input.f32")));
        out.push(Fixture { name, input, golden });
    }
    out
}
```
Add `serde_json = "1"` to `[dev-dependencies]`.

`tests/harness_smoke.rs`:
```rust
mod common;
#[test]
fn fixtures_load_and_are_nonempty() {
    let fx = common::load_all();
    assert!(!fx.is_empty(), "no fixtures found — run export_golden.py");
    assert!(fx.iter().any(|f| f.golden.detected));
    assert!(fx.iter().any(|f| !f.golden.detected));
    for f in &fx { assert!(!f.input.is_empty(), "empty input: {}", f.name); }
}
```

- [ ] **Step 4: Run the smoke test**

Run: `cd rust/board-projection-detector && cargo test --test harness_smoke`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/board-detection-2d/tools/export_golden.py rust/board-projection-detector/tests/
git commit -m "test(board-proj): golden-vector parity harness — exporter + fixtures + loader"
```

---

### Task 2: geometry — plane fit, projection, sanitize

**Files:**
- Create: `rust/board-projection-detector/src/geometry.rs`
- Modify: `rust/board-projection-detector/src/lib.rs` (add `pub mod geometry;`)
- Test: `rust/board-projection-detector/tests/geometry.rs`

**Interfaces:**
- Consumes: fixtures (`common`).
- Produces:
  - `struct PlaneModel { center: Point3<f64>, normal: Vector3<f64>, u: Vector3<f64>, v: Vector3<f64> }`
  - `fn fit_plane(points: &[Point3<f64>]) -> PlaneModel` — SVD; `u,v` = two largest right-singular vectors, `normal` = smallest (matches `geometry.py:fit_plane`).
  - `fn project_to_plane(points: &[Point3<f64>], plane: &PlaneModel) -> Vec<[f64; 2]>`
  - `fn unproject(coords: &[[f64;2]], plane: &PlaneModel) -> Vec<Point3<f64>>`
  - `fn finite_only(points: &[Point3<f64>]) -> Vec<Point3<f64>>`
  - `fn plane_rms(points: &[Point3<f64>], plane: &PlaneModel) -> f64`
  - `fn extent_2d(coords: &[[f64; 2]]) -> f64`
  - `fn voxel_downsample(points: &[Point3<f64>], voxel: f64) -> Vec<Point3<f64>>` — group points by `floor(p/voxel)` per axis, emit each voxel's **centroid** (matches open3d `voxel_down_sample`, which the Python `downsample` wraps). Add a test: two points in one voxel collapse to their midpoint; two points in different voxels both survive.

- [ ] **Step 1: Write the failing test**

```rust
// tests/geometry.rs
mod common;
use board_projection_detector::geometry::*;
use nalgebra::Point3;

#[test]
fn finite_only_drops_non_finite() {
    let pts = vec![Point3::new(1.0,2.0,3.0), Point3::new(f64::NAN,0.0,0.0),
                   Point3::new(0.0,f64::INFINITY,0.0), Point3::new(4.0,5.0,6.0)];
    let out = finite_only(&pts);
    assert_eq!(out.len(), 2);
    assert_eq!(out[0], Point3::new(1.0,2.0,3.0));
    assert_eq!(out[1], Point3::new(4.0,5.0,6.0));
}

#[test]
fn fit_plane_on_xy_plane_gives_z_normal() {
    // z ≈ 0 patch → normal ∥ ±z, projection preserves x,y extent
    let mut pts = vec![];
    for i in 0..10 { for j in 0..10 {
        pts.push(Point3::new(i as f64 * 0.1, j as f64 * 0.1, 0.0));
    }}
    let plane = fit_plane(&pts);
    assert!(plane.normal.z.abs() > 0.999, "normal={:?}", plane.normal);
    let coords = project_to_plane(&pts, &plane);
    assert!((extent_2d(&coords) - 0.9).abs() < 1e-6);
    assert!(plane_rms(&pts, &plane) < 1e-9);
}
```

- [ ] **Step 2: Run to verify fail**

Run: `cd rust/board-projection-detector && cargo test --test geometry`
Expected: FAIL — `geometry` unresolved.

- [ ] **Step 3: Implement `geometry.rs`**

Port `geometry.py`. Center = mean; SVD of centered points (`nalgebra::SVD` on the `N×3` matrix, or 3×3 covariance eigen — use `SVD` of the `3×N` to get right singular vectors as columns of `V`). Match numpy's `np.linalg.svd(q)` where `vt` rows are singular vectors: `u=vt[0], v=vt[1], normal=vt[2]`.
```rust
use nalgebra::{Point3, Vector3, DMatrix};

pub struct PlaneModel { pub center: Point3<f64>, pub normal: Vector3<f64>, pub u: Vector3<f64>, pub v: Vector3<f64> }

pub fn finite_only(points: &[Point3<f64>]) -> Vec<Point3<f64>> {
    points.iter().filter(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()).copied().collect()
}

pub fn fit_plane(points: &[Point3<f64>]) -> PlaneModel {
    let n = points.len() as f64;
    let center = points.iter().fold(Vector3::zeros(), |a, p| a + p.coords) / n;
    // q: rows = centered points (N×3). SVD → V columns are the 3 principal axes.
    let q = DMatrix::from_row_iterator(points.len(), 3,
        points.iter().flat_map(|p| { let d = p.coords - center; [d.x, d.y, d.z] }));
    let svd = q.svd(true, true);
    let vt = svd.v_t.expect("v_t"); // 3×3, rows = right singular vectors (matches numpy vt)
    let row = |i: usize| Vector3::new(vt[(i,0)], vt[(i,1)], vt[(i,2)]);
    PlaneModel { center: Point3::from(center), normal: row(2), u: row(0), v: row(1) }
}

pub fn project_to_plane(points: &[Point3<f64>], plane: &PlaneModel) -> Vec<[f64;2]> {
    points.iter().map(|p| { let q = p.coords - plane.center.coords; [q.dot(&plane.u), q.dot(&plane.v)] }).collect()
}
pub fn unproject(coords: &[[f64;2]], plane: &PlaneModel) -> Vec<Point3<f64>> {
    coords.iter().map(|c| Point3::from(plane.center.coords + c[0]*plane.u + c[1]*plane.v)).collect()
}
pub fn plane_rms(points: &[Point3<f64>], plane: &PlaneModel) -> f64 {
    let s: f64 = points.iter().map(|p| { let d = (p.coords - plane.center.coords).dot(&plane.normal); d*d }).sum();
    (s / points.len() as f64).sqrt()
}
pub fn extent_2d(coords: &[[f64;2]]) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY;2], [f64::NEG_INFINITY;2]);
    for c in coords { for k in 0..2 { lo[k]=lo[k].min(c[k]); hi[k]=hi[k].max(c[k]); } }
    (hi[0]-lo[0]).max(hi[1]-lo[1])
}
```
Note: SVD sign is arbitrary (like numpy). Downstream `pose::board_pose` fixes normal sign toward the sensor and `stance`/projection are sign-robust; do NOT try to pin SVD signs to numpy.

- [ ] **Step 4: Run to verify pass**

Run: `cd rust/board-projection-detector && cargo test --test geometry`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/geometry.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/geometry.rs
git commit -m "feat(board-proj): geometry — SVD plane fit, projection, finite_only"
```

---

### Task 3: dbscan — anisotropic-scaled Euclidean clustering

**Files:**
- Create: `rust/board-projection-detector/src/dbscan.rs`
- Modify: `src/lib.rs`
- Test: `rust/board-projection-detector/tests/dbscan.rs`

**Interfaces:**
- Produces:
  - `fn anisotropic_scaled(points: &[Point3<f64>], eps_h: f64, vertical_gap_deg: f64) -> Vec<Point3<f64>>` — z-compressed COPY (ports `_anisotropic_scaled`).
  - `fn dbscan(points: &[Point3<f64>], eps: f64, min_points: usize) -> Vec<i64>` — labels, `-1` = noise; core/reachable semantics matching open3d `cluster_dbscan` (a point is core if it has `>= min_points` neighbours within `eps` INCLUDING itself; clusters are connected components of core points plus their border points).
  - `fn cluster_anisotropic(points: &[Point3<f64>], eps: f64, min_points: usize, vertical_gap_deg: f64) -> Vec<i64>` — scale, cluster, return labels indexed back to ORIGINAL `points`.

**Note on parity:** open3d's exact label integers / tie-breaks are NOT reproducible. Assert on **cluster membership of the board**, not label equality: the Rust clustering must produce a cluster whose point set matches the Python-selected board cluster by ≥95% (checked end-to-end in Task 9; here, unit-test structural properties + a synthetic two-blob separation).

- [ ] **Step 1: Write the failing test**

```rust
// tests/dbscan.rs
use board_projection_detector::dbscan::*;
use nalgebra::Point3;

#[test]
fn two_separated_blobs_get_two_labels() {
    let mut pts = vec![];
    for i in 0..40 { pts.push(Point3::new(0.0 + (i%5) as f64*0.01, (i/5) as f64*0.01, 0.0)); }
    for i in 0..40 { pts.push(Point3::new(5.0 + (i%5) as f64*0.01, (i/5) as f64*0.01, 0.0)); }
    let labels = dbscan(&pts, 0.05, 5);
    let uniq: std::collections::BTreeSet<_> = labels.iter().filter(|&&l| l >= 0).collect();
    assert_eq!(uniq.len(), 2);
    assert_ne!(labels[0], labels[79]);
}

#[test]
fn anisotropic_scaling_compresses_z_with_range() {
    // far point: z scaled DOWN so ring gaps merge; near point ~unchanged
    let near = Point3::new(0.5, 0.0, 1.0);
    let far  = Point3::new(20.0, 0.0, 1.0);
    let out = anisotropic_scaled(&[near, far], 0.15, 3.0);
    assert!((out[0].z - 1.0).abs() < 1e-6, "near z unchanged");
    assert!(out[1].z < 0.5, "far z compressed, got {}", out[1].z);
}
```

- [ ] **Step 2: Run to verify fail**

Run: `cd rust/board-projection-detector && cargo test --test dbscan`
Expected: FAIL — unresolved `dbscan`.

- [ ] **Step 3: Implement `dbscan.rs`**

`anisotropic_scaled` ports `cluster_after_ground.py:19-47` exactly:
```rust
use nalgebra::Point3;
pub fn anisotropic_scaled(points: &[Point3<f64>], eps_h: f64, vertical_gap_deg: f64) -> Vec<Point3<f64>> {
    if vertical_gap_deg <= 0.0 { return points.to_vec(); }
    let t = vertical_gap_deg.to_radians().tan();
    points.iter().map(|p| {
        let r = (p.x*p.x + p.y*p.y).sqrt();
        let eps_v = eps_h.max(2.0 * r * t);
        Point3::new(p.x, p.y, p.z * (eps_h / eps_v))
    }).collect()
}
```
`dbscan`: grid-accelerated (cell = `eps`), region query over the 27 neighbouring cells, classic DBSCAN. Deterministic point iteration order (0..N) so results are reproducible run-to-run (open3d seeds its own; ours is order-deterministic which is all parity needs). Core condition counts the point itself, matching open3d.
```rust
use std::collections::HashMap;
pub fn dbscan(points: &[Point3<f64>], eps: f64, min_points: usize) -> Vec<i64> {
    let n = points.len();
    let mut labels = vec![-1_i64; n];   // -1 = unvisited/noise
    let mut visited = vec![false; n];
    let eps2 = eps * eps;
    let key = |p: &Point3<f64>| ((p.x/eps).floor() as i64, (p.y/eps).floor() as i64, (p.z/eps).floor() as i64);
    let mut grid: HashMap<(i64,i64,i64), Vec<usize>> = HashMap::new();
    for (i, p) in points.iter().enumerate() { grid.entry(key(p)).or_default().push(i); }
    let region = |i: usize| -> Vec<usize> {
        let (cx,cy,cz) = key(&points[i]);
        let mut out = vec![];
        for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
            if let Some(c) = grid.get(&(cx+dx, cy+dy, cz+dz)) {
                for &j in c { if (points[i].coords - points[j].coords).norm_squared() <= eps2 { out.push(j); } }
            }
        }}}
        out
    };
    let mut cluster = 0_i64;
    for i in 0..n {
        if visited[i] { continue; }
        visited[i] = true;
        let neigh = region(i);
        if neigh.len() < min_points { continue; } // stays noise (-1)
        labels[i] = cluster;
        let mut queue = neigh;
        let mut qi = 0;
        while qi < queue.len() {
            let j = queue[qi]; qi += 1;
            if !visited[j] { visited[j] = true; let jn = region(j); if jn.len() >= min_points { queue.extend(jn); } }
            if labels[j] < 0 { labels[j] = cluster; }
        }
        cluster += 1;
    }
    labels
}
pub fn cluster_anisotropic(points: &[Point3<f64>], eps: f64, min_points: usize, vertical_gap_deg: f64) -> Vec<i64> {
    let scaled = anisotropic_scaled(points, eps, vertical_gap_deg);
    dbscan(&scaled, eps, min_points) // labels index back to `points` by position
}
```

- [ ] **Step 4: Run to verify pass**

Run: `cd rust/board-projection-detector && cargo test --test dbscan`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/dbscan.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/dbscan.rs
git commit -m "feat(board-proj): anisotropic-scaled grid DBSCAN"
```

---

### Task 4: background — voxel occupancy + consensus + dilated foreground

**Files:**
- Create: `rust/board-projection-detector/src/background.rs`
- Modify: `src/lib.rs`
- Test: `rust/board-projection-detector/tests/background.rs`

**Interfaces:**
- Produces:
  - `struct BackgroundModel { voxel: f64, dilation_radius: i64, min_sources: usize, sources: HashMap<String, BTreeSet<i64>>, keys: Option<Vec<i64>> }`
  - `fn new(voxel: f64, dilation_radius: i64, min_sources: usize) -> Self`
  - `fn observe(&mut self, points: &[Point3<f64>], source: &str)`
  - `fn finalize(&mut self)`
  - `fn foreground_points(&self, dn: &[Point3<f64>]) -> Vec<Point3<f64>>`
  - `fn keys(&self) -> &[i64]` (for fixture parity)
  - free `fn pack(idx: [i64;3]) -> i64` (ports `background.py:_pack`, bit layout EXACT: 21 bits/axis, `_KEY_OFFSET = 1<<20`).

- [ ] **Step 1: Write the failing test**

```rust
// tests/background.rs
mod common;
use board_projection_detector::background::BackgroundModel;
use nalgebra::Point3;

#[test]
fn foreground_keeps_new_geometry_drops_static() {
    let mut bg = BackgroundModel::new(0.06, 1, 1);
    // static wall at x=2
    let wall: Vec<_> = (0..50).map(|i| Point3::new(2.0, i as f64*0.02, 0.0)).collect();
    bg.observe(&wall, "live");
    bg.finalize();
    // query: wall + a new blob at x=5
    let mut q = wall.clone();
    q.push(Point3::new(5.0, 0.0, 0.0));
    let fg = bg.foreground_points(&q);
    assert!(fg.iter().all(|p| p.x > 4.0), "static wall not suppressed: {fg:?}");
    assert_eq!(fg.len(), 1);
}

#[test]
fn foreground_parity_against_python() {
    for f in common::load_all().into_iter().filter(|f| f.generator_is_bg()) {
        // rebuild background from exported keys, then compare foreground on dn
        // (helper `from_keys` + downsample covered in impl/loader)
        // assert ≥95% membership match — see common::foreground_match
        common::assert_foreground_parity(&f);
    }
}
```
(Add `generator_is_bg`, `from_keys`, `assert_foreground_parity` to `common/mod.rs`: rebuild `BackgroundModel` via a `from_keys(keys, params)` constructor, downsample `f.input`, compute Rust foreground, and match against `f.golden.foreground_xyz` by nearest-neighbour within `voxel`.)

- [ ] **Step 2: Run to verify fail**

Run: `cd rust/board-projection-detector && cargo test --test background`
Expected: FAIL — unresolved `background`.

- [ ] **Step 3: Implement `background.rs`**

Port `background.py` exactly. Bit-packing (`_KEY_BITS=21`, mask, offset `1<<20`); `observe` unions per-source key sets; `finalize` keeps keys seen by `>= min_sources` sources; `foreground_points` marks a query point background if its own OR any dilation-stencil-neighbour voxel is in `keys`. Add `from_keys(keys: Vec<i64>, voxel, dilation_radius, min_sources)` for fixture rebuild.
```rust
use nalgebra::Point3;
use std::collections::{BTreeSet, HashMap};

const KEY_BITS: i64 = 21;
const KEY_MASK: i64 = (1 << KEY_BITS) - 1;
const KEY_OFFSET: i64 = 1 << (KEY_BITS - 1);

pub fn pack(idx: [i64;3]) -> i64 {
    (idx[0] & KEY_MASK) | ((idx[1] & KEY_MASK) << KEY_BITS) | ((idx[2] & KEY_MASK) << (2*KEY_BITS))
}
fn voxel_idx(p: &Point3<f64>, voxel: f64) -> [i64;3] {
    [ (p.x/voxel).floor() as i64 + KEY_OFFSET,
      (p.y/voxel).floor() as i64 + KEY_OFFSET,
      (p.z/voxel).floor() as i64 + KEY_OFFSET ]
}
pub struct BackgroundModel {
    pub voxel: f64, pub dilation_radius: i64, pub min_sources: usize,
    sources: HashMap<String, BTreeSet<i64>>, keys: Option<Vec<i64>>, stencil: Vec<[i64;3]>,
}
impl BackgroundModel {
    pub fn new(voxel: f64, dilation_radius: i64, min_sources: usize) -> Self {
        let r = dilation_radius;
        let mut stencil = vec![];
        for dx in -r..=r { for dy in -r..=r { for dz in -r..=r { stencil.push([dx,dy,dz]); }}}
        Self { voxel, dilation_radius, min_sources, sources: HashMap::new(), keys: None, stencil }
    }
    pub fn from_keys(keys: Vec<i64>, voxel: f64, dilation_radius: i64, min_sources: usize) -> Self {
        let mut m = Self::new(voxel, dilation_radius, min_sources);
        let mut k = keys; k.sort_unstable(); k.dedup(); m.keys = Some(k); m
    }
    pub fn observe(&mut self, points: &[Point3<f64>], source: &str) {
        if points.is_empty() { return; }
        let e = self.sources.entry(source.to_string()).or_default();
        for p in points { e.insert(pack(voxel_idx(p, self.voxel))); }
        self.keys = None;
    }
    pub fn finalize(&mut self) {
        let mut counts: HashMap<i64, usize> = HashMap::new();
        for s in self.sources.values() { for &k in s { *counts.entry(k).or_default() += 1; } }
        let mut keys: Vec<i64> = counts.into_iter().filter(|&(_,c)| c >= self.min_sources).map(|(k,_)| k).collect();
        keys.sort_unstable();
        self.keys = Some(keys);
    }
    pub fn keys(&self) -> &[i64] { self.keys.as_deref().unwrap_or(&[]) }
    pub fn foreground_points(&self, dn: &[Point3<f64>]) -> Vec<Point3<f64>> {
        let keys = self.keys.as_ref().expect("finalize() before foreground_points()");
        if keys.is_empty() || dn.is_empty() { return dn.to_vec(); }
        dn.iter().filter(|p| {
            let base = voxel_idx(p, self.voxel);
            let is_bg = self.stencil.iter().any(|d| {
                let k = pack([base[0]+d[0], base[1]+d[1], base[2]+d[2]]);
                keys.binary_search(&k).is_ok()
            });
            !is_bg
        }).copied().collect()
    }
}
```

- [ ] **Step 4: Run to verify pass**

Run: `cd rust/board-projection-detector && cargo test --test background`
Expected: PASS (synthetic + parity).

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/background.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/background.rs rust/board-projection-detector/tests/common/mod.rs
git commit -m "feat(board-proj): BackgroundModel voxel occupancy + consensus + dilated foreground"
```

---

### Task 5: candidates — foreground methods + cluster-and-gate

**Files:**
- Create: `rust/board-projection-detector/src/candidates.rs`
- Modify: `src/lib.rs`
- Test: `rust/board-projection-detector/tests/candidates.rs`

**Interfaces:**
- Consumes: `geometry`, `dbscan`, `background`, `config::BoardConfig`, `arrsac::Arrsac`, `plane_estimator::PlaneEstimator`, `sample_consensus::Consensus`.
- Produces:
  - `struct Candidate { points: Vec<Point3<f64>>, plane: PlaneModel }`
  - `fn plausible_board_patch(points: &[Point3<f64>], board: &BoardConfig) -> Option<Candidate>` (ports `candidates/__init__.py:plausible_board_patch`, uses `board.flatness_rms_max`).
  - `fn remove_big_planes(points: &[Point3<f64>], board: &BoardConfig, dist: f64, min_frac: f64, vertical_gap_deg: f64) -> Vec<Point3<f64>>` (ports `_remove_big_planes`).
  - `fn cluster_and_gate(fg: &[Point3<f64>], board: &BoardConfig, cluster_eps: f64, cluster_min_points: usize, vertical_gap_deg: f64) -> Vec<Candidate>` (ports `_cluster_and_gate` + `_merge_coplanar_clusters`).
  - `fn generate_plane_strip(points: &[Point3<f64>], board: &BoardConfig) -> Vec<Candidate>` (ports `generate_cluster_after_ground`; constants `_BIG_PLANE_DIST=0.05`, `_BIG_PLANE_MIN_FRAC=0.08`, `cluster_eps=0.15`).
  - `fn generate_background_diff(dn: &[Point3<f64>], board: &BoardConfig, background: &BackgroundModel) -> Vec<Candidate>` (ports `generate_background_diff`; `cluster_eps=0.15`).

**RANSAC (no hollow-board-detector):** `remove_big_planes` uses open3d `segment_plane` in Python. Implement it locally with the same primitives `hollow-board-detector`'s `fit_plane_ransac` uses:
```rust
use arrsac::Arrsac;
use sample_consensus::Consensus;
use plane_estimator::PlaneEstimator;
// inside remove_big_planes, per iteration:
let mut arrsac = Arrsac::new(dist, rand::thread_rng()).max_candidate_hypotheses(300);
let est = PlaneEstimator::new();
let (_model, inlier_idx) = match arrsac.model_inliers(&est, remaining.iter().cloned()) {
    Some(r) => r, None => break,
};
```
`inlier_idx: Vec<usize>` indexes `remaining`. Port the strip loop's "biggest connected component extent vs board scale" logic on those inliers using `dbscan::cluster_anisotropic(&inliers, 0.20, 10, vertical_gap_deg)` (Python uses eps=0.20, min_points=10 for the big-vs-board judgement). `plane_estimator::PlaneModel` has only `center`/`normal`; for the extent check, refit a `geometry::PlaneModel` (with `u`,`v`) on the biggest component via `geometry::fit_plane`.
Note: `arrsac`/open3d RANSAC are both randomized; parity is asserted on the SELECTED-cluster outcome (Task 9), not on exact stripped-point sets. Seed `rand` deterministically per call (`rand::rngs::StdRng::seed_from_u64(0)`) to keep runs reproducible, mirroring the Python `o3d.utility.random.seed(0)`.

**Constants (copy verbatim from Python):** big-plane loop max 6 iterations, `len(remaining) < 100` break, `len(idx) < max(100, min_frac*len(remaining))` break, `ext <= 2.0*diag` stop. `_merge_coplanar_clusters`: `seed_min_points=40`, `offset_tol=0.02`, `merge_dist_factor=1.6`, `diag = side_m*sqrt(2)`. `plausible_board_patch`: `_MIN_PATCH_POINTS=60`, extent band `0.5*side_m ..= 1.8*diag`.

- [ ] **Step 1: Write the failing test** (synthetic board patch passes the gate; a too-small blob fails)

```rust
// tests/candidates.rs
mod common;
use board_projection_detector::{candidates::*, config::production_config};
use nalgebra::Point3;

#[test]
fn plausible_patch_accepts_board_rejects_small() {
    let board = production_config(1.0, [0.0,0.0,1.0], 30);
    // a ~1 m flat square patch in the x=2 plane
    let mut patch = vec![];
    for i in 0..40 { for j in 0..40 {
        patch.push(Point3::new(2.0, -0.5 + i as f64*0.025, -0.5 + j as f64*0.025));
    }}
    assert!(plausible_board_patch(&patch, &board).is_some());
    let tiny: Vec<_> = patch.iter().take(10).copied().collect();
    assert!(plausible_board_patch(&tiny, &board).is_none());
}

#[test]
fn candidate_parity_against_python() {
    // For each fixture: run the matching generator on downsample(input),
    // assert candidate count and that some candidate centroid matches the
    // Python selected_centroid within 0.02 m when detected.
    for f in common::load_all() { common::assert_candidate_parity(&f); }
}
```
(`assert_candidate_parity` in `common`: downsample input, dispatch generator by `f.golden.generator` — rebuilding the background from keys for `background_subtraction` — and compare against `n_candidates` (± tolerance is NOT allowed; assert exact only if clustering proves stable, otherwise assert the selected board centroid is among candidate centroids). Prefer asserting the weaker-but-robust property: when `detected`, at least one candidate centroid is within 0.02 m of `selected_centroid`.)

- [ ] **Step 2: Run to verify fail** — `cd rust/board-projection-detector && cargo test --test candidates` → FAIL.

- [ ] **Step 3: Implement `candidates.rs`** — port each function from the Python reference cited above. Structure mirrors `cluster_after_ground.py`. `cluster_and_gate` = `cluster_anisotropic` → `merge_coplanar_clusters` → `plausible_board_patch` per group. Keep the merge loop's "grow from a reliable-plane seed by point-to-plane distance" semantics exactly (do NOT substitute a normal-similarity test — the Python comment documents why it fails).

- [ ] **Step 4: Run to verify pass** — expected PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/candidates.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/candidates.rs rust/board-projection-detector/tests/common/mod.rs
git commit -m "feat(board-proj): candidates — plane_strip + background_diff + cluster-and-gate"
```

---

### Task 6: scorer (reduced) — minAreaRect seed

**Files:**
- Create: `rust/board-projection-detector/src/scorer.rs`
- Modify: `src/lib.rs`
- Test: `rust/board-projection-detector/tests/scorer.rs`

**Interfaces:**
- Produces:
  - `struct MinAreaRect { center: [f64;2], size: [f64;2], corners: [[f64;2];4] }`
  - `fn min_area_rect(coords: &[[f64;2]]) -> Option<MinAreaRect>` — convex hull (Andrew monotone chain) + rotating calipers; matches `cv2.minAreaRect`/`boxPoints` geometry (center + 4 corners). Corner order need not match OpenCV; only the CENTER is consumed downstream.
  - `fn seed_center(coords: &[[f64;2]], board: &BoardConfig) -> [f64;2]` — ports the `square_icp` seed logic: compute `min_area_rect`; if it exists AND `min(size) >= 3*cell` AND `lo < mean_side < hi` (size gate, `lo=side_m*(1-2*side_tol)`, `hi=side_m*(1+2*side_tol)`), return rect center; else return centroid of `coords`.

**Why reduced:** see Global Constraints. `seed_center` is the ENTIRE contribution of `scorer` to the production path.

- [ ] **Step 1: Write the failing test**

```rust
// tests/scorer.rs
use board_projection_detector::{scorer::*, config::production_config};

#[test]
fn min_area_rect_of_axis_square() {
    let sq = vec![[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]];
    let r = min_area_rect(&sq).unwrap();
    assert!((r.center[0]-0.5).abs() < 1e-9 && (r.center[1]-0.5).abs() < 1e-9);
    assert!((r.size[0]-1.0).abs() < 1e-6 && (r.size[1]-1.0).abs() < 1e-6);
}

#[test]
fn min_area_rect_of_rotated_square() {
    // 1×1 square rotated 30°, still area ~1, min side ~1
    let th = 30f64.to_radians(); let (c,s) = (th.cos(), th.sin());
    let base = [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]];
    let sq: Vec<_> = base.iter().map(|p| [p[0]*c - p[1]*s, p[0]*s + p[1]*c]).collect();
    let r = min_area_rect(&sq).unwrap();
    assert!((r.size[0]*r.size[1] - 1.0).abs() < 1e-3, "area {:?}", r.size);
}

#[test]
fn seed_center_falls_back_to_centroid_when_wrong_size() {
    let board = production_config(1.0, [0.0,0.0,1.0], 30);
    // a 0.1 m blob → fails size gate → centroid fallback
    let blob = vec![[0.0,0.0],[0.1,0.0],[0.1,0.1],[0.0,0.1]];
    let c = seed_center(&blob, &board);
    assert!((c[0]-0.05).abs() < 1e-9 && (c[1]-0.05).abs() < 1e-9);
}
```

- [ ] **Step 2: Run to verify fail** — FAIL (unresolved `scorer`).

- [ ] **Step 3: Implement `scorer.rs`** — Andrew monotone-chain hull, then rotating calipers: for each hull edge, rotate points so the edge is axis-aligned, take the axis-aligned bounding box, track min area. Return center (transformed back), size, corners. Then `seed_center` applies the size gate exactly as `score_candidate` does before returning the quad center.

- [ ] **Step 4: Run to verify pass** — PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/scorer.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/scorer.rs
git commit -m "feat(board-proj): minAreaRect (hull+calipers) + square_icp seed center"
```

---

### Task 7: square_fit — fixed-side square fit

**Files:**
- Create: `rust/board-projection-detector/src/square_fit.rs`
- Modify: `src/lib.rs`
- Test: `rust/board-projection-detector/tests/square_fit.rs`

**Interfaces:**
- Produces:
  - `struct SquareFit { center: [f64;2], theta: f64, residual: f64, corners_2d: [[f64;2];4] }`
  - `fn fit_fixed_square(coords: &[[f64;2]], side: f64, init_center: Option<[f64;2]>, init_theta: Option<f64>) -> Option<SquareFit>` — ports `square_fit.py:fit_fixed_square` (pure numpy → pure Rust). Constants: `_MIN_POINTS=20`, `_EXTENT_PCTL=2.0`, `_N_BINS_PER_SIDE=10`, `_BAND_FRAC=0.06`, `_COARSE_STEPS=37`, `_COARSE_REFINE_WINDOW_DEG=4.0`, `_RESOLUTION_DEG=0.25`, `_LOCALIZE_RADIUS_FACTOR=1.5`.
  - Detector always calls with `init_theta=None` (see `detector.py:168-170`), so the coarse full `[0,90°)` sweep path is the one that must be right; implement the `init_theta=Some` window too for completeness/tests.

**Percentile:** port `np.percentile(..., pctl, axis)` with linear interpolation (numpy default) — a small helper `percentile(sorted_vals, p)`.

- [ ] **Step 1: Write the failing test**

```rust
// tests/square_fit.rs
use board_projection_detector::square_fit::fit_fixed_square;

#[test]
fn fits_clean_unit_square_zero_residual() {
    // dense border of a 1 m square at ~15°
    let th = 15f64.to_radians(); let (c,s)=(th.cos(), th.sin());
    let mut pts = vec![];
    let n = 60;
    for i in 0..n { let t = i as f64/n as f64;
        for e in [[t,0.0],[t,1.0],[0.0,t],[1.0,t]] {
            let (x,y) = (e[0]-0.5, e[1]-0.5);
            pts.push([x*c - y*s, x*s + y*c]);
        }
    }
    let fit = fit_fixed_square(&pts, 1.0, None, None).unwrap();
    assert!(fit.residual < 0.05, "residual {}", fit.residual);
    // theta determined mod 90°
    let deg = fit.theta.to_degrees().rem_euclid(90.0);
    assert!((deg - 15.0).abs() < 3.0 || (deg - 75.0).abs() < 3.0, "theta {deg}");
}

#[test]
fn too_few_points_returns_none() {
    assert!(fit_fixed_square(&[[0.0,0.0];5], 1.0, None, None).is_none());
}
```

- [ ] **Step 2: Run to verify fail** — FAIL.

- [ ] **Step 3: Implement `square_fit.rs`** — direct port. `_fit_at_theta`: rotate coords by `-theta` (`p @ rot` with `rot=[[c,-s],[s,c]]` — note Python does `coords @ rot` which is rotate by `+theta` of the frame; replicate the exact matrix), robust per-axis extent midpoint via percentiles, `_coverage_residual` (mean outside + perimeter-band miss fraction). `_best_over` = argmin over theta grid. `fit_fixed_square` = localize by `init_center` radius, coarse sweep (`init_theta=None`) then ±4° refine, or ±window when seeded.

- [ ] **Step 4: Run to verify pass** — PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/square_fit.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/square_fit.rs
git commit -m "feat(board-proj): fit_fixed_square — brute theta sweep + coverage residual"
```

---

### Task 8: pose + gates — board_pose, stance_3d, isolation_density

**Files:**
- Create: `rust/board-projection-detector/src/pose.rs`
- Modify: `src/lib.rs`
- Test: `rust/board-projection-detector/tests/pose.rs`

**Interfaces:**
- Produces:
  - `struct BoardDetection { center: Point3<f64>, rotation: Matrix3<f64>, corners_3d: [Point3<f64>;4], score: f64 }` (columns of `rotation` = board x, y, normal).
  - `fn board_pose(plane: &PlaneModel, corners_2d: &[[f64;2];4], score: f64, up: [f64;3]) -> BoardDetection` — ports `pose.py:board_pose` EXACTLY: normal toward sensor, X = center→up-most corner projected in-plane, `y = n × x`, CCW winding via `atan2(rel·y, rel·x)` sort.
  - `fn stance_3d(corners_3d: &[Point3<f64>;4], up: [f64;3]) -> f64` — ports `detector.py:_stance` (max |diagonal·up|; corners CCW so `[2]-[0]`, `[3]-[1]` are diagonals).
  - `fn isolation_density(dn: &[Point3<f64>], plane: &PlaneModel, corners_2d: &[[f64;2];4]) -> f64` — port `isolation.py:isolation_density` (read the file; points-per-perimeter-metre of coplanar continuation just outside the quad).

- [ ] **Step 1: Write the failing test**

```rust
// tests/pose.rs
mod common;
use board_projection_detector::{pose::*, geometry::PlaneModel};
use nalgebra::{Point3, Vector3};

#[test]
fn board_pose_normal_faces_sensor_and_winds_ccw() {
    // vertical plane at x=2, in-plane u=y, v=z
    let plane = PlaneModel { center: Point3::new(2.0,0.0,0.0), normal: Vector3::new(1.0,0.0,0.0),
        u: Vector3::new(0.0,1.0,0.0), v: Vector3::new(0.0,0.0,1.0) };
    // diamond corners (top/left/bottom/right) in (u,v)
    let corners = [[0.0,0.7],[-0.7,0.0],[0.0,-0.7],[0.7,0.0]];
    let det = board_pose(&plane, &corners, 1.0, [0.0,0.0,1.0]);
    // normal faces origin → -x
    assert!(det.rotation.column(2).x < 0.0);
    // center ~ plane center
    assert!((det.center.x - 2.0).abs() < 1e-9);
}

#[test]
fn pose_corners_parity_against_python() {
    for f in common::load_all().into_iter().filter(|f| f.golden.detected) {
        common::assert_pose_corners_parity(&f); // corners_3d set within a few cm of Python
    }
}
```
(`assert_pose_corners_parity`: run the full detector once available — OR, at this task, reconstruct pose from the golden's own `foreground`→candidate is not available yet, so gate this second test `#[ignore]` here and REMOVE the ignore in Task 9 where `detect()` exists. Keep the first, self-contained test active now.)

- [ ] **Step 2: Run to verify fail** — FAIL.

- [ ] **Step 3: Implement `pose.rs`** — port `pose.py`, `_stance`, and `isolation.py`. For `board_pose`, `top = corners_3d[argmax(corners_3d · up)]`; project X in-plane by removing the normal component; build rotation `[x|y|n]`; sort corners CCW.

- [ ] **Step 4: Run to verify pass** — first test PASS; parity test `#[ignore]`d until Task 9.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/pose.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/pose.rs
git commit -m "feat(board-proj): board_pose (CCW, sensor-facing) + stance_3d + isolation_density"
```

---

### Task 9: detector — detect() orchestration + end-to-end parity + recall/precision gate

**Files:**
- Create: `rust/board-projection-detector/src/detector.rs`
- Modify: `src/lib.rs`
- Modify: `rust/board-projection-detector/tests/pose.rs` (un-ignore the parity test)
- Test: `rust/board-projection-detector/tests/detect_parity.rs`

**Interfaces:**
- Produces:
  - `enum RejectReason { NoClusters, Flatness, Extent, SizeGate, SquareResidual, Stance, Isolation }`
  - `struct DetectOutcome { detection: Option<BoardDetection>, selected_points: Option<Vec<Point3<f64>>>, selected_plane: Option<PlaneModel>, n_candidates: usize, reject: Option<RejectReason> }`
  - `fn detect(points: &[Point3<f64>], board: &BoardConfig, method: ForegroundMethod, voxel: f64, background: Option<&BackgroundModel>) -> DetectOutcome`
  - **The `selected_points` + `selected_plane` are the sub-project-2 output** — the winning cluster fed to RANSAC+ICP. `detection` (square-fit pose) is used only for gating/selection here.

**Orchestration (ports `detector.py:detect` square_icp branch only):**
1. `points = finite_only(points)`; `dn = geometry::voxel_downsample(&points, voxel)` (local, centroid-per-voxel — matches Python `downsample`).
2. Foreground: `PlaneStrip` → `generate_plane_strip(&dn, board)`; `BackgroundSubtraction` → requires `background` (else return `NoClusters`), `generate_background_diff(&dn, board, bg)`.
3. Per candidate: `coords = project_to_plane(cand.points, cand.plane)`; `seed = scorer::seed_center(&coords, board)`; `fit = square_fit::fit_fixed_square(&coords, board.side_m, Some(seed), None)`; reject if `None` or `fit.residual >= board.square_icp_residual_max`.
4. `det = board_pose(&cand.plane, &fit.corners_2d, 1/(1+fit.residual), board.up_axis)`.
5. Gates: `stance_3d(det.corners_3d, up) <= stance_floor` → reject; `isolation` on → `isolation_density > isolation_max_density` → reject.
6. Keep candidate with **lowest `fit.residual`** as best (mirrors `best_residual` selection); record its `cand.points`/`cand.plane`.
7. `DetectOutcome{ detection: best_det, selected_points: best_cand_points, selected_plane: best_cand_plane, n_candidates, reject }`.

- [ ] **Step 1: Write the failing end-to-end parity test**

```rust
// tests/detect_parity.rs
mod common;
use board_projection_detector::{detector::detect, config::production_config,
    config::ForegroundMethod, background::BackgroundModel};

#[test]
fn per_frame_detection_decision_matches_python() {
    let mut mism = vec![];
    for f in common::load_all() {
        let board = production_config(1.0, f.golden.up_axis, f.golden.cluster_min_points);
        let (method, bg) = common::method_and_background(&f); // rebuild bg from keys for E
        let out = detect(&f.input, &board, method, f.golden.voxel, bg.as_ref());
        if out.detection.is_some() != f.golden.detected { mism.push(f.name.clone()); continue; }
        if let (Some(d), Some(sc)) = (&out.detection, &f.golden.selected_centroid) {
            let c = d.center;
            let dist = ((c.x-sc[0]).powi(2)+(c.y-sc[1]).powi(2)+(c.z-sc[2]).powi(2)).sqrt();
            assert!(dist < 0.02, "{}: centroid off {dist:.3} m", f.name);
        }
    }
    assert!(mism.is_empty(), "detect/no-detect mismatches: {mism:?}");
}

#[test]
fn recall_precision_parity_per_dataset() {
    // Aggregate detected-count per dataset (parsed from fixture name prefix)
    // and assert it equals the Python detected-count (± the ±1-frame tolerance
    // in Global Constraints). Fixture names encode dataset id: "ds3_frame017_bg".
    common::assert_recall_precision_parity(&common::load_all(), |f| {
        let board = production_config(1.0, f.golden.up_axis, f.golden.cluster_min_points);
        let (m, bg) = common::method_and_background(f);
        detect(&f.input, &board, m, f.golden.voxel, bg.as_ref()).detection.is_some()
    });
}
```

- [ ] **Step 2: Run to verify fail** — FAIL (unresolved `detector`).

- [ ] **Step 3: Implement `detector.rs`** per the orchestration above; un-ignore `assert_pose_corners_parity` in `tests/pose.rs` (now that `detect` exists, that helper runs the full pipeline and compares `corners_3d`).

- [ ] **Step 4: Run to verify pass**

Run: `cd rust/board-projection-detector && cargo test`
Expected: ALL tests PASS — including `detect_parity` (per-frame decision + centroid) and `recall_precision_parity`. If a handful of frames mismatch, DO NOT loosen tolerances: debug via the stage-by-stage fixtures (foreground → candidates → seed → fit) to find the diverging stage. Loosening the parity gate is a plan failure.

- [ ] **Step 5: Commit**

```bash
git add rust/board-projection-detector/src/detector.rs rust/board-projection-detector/src/lib.rs rust/board-projection-detector/tests/detect_parity.rs rust/board-projection-detector/tests/pose.rs rust/board-projection-detector/tests/common/mod.rs
git commit -m "feat(board-proj): detect() orchestration + end-to-end parity + recall/precision gate"
```

---

## Self-Review

**Spec coverage** (design decisions from brainstorming → tasks):
- Rust port, new crate → Task 0. ✅
- Both methods, renamed `plane_strip`/`background_subtraction` → Task 5 generators + Task 0 enum. ✅
- Option C (foreground + discriminator, KEEP ICP): `detect()` returns `selected_points`/`selected_plane` for ICP, uses square-fit pose only for gating → Task 9 interfaces. ✅
- Parity harness (golden vectors, stage-by-stage, recall/precision) → Task 1 + per-task parity tests + Task 9 gate. ✅
- Minimal reject enum → Task 9 `RejectReason`. ✅
- `production_config` only, scorer reduced to minAreaRect seed → Global Constraints + Task 6. ✅
- Local `voxel_downsample` (Task 2) + RANSAC via `plane-estimator`/`arrsac` (Task 5); no `hollow-board-detector` (native SFCGAL). Standalone workspace. ✅
- Carry-forward #1 (corner ordering: ArUco uses `corners_3d`) → Task 8 `board_pose` CCW; the ROS wiring enforces "consume `corners_3d`" in sub-project 2 (out of scope here, noted).
- Carry-forward #2 (far-board `cluster_min_points`) → `production_config(cluster_min_points)` param, threaded from fixture per-dataset. ✅

**Deferred to sub-project 2 (ROS node) — NOT in this plan:** **Fold `board-projection-detector` from a standalone workspace back into the root workspace as a normal `rust/*` member** (remove its `[workspace]` table + the root `exclude` entry, switch inline dep versions to `{ workspace = true }` where a root workspace dep exists), and update the root `Cargo.lock` via `just build` (colcon, in the sourced ROS env — plain cargo cannot re-resolve the wildcard ROS crates). This is the agreed "convention-first, deferred for dev speed" debt: standalone now for fast parity-test iteration, member at integration when `lidar_board_detector` (a root member) depends on it. Also: `detection_mode` param + `select_board_cluster` wiring into `process_pointcloud`; E warmup lifecycle (param-driven + `reset_background` service); mapping `selected_points`→existing Stage 2/3; json5 config file surfacing the ~20 constants; launch/`book` docs. This plan's deliverable is the parity-validated library only.

**Placeholder scan:** Every code step carries real Rust or a real Python skeleton. Two spots depend on names in existing files, flagged explicitly to confirm before use, not guess: Task 1 the `boarddet.ingest` frame accessor; Task 8 `isolation.py:isolation_density` (read the file — it was not quoted in full during design). No "TBD"/"add error handling"/"similar to Task N".

**Type consistency:** `PlaneModel{center,normal,u,v}` (Task 2) consumed identically in Tasks 5/8/9. `Candidate{points,plane}` (Task 5) consumed in Task 9. `SquareFit{center,theta,residual,corners_2d}` (Task 7) → `board_pose(plane, corners_2d, score, up)` (Task 8) → `detect` (Task 9). `BoardConfig` fields (Task 0) referenced by exact name in Tasks 5/6/8/9. `ForegroundMethod` (Task 0) dispatched in Task 9. `BackgroundModel` (Task 4) `from_keys`/`foreground_points` used in Tasks 5/9 parity.

**Open items to confirm during execution (do not guess):**
1. `boarddet.ingest` cached-`.npz` frame accessor API (Task 1 exporter loop).
2. `isolation.py:isolation_density` exact formula (Task 8) — read the file; it was not fully quoted in the design pass.
3. numpy `q @ rot` vs `rot @ q` sign convention in `square_fit._fit_at_theta` (Task 7) — replicate the exact Python matrix multiply orientation.
