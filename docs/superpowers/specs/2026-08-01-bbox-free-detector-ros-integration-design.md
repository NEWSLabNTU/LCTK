# Design — Crop-box-free board detection in `lidar_board_detector` (sub-project 2)

**Date:** 2026-08-01
**Branch:** `feat/method-e-background-subtraction`
**Predecessor:** sub-project 1 (HEAD ~`d51c278`) delivered `rust/board-cluster-detector`, a
parity-validated, OpenCV/open3d-free Rust port of the `boarddet` crop-box-free detector.

## Goal

Let a user locate the calibration board with **no bounding box** by setting `detection_mode:
bbox_free` in the board config. The new detector selects/confirms the board *cluster*; its
`selected_points` feed the node's **existing** RANSAC/PCA + ICP pose engine unchanged (Option C,
settled in sub-project 1). The square-fit corners are used only for cluster selection/gating,
never as the final pose.

## Non-goals

- **Method B (`plane_strip`) perf** (157–202 ms/frame, over the ~100 ms Jetson budget) — deferred
  to a follow-up (coarse-voxel strip + cap-and-early-exit). B ships but is not the default path.
- Changing the parity-validated crate's detection logic. The crate is consumed as-is.
- ArUco corner-correspondence rework: Option C uses the node's ICP pose, so the crate's
  `corners_3d` ordering seam does not surface here.

## Scope

Tasks 1–5 of the handoff. Method E (`background_subtraction`) is the validated, in-budget
(34–69 ms/frame) shipping path.

---

## Component 1 — Fold crate into root workspace (task 1)

The crate is a **standalone** cargo workspace today (own `[workspace]` table + root `exclude`
entry), a user-approved dev-speed shortcut. Integration pays that debt: `lidar_board_detector` is a
root member and must path-depend on the detector.

- **Crate `Cargo.toml`:** remove the `[workspace]` table. Switch inline deps to
  `{ workspace = true }` — all are present in the root `[workspace.dependencies]`: `nalgebra`,
  `anyhow`, `serde`, `json5`, `log`, `rand`; dev-deps `approx`, `serde_json`.
- **Root `Cargo.toml`:** delete the `rust/board-cluster-detector` line from `exclude`; the
  `rust/*` members glob then picks it up.
- **Node `Cargo.toml`:** add
  `board-cluster-detector = { version = "0.1.0", path = "../../rust/board-cluster-detector" }`.
- **Lockfile:** regenerate via `just build` (colcon, in the sourced ROS env:
  `source /opt/ros/humble/setup.bash && source install/setup.bash`). Plain `cargo update` aborts on
  the yanked wildcard `sensor_msgs` — see CLAUDE.md.

**Consequence to document in the crate header:** once a root member, the crate shares the
ROS-poisoned root resolve, so plain `cargo test` in the crate dir no longer works. The crate's
parity tests now run **only** under colcon (`just test`). The 51 MB parity fixtures remain local /
gitignored (regenerate via `experiments/board-detection-2d/tools/export_golden.py`).

---

## Component 2 — Config surfacing (task 2)

`board_detector.json5` today holds the **old hollow-board** node schema (RANSAC/ICP/voxel/
`board_width`/`hole_radius`). The new `BoardConfig` is a different shape. Chosen surfacing: a
**nested `bbox_free` block** in the same file; the old keys and the entire bbox path stay untouched.

```json5
{
    // ... existing hollow-board RANSAC/ICP/voxel/geometry keys, unchanged ...

    // Detection mode: "bbox" (default, existing behavior) | "bbox_free"
    "detection_mode": "bbox",

    // Only read when detection_mode == "bbox_free".
    "bbox_free": {
        // "background_subtraction" (Method E, shipping) | "plane_strip" (Method B, slow)
        "foreground_method": "background_subtraction",

        // Voxel edge (m) for the detector's internal downsample — the `voxel`
        // arg to detect(). NOT a BoardConfig field.
        "voxel": 0.05,

        // Deserialized directly into the crate's BoardConfig. MUST spell out the
        // production operating point explicitly: BoardConfig's serde defaults are
        // the frozen library defaults (flatness 0.035, stance_floor 0.0,
        // isolation false), NOT the production values below. See warning.
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

        // Method-E background model + warmup. min_sources is fixed at 1
        // (single live session). See Component 4.
        "background": {
            "dilation_radius": 1,
            "warmup_frames": 20
        }
    }
}
```

**⚠️ Production-values warning (from sub-project 1 ledger):** `BoardConfig` uses
`#[serde(default)]` on every field, and those defaults are the frozen library defaults, which
differ from the production operating point (`production_config()`): `flatness_rms_max` 0.035→0.045,
`stance_floor` 0.0→0.9, `isolation` false→true. The shipped `bbox_free.board` block must therefore
write the production values explicitly (as above). A test asserts the shipped config parses to the
production operating point.

**Parsing:** node reads `detection_mode` (default `"bbox"`), and when `bbox_free`, extracts the
`bbox_free` sub-object, deserializes `bbox_free.board` into `BoardConfig` (via `json5`), parses
`foreground_method` through `ForegroundMethod::from_str`, and reads `voxel` / `background.*`. Config
is loaded once at node construction (same as today's board config).

---

## Component 3 — `select_board_cluster` splice (task 3)

Single, minimal splice: `bbox_free` replaces the **Stage-1 output**. Everything downstream
(Stage 2 `skip_ransac` PCA plane → Stage 3a voxel → Stage 3b ICP → `icp_good_fit_threshold` gate)
is unchanged.

In `process_pointcloud` (`ros/lidar_board_detector/src/main.rs`), after
`convert_pointcloud2_to_points` and before Stage 1:

```
if detection_mode == BboxFree:
    // (Method E warmup handled first — see Component 4; may short-circuit to empty)
    let outcome = detect(&points, &board_cfg, method, voxel, background_opt);
    match outcome.selected_points {
        Some(pts) if !pts.is_empty() => active_points = pts,   // → existing Stage 2/3
        _ => {
            log reject diagnostics (Component 5);
            return empty Detection3DArray;
        }
    }
else:
    active_points = filter_points_by_bbox(...);   // existing code, verbatim
```

`selected_points` are already finite-filtered and voxel-downsampled by the crate; the node's Stage
3a voxel re-runs harmlessly. With `skip_ransac: true` (the shipped default), Stage 2 computes a PCA
plane over `selected_points` and hands them straight to ICP — exactly the intended Option-C
handoff. `bbox_params` / bbox debug publishers are simply not exercised in this branch.

---

## Component 4 — Method E warmup lifecycle (task 4)

Method E needs a `BackgroundModel` built from board-free frames before it can diff. The node runs a
small state machine, decoupled from the subscription callback per CLAUDE.md's high-frequency-sensor
`ArcSwap` pattern.

**State** (`ArcSwap<BackgroundState>`):
- `Warming { model: BackgroundModel, seen: usize }`
- `Ready { model: Arc<BackgroundModel> }`

**Per cloud, when `detection_mode == bbox_free && foreground_method == background_subtraction`:**
- **Warming:** `model.observe(&points, "live")`; `seen += 1`. When `seen == warmup_frames`:
  `model.finalize()`, swap to `Ready`. While warming, publish an empty `Detection3DArray` and log
  (throttled) `"background warmup N/M"`.
- **Ready:** call `detect(..., Some(&model))` (Component 3).

**`reset_background` control:** swaps state back to a fresh `Warming { new BackgroundModel, 0 }` so
the operator can re-capture the empty scene (e.g. after moving the rig).
- Primary: a `std_srvs/srv/Trigger` service.
- **Open impl item / fallback:** no Rust node in this repo creates an rclrs service yet
  (`advanced_extrinsic_solver` is Python). If rclrs 0.7 service wiring + a `std_srvs` dep proves
  heavy during execution, fall back to a watched bool parameter (`reset_background_request`: node
  observes `true`, resets, then is documented as operator-cleared). Decide in the plan after
  confirming the rclrs 0.7 `create_service` API.

`BackgroundModel::new(voxel, dilation_radius, min_sources=1)`. `plane_strip` skips this machine
entirely (no background); warmup gates **only** `background_subtraction`.

---

## Component 5 — Reject diagnostics + None-background error (task 5)

- **Diagnostics:** on a no-detection in `bbox_free`, log `outcome.reject` — the crate's
  `RejectReason` (`NoClusters` | `Flatness` | `Extent` | `SizeGate` | `SquareResidual` | `Stance`
  | `Isolation`) names the furthest-progressing candidate's killer gate. Turns today's silent empty
  publishes into an actionable line.
- **None-background as a real error, not silent `NoClusters`:** the node guarantees `background =
  Some` before it ever enters `Ready`, so a live `detect` call in `bbox_free` always has a
  background. If `detect` is ever reached with `background = None` under `BackgroundSubtraction`
  (a node-logic bug), the node emits an explicit `log_error` at the call site rather than letting
  the crate return a silent `NoClusters`. The crate stays frozen (parity-validated); the guarantee
  lives at the node.

---

## Testing

- **Crate parity:** unchanged; runs under colcon after the fold (`just test`), fixtures local.
- **Config parse:** unit test — `board_detector.json5` with `detection_mode: bbox_free` parses to
  the expected `BoardConfig` and asserts the shipped `bbox_free.board` equals the production
  operating point (flatness 0.045, stance_floor 0.9, isolation true) — guards the serde-default
  trap.
- **Warmup state machine:** observe `warmup_frames` clouds → `finalize` → `Ready`; a cloud in
  `Warming` yields empty detections; `reset_background` returns to `Warming`.
- **Bbox path regression:** `detection_mode: bbox` (default) produces byte-identical behavior to
  today (the splice is fully guarded behind the mode check).

## Open items to confirm during execution (do not guess)

1. rclrs 0.7 `create_service` API + `std_srvs` availability under this workspace → picks the
   `reset_background` mechanism (service vs watched-param fallback). Component 4.
2. Exact node-construction config-load site to thread the new `detection_mode` / `bbox_free`
   parsing into (mirrors the existing `board_detector_file` load).
3. Whether `selected_points` (already downsampled) should bypass Stage 3a voxel — measure; default
   is to leave Stage 3a on (harmless idempotent-ish re-voxel), do not optimize unless it matters.
