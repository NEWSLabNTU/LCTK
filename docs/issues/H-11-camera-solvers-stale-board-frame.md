# H-11 · Camera solvers still build marker geometry in the old edge-aligned board frame → every LiDAR-camera extrinsic is wrong by 45°, half of it silently

- **Severity:** High
- **Area:** `ros/extrinsic_solver_node`, `ros/advanced_extrinsic_solver` (renamed
  `ros/lidar_to_camera_solver` — see the 2026-08-28 note at the bottom of this file) / board frame
  convention
- **Status:** Open — this is **Phase 2** of the corner-aligned board-frame work; Phase 1 has landed and was field-validated on the two-LiDAR rig 2026-08-14, which makes the error below a **confirmed live defect** rather than a predicted one
- **Verified:** 2026-08-13 — `extrinsic_solver_node/main.py:475-575`, `advanced_extrinsic_solver/main.py:1589-1656` (path predates the `ecba23c` rename to `lidar_to_camera_solver`), read against the landed `rust/hollow-board-config/src/lib.rs` (deleted W5-E2; successor `rust/calibration-target/src/lib.rs`)
- **Spec (the fix):** [`2026-08-14-lidar-to-camera-solver-diamond-frame.md`](../superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md) — **this is the target.** Closing H-11 means implementing that spec's three stages. It fixes the names, the staging and the validation gate; the description below is the diagnosis it was written from.
- **Spec (the cause):** [`2026-08-13-corner-aligned-board-frame.md`](../superpowers/specs/2026-08-13-corner-aligned-board-frame.md) — Phase 1, see "Out of Scope" and "Why the phase gap needs a guard rather than a note"
- **Related:** [M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md) (the Phase 1 issue), [M-14](./M-14-corner-order-brittle.md) (the duplicated corner layout — closed by the Phase 2 geometry extraction), [M-12](./archive/M-12-no-robust-estimation-or-refinement.md) (the estimator asymmetry — closed by Phase 2 Stage 2), [C-01](./archive/C-01-aruco-corners-discarded.md), [H-10](./archive/H-10-dump-load-regresses-c01.md) (the saved-file format this bumps to version 4), [L-22](./archive/L-22-advanced-solver-undeclared-lctk-interfaces-dep.md) and [L-23](./L-23-debug-mode-parameter-never-read.md) (found while scoping the fix)

## Problem

Phase 1 redefined the calibration board's canonical local frame in what was then
`rust/hollow-board-config` (deleted W5-E2, `21142ac`; the model now lives in
`rust/calibration-target`): the origin moved from the plate's bottom corner to the plate
**centre**, and the in-plane axes now run corner to corner (the plate is the diamond
`|x| + |y| ≤ R`, `R = board_width/√2`) instead of along the plate's edges. `lidar_board_detector`
publishes board poses in that frame today.

The two Python solvers do **not** consume that model. Each reimplements the board's marker geometry
from the ArUco pattern config in its own `_compute_multi_marker_corners`:

- `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:475`
- `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1589` — this file no longer
  exists at this path; `ros/advanced_extrinsic_solver` was renamed to `ros/lidar_to_camera_solver`
  in `ecba23c`, ahead of the diagnosis below being written up. Whether the renamed package's
  `main.py` still carries the described reimplementation, or now reads the shared model (see
  `board_geometry.py`'s use of `lctk_target.ValidatedTarget`, which postdates this diagnosis), is
  exactly the kind of re-audit this dated pointer-repair pass does not do — see the 2026-08-28 note
  at the bottom of this file

Both still emit corners as `(base_x, base_y, 0)` with the origin at the plate's origin corner and
the axes along the plate's edges — the **previous** convention. Neither reads the new
`paper_placement` field that Phase 1 added to `aruco_pattern.json5` (deleted W5-E1; the equivalent
placement data — `paper_center`, `paper_side`, etc. — now lives in the target manifest's
`fiducial` block, e.g. `ros/lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5`), which is now
the one place stating where the printed sheet sits on the plate.

## Why this is High, not a tidy-up

The published board pose is the transform **from board coordinates to sensor coordinates**, and the
solvers supply *board-local* marker coordinates to it:

```python
world_corners = (board_rotation @ local_corners.T).T + board_position
```

The convention therefore appears on **both sides of that product**. Changing only the LiDAR side
leaves two errors, and they are not equally visible:

1. **A 45° in-plane rotation — silent.** The marker grid is a symmetric 2×2, so a quarter-turn's
   worth of rotational error still produces a clean-looking PnP solve with low reprojection error.
   Nothing in the pipeline reports it. This is the same class of failure as
   [M-14](./M-14-corner-order-brittle.md)'s corner permutation and
   [H-09](./archive/H-09-no-extrinsic-quality-metric.md)'s central point: the system has no way to
   tell you it is not working.
2. **An origin shift of `board_width/√2` ≈ 707 mm — probably caught.** The board-local origin moved
   from a corner to the centre, so every object point is displaced by the plate's half-diagonal.

Half the error is silent. That is precisely why documentation alone is insufficient here and a
runtime guard is required.

## Impact

- **LiDAR-camera calibration is untrustworthy until this lands.** Any extrinsic solved from the
  current tree — live, or exported to Autoware via `lctk_autoware_export` — carries the error above.
- **LiDAR-to-LiDAR calibration is unaffected and must keep working.** Both sides of that solve come
  from `lidar_board_detector`, so the convention cancels. Whatever guard lands must not block it
  ([M-16](./M-16-l2l-pipeline-untested.md)).
- **Saved detections predate the change.** Files written before Phase 1 are `version: 3`
  (see CLAUDE.md, "Detection File Format"), which records no frame convention at all, so a v3 file
  cannot be told apart from a post-change one.

## Fix

**The fix is the spec:
[`2026-08-14-lidar-to-camera-solver-diamond-frame.md`](../superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md).**
It supersedes the sketch below, which was written from the diagnosis before the design existed. Where
the two disagree, the spec wins. Two differences are load-bearing:

- The sketch says port *both* solvers. The spec ports **only** `advanced_extrinsic_solver`, migrated
  to `lidar_to_camera_solver`. `extrinsic_solver_node` is never ported — it becomes unreachable from
  the config-driven launch path when `use_advanced_solver` is removed, and is deleted at Stage 3.
  Investigation established the two do not share a backend (`SOLVEPNP_ITERATIVE` vs `SOLVEPNP_SQPNP`
  plus refinement, float32 vs float64, covariance discarded vs propagated), so porting both would have
  meant porting the weaker estimator forward.
- The sketch says v3 files load with a loud warning. The spec **rejects** them, with an explicit
  one-shot conversion command. Auto-migration would make a file's meaning depend on which build opened
  it — the same silent-difference problem this phase exists to remove.

Closing H-11 means implementing that spec's three stages, and Stage 1 is the stage that ends the
silent 45° error.

<details>
<summary>Original sketch, superseded — kept for the reasoning, not the plan</summary>

1. **Port both `_compute_multi_marker_corners` to the corner-aligned frame.** Corners must come out
   where `BoardModel::marker_paper_point` (successor: `CalibrationTarget::local_marker_paper_point`,
   `rust/calibration-target/src/lib.rs`) puts them: paper coordinates run along the paper's edges,
   at 45° to the board frame's axes, and the sheet's position on the plate comes from
   `paper_placement` in `aruco_pattern.json5` (deleted W5-E1; see the `fiducial.paper_center` field
   in the target manifest now) rather than being re-derived from the board width.
2. **Assert it, cross-language.** `fixtures/targets/marker_corners_world.golden.json` is the
   checked-in seam (this path was already wrong when first written — the golden lived under
   `fixtures/board/` at the time, not `rust/hollow-board-config/tests/fixtures/`; both are gone now,
   repointed 2026-08-28): world-coordinate marker corners at a stated physical mounting, keyed by
   ArUco marker id, with an independent Python generator alongside it
   (`fixtures/targets/generate_marker_corners_world.py`). The Rust side asserts against it already
   (`rust/calibration-target/tests/geometry_contract.rs`); a pytest in `ros/lidar_to_camera_solver`'s
   test directory should assert the Python implementations against the same file — and, per the
   2026-08-28 note below, `ros/lidar_to_camera_solver/test/test_marker_corners_world_golden.py`
   already exists and does exactly that. That also discharges
   [M-14](./M-14-corner-order-brittle.md)'s "corner order is defined twice and never verified", and
   the shared golden is the natural place to finally collapse the two copies into one helper.
3. **Add the frame-convention check.** The publishing half already exists:
   `lidar_board_detector` publishes `corner_aligned_plate_center_v1` as a `std_msgs/String`, once,
   latched (transient-local, depth 1), on the fixed topic `/lctk/board_frame_convention`
   (`main.rs:300-323`, `:528-545`). Both solvers must subscribe and **refuse to start on mismatch**,
   naming the reason — the board frame changed and this solver has not been updated. **Absence of
   the tag must be treated as failure, not as consent**: a solver that starts before any detector
   must not read silence as agreement. The topic is deliberately absolute rather than node-relative,
   because the launch system generates one detector node per sensor-marker pair.
4. **Bump the saved-detection format to version 4**, carrying that same identifier, so a
   file solved under one convention cannot be silently reloaded under another. v3 files must load
   with a loud warning, in the manner [H-10](./archive/H-10-dump-load-regresses-c01.md) established.

</details>

## Notes

Deliberately deferred from Phase 1, not overlooked: the recordings available here
(`ros/lctk_sample_data/bags/TWO_LIDAR_*`) contain **no camera stream**, so nothing camera-side could
be verified end to end. Landing an unverifiable rewrite of the camera geometry alongside the
verifiable LiDAR-side one would have made a failure unattributable. Closing this issue requires a
recording with both a LiDAR and a camera observing the board.

**Update (2026-08-14).** The LiDAR side is now field-validated on the two-LiDAR rig
([M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md), fixed): the published board
pose is confirmed corner-aligned in the intended sense — the `+Y` arrow points at the physically
up-most plate corner, which rules out the `−45°` conjugation. The mismatch described above is
therefore no longer a prediction about what the detector emits; it is a **measured** property of one
side of the product against unchanged Python on the other. Nothing camera-side was run — the
`TWO_LIDAR_*` bags still carry no camera stream — so the closing condition is unchanged. Relatedly,
`paper_placement`'s shipped values are now confirmed to be the board's **measured** sheet placement
(lower quarter, top corner at the plate centre), so fix step 1 must read them rather than assume a
centred sheet.

**Update (2026-08-14, second).** The closing condition above — "requires a recording with both a
LiDAR and a camera observing the board" — is **already satisfied**, and was when this issue was
filed. All five `ros/lctk_sample_data/data/` datasets pair a VLP-32C pcap with a synchronised avi;
dataset 3 carries 270 frames at 1920×1080 with the board filling the image. Frame inspection confirms
it is the **same physical board** Phase 1 validated: diamond-hung square plate, three holes with none
at the bottom, ArUco sheet in the lower quarter with its top corner at the plate centre — matching
`paper_placement`. Only the `TWO_LIDAR_*` bags lack a camera; the pcap/avi sample data never did.

So Phase 2 is validatable today with data already in the repository, and the spec makes dataset 3 the
gate. One caveat carried forward: the gate is **visual**, not numerical. A 45° in-plane error leaves
reprojection RMS low because the 2×2 marker grid is symmetric, so a low residual must never be
accepted as evidence that this issue is fixed. The observable signatures are the ~707 mm origin shift
and the overlay picture.

**Update (2026-08-18, Stage 2).** `lidar_to_camera_solver` now owns both exact operating modes:
`continuous` (default, latest-pair auto-solve) and `manual` (service-driven multi-pose buffer).
Config-driven launch and every justfile path always start that package and pass `solver_mode`; the
removed `use_advanced_solver` boolean has no compatibility alias, and `extrinsic_solver_node` is no
longer reachable from those paths. Continuous mode replaces its one retained pair atomically and
uses the same float64 SQPnP, LM-refined, covariance-aware backend as manual mode. Focused coverage
pins latest-pair replacement and the SQPnP-plus-LM calls. `just build`, all 240 Rust tests, and all
181 Python tests pass. H-11 remains open until Stage 3 deletes the superseded packages and references.

**Update (2026-08-28) — evidence pointers repaired; status and severity unchanged.**
W5-E1/E2 deleted `rust/hollow-board-config` (successor: `rust/calibration-target`) and the
`fixtures/board/` golden (successor: `fixtures/targets/`, now with a checked-in generator at
`fixtures/targets/generate_marker_corners_world.py`, and consumed by
`rust/calibration-target/tests/geometry_contract.rs`,
`ros/lidar_to_camera_solver/test/test_marker_corners_world_golden.py` and
`ros/lctk_target/test/test_target.py`). Every reference to those two deleted paths above has been
repointed in place.

Separately, and predating this phase: `ros/advanced_extrinsic_solver` no longer exists under that
name — it was renamed to `ros/lidar_to_camera_solver` in `ecba23c`, part of this same issue's own
Stage 2 (2026-08-18, above). `ros/extrinsic_solver_node` still exists but is superseded and pending
deletion per the diamond-frame plan; the maintained solver is `ros/lidar_to_camera_solver`. This
means several of this issue's still-`Open` sections — the header **Area**, the **Verified** line,
the **Problem** section's file list, and the "Original sketch" `<details>` block — describe the
diagnosis using a package name and file paths from before that rename. Inline notes have been added
at each so a reader is pointed at the current location rather than a dead path, but **the
substantive question the original diagnosis asked — does the code at the new path still reimplement
marker geometry independently, or does it now read the shared `lctk_target`/`calibration-target`
model — has not been re-audited here.** That re-audit is out of scope for this pointer-repair pass
(W5-E3's brief is reference repair, not re-verification of open findings), and is worth doing before
this issue is next touched substantively: `ros/lidar_to_camera_solver/lidar_to_camera_solver/board_geometry.py`
already imports `ValidatedTarget`/`load_target` from `lctk_target` rather than reimplementing corner
math, and a Stage-2-era `test_marker_corners_world_golden.py` already asserts against the shared
golden — both suggestive that some or all of what this issue describes may already be addressed, but
neither confirms it, and this update does not claim it. Status, severity and every substantive claim
above are left exactly as written; only the dangling path pointers were repaired.
