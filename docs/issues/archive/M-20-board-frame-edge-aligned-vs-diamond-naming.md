# M-20 · Board model's local axes run along edges while every accessor names a diamond → `initial_inplane_rotation_deg: 45.0` is mandatory

- **Severity:** Medium
- **Area:** `rust/hollow-board-config` / board detection
- **Status:** Fixed (2026-08-14) — Phase 1 (the Rust model, the detector node, the presets) landed 2026-08-13 and was field-validated on the two-LiDAR rig 2026-08-14; see [Field validation](#field-validation-2026-08-14) below. The camera-side remainder is **not** tracked by this issue — it is [H-11](./H-11-camera-solvers-stale-board-frame.md), Phase 2
- **Verified:** 2026-08-13 — source walkthrough; detection empirically fails at `0.0` and works at `45.0`. Fix validated 2026-08-14 against the `TWO_LIDAR_*` recordings on the real Seyond + Velodyne rig
- **Analysis:** [`2026-08-12-initial-board-pose-inplane-rotation.md`](../../superpowers/specs/2026-08-12-initial-board-pose-inplane-rotation.md)
- **Implementation spec:** [`2026-08-13-corner-aligned-board-frame.md`](../../superpowers/specs/2026-08-13-corner-aligned-board-frame.md)
- **Related:** [M-14](./M-14-corner-order-brittle.md), [M-17](../M-17-initial-pose-rewrite-unverified-bbox-path.md), [M-19](../M-19-debug-assertions-compiled-out.md), [L-21 (archived)](./L-21-find-correspondences-duplicated-tests-wrong-body.md), [H-11](./H-11-camera-solvers-stale-board-frame.md)

## Problem

`BoardModel`'s local X/Y axes run along the board's **edges**, with the origin at `bottom_corner`. But
every accessor name — `top_corner`, `bottom_corner`, `left_corner`, `right_corner`, and the three hole
centres — describes a **diamond**, in which the axes run corner to corner. Decomposed onto the
diagonal basis the naming is exactly self-consistent: top and bottom lie purely on one diagonal, left
and right purely on the other, and all three holes sit at radius `hole_center_shift · √2`.

The board is physically hung as a diamond. The two conventions are 45° apart, and
`initial_inplane_rotation_deg: 45.0` has been bridging the gap.

**All rigs in this repo are diamond-mounted**, so this is a convention bug, not a per-rig mounting
parameter. Stance (normalised max diagonal-vs-up alignment; ≈1.0 corner-standing, ≈0.71 edge-aligned)
computed over 25 golden fixtures spanning all five sample datasets: **0.9986–1.0000**. Confirmed
independently by pre-gate overlay renders for both recorded rigs.

## Impact

- Both rig presets carry `45.0`; `board_detector.json5` ships **`0.0`**, so `sample_data.yaml` and
  `vehicle.yaml` run a 45°-off ICP seed. Detection empirically fails at `0.0`. This is the concrete
  case [M-17](../M-17-initial-pose-rewrite-unverified-bbox-path.md) is tracking.
- ICP cannot recover: 45° is the exact saddle between two of the square's four 90°-symmetric
  attractors, and board-interior correspondences carry **zero** in-plane information — only
  boundary-clamped and hole-rim points constrain in-plane pose.
- No configuration comment, log line, or doc tells an operator the parameter exists or that `45.0` is
  the only working value.
- The `bbox_free` detector already computes a correct diamond-oriented pose and **discards** it,
  forwarding only its point set; the node then re-derives an edge-aligned pose from the plane normal
  plus this constant.

## History

Before commit `162a28e` the seed was `Ry(−90°)·Rz(−45°)` hard-coded, whose `(1,1)` diagonal is exactly
world-up. That commit removed the `−45°` and re-exposed it as this config parameter, documented only as
correcting "a fixed rotational bias visible in RViz".

## Fix

Redefine the canonical frame so the in-plane axes run along the diagonals and the origin sits at the
plate centre — see the implementation spec. `initial_inplane_rotation_deg` then becomes `0.0` for every
supported rig and survives only as a genuine escape hatch.

Phased: Phase 1 is the Rust model, the detector node, configs, and a frame-convention tag that makes
the phase boundary loud. Phase 2 — the two camera-side solver reimplementations, their tag check, and
the saved-file format bump — is deferred because the available recordings contain no camera stream,
and is tracked as [H-11](./H-11-camera-solvers-stale-board-frame.md).

## Phase 1 status (2026-08-13)

**Landed** in `rust/hollow-board-config` and the shipped configs:

- The canonical frame is corner-aligned: origin at the plate **centre**, `+Z` the normal
  (unchanged), `+Y` toward the top corner, `+X = Y × Z` toward the left corner. The plate is the
  diamond `|x| + |y| ≤ R` with `R = board_width/√2`; hole centres sit at `hole_center_shift·√2` from
  the centre, at `(+d,0)`, `(−d,0)`, `(0,+d)`. The module now carries the convention as a
  module-level doc comment, including the two "looks like a mistake and is not" notes (the left
  corner appearing on an observer's right; Z rather than X as the normal).
- The componentwise clamp boundary projection is replaced by a true L¹-ball projection.
- The 51 rotation-invariant `debug_assert!`s are deleted and replaced by real tests —
  `tests/board_frame.rs`, `tests/boundary_projection.rs`, `tests/marker_layout_golden.rs` (a
  world-coordinate golden keyed by ArUco marker id, with an independent Python generator in
  `tests/fixtures/`). See [M-19](../M-19-debug-assertions-compiled-out.md).
- `MarkerPaperPlacement` (defined in `rust/aruco-config`, re-exported from `hollow_board_config`)
  states where the printed sheet sits on the plate, as offsets along the plate's diagonals, and is
  now an explicit optional `paper_placement` field in `aruco_pattern.json5`. Its shipped values
  are the **measured** placement of this repository's board — the sheet in the plate's lower
  quarter, its top corner at the plate centre — confirmed with the hardware owners on 2026-08-14.
- `BoardModel::marker_pose` is deleted: its rotation would now disagree with the physical paper by
  45°, and it had no callers.
- `find_correspondences` is collapsed to one shared per-point body behind two thin feature-gated
  wrappers — [L-21](./L-21-find-correspondences-duplicated-tests-wrong-body.md), now fixed.
- `ros/lidar_board_detector`: the initial pose's translation is now the plane-inlier centroid
  directly (the plate centre *is* the frame origin, so the old corner offset — and the board-width
  argument that computed it — are gone); the RViz board outline is a closed line strip through the
  four corner accessors, so the picture cannot drift from the maths; and the node publishes its
  frame convention as `corner_aligned_plate_center_v1`, latched (transient-local) on
  `/lctk/board_frame_convention`, for Phase 2's consumers to check.
- All three board-detector configs carry `initial_inplane_rotation_deg: 0.0`, with a comment saying
  plainly that this is the correct value and not a tuning dial. Documented in the book under
  [Configuration](../../../book/src/user-guide/configuration.md) and
  [Architecture](../../../book/src/developer-guide/architecture.md).

## Field validation (2026-08-14)

Phase 1 was run against the real two-LiDAR rig (Seyond + Velodyne VLP-32C) using the
`ros/lctk_sample_data/bags/TWO_LIDAR_*` recordings. Reported by the operator who owns the hardware:

- **The board's green `+Y` arrow points at the physically up-most corner of the plate.** This is the
  decisive observation. It is the *only* thing that separates a `+45°` conjugation from a `−45°`
  one, which would produce a geometrically identical diamond with the corner labels rotated a
  quarter turn — the spec's one stated residual risk, the "silent quarter-turn". That risk is now
  discharged by observation, not by argument.
- The RViz board outline traces the actual plate corners and coincides with the LiDAR returns from
  the real board.
- Detection works with `initial_inplane_rotation_deg: 0.0` on **both** rigs, and the before/after
  behaviour is indistinguishable — the magic `45.0` is genuinely gone, not merely relabelled.
- The LiDAR-to-LiDAR extrinsic publishes on
  `/calibration/top_lidar_front_lidar/lidar_to_lidar_transform`, and the resulting Seyond/Velodyne
  relative poses look correct against the vehicle.
- Separately confirmed by the team that physically placed it: **the ArUco sheet really does sit in
  the plate's lower quarter, with its top corner at the plate centre.** The shipped
  `paper_placement` values are therefore a measurement of this board, not a stand-in for one. The
  earlier notes calling them unmeasured were wrong and have been corrected in
  `rust/aruco-config/src/multi_aruco.rs`, `ros/lctk_launch/config/aruco/aruco_pattern.json5`, and
  the book's [Configuration](../../../book/src/user-guide/configuration.md) page. A
  differently-built board still needs its own measurement.

**What this run did not cover**, and what this issue therefore does *not* close:

- **Nothing camera-side.** The `TWO_LIDAR_*` bags carry no camera stream. The two Python solvers
  still build marker geometry in the old edge-aligned frame, so the LiDAR-camera path remains
  actively wrong with half the error silent. That is
  [H-11](./H-11-camera-solvers-stale-board-frame.md), Phase 2, tracked separately — closing M-20
  does not imply the camera path is sound.
- **The crop-box (`bbox`) detection path**, which was not exercised or measured by this run —
  [M-17](../M-17-initial-pose-rewrite-unverified-bbox-path.md).

## Notes

Why this went unnoticed: `hollow-board-config`'s 51 `debug_assert!`s are the mechanism that should
have caught it, and they are both compiled out of every sanctioned build and rotation-invariant. See
[M-19](../M-19-debug-assertions-compiled-out.md).
