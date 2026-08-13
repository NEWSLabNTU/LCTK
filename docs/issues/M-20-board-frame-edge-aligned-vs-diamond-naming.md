# M-20 · Board model's local axes run along edges while every accessor names a diamond → `initial_inplane_rotation_deg: 45.0` is mandatory

- **Severity:** Medium
- **Area:** `rust/hollow-board-config` / board detection
- **Status:** Open — analysed, specced, not implemented
- **Verified:** 2026-08-13 — source walkthrough; detection empirically fails at `0.0` and works at `45.0`
- **Analysis:** [`2026-08-12-initial-board-pose-inplane-rotation.md`](../superpowers/specs/2026-08-12-initial-board-pose-inplane-rotation.md)
- **Implementation spec:** [`2026-08-13-corner-aligned-board-frame.md`](../superpowers/specs/2026-08-13-corner-aligned-board-frame.md)
- **Related:** [M-14](./M-14-corner-order-brittle.md), [M-17](./M-17-initial-pose-rewrite-unverified-bbox-path.md), [M-19](./M-19-debug-assertions-compiled-out.md), [L-21](./L-21-find-correspondences-duplicated-tests-wrong-body.md)

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
  case [M-17](./M-17-initial-pose-rewrite-unverified-bbox-path.md) is tracking.
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
the phase boundary loud. Phase 2 is the two camera-side solver reimplementations and the saved-file
format bump, deferred because the available recordings contain no camera stream.

## Notes

Why this went unnoticed: `hollow-board-config`'s 51 `debug_assert!`s are the mechanism that should
have caught it, and they are both compiled out of every sanctioned build and rotation-invariant. See
[M-19](./M-19-debug-assertions-compiled-out.md).
