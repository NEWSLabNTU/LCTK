# H-17 · The `solid_600` detector preset rejects every frame of real data

- **Severity:** High
- **Area:** lidar_board_detector / config/board/solid_600
- **Status:** Open
- **Found:** 2026-08-31, first run of the solid target against real sensor data
- **Data:** `~/Downloads/new_LCTK_board/` (`newtype_background`, `newtype_1`, `newtype_2`),
  ZED + VLP-32C + Seyond, from the 2026-08-12 capture

## Problem

The first real-data run of the 600 mm solid target produced **zero accepted board
detections in 369 frames**. The camera side works; the LiDAR side rejects everything, so
no LiDAR-camera pair is ever formed and no extrinsic is ever solved.

This is the first time `solid_600/*` has been run against a real LiDAR — Phase 8's W7-B
is still outstanding — so the presets were never tuned, only written.

## What is *not* wrong

Worth recording, because each was suspected and cleared by measurement:

- **Background subtraction works.** With warmup on the board-free `newtype_background`
  bag, the node's foreground is median 273, max 483 points/frame, matching an independent
  offline computation (median 313). The `BackgroundState` machine stops accumulating once
  `Ready`, so a static board is *not* absorbed into the background.
- **The board is separable.** Offline single-linkage clustering of the foreground finds a
  board-shaped cluster in most frames: 63–420 points, flatness RMS 0.012–0.030 for the
  clean ones, at 6–8 m range.
- **`patch_min_points: 60` is not the blocker.** No sampled frame's best cluster fell
  below it.
- **The anisotropic clustering was ported correctly.** `dbscan.rs` implements the
  range-scaled vertical widening (`anisotropic_scaled`) that the Python original uses.

## What is wrong

**1. `cluster_eps` is double the validated value.** The presets ship `0.30`; the Python
original that produced the 88.4%/100% Method-E result defaults to `0.15`
(`background_diff.py:26`). Since the anisotropic scaling already widens the *vertical*
tolerance with range, a doubled *horizontal* eps merges the board into whatever is behind
it. Measured effect of `0.30 -> 0.15`: "no candidate clusters survived foreground
extraction" fell from 520 frames to 315, a 39% reduction. Still not sufficient alone.

**2. The isolation gate is hostile to handheld capture.** With `isolation: true` and
`isolation_max_density: 0.3`, 66 frames failed specifically on "embedded clutter". A
handheld board always has a person inside the isolation band. `solid_600_handheld.yaml`
exists as a shipped example, so handheld is an intended capture mode, and the gate as
tuned contradicts it.

**3. `icp_min_inlier_points: 100` exceeds what the board yields.** The cleanest
board-only clusters measure 63–100 points at 6–8 m. The 1000 mm perforated plate returns
far more, which is where this number came from.

**4. The residual failures are shape gates.** After relaxing 1–3: 315 frames "no
candidate clusters survived", 54 "square fit residual exceeded
`square_icp_residual_max`". The merged board+holder clusters measure 0.7–1.5 m in extent
against a 0.6 m plate (0.849 m diagonal) with flatness 0.06–0.11 versus
`flatness_rms_max: 0.045` — correctly rejected, since they contain a person. The
board-only cluster is not surviving as a separate candidate.

## Why it is High

The solid target is undetectable as shipped. Every gate value in `solid_600/*` was
inherited from the 1000 mm perforated plate, which returns several times more points and
is mounted rather than handheld. Nothing in the repo would have caught this: the presets
have tests for *existence* (`test_target_presets.py`) but nothing exercises them against
data.

## Suggested fix

1. Adopt `cluster_eps: 0.15`, matching the value the offline result was measured at, for
   the `solid_600` presets. Consider it for `hollow_1000` too, since the same 0.30 appears
   there and was equally unvalidated.
2. Decide whether handheld capture is supported. If yes, `isolation` must be off or
   re-tuned for `solid_600`; if no, `solid_600_handheld.yaml` should go.
3. Lower `icp_min_inlier_points` for the solid target, from measurement rather than guess.
4. Then re-measure on `newtype_1` and `newtype_2` and record the accepted-frame rate,
   the way Method E's 88.4%/100% was recorded. Do not tune a gate below what the sensor
   can deliver — that is how C-04 made the detector silently accept nothing.

## Related

- [C-04](./archive/C-04-board-detector-gate-unreachable.md) — the same failure class: a
  gate set beyond what the data can satisfy, producing silence rather than an error.
## The marker-ID finding (camera side, separate from the above)

`solid_600_aruco_1_v1.json5` declares `marker_ids: [1]`. The physical board recorded on
2026-08-12 carries **id 24** — confirmed by scanning every predefined OpenCV dictionary
over 30 frames: the only marker present is id 24, DICT_5X5_*, in 25 of 30 frames. Against
the shipped manifest the locator reports "No ArUco markers detected" on every frame; with
id 24 it detects reliably and the solver receives `counts=(1, 0)`.

Shipped as a **new** manifest, `solid_600_aruco_24_v1.json5`, rather than by editing the
existing one. Editing it was tried and reverted: `marker_ids: [1]` is woven through the
identity goldens, the cross-language corner goldens and five test suites, so changing it
ripples widely — and only the *marker id* has been confirmed against a physical board.
The plate and paper dimensions in both manifests are unverified. That matters: a wrong
`paper_side` scales every 3D corner and biases the extrinsic silently, which is the same
shape of failure as everything else in this tracker.

**Open question for the operator:** does a board with marker id 1 physically exist, or was
that a placeholder? If it was, `solid_600_aruco_1_v1` should be retired and its goldens
regenerated in one deliberate change. Measure the printed marker and plate before either.
