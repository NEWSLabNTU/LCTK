# H-17 · The `solid_600` detector preset rejects every frame of real data

- **Severity:** High
- **Area:** lidar_board_detector / config/board/solid_600
- **Status:** Open (root-caused; fix is a code change, see "Two candidate fixes")
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

## Root cause: the square-fit coverage gate is unreachable for this board and sensor

Measured 2026-08-31. This is the decisive finding; the gates in the previous section are
real but secondary, and relaxing them only moves the failure here.

`square_fit.rs::coverage_residual` returns `mean_outside / side + coverage_penalty`,
where `coverage_penalty` is the fraction of 40 perimeter bins (4 sides x 10) that hold no
point within `BAND_FRAC * side` = **3.6 cm** for a 600 mm board.

Computing that same residual over 30 real, planar board clusters from `newtype_1`:

| quantity | best | median |
|---|---|---|
| total residual | 0.436 | 0.684 |
| coverage penalty alone | 0.425 | 0.675 |

Against `square_icp_residual_max: 0.45`, **29 of 30 clusters are rejected.** The best
frame the sensor produced misses the gate by 0.014.

The residual is almost entirely coverage penalty: `mean_outside / side` contributes about
0.01, meaning the square model **fits the geometry well** -- the board really is a
600 mm square where it is sampled. What fails is the demand that points appear all the
way round the perimeter.

They cannot. A VLP-32C at 7-8 m samples anisotropically: roughly **2.8 cm between points
within a ring, but ~15 cm between rings**. A 600 mm plate is therefore crossed by only
about four rings. The two horizontal edges fall between rings and can never hold a point
within 3.6 cm, so ~half the bins are unfillable by construction.

An adaptive band was tried and rejected: widening it to the cloud's own median
nearest-neighbour spacing changes nothing, because that spacing (2.4-3.0 cm measured) is
the *horizontal* one and is already smaller than the fixed band. The quantity that
matters is the vertical ring gap.

### Why this is a C-04 repeat

[C-04](./archive/C-04-board-detector-gate-unreachable.md) set `icp_good_fit_threshold`
below the sensor's noise floor, so the detector silently accepted nothing. This is the
same shape with a different quantity: a gate placed beyond what the sensor can deliver,
producing silence rather than an error. Both were invisible because the detector reports
"no board selected" either way.

The general lesson the tracker keeps relearning: **a gate must be set from what the
sensor produces, not from what a clean model would produce.**

### Two candidate fixes

1. **Config only -- tried, and it does not work.** Raising
   `square_icp_residual_max` to 0.75 (above the 0.684 median measured offline) still
   accepted nothing: the node's own reported residuals are 0.752-0.950, median 0.838,
   with 52 frames still failing this gate. Even a threshold that accepted them would be
   meaningless, since 0.95 means almost no perimeter coverage is required at all.

   The gap between the node's residuals (best 0.752) and the same metric computed offline
   on hand-clustered board points (best 0.436) is a **second finding**: the detector's
   candidate formation is handing the square fit worse point sets than the data supports,
   i.e. clusters still carrying the holder or fragments. Candidate formation and the
   coverage metric both need work; fixing either alone is not enough.
2. **Code** -- make the coverage band anisotropic, mirroring what `dbscan.rs` already
   does for clustering (`anisotropic_scaled` widens the vertical tolerance with range).
   A bin should only be charged as a miss if a point could physically have landed in it.
   This is the principled fix and would also serve the perforated board at long range.
   It changes detection outcomes, so it will move the golden parity fixtures and must be
   sequenced with that in mind.

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
