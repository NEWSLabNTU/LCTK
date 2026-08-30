# H-16 · main's M-01 and M-12 fixes were not present in the Phase-8 solver

- **Severity:** High
- **Area:** solver correctness
- **Status:** Fixed (2026-08-30)
- **Found:** 2026-08-30, rebasing `feat/bbox-free-parity-validation` onto `main`

## Problem

Rebasing this branch onto `main` replayed 237 commits. Fixes that `main` had made to
code Phase 8 restructured could not carry across, because the code they patch no longer
exists in that shape. Two were real, and both are now ported.

### 1. M-01 — the published transform meant the opposite of its labels

`cv2.solvePnP` returns `(R, t)` with `p_cam = R @ p_lidar + t`, i.e. `T_camera<-lidar`.
A transform labelled `frame_id=lidar, child_frame_id=camera` means the *opposite* under
ROS TF: the camera's pose expressed in lidar coordinates. `lidar_to_camera_solver`
published the raw solve under those labels, so every tf2 consumer got the inverse of
what it asked for.

The branch was already inconsistent with itself about this, which is what made it
findable. The rebase brought in `main`'s post-M-01 `pointcloud_image_overlay`, which
inverts the topic back before `projectPoints` and says so in a comment referring to the
pre-M-01 behaviour — so the consumer expected TF semantics while the producer did not.

Fixed by inverting on publish. The overlay's projection is bit-for-bit unchanged; a tf2
consumer now gets the direction it asks for. The detection archive still stores the raw
`rvec`/`tvec`, deliberately: `lctk_autoware_export` consumes the archive and its
arithmetic is written and tested against the raw solve.

### 2. M-12 — pose-granularity outlier rejection was half-ported

`main`'s solver has two mechanisms. Phase 8 carried over one of them:

| mechanism | `main` | this branch, before this fix |
|---|---|---|
| LM refinement | `solvePnPRefineLM` / weighted `least_squares` | present, `detection_buffer.py` |
| pose-granularity outlier rejection | `_reject_outlier_poses` | **absent** |

The tracker's 🟢 on
[M-12](./M-12-no-robust-estimation-or-refinement.md) therefore overstated what shipped.
Without the gate, one bad placement corrupts the whole extrinsic and reports nothing: a
placement's corners share one rigid board transform, so when the pose is wrong every
corner is wrong *together* — one rigid misplacement observed N times, not N independent
draws. That correlated regime is the one least squares handles worst.

Ported as `reject_outlier_poses` / `pose_reprojection_rms` in `detection_buffer.py`,
keeping `main`'s robust `median + k * 1.4826 * MAD` threshold and both guards
(`floor_px`, so a tight clean buffer with near-zero MAD does not eat good data;
`min_keep`, so an under-constrained solve is never preferred to a contaminated one —
H-07). Rejection is a solve-time decision, not a deletion: the capture stays in the
buffer and the quality report's `n_frames` records the set actually solved over.

## What this issue got wrong when first filed

It also claimed **M-14** was unported. That was incorrect, and the correction is the
useful part of this record.

M-14 is "board origin corner picked by gravity". The original filing grepped
`ros/lidar_board_detector/src/main.rs`, found `posed.bottom_corner()`, and concluded the
branch still selected by gravity. That call site is **RViz marker drawing**, not pose
estimation. The real logic lives in `rust/calibration-target-detector/src/perforated.rs`,
which builds four quarter-turn hypotheses, runs ICP on each, ranks them by `avg_loss`,
and requires the winner to beat the runner-up by `min_hypothesis_loss_separation_m`.

That is M-14's idea — let the hole asymmetry, not gravity, decide the origin corner —
implemented more strictly than `main`'s version, which keeps gravity as a tie-break where
this one *rejects* an ambiguous quarter-turn outright. The branch had solved M-14
independently and its tracker row was simply stale.

The lesson: a grep for a symbol is not evidence about where a decision is made. Both
`main` and this branch had archived M-14 as fixed by different routes; the tracker row
here disagreed with the archive it pointed at.

## How this surfaced

`main`'s two test files travelled into `ros/lidar_to_camera_solver/test/` during the
rebase, because git followed the `advanced_extrinsic_solver` → `lidar_to_camera_solver`
rename. They imported `advanced_extrinsic_solver.main`, so they failed at *collection*
and took the whole Python suite down — 412 tests not run, because of two files.

## Fix

- `fix(H-16): port M-12's pose-granularity outlier rejection` — the gate, its wiring into
  `DetectionBuffer._derive` via a shared `_solve` helper so the retry re-runs the same
  SQPnP+LM path, and new `reject_outlier_poses` / `outlier_pose_mad_k` parameters.
- `fix(H-16): port M-01 — publish the extrinsic with ROS TF semantics`.

Both of `main`'s test files are ported onto the new API and pass (5 + 4 tests), plus 3
buffer-level tests covering the M-12 wiring, since the unit tests exercise only the free
function. Each was confirmed to fail with its fix deliberately reverted and pass when
restored. Python suite: 424 passed.
