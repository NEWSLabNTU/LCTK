# H-16 · main's M-01, M-12 and M-14 fixes are not present in the Phase-8 solver

- **Severity:** High
- **Area:** solver / detector correctness
- **Status:** Open
- **Found:** 2026-08-30, rebasing `feat/bbox-free-parity-validation` onto `main`

## Problem

Three fixes that landed on `main` patch code that Phase 8 restructured, so replaying this
branch onto `main` could not carry them. Each is a correctness fix, and each is now absent
from the code this branch actually runs.

### 1. M-14 — board origin corner from hole asymmetry (`main` commit `7c82909`)

`main` picks the board's origin corner from the three-hole asymmetry, keeping gravity only
as a tie-break, because "lowest corner" breaks under a 45° roll or a diamond-mounted board.
It patches `ros/lidar_board_detector/src/main.rs`, in the block this branch moved into
`TargetPoseEstimator`. The branch still selects by gravity (`posed.bottom_corner()`).

This branch's tracker already carries
[M-14](./M-14-corner-order-brittle.md) as 🟡, which is the right home for the port. Noted
here so the rebase does not read as having delivered it.

### 2. M-12 — pose-granularity outlier rejection (partially lost)

`main`'s `advanced_extrinsic_solver` has two mechanisms:

| mechanism | `main` | this branch |
|---|---|---|
| LM refinement | `least_squares(..., method="lm")` | present, `detection_buffer.py:549` |
| pose-granularity outlier rejection | `_reject_outlier_poses` | **absent** |

The rejection half exists nowhere in `ros/lidar_to_camera_solver/`. A pose's 16 corners
share one rigid `T_board`, so a bad pose makes all 16 outliers *together* — the regime
least-squares handles worst. Without the gate, one occluded or mis-solved board pose
corrupts the whole extrinsic.

The tracker currently lists
[M-12](./archive/M-12-no-robust-estimation-or-refinement.md) as 🟢, which overstates what
is implemented. It should be reopened, or narrowed to the refinement half.

### 3. M-01 — transform direction (`main` commit `ac1f1b9`)

`main` publishes the extrinsic with ROS TF semantics. This branch's tracker still carries
[M-01](./M-01-transform-direction-inverted.md) as 🟡 and the restructured solver carries no
marker for it. Confirm which convention `lidar_to_camera_solver` actually publishes before
assuming either state.

## How this surfaced

`main`'s two test files for the above travelled into `ros/lidar_to_camera_solver/test/`
during the rebase, because git followed the `advanced_extrinsic_solver` →
`lidar_to_camera_solver` rename. They import `advanced_extrinsic_solver.main` and call
`S._reject_outlier_poses`, neither of which exists here, so they failed at *collection* and
took the entire Python suite down with them — 412 tests not run, for two files.

They were removed rather than left broken. Their content is recoverable from `main`:

```bash
git show origin/main:ros/advanced_extrinsic_solver/test/test_pose_gating.py
git show origin/main:ros/advanced_extrinsic_solver/test/test_transform_direction.py
```

Port them alongside the features; they are the acceptance criteria for this issue.

## Why it is High

M-12's missing gate silently degrades every extrinsic solved from a buffer containing one
bad pose, and gives no signal that it did. M-14 mis-orients the board frame by a quarter
turn on rigs where gravity does not disambiguate. Both are the kind of wrong-but-plausible
output this tracker exists to prevent.

## Suggested fix

1. Port `_reject_outlier_poses` into `lidar_to_camera_solver`'s solve path, with
   `test_pose_gating.py` retargeted at `LidarToCameraSolver`.
2. Port M-14's candidate-loss origin selection into `calibration-target-detector`, against
   this branch's board-frame conventions (see
   [M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md)).
3. Establish which direction `lidar_to_camera_solver` publishes and settle M-01 either way.
