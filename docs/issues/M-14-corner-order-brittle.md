# M-14 · Board origin-corner is disambiguated by gravity, and corner order is duplicated in two languages with no cross-check

- **Severity:** Medium
- **Area:** lidar_board_detector, hollow-board-config, advanced_extrinsic_solver
- **Status:** Open
- **Verified:** Yes (confirmed against live source, 2026-07-12)
- **Location:**
  - `ros/lidar_board_detector/src/main.rs:1341-1397` (post-ICP origin-corner fixup)
  - `rust/hollow-board-config/src/lib.rs:123-151` (`BoardModel::multi_marker_corners`)
  - `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py:1234-1288` (`_compute_multi_marker_corners`, a Python re-implementation of the above)

## Problem

Two separate brittle couplings, both of which produce a *silently* wrong extrinsic rather than
an error.

### 1. The board's 4-fold in-plane symmetry is broken by gravity

A square plate with a symmetric hole pattern has a 90° in-plane ambiguity that ICP cannot
resolve. After ICP converges, the detector picks the board origin as **whichever of the 4
corners has the lowest world z**, then rotates the frame by `90° × lowest_index` about the board
normal (`main.rs:1341-1397`). This is what makes the downstream ArUco-corner-to-board-frame
mapping line up at all.

It is an unstated assumption that the board is mounted so that exactly one corner is
unambiguously lowest. Near a 45° roll, or on a board mounted diamond-wise, or with a LiDAR
whose frame is not gravity-aligned, the wrong corner wins and the 3D corners rotate 90°
relative to the image corners. The solve then still "succeeds" — it just returns a transform
that is wrong by roughly a quarter turn about the board normal, blended against the true
solution by whatever poses were correct.

### 2. Corner order is defined twice, in two languages, and never verified

`BoardModel::multi_marker_corners` (Rust, `hollow-board-config/src/lib.rs:123-151`) emits each
marker's 4 corners in the order `[right, top, left, bottom]`, intended to line up with OpenCV
`detectMarkers`' `TL, TR, BR, BL`. `_compute_multi_marker_corners` (Python,
`advanced_extrinsic_solver/main.py:1234-1288`) reimplements the same layout independently.
Nothing checks that the two orders agree, and nothing checks that either agrees with OpenCV's.
A mismatch is a silent corner permutation — a 90°/180° error folded into the extrinsic.

## Failure scenario

Board is mounted at ~45° roll on a rig whose LiDAR frame is not gravity-aligned. Some poses
resolve to the correct origin corner, some to the neighbour. The concatenated correspondence
set is a mix of correct and quarter-turned object points, and the least-squares solve returns
something between them. Reprojection error is large but nothing reports it
([H-09](./H-09-no-extrinsic-quality-metric.md)), so the operator sees only a mysteriously bad
overlay.

## Suggested fix

- **Break the symmetry in the target, not in the code.** The 3-hole pattern *is* asymmetric
  (`hole_center_shift` offsets); use it. Score all 4 candidate in-plane rotations by ICP loss
  against the asymmetric hole layout and take the best, instead of using world-z. Keep gravity
  only as a tie-break.
- **Cross-validate the resolved orientation against the camera.** The ArUco IDs are distinct and
  their layout on the board is known, so the image already tells you the board's in-plane
  orientation unambiguously. Reject any pose where the LiDAR-derived and image-derived in-plane
  orientation disagree by more than a tolerance. This turns a silent 90° error into a rejected
  detection.
- **Delete the Python re-implementation.** Export the corner layout once (from the Rust
  `hollow-board-config`, or from the JSON5 config), and have the solver read it — or at least
  add a test that asserts the two implementations agree corner-for-corner.
- Once [H-09](./H-09-no-extrinsic-quality-metric.md) lands, a per-pose reprojection residual
  makes a permuted pose trivially detectable, and [M-12](./M-12-no-robust-estimation-or-refinement.md)
  can reject it.
