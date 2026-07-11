# M-05 · L2L solver reads board pose from the wrong message field

- **Severity:** Medium
- **Area:** lidar_to_lidar_solver
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/lidar_to_lidar_solver/lidar_to_lidar_solver/main.py:191-193` (`det.bbox.center`)
  - vs camera path: `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:414` / `advanced_extrinsic_solver/main.py:1180` (`results[0].pose.pose`)

## Problem

The L2L solver extracts the board pose from `det.bbox.center`, whereas both camera solvers read the board pose from `results[0].pose.pose`. If the `lidar_board_detector` populates pose in `results[0].pose` (as the camera path assumes), the L2L solver is reading an unset/zero `bbox.center` pose.

## Failure scenario

Running L2L calibration produces garbage output. Consistent with CLAUDE.md's "this pipeline is not yet tested" note.

## Suggested fix

Read the board pose from the same field the detector populates (`results[0].pose.pose`), matching the camera solvers. Add an integration test for the L2L path.
