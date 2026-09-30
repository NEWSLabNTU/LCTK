# Architecture

LCTK combines sensor-facing ROS 2 nodes with libraries and services for target
detection, synchronized observations, calibration, review, and export. The codebase is
mixed Rust and Python; algorithms and application orchestration do not all live in one
language or one layer.

## Main data flow

```text
LiDAR point clouds ──> LiDAR target detector ──┐
                                               ├─> LiDAR-camera solver ──> estimate
Camera images ───────> ArUco detector ─────────┘

LiDAR point clouds ──> two target detectors ──> LiDAR-LiDAR solver ──> estimate
```

For LiDAR-camera calibration, the detector nodes publish observations of the same
Calibration Target. `lidar_to_camera_solver` consumes synchronized pairs, maintains a
Detection Buffer in manual or assisted operation, computes an estimate and
`lctk_quality` reports how the observed geometry constrains it. Assisted review is
served by the solver node. `lctk_autoware_export` consumes a saved Detection Archive;
it is a separate export tool, not part of the solve.

## Package responsibilities

- **`rust/calibration-target` and `ros/lctk_target`** load and validate Target
  Definitions and Target Identity. The target description is separate from
  sensor-specific Detector Tuning.
- **`rust/board-cluster-detector` and `rust/calibration-target-detector`** provide
  LiDAR point-cloud candidate selection and target pose estimation.
- **`ros/aruco_locator_node`** detects camera-side fiducial observations;
  **`ros/lidar_board_detector`** publishes LiDAR target observations.
- **`ros/lctk_launch`** parses session/configuration YAML, validates sensor and target
  settings, builds a `NodePlan`, and translates it into ROS launch actions. `NodePlan`
  is an ordered data value and does not contain launch action types.
- **`ros/lctk_sync`** owns synchronized detection-pair intake for the maintained
  solvers, including timing-window validation, replay recovery, and diagnostics.
- **`ros/lidar_to_camera_solver`** owns the multi-pose Detection Buffer, estimate,
  Quality Verdict, archive operations, assisted review, and transform publication.
  **`ros/lidar_to_lidar_solver`** estimates the relative transform from two LiDAR
  target-pose streams.
- **`ros/lctk_quality`** assesses geometric support; a degenerate Quality Verdict is
  distinct from a failed numerical solve.
- **`ros/lctk_autoware_export`** converts a validated LiDAR-camera archive to an
  Autoware sensor-kit calibration entry.

Other ROS packages provide interfaces, sample playback, overlays, assisted/manual
controls, and supporting utilities. See the repository's `AGENTS.md` for the current
package map and domain terminology.

## Rust and ROS 2 workspace boundary

The root Cargo workspace contains `rust/*` and selected Rust ROS packages. Python ROS
packages and launch-only packages are managed by colcon but are excluded from the
Cargo workspace. ROS interface bindings are generated during the ROS build. The
`ros/conflux` submodule provides the C++/Python synchronization dependency; the root
build selects the packages LCTK uses rather than building its separate Rust ROS 2
integration.

Use `just build` and `just test` for the complete workspace workflow. See
[Build System](./build-system.md) and [Testing](./testing.md).
