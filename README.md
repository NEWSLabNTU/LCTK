# LCTK

This is the repository for LiDAR and camera calibration tools and
supporting librariesx.

## Libraries

- [aruco-config](rust-lib/aruco-config/README.md) - Serializable types
  to describe ArUco patterns.

- [aruco-detector](rust-lib/aruco-detector/README.md) - ArUco board
  detector.

- [hollow-board-config](rust-lib/hollow-board-config/README.md) -
  Serializable types to describe hollow-board shapes.

- [hollow-board-detector](rust-lib/hollow-board-detector/README.md) -
  Locate a hollow-board inside a point cloud.

- [plane-estimator](rust-lib/plane-estimator/README.md) - Fit a plane
  against a point cloud.

- [pnp-solver](rust-lib/pnp-solver/README.md) - A Rust wrapper around
  OpenCV `solve_pnp`.

## Programs

- [aruco-generator](rust-bin/aruco-generator/README.md) - Generate
  ArUco board image according to the configuration described by
  [aruco-config](rust-lib/aruco-config/README.md).

