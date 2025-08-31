# Core Libraries

The core libraries are located in the `src/lib` directory and provide the fundamental algorithms and data structures used throughout LCTK.

## ArUco Marker Handling

### aruco-config
Defines the data structures for ArUco marker patterns, including board layouts and marker configurations.

### aruco-detector
Implements ArUco marker detection in images using OpenCV's ArUco module. Provides robust detection with configurable dictionaries and refinement strategies.

### aruco-generator
Generates ArUco marker board images for printing calibration targets. Supports various marker dictionaries and custom board layouts.

## Calibration Board Handling

### hollow-board-config
Defines data structures for hollow calibration boards used in LiDAR calibration. Includes specifications for board geometry and hole patterns.

### hollow-board-detector
Detects hollow calibration boards in point clouds using plane fitting and geometric pattern matching.

### board-fitter-config
Advanced board shape configurations supporting rectangles, circles, and polygons for flexible calibration scenarios.

### board-fitter
Advanced board detection using small_gicp library with SVD-based ICP refinement for precise board pose estimation.

## Point Cloud Processing

### plane-estimator
Implements RANSAC-based plane fitting algorithms for point cloud data. Used for detecting flat surfaces in calibration boards.

### small_gicp_rust
Rust wrapper for the small_gicp library, providing efficient point cloud registration using Generalized ICP algorithms.

## Calibration Algorithms

### pnp-solver
Solves the Perspective-n-Point (PnP) problem to determine object pose from 2D-3D correspondences. Supports multiple solving methods including SQPNP and iterative refinement.

## Utilities

### multi-stream-synchronizer
Synchronizes data from multiple sensor streams based on timestamps, ensuring temporally aligned sensor measurements for calibration.

### serde-types
Common serializable types used across the project for configuration and data exchange.