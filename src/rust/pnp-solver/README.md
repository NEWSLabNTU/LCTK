# pnp-solver

This library provides a Rust wrapper around `opencv::calib3d::solve_pnp()` for solving the Perspective-n-Point (PnP) problem.

## Overview

The PnP problem involves finding the position and orientation of a camera given:
- A set of 3D object points
- Their corresponding 2D image projections
- Camera intrinsic parameters

This library simplifies the process by providing a safe Rust interface to OpenCV's PnP solver.

## Usage

```rust
use pnp_solver::{PnpSolver, PnpMethod};
use sensor_msgs::msg::CameraInfo;
use opencv::core::{Point3d, Point2d};

// Create a PnP solver with camera info
let camera_info = CameraInfo { /* ... */ };
let solver = PnpSolver::new(&camera_info, PnpMethod::SQPNP);

// Define 3D-2D point correspondences
let point_pairs = vec![
    (Point3d::new(0.0, 0.0, 0.0), Point2d::new(100.0, 100.0)),
    (Point3d::new(1.0, 0.0, 0.0), Point2d::new(200.0, 100.0)),
    (Point3d::new(0.0, 1.0, 0.0), Point2d::new(100.0, 200.0)),
    // ... more points
];

// Solve for camera pose
if let Some(transform) = solver.solve(point_pairs) {
    println!("Camera pose: {:?}", transform);
}
```

## PnP Methods

The library supports multiple PnP solving algorithms:

- `ITERATIVE`: Iterative method based on Levenberg-Marquardt optimization
- `EPNP`: Efficient PnP method for n≥4 points
- `IPPE`: Infinitesimal Plane-based Pose Estimation (requires coplanar points)
- `SQPNP`: A non-iterative solution with better accuracy

## Dependencies

- OpenCV 4.x with calib3d module
- ROS 2 sensor_msgs for CameraInfo type

## Camera Info Format

The solver expects camera parameters in ROS CameraInfo format:
- `k`: 3x3 camera matrix as a 9-element array [fx, 0, cx, 0, fy, cy, 0, 0, 1]
- `d`: Distortion coefficients [k1, k2, p1, p2, k3, ...]

The library automatically handles conversion to OpenCV format.