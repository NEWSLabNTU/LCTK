//! Point cloud plane fitting.
//!
//! # Example
//!
//! ```rust
//! use plane_estimator::PlaneEstimator;
//! use sample_consensus::Estimator;
//!
//! let points = vec![
//!     [1.0, 0.0, 0.0],
//!     [1.0, 1.2, 2.4],
//!     [1.0, -0.9, 1.7],
//!     [1.0, 0.5, -1.8],
//!     [1.0, 4.4, 6.1],
//! ];
//!
//! let estimator = PlaneEstimator::new();
//! let plane = estimator.estimate(points.iter());
//!
//! if let Some(plane) = plane {
//!     println!("A plane is found with pose {}", plane.pose());
//! }
//! ```

pub use model::*;
mod model;

pub use estimator::*;
mod estimator;
