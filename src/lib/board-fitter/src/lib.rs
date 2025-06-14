//! Board Fitter Detector Library
//!
//! This library provides diamond-oriented square calibration board detection
//! in point cloud data, with support for circular hole pattern matching.
//!
//! ## Features
//!
//! - **Diamond-oriented board detection**: Specialized for 45° rotated square boards
//! - **Asymmetric hole patterns**: Support for orientation determination using different hole sizes
//! - **Multi-board tracking**: Kalman filter-based motion prediction and tracking
//! - **Adaptive ROI management**: Efficient processing with region-of-interest focusing
//! - **Robust algorithms**: RANSAC plane detection, PCA-based square fitting, circle fitting
//! - **Real-time performance**: Optimized for real-time processing with configurable timeouts
//!
//! ## Quick Start
//!
//! ```rust
//! use board_fitter::{BoardDetector, BoardDetectorBuilder, PointCloud};
//! use board_fitter_config::{BoardConfig, Point2D, SquareBoard};
//! use measurements::Length;
//! use nalgebra::Point3;
//!
//! // Create board configuration with holes for diamond detection
//! let mut board = SquareBoard::new(Length::from_meters(1.0));
//! board.add_hole(
//!     Length::from_meters(0.1),
//!     Point2D {
//!         x: Length::from_meters(0.0),
//!         y: Length::from_meters(0.5),
//!     },
//!     Some("top_hole".to_string()),
//! );
//! board.add_hole(
//!     Length::from_meters(0.05),
//!     Point2D {
//!         x: Length::from_meters(-0.5),
//!         y: Length::from_meters(0.0),
//!     },
//!     Some("left_hole".to_string()),
//! );
//!
//! let config = BoardConfig {
//!     board,
//!     detection: None,
//!     metadata: None,
//! };
//!
//! // Create detector
//! let mut detector = BoardDetector::new(board_fitter::DetectionConfig::new_with_default(config));
//!
//! // Create point cloud
//! let points = vec![Point3::new(0.0, 0.0, 1.0)];
//! let point_cloud = PointCloud::new(points, "sensor_frame".to_string());
//!
//! // Detect boards
//! let result = detector.detect(&point_cloud)?;
//! println!("Found {} boards", result.detections.len());
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ## Advanced Usage
//!
//! ```rust
//! use board_fitter::BoardDetectorBuilder;
//! use board_fitter_config::{BoardConfig, Point2D, SquareBoard};
//! use measurements::Length;
//!
//! // Create board configuration with holes
//! let mut board = SquareBoard::new(Length::from_meters(1.0));
//! board.add_hole(
//!     Length::from_meters(0.1),
//!     Point2D {
//!         x: Length::from_meters(0.0),
//!         y: Length::from_meters(0.5),
//!     },
//!     Some("top_hole".to_string()),
//! );
//! board.add_hole(
//!     Length::from_meters(0.05),
//!     Point2D {
//!         x: Length::from_meters(-0.5),
//!         y: Length::from_meters(0.0),
//!     },
//!     Some("left_hole".to_string()),
//! );
//!
//! let config = BoardConfig {
//!     board,
//!     detection: None,
//!     metadata: None,
//! };
//!
//! // Use builder pattern for advanced configuration
//! let detector = BoardDetectorBuilder::new(config)
//!     .min_confidence(0.8)
//!     .max_detections(5)
//!     .timeout_ms(2000)
//!     .parallel_processing(true)
//!     .build()?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod debug;
pub mod detection;
pub mod diamond;
pub mod hole;
pub mod io;
pub mod plane;
pub mod roi;
pub mod tracking;
pub mod types;

pub use debug::{
    AlgorithmStats, DataCallback, DebugConfig, DebugConfigBuilder, DebugContext, DebugData,
    MetricsCallback, StageMetrics, TimingCallback,
};
pub use detection::{BoardDetector, BoardDetectorBuilder, DetectionConfig, DetectionResult};
pub use tracking::{BoardTracker, TrackedBoard, TrackingConfig};
pub use types::{
    BoardDetection, BoundingBox, DetectedHole, DetectedPlane, DetectionConfidence, PointCloud,
    ProcessingStats, RegionOfInterest, RoiType,
};
