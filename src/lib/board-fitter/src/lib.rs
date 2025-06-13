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
//! use board_fitter_config::{Config, Point2D, SquareBoard};
//! use board_fitter_detector::{DiamondBoardDetector, PointCloud};
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
//! let config = Config {
//!     board,
//!     detection: None,
//!     metadata: None,
//! };
//!
//! // Create detector
//! let mut detector = DiamondBoardDetector::new(config)?;
//!
//! // Create point cloud
//! let points = vec![Point3::new(0.0, 0.0, 1.0)];
//! let point_cloud = PointCloud::new(points, "sensor_frame".to_string());
//!
//! // Detect boards
//! let detections = detector.detect(&point_cloud)?;
//! println!("Found {} boards", detections.len());
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ## Advanced Usage
//!
//! ```rust
//! use board_fitter_config::{Config, Point2D, SquareBoard};
//! use board_fitter_detector::DiamondBoardDetector;
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
//! let config = Config {
//!     board,
//!     detection: None,
//!     metadata: None,
//! };
//!
//! // Use builder pattern for advanced configuration
//! let detector = DiamondBoardDetector::builder()
//!     .min_confidence(0.8)
//!     .max_detections(5)
//!     .timeout_ms(2000)
//!     .parallel_processing(true)
//!     .build(config)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use anyhow::Result;
use board_fitter_config::Config;

pub mod detection;
pub mod diamond;
pub mod hole;
pub mod plane;
pub mod roi;
pub mod tracking;
pub mod types;

pub use detection::{BoardDetector, DetectionConfig, DetectionResult};
pub use tracking::{BoardTracker, TrackedBoard, TrackingConfig};
pub use types::{
    BoardDetection, BoundingBox, DetectedHole, DetectedPlane, DetectionConfidence, PointCloud,
    ProcessingStats, RegionOfInterest, RoiType,
};

/// Default detection timeout in milliseconds
pub const DEFAULT_DETECTION_TIMEOUT_MS: u64 = 1000;

/// Default maximum number of detections to return
pub const DEFAULT_MAX_DETECTIONS: usize = 10;

/// Default minimum confidence threshold for valid detections
pub const DEFAULT_MIN_CONFIDENCE: f64 = 0.5;

/// Main board fitter detector for diamond-oriented square boards
pub struct DiamondBoardDetector {
    config: Config,
    detector: Box<dyn BoardDetector>,
}

impl DiamondBoardDetector {
    /// Create a new diamond board detector with the given configuration
    pub fn new(config: Config) -> Result<Self> {
        // Validate configuration before creating detector
        Self::validate_config(&config)?;

        let detector = Box::new(detection::DiamondDetector::with_board_config(
            config.clone(),
        ));
        Ok(Self { config, detector })
    }

    /// Validate board configuration for diamond detection
    fn validate_config(config: &Config) -> Result<()> {
        let board = &config.board;

        // Check board size is reasonable
        let board_size = board.size.as_meters();
        if board_size < 0.1 || board_size > 10.0 {
            return Err(anyhow::anyhow!(
                "Board size must be between 0.1m and 10.0m, got: {}m",
                board_size
            ));
        }

        // Check minimum holes for diamond detection
        if board.holes.len() < 2 {
            return Err(anyhow::anyhow!(
                "Diamond board detection requires at least 2 holes for orientation determination, got: {}", 
                board.holes.len()
            ));
        }

        // Validate hole sizes are reasonable
        for hole in &board.holes {
            let radius = hole.radius.as_meters();
            if radius < 0.005 || radius > 0.5 {
                return Err(anyhow::anyhow!(
                    "Hole radius must be between 0.5cm and 50cm, got: {}m for hole {:?}",
                    radius,
                    hole.id
                ));
            }
        }

        Ok(())
    }

    /// Detect boards in the given point cloud
    pub fn detect(&mut self, point_cloud: &PointCloud) -> Result<Vec<BoardDetection>> {
        // Run the main detection pipeline through the detector
        let detection_result = self.detector.detect(point_cloud)?;

        // Extract the detected boards from the result
        Ok(detection_result.detections)
    }

    /// Get the current configuration
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Update the detector configuration
    pub fn update_config(&mut self, config: Config) -> Result<()> {
        // Validate new configuration
        Self::validate_config(&config)?;

        self.detector.update_config(config.clone())?;
        self.config = config;
        Ok(())
    }

    /// Get the last processing statistics
    pub fn last_stats(&self) -> &types::ProcessingStats {
        self.detector.last_stats()
    }

    /// Reset the detector internal state
    pub fn reset(&mut self) {
        self.detector.reset();
    }

    /// Detect boards and return detailed results with statistics
    pub fn detect_with_stats(&mut self, point_cloud: &PointCloud) -> Result<DetectionResult> {
        self.detector.detect(point_cloud)
    }

    /// Create a builder for configuring detection parameters
    pub fn builder() -> DiamondBoardDetectorBuilder {
        DiamondBoardDetectorBuilder::new()
    }
}

/// Builder for DiamondBoardDetector with fluent API
pub struct DiamondBoardDetectorBuilder {
    detection_config: DetectionConfig,
}

impl DiamondBoardDetectorBuilder {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self {
            detection_config: DetectionConfig::default(),
        }
    }

    /// Set the minimum confidence threshold
    pub fn min_confidence(mut self, confidence: f64) -> Self {
        self.detection_config.min_confidence = confidence;
        self
    }

    /// Set the maximum number of detections
    pub fn max_detections(mut self, max: usize) -> Self {
        self.detection_config.max_detections = max;
        self
    }

    /// Set the detection timeout
    pub fn timeout_ms(mut self, timeout: u64) -> Self {
        self.detection_config.timeout_ms = timeout;
        self
    }

    /// Enable or disable parallel processing
    pub fn parallel_processing(mut self, enabled: bool) -> Self {
        self.detection_config.parallel_processing = enabled;
        self
    }

    /// Build the detector with the given board configuration
    pub fn build(self, board_config: Config) -> Result<DiamondBoardDetector> {
        // Validate configuration before creating detector
        DiamondBoardDetector::validate_config(&board_config)?;

        let detector = Box::new(detection::DiamondDetector::new(
            board_config.clone(),
            self.detection_config,
        ));

        Ok(DiamondBoardDetector {
            config: board_config,
            detector,
        })
    }
}

impl Default for DiamondBoardDetectorBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use board_fitter_config::{CircleHole, Point2D, SquareBoard};
    use measurements::Length;

    fn create_test_config() -> Config {
        let mut board = SquareBoard::new(Length::from_meters(1.0));

        // Add three holes for diamond board
        board.add_hole(
            Length::from_meters(0.1),
            Point2D {
                x: Length::from_meters(0.0),
                y: Length::from_meters(0.5),
            },
            Some("top_hole".to_string()),
        );
        board.add_hole(
            Length::from_meters(0.05),
            Point2D {
                x: Length::from_meters(-0.5),
                y: Length::from_meters(0.0),
            },
            Some("left_hole".to_string()),
        );
        board.add_hole(
            Length::from_meters(0.05),
            Point2D {
                x: Length::from_meters(0.5),
                y: Length::from_meters(0.0),
            },
            Some("right_hole".to_string()),
        );

        Config {
            board,
            detection: None,
            metadata: None,
        }
    }

    #[test]
    fn test_detector_creation() {
        let config = create_test_config();
        let detector = DiamondBoardDetector::new(config);
        assert!(detector.is_ok());
    }

    #[test]
    fn test_detector_builder() {
        let config = create_test_config();
        let detector = DiamondBoardDetector::builder()
            .min_confidence(0.8)
            .max_detections(5)
            .timeout_ms(2000)
            .parallel_processing(true)
            .build(config);
        assert!(detector.is_ok());
    }

    #[test]
    fn test_empty_point_cloud_detection() {
        let config = create_test_config();
        let mut detector = DiamondBoardDetector::new(config).unwrap();
        let empty_cloud = PointCloud::new(Vec::new(), "test".to_string());

        let result = detector.detect(&empty_cloud);
        assert!(result.is_ok());
        assert!(result.unwrap().is_empty());
    }
}
