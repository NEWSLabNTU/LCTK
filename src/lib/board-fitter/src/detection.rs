//! Core detection traits and implementation

use anyhow::Result;
use board_fitter_config::Config;
use std::time::{Duration, Instant};

use crate::{
    debug::{stages, AlgorithmStats, DebugContext, DebugData, StageMetrics},
    types::{BoardDetection, PointCloud, ProcessingStage, ProcessingStats},
};

/// Main trait for board detection algorithms
pub trait BoardDetector: Send + Sync {
    /// Detect boards in a point cloud
    fn detect(&mut self, point_cloud: &PointCloud) -> Result<DetectionResult>;

    /// Get the current configuration
    fn config(&self) -> &Config;

    /// Update the detector configuration
    fn update_config(&mut self, config: Config) -> Result<()>;

    /// Get processing statistics from the last detection run
    fn last_stats(&self) -> &ProcessingStats;

    /// Reset internal state (useful for testing)
    fn reset(&mut self);
}

/// Result of a detection operation
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Detected boards
    pub detections: Vec<BoardDetection>,
    /// Processing statistics
    pub stats: ProcessingStats,
    /// Processing timestamp
    pub timestamp: Instant,
}

impl DetectionResult {
    /// Create a new detection result
    pub fn new(detections: Vec<BoardDetection>, stats: ProcessingStats) -> Self {
        Self {
            detections,
            stats,
            timestamp: Instant::now(),
        }
    }

    /// Get the number of detected boards
    pub fn count(&self) -> usize {
        self.detections.len()
    }

    /// Get detections above a confidence threshold
    pub fn high_confidence_detections(&self, threshold: f64) -> Vec<&BoardDetection> {
        self.detections
            .iter()
            .filter(|d| d.confidence.above_threshold(threshold))
            .collect()
    }

    /// Get the best detection (highest confidence)
    pub fn best_detection(&self) -> Option<&BoardDetection> {
        self.detections
            .iter()
            .max_by(|a, b| a.confidence.partial_cmp(&b.confidence).unwrap())
    }
}

/// Configuration for detection algorithms
#[derive(Debug, Clone)]
pub struct DetectionConfig {
    /// Minimum confidence threshold for valid detections
    pub min_confidence: f64,
    /// Maximum number of detections to return
    pub max_detections: usize,
    /// Enable parallel processing
    pub parallel_processing: bool,
    /// Timeout for detection operations
    pub timeout_ms: u64,
}

impl Default for DetectionConfig {
    fn default() -> Self {
        Self {
            min_confidence: 0.5,
            max_detections: 10,
            parallel_processing: true,
            timeout_ms: 1000,
        }
    }
}

/// Diamond board detector implementation
pub struct DiamondDetector {
    config: Config,
    detection_config: DetectionConfig,
    stats: ProcessingStats,
    debug_ctx: Option<DebugContext>,
}

impl DiamondDetector {
    /// Create a new diamond detector
    pub fn new(config: Config, detection_config: DetectionConfig) -> Self {
        Self {
            config,
            detection_config,
            stats: ProcessingStats::new(),
            debug_ctx: None,
        }
    }

    /// Create with default detection configuration
    pub fn with_board_config(config: Config) -> Self {
        Self::new(config, DetectionConfig::default())
    }

    /// Set debug context for instrumentation
    pub fn with_debug_context(mut self, debug_ctx: DebugContext) -> Self {
        self.debug_ctx = Some(debug_ctx);
        self
    }
}

impl BoardDetector for DiamondDetector {
    fn detect(&mut self, point_cloud: &PointCloud) -> Result<DetectionResult> {
        let start_time = Instant::now();
        let timeout = Duration::from_millis(self.detection_config.timeout_ms);

        // Initialize statistics and debug context
        self.stats = ProcessingStats::new();
        self.stats.points_processed = point_cloud.len();

        if let Some(debug_ctx) = &mut self.debug_ctx {
            debug_ctx.start_stage(stages::PREPROCESSING);

            // Log initial point cloud statistics
            let mut metadata = std::collections::HashMap::new();
            metadata.insert("input_points".to_string(), point_cloud.len().to_string());
            metadata.insert(
                "has_intensity".to_string(),
                point_cloud.intensities.is_some().to_string(),
            );
            metadata.insert(
                "has_colors".to_string(),
                point_cloud.colors.is_some().to_string(),
            );
            metadata.insert("frame_id".to_string(), point_cloud.frame_id.clone());

            let debug_data = DebugData::PointCloud {
                cloud: point_cloud.clone(),
                metadata,
            };
            debug_ctx.emit_data(stages::PREPROCESSING, &debug_data);
            debug_ctx.emit_point_cloud(stages::PREPROCESSING, point_cloud);

            let preprocess_metrics = StageMetrics::new(
                point_cloud.len(),
                point_cloud.len(),
                Duration::from_nanos(1000), // Minimal preprocessing time
            );
            debug_ctx.emit_metrics(stages::PREPROCESSING, &preprocess_metrics);
            debug_ctx.end_stage(stages::PREPROCESSING);
        }

        // Early return for empty point clouds
        if point_cloud.is_empty() {
            if let Some(debug_ctx) = &self.debug_ctx {
                println!("DEBUG: Empty point cloud provided, returning no detections");
            }
            let detection_time = start_time.elapsed();
            self.stats
                .add_time(ProcessingStage::Detection, detection_time);
            return Ok(DetectionResult::new(Vec::new(), self.stats.clone()));
        }

        // STEP 1: Plane Detection with Debug Instrumentation
        if let Some(debug_ctx) = &mut self.debug_ctx {
            debug_ctx.start_stage(stages::PLANE_DETECTION);
            println!(
                "DEBUG: Starting plane detection on {} points",
                point_cloud.len()
            );
        }

        let plane_start = Instant::now();
        let mut plane_detector = crate::plane::RansacPlaneDetector::default();
        let detected_planes = plane_detector.detect_planes(point_cloud)?;
        let plane_time = plane_start.elapsed();
        self.stats.planes_detected = detected_planes.len();

        if let Some(debug_ctx) = &mut self.debug_ctx {
            println!(
                "DEBUG: Plane detection found {} planes in {:.2}ms",
                detected_planes.len(),
                plane_time.as_secs_f64() * 1000.0
            );

            // Emit plane detection debug data
            let mut metadata = std::collections::HashMap::new();
            metadata.insert(
                "planes_found".to_string(),
                detected_planes.len().to_string(),
            );
            metadata.insert(
                "processing_time_ms".to_string(),
                (plane_time.as_secs_f64() * 1000.0).to_string(),
            );

            let debug_data = DebugData::PlaneData {
                planes: detected_planes.clone(),
                inlier_counts: detected_planes.iter().map(|p| p.inliers.len()).collect(),
                quality_scores: detected_planes.iter().map(|p| p.score).collect(),
                metadata,
            };
            debug_ctx.emit_data(stages::PLANE_DETECTION, &debug_data);

            let plane_metrics =
                StageMetrics::new(point_cloud.len(), detected_planes.len(), plane_time);
            debug_ctx.emit_metrics(stages::PLANE_DETECTION, &plane_metrics);
            debug_ctx.end_stage(stages::PLANE_DETECTION);
        }

        // Check timeout after plane detection
        if start_time.elapsed() > timeout {
            if let Some(debug_ctx) = &self.debug_ctx {
                println!("DEBUG: Timeout exceeded during plane detection");
            }
            return Err(anyhow::anyhow!(
                "Detection timeout exceeded during plane detection"
            ));
        }

        if detected_planes.is_empty() {
            if let Some(debug_ctx) = &self.debug_ctx {
                println!("DEBUG: No planes detected, returning no detections");
            }
            let detection_time = start_time.elapsed();
            self.stats
                .add_time(ProcessingStage::Detection, detection_time);
            return Ok(DetectionResult::new(Vec::new(), self.stats.clone()));
        }

        // STEP 2: Plane Filtering with Debug
        let plane_filter = crate::plane::PlaneFilter::for_diamond_boards();

        if let Some(debug_ctx) = &self.debug_ctx {
            println!("DEBUG: Plane filtering criteria:");
            println!("  Min Z-angle: {:.1}°, Max Z-angle: {:.1}°", 30.0, 150.0);
            println!("  Min dimensions: 0.5m x 0.5m, Max dimensions: 2.0m x 2.0m");

            for (i, plane) in detected_planes.iter().enumerate() {
                let z_axis = nalgebra::Vector3::new(0.0, 0.0, 1.0);
                let angle_rad = plane.normal.angle(&z_axis);
                let angle_deg = angle_rad.to_degrees();
                let size = plane.bbox.size();

                println!(
                    "  Plane {}: normal={:.3?}, angle_with_z={:.1}°, size={:.2}x{:.2}m, inliers={}",
                    i,
                    plane.normal,
                    angle_deg,
                    size.x,
                    size.y,
                    plane.inliers.len()
                );

                let angle_ok =
                    angle_rad >= 30.0_f64.to_radians() && angle_rad <= 150.0_f64.to_radians();
                let size_ok = size.x >= 0.5 && size.y >= 0.5 && size.x <= 2.0 && size.y <= 2.0;

                println!(
                    "    Angle check: {} ({}°), Size check: {} ({:.2}x{:.2}m)",
                    if angle_ok { "PASS" } else { "FAIL" },
                    angle_deg,
                    if size_ok { "PASS" } else { "FAIL" },
                    size.x,
                    size.y
                );
            }
        }

        let filtered_planes = plane_filter.filter_planes(detected_planes);

        if let Some(debug_ctx) = &self.debug_ctx {
            println!(
                "DEBUG: Filtered to {} suitable planes for diamond boards",
                filtered_planes.len()
            );
        }

        if filtered_planes.is_empty() {
            if let Some(debug_ctx) = &self.debug_ctx {
                println!("DEBUG: No suitable planes for diamond boards, returning no detections");
            }
            let detection_time = start_time.elapsed();
            self.stats
                .add_time(ProcessingStage::Detection, detection_time);
            return Ok(DetectionResult::new(Vec::new(), self.stats.clone()));
        }

        let mut board_detections = Vec::new();

        // STEP 3: Diamond Fitting with Debug Instrumentation
        if let Some(debug_ctx) = &mut self.debug_ctx {
            debug_ctx.start_stage(stages::DIAMOND_FITTING);
            println!(
                "DEBUG: Starting diamond fitting on {} planes",
                filtered_planes.len()
            );
        }

        let diamond_start = Instant::now();
        let diamond_fitter =
            crate::diamond::DiamondSquareFitter::from_board_config(&self.config.board);
        let mut squares_fitted = 0;

        for (plane_idx, plane) in filtered_planes.iter().enumerate() {
            if let Some(debug_ctx) = &self.debug_ctx {
                println!(
                    "DEBUG: Processing plane {} with {} inliers",
                    plane_idx,
                    plane.inliers.len()
                );
            }

            // Check timeout during processing
            if start_time.elapsed() > timeout {
                if let Some(debug_ctx) = &self.debug_ctx {
                    println!("DEBUG: Timeout exceeded during square fitting");
                }
                return Err(anyhow::anyhow!(
                    "Detection timeout exceeded during square fitting"
                ));
            }

            if let Some(diamond_square) = diamond_fitter.fit_square(point_cloud, plane)? {
                squares_fitted += 1;
                if let Some(debug_ctx) = &self.debug_ctx {
                    println!(
                        "DEBUG: Successfully fitted diamond square on plane {}",
                        plane_idx
                    );
                }

                // STEP 4: Hole Detection with Debug
                if let Some(debug_ctx) = &mut self.debug_ctx {
                    debug_ctx.start_stage(stages::HOLE_DETECTION);
                    println!("DEBUG: Starting hole detection in fitted square");
                }

                let hole_start = Instant::now();
                let hole_detector = crate::hole::HoleDetector::default();
                let detected_holes =
                    hole_detector.detect_holes_in_square(point_cloud, &diamond_square)?;
                let hole_time = hole_start.elapsed();

                if let Some(debug_ctx) = &mut self.debug_ctx {
                    println!(
                        "DEBUG: Found {} holes in {:.2}ms",
                        detected_holes.len(),
                        hole_time.as_secs_f64() * 1000.0
                    );

                    // Emit hole detection debug data
                    let mut metadata = std::collections::HashMap::new();
                    metadata.insert("holes_found".to_string(), detected_holes.len().to_string());
                    metadata.insert(
                        "processing_time_ms".to_string(),
                        (hole_time.as_secs_f64() * 1000.0).to_string(),
                    );
                    metadata.insert("plane_index".to_string(), plane_idx.to_string());

                    let debug_data = DebugData::CircleData {
                        holes: detected_holes.clone(),
                        fitting_residuals: vec![0.0; detected_holes.len()], // TODO: Get actual residuals
                        iteration_counts: vec![0; detected_holes.len()], // TODO: Get actual iteration counts
                        metadata,
                    };
                    debug_ctx.emit_data(stages::HOLE_DETECTION, &debug_data);

                    let hole_metrics =
                        StageMetrics::new(plane.inliers.len(), detected_holes.len(), hole_time);
                    debug_ctx.emit_metrics(stages::HOLE_DETECTION, &hole_metrics);
                    debug_ctx.end_stage(stages::HOLE_DETECTION);
                }

                // Check timeout during hole detection
                if start_time.elapsed() > timeout {
                    if let Some(debug_ctx) = &self.debug_ctx {
                        println!("DEBUG: Timeout exceeded during hole detection");
                    }
                    return Err(anyhow::anyhow!(
                        "Detection timeout exceeded during hole detection"
                    ));
                }

                // STEP 5: Pattern Matching with Debug
                let hole_pattern = &self.config.board.holes;
                let hole_match = hole_detector.match_hole_pattern(&detected_holes, hole_pattern)?;

                if let Some(debug_ctx) = &self.debug_ctx {
                    println!(
                        "DEBUG: Pattern matching found {} hole matches",
                        hole_match.matches.len()
                    );
                }

                // STEP 6: Pattern Analysis with Debug
                let pattern_analyzer =
                    crate::hole::AsymmetricPatternAnalyzer::for_diamond_board(&self.config.board);
                let pattern_analysis = pattern_analyzer.analyze_pattern(&hole_match);

                if let Some(debug_ctx) = &self.debug_ctx {
                    println!(
                        "DEBUG: Pattern analysis - orientation_determined: {}, confidence: {:.3}",
                        pattern_analysis.orientation_determined, pattern_analysis.confidence
                    );
                }

                // STEP 7: Create board detection if pattern is acceptable
                if pattern_analysis.orientation_determined && pattern_analysis.confidence > 0.5 {
                    let confidence =
                        crate::types::DetectionConfidence::new(pattern_analysis.confidence);
                    let mut board_detection = diamond_square
                        .to_board_detection_with_points(confidence, plane.inliers.clone());

                    // Add detected holes to the board detection
                    board_detection.holes = hole_match
                        .matches
                        .into_values()
                        .map(|(hole, _error)| hole)
                        .collect();

                    board_detections.push(board_detection);

                    if let Some(debug_ctx) = &self.debug_ctx {
                        println!(
                            "DEBUG: Created board detection with confidence {:.3}",
                            pattern_analysis.confidence
                        );
                    }
                } else {
                    if let Some(debug_ctx) = &self.debug_ctx {
                        println!("DEBUG: Pattern rejected - orientation_determined: {}, confidence: {:.3}", 
                            pattern_analysis.orientation_determined, pattern_analysis.confidence);
                    }
                }
            } else {
                if let Some(debug_ctx) = &self.debug_ctx {
                    println!("DEBUG: Failed to fit diamond square on plane {}", plane_idx);
                }
            }

            // Early exit if we have enough detections or approaching timeout
            if board_detections.len() >= self.detection_config.max_detections
                || start_time.elapsed() > timeout * 3 / 4
            {
                if let Some(debug_ctx) = &self.debug_ctx {
                    println!(
                        "DEBUG: Early exit - detections: {}, timeout approaching",
                        board_detections.len()
                    );
                }
                break;
            }
        }

        let diamond_time = diamond_start.elapsed();
        if let Some(debug_ctx) = &mut self.debug_ctx {
            println!(
                "DEBUG: Diamond fitting completed - {} squares fitted from {} planes in {:.2}ms",
                squares_fitted,
                filtered_planes.len(),
                diamond_time.as_secs_f64() * 1000.0
            );

            let diamond_metrics =
                StageMetrics::new(filtered_planes.len(), squares_fitted, diamond_time);
            debug_ctx.emit_metrics(stages::DIAMOND_FITTING, &diamond_metrics);
            debug_ctx.end_stage(stages::DIAMOND_FITTING);
        }

        // STEP 8: Validation with Debug
        if let Some(debug_ctx) = &mut self.debug_ctx {
            debug_ctx.start_stage(stages::VALIDATION);
            println!(
                "DEBUG: Validating {} board detections",
                board_detections.len()
            );
        }

        let validation_start = Instant::now();
        let validator = DetectionValidator::from_config(&self.config.board);
        let input_detections_count = board_detections.len();
        let validated_detections = validator.validate_detections(board_detections);
        let validation_time = validation_start.elapsed();

        if let Some(debug_ctx) = &mut self.debug_ctx {
            println!(
                "DEBUG: Validation completed - {} detections validated in {:.2}ms",
                validated_detections.len(),
                validation_time.as_secs_f64() * 1000.0
            );

            // Emit final detection results
            let mut metadata = std::collections::HashMap::new();
            metadata.insert(
                "final_detections".to_string(),
                validated_detections.len().to_string(),
            );
            metadata.insert(
                "validation_time_ms".to_string(),
                (validation_time.as_secs_f64() * 1000.0).to_string(),
            );

            let debug_data = DebugData::DetectionResult {
                detections: validated_detections.clone(),
                confidence_scores: validated_detections
                    .iter()
                    .map(|d| d.confidence.value())
                    .collect(),
                metadata,
            };
            debug_ctx.emit_data(stages::VALIDATION, &debug_data);

            let validation_metrics = StageMetrics::new(
                input_detections_count,
                validated_detections.len(),
                validation_time,
            );
            debug_ctx.emit_metrics(stages::VALIDATION, &validation_metrics);
            debug_ctx.end_stage(stages::VALIDATION);
        }

        // Update final statistics
        let detection_time = start_time.elapsed();
        self.stats
            .add_time(ProcessingStage::Detection, detection_time);
        self.stats.boards_detected = validated_detections.len();

        if let Some(debug_ctx) = &self.debug_ctx {
            println!(
                "DEBUG: Detection pipeline completed - {} detections in {:.2}ms",
                validated_detections.len(),
                detection_time.as_secs_f64() * 1000.0
            );
        }

        Ok(DetectionResult::new(
            validated_detections,
            self.stats.clone(),
        ))
    }

    fn config(&self) -> &Config {
        &self.config
    }

    fn update_config(&mut self, config: Config) -> Result<()> {
        // Validate configuration compatibility
        if config.board.holes.is_empty() {
            return Err(anyhow::anyhow!(
                "Board configuration must have at least one hole for detection"
            ));
        }

        // Check for asymmetric pattern requirement
        if config.board.holes.len() < 3 {
            return Err(anyhow::anyhow!(
                "Diamond board detection requires at least 3 holes for orientation determination"
            ));
        }

        // Validate board size is reasonable
        let board_size = config.board.size.as_meters();
        if board_size < 0.1 || board_size > 10.0 {
            return Err(anyhow::anyhow!(
                "Board size must be between 0.1m and 10.0m, got: {}m",
                board_size
            ));
        }

        self.config = config;
        Ok(())
    }

    fn last_stats(&self) -> &ProcessingStats {
        &self.stats
    }

    fn reset(&mut self) {
        self.stats = ProcessingStats::new();
    }
}

/// Validator for detection results
pub struct DetectionValidator {
    min_board_size: f64,
    max_board_size: f64,
    min_hole_radius: f64,
    max_hole_radius: f64,
}

impl DetectionValidator {
    /// Create a new validator with size constraints
    pub fn new(
        min_board_size: f64,
        max_board_size: f64,
        min_hole_radius: f64,
        max_hole_radius: f64,
    ) -> Self {
        Self {
            min_board_size,
            max_board_size,
            min_hole_radius,
            max_hole_radius,
        }
    }

    /// Validate a board detection
    pub fn validate(&self, detection: &BoardDetection) -> bool {
        // Check board size
        let board_size = detection.dimensions.x.max(detection.dimensions.y);
        if board_size < self.min_board_size || board_size > self.max_board_size {
            return false;
        }

        // Check hole sizes
        for hole in &detection.holes {
            if hole.radius < self.min_hole_radius || hole.radius > self.max_hole_radius {
                return false;
            }
        }

        // Check aspect ratio - diamond boards should be roughly square
        let aspect_ratio = detection.dimensions.x / detection.dimensions.y;
        if aspect_ratio < 0.8 || aspect_ratio > 1.2 {
            return false;
        }

        // Check that we have expected holes for asymmetric pattern
        if detection.holes.len() < 2 {
            return false; // Need at least 2 holes for orientation determination
        }

        // Check pose plausibility - board normal should not be pointing straight down
        let normal_z = detection
            .pose
            .rotation
            .transform_vector(&nalgebra::Vector3::z())
            .z;
        if normal_z < -0.9 {
            return false; // Reject boards facing straight down
        }

        true
    }

    /// Create validator from board configuration
    pub fn from_config(board: &board_fitter_config::SquareBoard) -> Self {
        let board_size = board.size.as_meters();
        let min_hole_radius = board
            .holes
            .iter()
            .map(|h| h.radius.as_meters())
            .fold(f64::INFINITY, f64::min);
        let max_hole_radius = board
            .holes
            .iter()
            .map(|h| h.radius.as_meters())
            .fold(0.0, f64::max);

        Self::new(
            board_size * 0.7,      // Allow 30% tolerance below expected size
            board_size * 1.3,      // Allow 30% tolerance above expected size
            min_hole_radius * 0.5, // Allow 50% tolerance for hole sizes
            max_hole_radius * 1.5,
        )
    }

    /// Validate multiple detections
    pub fn validate_detections(&self, detections: Vec<BoardDetection>) -> Vec<BoardDetection> {
        detections
            .into_iter()
            .filter(|detection| self.validate(detection))
            .collect()
    }

    /// Filter valid detections from a list
    pub fn filter_valid(&self, detections: Vec<BoardDetection>) -> Vec<BoardDetection> {
        detections
            .into_iter()
            .filter(|d| self.validate(d))
            .collect()
    }
}

/// Detection pipeline coordinator
pub struct DetectionPipeline {
    detector: Box<dyn BoardDetector>,
    validator: DetectionValidator,
    config: DetectionConfig,
}

impl DetectionPipeline {
    /// Create a new detection pipeline
    pub fn new(
        detector: Box<dyn BoardDetector>,
        validator: DetectionValidator,
        config: DetectionConfig,
    ) -> Self {
        Self {
            detector,
            validator,
            config,
        }
    }

    /// Run the full detection pipeline
    pub fn process(&mut self, point_cloud: &PointCloud) -> Result<DetectionResult> {
        let start_time = Instant::now();

        // Run detection
        let detection_result = self.detector.detect(point_cloud)?;
        let mut detections = detection_result.detections;

        // Validate detections
        detections = self.validator.filter_valid(detections);

        // Apply confidence threshold
        detections.retain(|d| d.confidence.above_threshold(self.config.min_confidence));

        // Limit number of detections
        detections.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());
        detections.truncate(self.config.max_detections);

        // Get final statistics
        let mut stats = detection_result.stats;
        stats.total_time = start_time.elapsed();
        stats.boards_detected = detections.len();

        Ok(DetectionResult::new(detections, stats))
    }

    /// Get the underlying detector
    pub fn detector(&self) -> &dyn BoardDetector {
        self.detector.as_ref()
    }

    /// Get the underlying detector mutably
    pub fn detector_mut(&mut self) -> &mut dyn BoardDetector {
        self.detector.as_mut()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DetectionConfidence;
    use board_fitter_config::{Config, SquareBoard};
    use measurements::Length;
    use nalgebra::{Isometry3, Point3, Vector3};

    fn create_test_config() -> Config {
        let board = SquareBoard::new(Length::from_meters(1.0));
        Config {
            board,
            detection: None,
            metadata: None,
        }
    }

    #[test]
    fn test_detection_result_creation() {
        let detections = vec![
            BoardDetection::new(
                Isometry3::identity(),
                DetectionConfidence::new(0.8),
                Vector3::new(1.0, 1.0, 0.02),
            ),
            BoardDetection::new(
                Isometry3::identity(),
                DetectionConfidence::new(0.6),
                Vector3::new(1.0, 1.0, 0.02),
            ),
        ];
        let stats = ProcessingStats::new();
        let result = DetectionResult::new(detections, stats);

        assert_eq!(result.count(), 2);
        assert_eq!(result.high_confidence_detections(0.7).len(), 1);
        assert!(result.best_detection().is_some());
        assert_eq!(result.best_detection().unwrap().confidence.value(), 0.8);
    }

    #[test]
    fn test_detection_validator() {
        let validator = DetectionValidator::new(0.5, 2.0, 0.01, 0.2);

        let mut valid_detection = BoardDetection::new(
            Isometry3::identity(),
            DetectionConfidence::new(0.8),
            Vector3::new(1.0, 1.0, 0.02),
        );

        // Add required holes for validation
        valid_detection.holes = vec![
            crate::types::DetectedHole {
                center: Point3::new(0.0, 0.5, 0.0),
                radius: 0.05,
                confidence: DetectionConfidence::new(0.9),
                id: Some("hole1".to_string()),
            },
            crate::types::DetectedHole {
                center: Point3::new(0.5, 0.0, 0.0),
                radius: 0.05,
                confidence: DetectionConfidence::new(0.9),
                id: Some("hole2".to_string()),
            },
        ];

        let invalid_detection = BoardDetection::new(
            Isometry3::identity(),
            DetectionConfidence::new(0.8),
            Vector3::new(3.0, 3.0, 0.02), // Too large
        );

        assert!(validator.validate(&valid_detection));
        assert!(!validator.validate(&invalid_detection));
    }

    #[test]
    fn test_diamond_detector_creation() {
        let config = create_test_config();
        let detector = DiamondDetector::with_board_config(config);
        assert_eq!(detector.config().board.size.as_meters(), 1.0);
    }
}
