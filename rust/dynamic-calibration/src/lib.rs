//! Dynamic calibration parameter adjustment library
//!
//! This library provides intelligent parameter adjustment for calibration
//! algorithms based on real-time detection confidence and scene analysis.

use anyhow::Result;
use calibration_quality::{CalibrationMetrics, QualityScore};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use thiserror::Error;

pub mod adjuster;
pub mod confidence;
pub mod scene_analyzer;

pub use adjuster::{AdjustmentStrategy, ParameterAdjuster};
pub use confidence::{ConfidenceAnalyzer, ConfidenceMetrics};
pub use scene_analyzer::{SceneAnalyzer, SceneCharacteristics};

#[derive(Error, Debug)]
pub enum DynamicCalibrationError {
    #[error("Invalid parameter range: {0}")]
    InvalidParameterRange(String),
    #[error("Adjustment failed: {0}")]
    AdjustmentFailed(String),
    #[error("Scene analysis error: {0}")]
    SceneAnalysisError(String),
}

/// Calibration parameters that can be dynamically adjusted
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationParameters {
    /// Detection threshold (0.0 to 1.0)
    pub detection_threshold: f64,
    /// Minimum number of inliers required
    pub min_inliers: usize,
    /// Maximum number of RANSAC iterations
    pub ransac_iterations: usize,
    /// Outlier rejection threshold
    pub outlier_threshold: f64,
    /// ICP convergence criteria
    pub icp_convergence_threshold: f64,
    /// Maximum ICP iterations
    pub icp_max_iterations: usize,
    /// Point cloud downsampling ratio
    pub downsampling_ratio: f64,
    /// ROI (Region of Interest) size multiplier
    pub roi_size_multiplier: f64,
    /// Noise filtering threshold
    pub noise_threshold: f64,
    /// Feature matching distance threshold
    pub matching_threshold: f64,
}

impl Default for CalibrationParameters {
    fn default() -> Self {
        Self {
            detection_threshold: 0.5,
            min_inliers: 10,
            ransac_iterations: 100,
            outlier_threshold: 0.05,
            icp_convergence_threshold: 0.001,
            icp_max_iterations: 20,
            downsampling_ratio: 1.0,
            roi_size_multiplier: 1.0,
            noise_threshold: 0.01,
            matching_threshold: 0.1,
        }
    }
}

impl CalibrationParameters {
    /// Apply constraints to ensure parameters are valid
    pub fn constrain(&mut self) {
        self.detection_threshold = self.detection_threshold.clamp(0.1, 0.95);
        self.min_inliers = self.min_inliers.max(3);
        self.ransac_iterations = self.ransac_iterations.clamp(10, 1000);
        self.outlier_threshold = self.outlier_threshold.clamp(0.001, 0.5);
        self.icp_convergence_threshold = self.icp_convergence_threshold.clamp(0.0001, 0.01);
        self.icp_max_iterations = self.icp_max_iterations.clamp(1, 100);
        self.downsampling_ratio = self.downsampling_ratio.clamp(0.1, 1.0);
        self.roi_size_multiplier = self.roi_size_multiplier.clamp(0.5, 3.0);
        self.noise_threshold = self.noise_threshold.clamp(0.001, 0.1);
        self.matching_threshold = self.matching_threshold.clamp(0.01, 1.0);
    }

    /// Get parameter summary
    pub fn summary(&self) -> String {
        format!(
            "Detection: {:.2}, Inliers: {}, RANSAC: {}, ICP: {} iterations",
            self.detection_threshold,
            self.min_inliers,
            self.ransac_iterations,
            self.icp_max_iterations
        )
    }
}

/// Dynamic calibration controller
pub struct DynamicCalibrationController {
    /// Current parameters
    parameters: CalibrationParameters,
    /// Parameter adjuster
    adjuster: ParameterAdjuster,
    /// Confidence analyzer
    confidence_analyzer: ConfidenceAnalyzer,
    /// Scene analyzer
    scene_analyzer: SceneAnalyzer,
    /// History of adjustments
    adjustment_history: VecDeque<AdjustmentRecord>,
    /// Maximum history size
    max_history: usize,
}

#[derive(Debug, Clone)]
struct AdjustmentRecord {
    timestamp: std::time::SystemTime,
    #[allow(dead_code)]
    parameters_before: CalibrationParameters,
    parameters_after: CalibrationParameters,
    reason: String,
    quality_score: f64,
}

impl Default for DynamicCalibrationController {
    fn default() -> Self {
        Self::new()
    }
}

impl DynamicCalibrationController {
    /// Create a new dynamic calibration controller
    pub fn new() -> Self {
        Self::with_strategy(AdjustmentStrategy::Balanced)
    }

    /// Create with specific adjustment strategy
    pub fn with_strategy(strategy: AdjustmentStrategy) -> Self {
        Self {
            parameters: CalibrationParameters::default(),
            adjuster: ParameterAdjuster::new(strategy),
            confidence_analyzer: ConfidenceAnalyzer::new(),
            scene_analyzer: SceneAnalyzer::new(),
            adjustment_history: VecDeque::new(),
            max_history: 50,
        }
    }

    /// Update parameters based on current calibration results
    pub fn update(
        &mut self,
        metrics: &CalibrationMetrics,
        quality_score: &QualityScore,
    ) -> Result<CalibrationParameters> {
        // Analyze confidence
        let confidence_metrics = self.confidence_analyzer.analyze(metrics, quality_score)?;

        // Analyze scene
        let scene_characteristics = self.scene_analyzer.analyze(metrics)?;

        // Store current parameters
        let parameters_before = self.parameters.clone();

        // Adjust parameters
        self.parameters = self.adjuster.adjust(
            &self.parameters,
            &confidence_metrics,
            &scene_characteristics,
        )?;

        // Constrain parameters
        self.parameters.constrain();

        // Record adjustment
        let record = AdjustmentRecord {
            timestamp: std::time::SystemTime::now(),
            parameters_before,
            parameters_after: self.parameters.clone(),
            reason: self.get_adjustment_reason(&confidence_metrics, &scene_characteristics),
            quality_score: quality_score.overall,
        };

        self.adjustment_history.push_back(record);
        if self.adjustment_history.len() > self.max_history {
            self.adjustment_history.pop_front();
        }

        Ok(self.parameters.clone())
    }

    /// Get current parameters
    pub fn parameters(&self) -> &CalibrationParameters {
        &self.parameters
    }

    /// Set adjustment strategy
    pub fn set_strategy(&mut self, strategy: AdjustmentStrategy) {
        self.adjuster.set_strategy(strategy);
    }

    /// Get adjustment history
    pub fn history(&self) -> Vec<(std::time::SystemTime, String, f64)> {
        self.adjustment_history
            .iter()
            .map(|record| {
                (
                    record.timestamp,
                    record.reason.clone(),
                    record.quality_score,
                )
            })
            .collect()
    }

    /// Check if parameters are stable
    pub fn is_stable(&self, window_size: usize) -> bool {
        if self.adjustment_history.len() < window_size {
            return false;
        }

        // Check if recent adjustments resulted in significant changes
        let recent: Vec<_> = self
            .adjustment_history
            .iter()
            .rev()
            .take(window_size)
            .collect();

        for i in 1..recent.len() {
            if Self::parameters_differ_significantly(
                &recent[i - 1].parameters_after,
                &recent[i].parameters_after,
            ) {
                return false;
            }
        }

        true
    }

    /// Check if two parameter sets differ significantly
    fn parameters_differ_significantly(
        p1: &CalibrationParameters,
        p2: &CalibrationParameters,
    ) -> bool {
        (p1.detection_threshold - p2.detection_threshold).abs() > 0.1
            || (p1.min_inliers as i32 - p2.min_inliers as i32).abs() > 5
            || (p1.ransac_iterations as i32 - p2.ransac_iterations as i32).abs() > 50
            || (p1.icp_max_iterations as i32 - p2.icp_max_iterations as i32).abs() > 10
    }

    /// Get reason for adjustment
    fn get_adjustment_reason(
        &self,
        confidence: &ConfidenceMetrics,
        scene: &SceneCharacteristics,
    ) -> String {
        let mut reasons = Vec::new();

        if confidence.overall_confidence < 0.5 {
            reasons.push("Low confidence");
        }
        if scene.complexity > 0.8 {
            reasons.push("Complex scene");
        }
        if scene.noise_level > 0.5 {
            reasons.push("High noise");
        }
        if confidence.detection_stability < 0.7 {
            reasons.push("Unstable detections");
        }

        if reasons.is_empty() {
            "Routine optimization".to_string()
        } else {
            reasons.join(", ")
        }
    }

    /// Reset to default parameters
    pub fn reset(&mut self) {
        self.parameters = CalibrationParameters::default();
        self.adjustment_history.clear();
        self.confidence_analyzer.reset();
        self.scene_analyzer.reset();
    }
}

/// Preset parameter configurations for common scenarios
pub mod presets {
    use super::CalibrationParameters;

    /// High accuracy preset - prioritizes precision over speed
    pub fn high_accuracy() -> CalibrationParameters {
        CalibrationParameters {
            detection_threshold: 0.7,
            min_inliers: 20,
            ransac_iterations: 500,
            outlier_threshold: 0.02,
            icp_convergence_threshold: 0.0005,
            icp_max_iterations: 50,
            downsampling_ratio: 0.9,
            roi_size_multiplier: 1.2,
            noise_threshold: 0.005,
            matching_threshold: 0.05,
        }
    }

    /// Fast processing preset - prioritizes speed over accuracy
    pub fn fast_processing() -> CalibrationParameters {
        CalibrationParameters {
            detection_threshold: 0.4,
            min_inliers: 5,
            ransac_iterations: 50,
            outlier_threshold: 0.1,
            icp_convergence_threshold: 0.005,
            icp_max_iterations: 10,
            downsampling_ratio: 0.3,
            roi_size_multiplier: 0.8,
            noise_threshold: 0.02,
            matching_threshold: 0.2,
        }
    }

    /// Noisy environment preset - robust to sensor noise
    pub fn noisy_environment() -> CalibrationParameters {
        CalibrationParameters {
            detection_threshold: 0.6,
            min_inliers: 15,
            ransac_iterations: 200,
            outlier_threshold: 0.15,
            icp_convergence_threshold: 0.002,
            icp_max_iterations: 30,
            downsampling_ratio: 0.7,
            roi_size_multiplier: 1.5,
            noise_threshold: 0.05,
            matching_threshold: 0.15,
        }
    }

    /// Sparse data preset - for limited correspondences
    pub fn sparse_data() -> CalibrationParameters {
        CalibrationParameters {
            detection_threshold: 0.3,
            min_inliers: 3,
            ransac_iterations: 150,
            outlier_threshold: 0.08,
            icp_convergence_threshold: 0.001,
            icp_max_iterations: 25,
            downsampling_ratio: 1.0,
            roi_size_multiplier: 2.0,
            noise_threshold: 0.01,
            matching_threshold: 0.3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use calibration_quality::CalibrationMetrics;

    #[test]
    fn test_parameter_constraints() {
        let mut params = CalibrationParameters {
            detection_threshold: 1.5, // Out of range
            min_inliers: 0,           // Too low
            ..Default::default()
        };

        params.constrain();

        assert!(params.detection_threshold <= 0.95);
        assert!(params.min_inliers >= 3);
    }

    #[test]
    fn test_dynamic_controller() {
        let mut controller = DynamicCalibrationController::new();

        // Simulate low quality metrics
        let metrics = CalibrationMetrics {
            reprojection_error: 0.1,
            consistency_score: 0.5,
            detection_confidence: 0.3,
            num_inliers: 5,
            num_correspondences: 10,
            inlier_ratio: 0.5,
            geometric_error: Default::default(),
            statistical_metrics: Default::default(),
        };

        let quality = QualityScore {
            overall: 0.4,
            components: Default::default(),
        };

        let updated_params = controller.update(&metrics, &quality).unwrap();

        // Parameters should be adjusted for low confidence
        assert!(updated_params.detection_threshold < 0.5);
    }
}
