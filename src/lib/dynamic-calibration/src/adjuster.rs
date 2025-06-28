//! Parameter adjustment strategies

use crate::{CalibrationParameters, ConfidenceMetrics, SceneCharacteristics};
use anyhow::Result;
use serde::{Deserialize, Serialize};

/// Strategy for parameter adjustment
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum AdjustmentStrategy {
    /// Conservative - small incremental changes
    Conservative,
    /// Balanced - moderate adjustments
    Balanced,
    /// Aggressive - large adjustments for quick adaptation
    Aggressive,
    /// Adaptive - adjusts aggressiveness based on performance
    Adaptive,
}

/// Parameter adjuster with configurable strategies
pub struct ParameterAdjuster {
    strategy: AdjustmentStrategy,
    /// Learning rate for adjustments
    learning_rate: f64,
    /// Momentum for smoothing adjustments
    momentum: f64,
    /// Previous adjustment deltas
    previous_deltas: Option<ParameterDeltas>,
}

#[derive(Debug, Clone)]
struct ParameterDeltas {
    detection_threshold: f64,
    min_inliers: f64,
    ransac_iterations: f64,
    outlier_threshold: f64,
    icp_convergence_threshold: f64,
    icp_max_iterations: f64,
    downsampling_ratio: f64,
    roi_size_multiplier: f64,
    noise_threshold: f64,
    matching_threshold: f64,
}

impl ParameterAdjuster {
    /// Create a new parameter adjuster
    pub fn new(strategy: AdjustmentStrategy) -> Self {
        let learning_rate = match strategy {
            AdjustmentStrategy::Conservative => 0.1,
            AdjustmentStrategy::Balanced => 0.3,
            AdjustmentStrategy::Aggressive => 0.5,
            AdjustmentStrategy::Adaptive => 0.3, // Start balanced
        };

        Self {
            strategy,
            learning_rate,
            momentum: 0.7,
            previous_deltas: None,
        }
    }

    /// Adjust parameters based on confidence and scene analysis
    pub fn adjust(
        &mut self,
        current: &CalibrationParameters,
        confidence: &ConfidenceMetrics,
        scene: &SceneCharacteristics,
    ) -> Result<CalibrationParameters> {
        let mut adjusted = current.clone();

        // Compute target adjustments
        let deltas = self.compute_deltas(current, confidence, scene);

        // Apply adjustments with momentum
        if let Some(prev_deltas) = &self.previous_deltas {
            adjusted.detection_threshold += self.learning_rate * deltas.detection_threshold
                + self.momentum * prev_deltas.detection_threshold;
            adjusted.min_inliers += (self.learning_rate * deltas.min_inliers
                + self.momentum * prev_deltas.min_inliers)
                as usize;
            adjusted.ransac_iterations += (self.learning_rate * deltas.ransac_iterations
                + self.momentum * prev_deltas.ransac_iterations)
                as usize;
            adjusted.outlier_threshold += self.learning_rate * deltas.outlier_threshold
                + self.momentum * prev_deltas.outlier_threshold;
            adjusted.icp_convergence_threshold += self.learning_rate
                * deltas.icp_convergence_threshold
                + self.momentum * prev_deltas.icp_convergence_threshold;
            adjusted.icp_max_iterations += (self.learning_rate * deltas.icp_max_iterations
                + self.momentum * prev_deltas.icp_max_iterations)
                as usize;
            adjusted.downsampling_ratio += self.learning_rate * deltas.downsampling_ratio
                + self.momentum * prev_deltas.downsampling_ratio;
            adjusted.roi_size_multiplier += self.learning_rate * deltas.roi_size_multiplier
                + self.momentum * prev_deltas.roi_size_multiplier;
            adjusted.noise_threshold += self.learning_rate * deltas.noise_threshold
                + self.momentum * prev_deltas.noise_threshold;
            adjusted.matching_threshold += self.learning_rate * deltas.matching_threshold
                + self.momentum * prev_deltas.matching_threshold;
        } else {
            // First adjustment without momentum
            adjusted.detection_threshold += self.learning_rate * deltas.detection_threshold;
            adjusted.min_inliers += (self.learning_rate * deltas.min_inliers) as usize;
            adjusted.ransac_iterations += (self.learning_rate * deltas.ransac_iterations) as usize;
            adjusted.outlier_threshold += self.learning_rate * deltas.outlier_threshold;
            adjusted.icp_convergence_threshold +=
                self.learning_rate * deltas.icp_convergence_threshold;
            adjusted.icp_max_iterations +=
                (self.learning_rate * deltas.icp_max_iterations) as usize;
            adjusted.downsampling_ratio += self.learning_rate * deltas.downsampling_ratio;
            adjusted.roi_size_multiplier += self.learning_rate * deltas.roi_size_multiplier;
            adjusted.noise_threshold += self.learning_rate * deltas.noise_threshold;
            adjusted.matching_threshold += self.learning_rate * deltas.matching_threshold;
        }

        // Update adaptive learning rate
        if self.strategy == AdjustmentStrategy::Adaptive {
            self.update_learning_rate(confidence);
        }

        // Store deltas for next iteration
        self.previous_deltas = Some(deltas);

        Ok(adjusted)
    }

    /// Compute parameter deltas based on analysis
    fn compute_deltas(
        &self,
        _current: &CalibrationParameters,
        confidence: &ConfidenceMetrics,
        scene: &SceneCharacteristics,
    ) -> ParameterDeltas {
        let mut deltas = ParameterDeltas {
            detection_threshold: 0.0,
            min_inliers: 0.0,
            ransac_iterations: 0.0,
            outlier_threshold: 0.0,
            icp_convergence_threshold: 0.0,
            icp_max_iterations: 0.0,
            downsampling_ratio: 0.0,
            roi_size_multiplier: 0.0,
            noise_threshold: 0.0,
            matching_threshold: 0.0,
        };

        // Adjust based on overall confidence
        if confidence.overall_confidence < 0.5 {
            // Low confidence - relax thresholds
            deltas.detection_threshold = -0.1;
            deltas.min_inliers = -2.0;
            deltas.matching_threshold = 0.05;
            deltas.roi_size_multiplier = 0.2;
        } else if confidence.overall_confidence > 0.8 {
            // High confidence - tighten thresholds for accuracy
            deltas.detection_threshold = 0.1;
            deltas.min_inliers = 2.0;
            deltas.matching_threshold = -0.02;
            deltas.roi_size_multiplier = -0.1;
        }

        // Adjust based on detection stability
        if confidence.detection_stability < 0.6 {
            deltas.ransac_iterations = 50.0;
            deltas.outlier_threshold = 0.02;
        }

        // Adjust based on convergence behavior
        if confidence.convergence_rate < 0.3 {
            deltas.icp_convergence_threshold = -0.0002;
            deltas.icp_max_iterations = 10.0;
        }

        // Adjust based on scene complexity
        if scene.complexity > 0.7 {
            deltas.downsampling_ratio = -0.1; // Less downsampling for complex scenes
            deltas.noise_threshold = 0.005;
        } else if scene.complexity < 0.3 {
            deltas.downsampling_ratio = 0.1; // More downsampling for simple scenes
        }

        // Adjust based on noise level
        if scene.noise_level > 0.6 {
            deltas.noise_threshold = 0.01;
            deltas.outlier_threshold = 0.03;
            deltas.min_inliers = 3.0;
        }

        // Adjust based on data density
        if scene.point_density < 0.4 {
            deltas.roi_size_multiplier = 0.3;
            deltas.min_inliers = -3.0;
        }

        deltas
    }

    /// Update learning rate for adaptive strategy
    fn update_learning_rate(&mut self, confidence: &ConfidenceMetrics) {
        if confidence.improvement_trend > 0.0 {
            // Increasing confidence - maintain or increase learning rate
            self.learning_rate = (self.learning_rate * 1.1).min(0.7);
        } else {
            // Decreasing confidence - reduce learning rate
            self.learning_rate = (self.learning_rate * 0.9).max(0.1);
        }
    }

    /// Set adjustment strategy
    pub fn set_strategy(&mut self, strategy: AdjustmentStrategy) {
        self.strategy = strategy;
        self.learning_rate = match strategy {
            AdjustmentStrategy::Conservative => 0.1,
            AdjustmentStrategy::Balanced => 0.3,
            AdjustmentStrategy::Aggressive => 0.5,
            AdjustmentStrategy::Adaptive => self.learning_rate, // Keep current
        };
    }
}

/// Rule-based parameter adjustment for specific scenarios
pub struct RuleBasedAdjuster;

impl RuleBasedAdjuster {
    /// Apply rule-based adjustments
    pub fn apply_rules(
        params: &mut CalibrationParameters,
        confidence: &ConfidenceMetrics,
        scene: &SceneCharacteristics,
    ) {
        // Rule 1: Very low detection rate
        if confidence.detection_rate < 0.2 {
            params.detection_threshold *= 0.8;
            params.roi_size_multiplier *= 1.5;
        }

        // Rule 2: High false positive rate
        if confidence.false_positive_rate > 0.3 {
            params.detection_threshold *= 1.2;
            params.min_inliers = (params.min_inliers as f64 * 1.5) as usize;
        }

        // Rule 3: Oscillating quality
        if confidence.quality_variance > 0.3 {
            params.ransac_iterations = (params.ransac_iterations as f64 * 1.3) as usize;
            params.icp_max_iterations = (params.icp_max_iterations as f64 * 1.2) as usize;
        }

        // Rule 4: Very sparse data
        if scene.point_density < 0.2 {
            params.min_inliers = params.min_inliers.min(5);
            params.downsampling_ratio = params.downsampling_ratio.max(0.8);
        }

        // Rule 5: Extreme noise
        if scene.noise_level > 0.8 {
            params.noise_threshold *= 2.0;
            params.outlier_threshold *= 1.5;
        }
    }
}

/// Adjustment profiles for different use cases
pub mod profiles {
    use super::*;

    /// Indoor calibration profile
    pub fn indoor_adjustment(confidence: &ConfidenceMetrics) -> f64 {
        if confidence.overall_confidence < 0.6 {
            0.8 // More aggressive adjustments for indoor challenges
        } else {
            1.0
        }
    }

    /// Outdoor calibration profile
    pub fn outdoor_adjustment(scene: &SceneCharacteristics) -> f64 {
        if scene.noise_level > 0.5 || scene.ambient_light_variation > 0.7 {
            1.2 // More conservative for outdoor variability
        } else {
            1.0
        }
    }

    /// Industrial calibration profile
    pub fn industrial_adjustment(scene: &SceneCharacteristics) -> f64 {
        if scene.reflectivity_variation > 0.6 {
            0.7 // Adjust for reflective surfaces
        } else {
            1.0
        }
    }
}
