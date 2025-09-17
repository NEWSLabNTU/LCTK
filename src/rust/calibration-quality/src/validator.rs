//! Calibration validation module

use crate::CalibrationMetrics;
use anyhow::Result;
use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};

/// Configuration for calibration validation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationConfig {
    /// Maximum allowed translation magnitude (meters)
    pub max_translation: f64,
    /// Maximum allowed rotation angle (radians)
    pub max_rotation: f64,
    /// Minimum required inlier ratio
    pub min_inlier_ratio: f64,
    /// Maximum allowed reprojection error
    pub max_reprojection_error: f64,
    /// Minimum required consistency score
    pub min_consistency_score: f64,
    /// Enable physical constraint checks
    pub check_physical_constraints: bool,
    /// Enable temporal consistency checks
    pub check_temporal_consistency: bool,
}

impl Default for ValidationConfig {
    fn default() -> Self {
        Self {
            max_translation: 10.0,              // 10 meters
            max_rotation: std::f64::consts::PI, // 180 degrees
            min_inlier_ratio: 0.5,              // 50% inliers minimum
            max_reprojection_error: 0.1,        // 10cm maximum error
            min_consistency_score: 0.7,         // 70% consistency
            check_physical_constraints: true,
            check_temporal_consistency: false,
        }
    }
}

/// Validation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationResult {
    /// Whether calibration is valid
    pub is_valid: bool,
    /// Individual validation checks
    pub checks: ValidationChecks,
    /// Validation messages
    pub messages: Vec<String>,
    /// Confidence in validation (0.0 to 1.0)
    pub confidence: f64,
}

/// Individual validation checks
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationChecks {
    pub translation_valid: bool,
    pub rotation_valid: bool,
    pub inlier_ratio_valid: bool,
    pub reprojection_error_valid: bool,
    pub consistency_valid: bool,
    pub physical_constraints_valid: bool,
    pub temporal_consistency_valid: bool,
}

/// Calibration validator
pub struct CalibrationValidator {
    config: ValidationConfig,
    /// Previous transforms for temporal consistency checking
    transform_history: Vec<(Isometry3<f64>, std::time::SystemTime)>,
    max_history: usize,
}

impl CalibrationValidator {
    /// Create a new validator
    pub fn new(config: ValidationConfig) -> Self {
        Self {
            config,
            transform_history: Vec::new(),
            max_history: 10,
        }
    }

    /// Validate a calibration transform
    pub fn validate(
        &mut self,
        transform: &Isometry3<f64>,
        metrics: &CalibrationMetrics,
    ) -> Result<ValidationResult> {
        let mut checks = ValidationChecks {
            translation_valid: true,
            rotation_valid: true,
            inlier_ratio_valid: true,
            reprojection_error_valid: true,
            consistency_valid: true,
            physical_constraints_valid: true,
            temporal_consistency_valid: true,
        };
        let mut messages = Vec::new();

        // Check translation magnitude
        let translation = transform.translation.vector.norm();
        if translation > self.config.max_translation {
            checks.translation_valid = false;
            messages.push(format!(
                "Translation {} exceeds maximum allowed {}",
                translation, self.config.max_translation
            ));
        }

        // Check rotation magnitude
        let rotation_angle = transform.rotation.angle();
        if rotation_angle > self.config.max_rotation {
            checks.rotation_valid = false;
            messages.push(format!(
                "Rotation angle {} exceeds maximum allowed {}",
                rotation_angle, self.config.max_rotation
            ));
        }

        // Check inlier ratio
        if metrics.inlier_ratio < self.config.min_inlier_ratio {
            checks.inlier_ratio_valid = false;
            messages.push(format!(
                "Inlier ratio {} below minimum required {}",
                metrics.inlier_ratio, self.config.min_inlier_ratio
            ));
        }

        // Check reprojection error
        if metrics.reprojection_error > self.config.max_reprojection_error {
            checks.reprojection_error_valid = false;
            messages.push(format!(
                "Reprojection error {} exceeds maximum allowed {}",
                metrics.reprojection_error, self.config.max_reprojection_error
            ));
        }

        // Check consistency score
        if metrics.consistency_score < self.config.min_consistency_score {
            checks.consistency_valid = false;
            messages.push(format!(
                "Consistency score {} below minimum required {}",
                metrics.consistency_score, self.config.min_consistency_score
            ));
        }

        // Check physical constraints
        if self.config.check_physical_constraints {
            checks.physical_constraints_valid = self.check_physical_constraints(transform);
            if !checks.physical_constraints_valid {
                messages.push("Transform violates physical constraints".to_string());
            }
        }

        // Check temporal consistency
        if self.config.check_temporal_consistency && !self.transform_history.is_empty() {
            checks.temporal_consistency_valid = self.check_temporal_consistency(transform);
            if !checks.temporal_consistency_valid {
                messages.push("Transform shows temporal inconsistency".to_string());
            }
        }

        // Update history
        self.transform_history
            .push((*transform, std::time::SystemTime::now()));
        if self.transform_history.len() > self.max_history {
            self.transform_history.remove(0);
        }

        // Determine overall validity
        let is_valid = checks.translation_valid
            && checks.rotation_valid
            && checks.inlier_ratio_valid
            && checks.reprojection_error_valid
            && checks.consistency_valid
            && checks.physical_constraints_valid
            && checks.temporal_consistency_valid;

        // Compute confidence
        let confidence = self.compute_confidence(&checks, metrics);

        Ok(ValidationResult {
            is_valid,
            checks,
            messages,
            confidence,
        })
    }

    /// Check physical constraints
    fn check_physical_constraints(&self, transform: &Isometry3<f64>) -> bool {
        // Check that transform preserves handedness (determinant should be positive)
        let det = transform
            .rotation
            .to_rotation_matrix()
            .matrix()
            .determinant();
        if det < 0.0 {
            return false;
        }

        // Check that rotation is reasonable (no extreme rotations)
        let rotation = transform.rotation;
        let (roll, pitch, _yaw) = rotation.euler_angles();

        // Assuming sensors are roughly level, extreme roll/pitch are suspicious
        let max_roll_pitch = std::f64::consts::PI / 3.0; // 60 degrees
        if roll.abs() > max_roll_pitch || pitch.abs() > max_roll_pitch {
            return false;
        }

        true
    }

    /// Check temporal consistency
    fn check_temporal_consistency(&self, transform: &Isometry3<f64>) -> bool {
        if self.transform_history.is_empty() {
            return true;
        }

        // Get recent transforms
        let recent_transforms: Vec<_> = self
            .transform_history
            .iter()
            .rev()
            .take(3)
            .map(|(t, _)| t)
            .collect();

        // Check for sudden large changes
        for prev_transform in recent_transforms {
            let delta = prev_transform.inverse() * transform;
            let delta_translation = delta.translation.vector.norm();
            let delta_rotation = delta.rotation.angle();

            // Allow up to 10cm translation change and 5 degree rotation change
            if delta_translation > 0.1 || delta_rotation > 0.0873 {
                return false;
            }
        }

        true
    }

    /// Compute validation confidence
    fn compute_confidence(&self, checks: &ValidationChecks, metrics: &CalibrationMetrics) -> f64 {
        let mut score = 0.0;
        let mut count = 0.0;

        // Weight each check
        if checks.translation_valid {
            score += 1.0;
        }
        count += 1.0;

        if checks.rotation_valid {
            score += 1.0;
        }
        count += 1.0;

        if checks.inlier_ratio_valid {
            score += metrics.inlier_ratio;
        }
        count += 1.0;

        if checks.reprojection_error_valid {
            score += 1.0 - (metrics.reprojection_error / self.config.max_reprojection_error);
        }
        count += 1.0;

        if checks.consistency_valid {
            score += metrics.consistency_score;
        }
        count += 1.0;

        if count > 0.0 {
            score / count
        } else {
            0.0
        }
    }
}

/// Dynamic validation config adjustment based on scene
#[allow(dead_code)]
pub struct AdaptiveValidator {
    base_config: ValidationConfig,
    #[allow(dead_code)]
    scene_analyzer: SceneAnalyzer,
}

impl AdaptiveValidator {
    pub fn new(base_config: ValidationConfig) -> Self {
        Self {
            base_config,
            scene_analyzer: SceneAnalyzer::new(),
        }
    }

    /// Adjust validation config based on scene characteristics
    pub fn adapt_config(&mut self, metrics: &CalibrationMetrics) -> ValidationConfig {
        let mut config = self.base_config.clone();

        // Relax constraints if scene is difficult
        if metrics.num_correspondences < 10 {
            config.min_inlier_ratio *= 0.8;
            config.max_reprojection_error *= 1.5;
        }

        // Tighten constraints if we have high-quality data
        if metrics.detection_confidence > 0.95 && metrics.num_correspondences > 50 {
            config.min_inlier_ratio = config.min_inlier_ratio.min(0.8);
            config.max_reprojection_error *= 0.7;
        }

        config
    }
}

/// Scene complexity analyzer
struct SceneAnalyzer {
    #[allow(dead_code)]
    complexity_history: Vec<f64>,
}

impl SceneAnalyzer {
    fn new() -> Self {
        Self {
            complexity_history: Vec::new(),
        }
    }

    #[allow(dead_code)]
    fn analyze_complexity(&mut self, metrics: &CalibrationMetrics) -> f64 {
        // Simple complexity metric based on correspondence count and distribution
        let complexity = 1.0 / (1.0 + metrics.num_correspondences as f64 / 20.0);
        self.complexity_history.push(complexity);
        complexity
    }
}
