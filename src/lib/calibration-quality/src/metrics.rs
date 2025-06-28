//! Calibration quality metrics computation

use anyhow::Result;
use nalgebra::{Isometry3, Point3, Vector3};
use serde::{Deserialize, Serialize};

/// Individual quality metrics for calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationMetrics {
    /// Reprojection error (mean squared error)
    pub reprojection_error: f64,
    /// Transform consistency score (0.0 to 1.0)
    pub consistency_score: f64,
    /// Detection confidence (0.0 to 1.0)
    pub detection_confidence: f64,
    /// Number of inlier correspondences
    pub num_inliers: usize,
    /// Total number of correspondences
    pub num_correspondences: usize,
    /// Inlier ratio (0.0 to 1.0)
    pub inlier_ratio: f64,
    /// Geometric error metrics
    pub geometric_error: GeometricError,
    /// Statistical metrics
    pub statistical_metrics: StatisticalMetrics,
}

impl CalibrationMetrics {
    /// Compute metrics from calibration data
    pub fn compute(
        transform: &Isometry3<f64>,
        detection_pairs: &[(Point3<f64>, Point3<f64>)],
        detection_confidence: f64,
    ) -> Result<Self> {
        let reprojection_error = Self::compute_reprojection_error(transform, detection_pairs);
        let consistency_score = Self::compute_consistency_score(transform, detection_pairs);
        let (num_inliers, inlier_ratio) = Self::compute_inlier_stats(transform, detection_pairs);
        let geometric_error = GeometricError::compute(transform, detection_pairs);
        let statistical_metrics = StatisticalMetrics::compute(transform, detection_pairs);

        Ok(Self {
            reprojection_error,
            consistency_score,
            detection_confidence,
            num_inliers,
            num_correspondences: detection_pairs.len(),
            inlier_ratio,
            geometric_error,
            statistical_metrics,
        })
    }

    /// Compute reprojection error
    fn compute_reprojection_error(
        transform: &Isometry3<f64>,
        pairs: &[(Point3<f64>, Point3<f64>)],
    ) -> f64 {
        if pairs.is_empty() {
            return f64::INFINITY;
        }

        let total_error: f64 = pairs
            .iter()
            .map(|(source, target)| {
                let transformed = transform * source;
                (transformed - target).norm_squared()
            })
            .sum();

        total_error / pairs.len() as f64
    }

    /// Compute transform consistency score
    fn compute_consistency_score(
        transform: &Isometry3<f64>,
        pairs: &[(Point3<f64>, Point3<f64>)],
    ) -> f64 {
        if pairs.len() < 2 {
            return 0.0;
        }

        // Check if transform preserves distances between points
        let mut preserved_distances = 0;
        let mut total_comparisons = 0;

        for i in 0..pairs.len() {
            for j in (i + 1)..pairs.len() {
                let (source1, target1) = &pairs[i];
                let (source2, target2) = &pairs[j];

                let source_dist = (source1 - source2).norm();
                let target_dist = (target1 - target2).norm();

                let transformed1 = transform * source1;
                let transformed2 = transform * source2;
                let transformed_dist = (transformed1 - transformed2).norm();

                // Check if distance is preserved within threshold
                let error = (transformed_dist - target_dist).abs();
                let threshold = 0.05 * source_dist.max(0.1); // 5% tolerance

                if error < threshold {
                    preserved_distances += 1;
                }
                total_comparisons += 1;
            }
        }

        if total_comparisons > 0 {
            preserved_distances as f64 / total_comparisons as f64
        } else {
            0.0
        }
    }

    /// Compute inlier statistics
    fn compute_inlier_stats(
        transform: &Isometry3<f64>,
        pairs: &[(Point3<f64>, Point3<f64>)],
    ) -> (usize, f64) {
        if pairs.is_empty() {
            return (0, 0.0);
        }

        let threshold = 0.05; // 5cm threshold for inliers
        let num_inliers = pairs
            .iter()
            .filter(|(source, target)| {
                let transformed = transform * source;
                (transformed - target).norm() < threshold
            })
            .count();

        let ratio = num_inliers as f64 / pairs.len() as f64;
        (num_inliers, ratio)
    }

    /// Get overall metric score
    pub fn overall_score(&self) -> f64 {
        let mut score = 0.0;
        let mut weights = 0.0;

        // Reprojection error score (inverse, lower is better)
        let reproj_score = (-self.reprojection_error * 10.0).exp();
        score += reproj_score * 0.3;
        weights += 0.3;

        // Consistency score
        score += self.consistency_score * 0.2;
        weights += 0.2;

        // Detection confidence
        score += self.detection_confidence * 0.2;
        weights += 0.2;

        // Inlier ratio
        score += self.inlier_ratio * 0.3;
        weights += 0.3;

        if weights > 0.0 {
            score / weights
        } else {
            0.0
        }
    }
}

/// Geometric error metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometricError {
    /// Mean translation error
    pub mean_translation_error: f64,
    /// Mean rotation error (in radians)
    pub mean_rotation_error: f64,
    /// Maximum translation error
    pub max_translation_error: f64,
    /// Maximum rotation error (in radians)
    pub max_rotation_error: f64,
}

impl GeometricError {
    /// Compute geometric errors
    pub fn compute(transform: &Isometry3<f64>, pairs: &[(Point3<f64>, Point3<f64>)]) -> Self {
        if pairs.is_empty() {
            return Self {
                mean_translation_error: 0.0,
                mean_rotation_error: 0.0,
                max_translation_error: 0.0,
                max_rotation_error: 0.0,
            };
        }

        let mut translation_errors = Vec::new();
        let mut rotation_errors = Vec::new();

        for (source, target) in pairs {
            let transformed = transform * source;
            let translation_error = (transformed - target).norm();
            translation_errors.push(translation_error);

            // Estimate rotation error using neighboring points
            if let Some((next_source, next_target)) = pairs.iter().find(|(s, _)| s != source) {
                let _v1 = (next_source - source).normalize();
                let v2 = (next_target - target).normalize();
                let v1_transformed = (transform * next_source - transformed).normalize();

                let angle_error = v1_transformed.dot(&v2).clamp(-1.0, 1.0).acos();
                rotation_errors.push(angle_error);
            }
        }

        let mean_translation_error =
            translation_errors.iter().sum::<f64>() / translation_errors.len() as f64;
        let mean_rotation_error = if !rotation_errors.is_empty() {
            rotation_errors.iter().sum::<f64>() / rotation_errors.len() as f64
        } else {
            0.0
        };

        let max_translation_error = translation_errors.iter().cloned().fold(0.0, f64::max);
        let max_rotation_error = rotation_errors.iter().cloned().fold(0.0, f64::max);

        Self {
            mean_translation_error,
            mean_rotation_error,
            max_translation_error,
            max_rotation_error,
        }
    }
}

/// Statistical quality metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatisticalMetrics {
    /// Standard deviation of errors
    pub error_std_dev: f64,
    /// Median error
    pub median_error: f64,
    /// 95th percentile error
    pub percentile_95_error: f64,
    /// Outlier count (errors > 3 * std_dev)
    pub outlier_count: usize,
}

impl StatisticalMetrics {
    /// Compute statistical metrics
    pub fn compute(transform: &Isometry3<f64>, pairs: &[(Point3<f64>, Point3<f64>)]) -> Self {
        if pairs.is_empty() {
            return Self {
                error_std_dev: 0.0,
                median_error: 0.0,
                percentile_95_error: 0.0,
                outlier_count: 0,
            };
        }

        let mut errors: Vec<f64> = pairs
            .iter()
            .map(|(source, target)| {
                let transformed = transform * source;
                (transformed - target).norm()
            })
            .collect();

        errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let mean_error = errors.iter().sum::<f64>() / errors.len() as f64;
        let variance =
            errors.iter().map(|e| (e - mean_error).powi(2)).sum::<f64>() / errors.len() as f64;
        let error_std_dev = variance.sqrt();

        let median_error = if errors.len() % 2 == 0 {
            (errors[errors.len() / 2 - 1] + errors[errors.len() / 2]) / 2.0
        } else {
            errors[errors.len() / 2]
        };

        let percentile_95_idx = ((errors.len() as f64 * 0.95) as usize).min(errors.len() - 1);
        let percentile_95_error = errors[percentile_95_idx];

        let outlier_threshold = mean_error + 3.0 * error_std_dev;
        let outlier_count = errors.iter().filter(|&&e| e > outlier_threshold).count();

        Self {
            error_std_dev,
            median_error,
            percentile_95_error,
            outlier_count,
        }
    }
}

/// Quality score aggregation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityScore {
    /// Overall score (0.0 to 1.0)
    pub overall: f64,
    /// Individual component scores
    pub components: QualityComponents,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityComponents {
    pub accuracy: f64,
    pub precision: f64,
    pub robustness: f64,
    pub consistency: f64,
}

impl From<&CalibrationMetrics> for QualityScore {
    fn from(metrics: &CalibrationMetrics) -> Self {
        let accuracy = (-metrics.reprojection_error * 10.0).exp();
        let precision = 1.0 / (1.0 + metrics.geometric_error.mean_translation_error * 20.0);
        let robustness = metrics.inlier_ratio;
        let consistency = metrics.consistency_score;

        let overall = (accuracy * 0.3 + precision * 0.3 + robustness * 0.2 + consistency * 0.2)
            .clamp(0.0, 1.0);

        Self {
            overall,
            components: QualityComponents {
                accuracy,
                precision,
                robustness,
                consistency,
            },
        }
    }
}
