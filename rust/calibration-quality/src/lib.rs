//! Automatic calibration quality assessment library
//!
//! This library provides tools for assessing the quality of calibration results
//! in real-time, enabling automatic validation and confidence scoring.

use anyhow::Result;
use nalgebra::{Isometry3, Point3};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use thiserror::Error;

pub mod metrics;
pub mod monitor;
pub mod validator;

pub use metrics::{CalibrationMetrics, QualityScore};
pub use monitor::{ConvergenceMonitor, ConvergenceStatus};
pub use validator::{CalibrationValidator, ValidationConfig, ValidationResult};

#[derive(Error, Debug)]
pub enum QualityError {
    #[error("Insufficient data for quality assessment")]
    InsufficientData,
    #[error("Invalid calibration parameters: {0}")]
    InvalidParameters(String),
    #[error("Quality threshold not met: {0}")]
    ThresholdNotMet(String),
}

/// Overall calibration quality assessment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationQuality {
    /// Overall quality score (0.0 to 1.0)
    pub overall_score: f64,
    /// Individual metric scores
    pub metrics: CalibrationMetrics,
    /// Validation results
    pub validation: ValidationResult,
    /// Convergence status
    pub convergence: ConvergenceStatus,
    /// Timestamp of assessment
    pub timestamp: std::time::SystemTime,
}

impl CalibrationQuality {
    /// Create a new quality assessment
    pub fn new(
        metrics: CalibrationMetrics,
        validation: ValidationResult,
        convergence: ConvergenceStatus,
    ) -> Self {
        let overall_score = Self::compute_overall_score(&metrics, &validation, &convergence);

        Self {
            overall_score,
            metrics,
            validation,
            convergence,
            timestamp: std::time::SystemTime::now(),
        }
    }

    /// Compute overall quality score from components
    fn compute_overall_score(
        metrics: &CalibrationMetrics,
        validation: &ValidationResult,
        convergence: &ConvergenceStatus,
    ) -> f64 {
        let mut score = 0.0;
        let mut weight_sum = 0.0;

        // Metric scores with weights
        let metric_weight = 0.4;
        score += metrics.overall_score() * metric_weight;
        weight_sum += metric_weight;

        // Validation score
        let validation_weight = 0.3;
        if validation.is_valid {
            score += 1.0 * validation_weight;
        }
        weight_sum += validation_weight;

        // Convergence score
        let convergence_weight = 0.3;
        score += convergence.convergence_score() * convergence_weight;
        weight_sum += convergence_weight;

        if weight_sum > 0.0 {
            score / weight_sum
        } else {
            0.0
        }
    }

    /// Check if calibration meets quality requirements
    pub fn meets_requirements(&self, min_score: f64) -> bool {
        self.overall_score >= min_score && self.validation.is_valid
    }

    /// Get quality assessment summary
    pub fn summary(&self) -> String {
        format!(
            "Calibration Quality: {:.1}% (Metrics: {:.1}%, Valid: {}, Converged: {})",
            self.overall_score * 100.0,
            self.metrics.overall_score() * 100.0,
            self.validation.is_valid,
            self.convergence.is_converged
        )
    }
}

/// Quality assessment for a calibration transform
pub struct QualityAssessor {
    /// Configuration for validation
    validation_config: ValidationConfig,
    /// Convergence monitor
    convergence_monitor: ConvergenceMonitor,
    /// History of quality assessments
    history: VecDeque<CalibrationQuality>,
    /// Maximum history size
    max_history: usize,
}

impl QualityAssessor {
    /// Create a new quality assessor
    pub fn new(validation_config: ValidationConfig) -> Self {
        Self {
            validation_config,
            convergence_monitor: ConvergenceMonitor::new(),
            history: VecDeque::new(),
            max_history: 100,
        }
    }

    /// Assess calibration quality
    pub fn assess(
        &mut self,
        transform: &Isometry3<f64>,
        detection_pairs: &[(Point3<f64>, Point3<f64>)],
        detection_confidence: f64,
    ) -> Result<CalibrationQuality> {
        // Compute metrics
        let metrics =
            CalibrationMetrics::compute(transform, detection_pairs, detection_confidence)?;

        // Validate calibration
        let mut validator = CalibrationValidator::new(self.validation_config.clone());
        let validation = validator.validate(transform, &metrics)?;

        // Update convergence monitor
        self.convergence_monitor.update(transform, &metrics);
        let convergence = self.convergence_monitor.status();

        // Create quality assessment
        let quality = CalibrationQuality::new(metrics, validation, convergence);

        // Update history
        self.history.push_back(quality.clone());
        if self.history.len() > self.max_history {
            self.history.pop_front();
        }

        Ok(quality)
    }

    /// Get quality trend over time
    pub fn quality_trend(&self) -> Vec<f64> {
        self.history.iter().map(|q| q.overall_score).collect()
    }

    /// Get average quality over recent assessments
    pub fn average_quality(&self, window_size: usize) -> Option<f64> {
        if self.history.is_empty() {
            return None;
        }

        let window = window_size.min(self.history.len());
        let sum: f64 = self
            .history
            .iter()
            .rev()
            .take(window)
            .map(|q| q.overall_score)
            .sum();

        Some(sum / window as f64)
    }

    /// Check if quality is improving
    pub fn is_improving(&self, window_size: usize) -> bool {
        if self.history.len() < window_size * 2 {
            return false;
        }

        let recent_avg = self.average_quality(window_size).unwrap_or(0.0);
        let previous_avg = self
            .history
            .iter()
            .rev()
            .skip(window_size)
            .take(window_size)
            .map(|q| q.overall_score)
            .sum::<f64>()
            / window_size as f64;

        recent_avg > previous_avg
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // use approx::assert_relative_eq;

    #[test]
    fn test_quality_assessment() {
        let config = ValidationConfig::default();
        let mut assessor = QualityAssessor::new(config);

        let transform = Isometry3::identity();
        let pairs = vec![
            (Point3::new(0.0, 0.0, 0.0), Point3::new(0.01, 0.01, 0.01)),
            (Point3::new(1.0, 0.0, 0.0), Point3::new(1.01, 0.01, 0.01)),
        ];

        let quality = assessor.assess(&transform, &pairs, 0.9).unwrap();
        assert!(quality.overall_score > 0.0);
        assert!(quality.overall_score <= 1.0);
    }

    #[test]
    fn test_quality_trend() {
        let config = ValidationConfig::default();
        let mut assessor = QualityAssessor::new(config);

        let transform = Isometry3::identity();
        let pairs = vec![(Point3::new(0.0, 0.0, 0.0), Point3::new(0.01, 0.01, 0.01))];

        // Add multiple assessments
        for i in 0..5 {
            let confidence = 0.5 + (i as f64) * 0.1;
            assessor.assess(&transform, &pairs, confidence).unwrap();
        }

        let trend = assessor.quality_trend();
        assert_eq!(trend.len(), 5);

        // Quality should improve with confidence
        assert!(assessor.is_improving(2));
    }
}
