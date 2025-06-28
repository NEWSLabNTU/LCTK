//! Confidence analysis for calibration

use anyhow::Result;
use calibration_quality::{CalibrationMetrics, QualityScore};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Confidence metrics derived from calibration results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceMetrics {
    /// Overall confidence score (0.0 to 1.0)
    pub overall_confidence: f64,
    /// Detection rate (successful detections / attempts)
    pub detection_rate: f64,
    /// Detection stability over time
    pub detection_stability: f64,
    /// False positive rate estimate
    pub false_positive_rate: f64,
    /// Convergence rate
    pub convergence_rate: f64,
    /// Quality variance
    pub quality_variance: f64,
    /// Improvement trend
    pub improvement_trend: f64,
    /// Consistency across multiple runs
    pub consistency_score: f64,
}

impl Default for ConfidenceMetrics {
    fn default() -> Self {
        Self {
            overall_confidence: 0.5,
            detection_rate: 0.0,
            detection_stability: 0.5,
            false_positive_rate: 0.0,
            convergence_rate: 0.0,
            quality_variance: 0.0,
            improvement_trend: 0.0,
            consistency_score: 0.5,
        }
    }
}

/// Analyzer for confidence metrics
pub struct ConfidenceAnalyzer {
    /// History of calibration metrics
    metrics_history: VecDeque<CalibrationMetrics>,
    /// History of quality scores
    quality_history: VecDeque<QualityScore>,
    /// Detection attempts counter
    detection_attempts: usize,
    /// Successful detections counter
    successful_detections: usize,
    /// Maximum history size
    max_history: usize,
}

impl Default for ConfidenceAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfidenceAnalyzer {
    /// Create a new confidence analyzer
    pub fn new() -> Self {
        Self {
            metrics_history: VecDeque::new(),
            quality_history: VecDeque::new(),
            detection_attempts: 0,
            successful_detections: 0,
            max_history: 50,
        }
    }

    /// Analyze confidence from calibration results
    pub fn analyze(
        &mut self,
        metrics: &CalibrationMetrics,
        quality: &QualityScore,
    ) -> Result<ConfidenceMetrics> {
        // Update history
        self.metrics_history.push_back(metrics.clone());
        self.quality_history.push_back(quality.clone());

        if self.metrics_history.len() > self.max_history {
            self.metrics_history.pop_front();
            self.quality_history.pop_front();
        }

        // Update detection statistics
        self.detection_attempts += 1;
        if metrics.num_inliers > 0 && metrics.inlier_ratio > 0.3 {
            self.successful_detections += 1;
        }

        // Compute confidence metrics
        let overall_confidence = self.compute_overall_confidence(metrics, quality);
        let detection_rate = self.compute_detection_rate();
        let detection_stability = self.compute_detection_stability();
        let false_positive_rate = self.estimate_false_positive_rate();
        let convergence_rate = self.compute_convergence_rate();
        let quality_variance = self.compute_quality_variance();
        let improvement_trend = self.compute_improvement_trend();
        let consistency_score = self.compute_consistency_score();

        Ok(ConfidenceMetrics {
            overall_confidence,
            detection_rate,
            detection_stability,
            false_positive_rate,
            convergence_rate,
            quality_variance,
            improvement_trend,
            consistency_score,
        })
    }

    /// Compute overall confidence
    fn compute_overall_confidence(
        &self,
        metrics: &CalibrationMetrics,
        quality: &QualityScore,
    ) -> f64 {
        let mut confidence = quality.overall;

        // Adjust based on detection confidence
        confidence *= metrics.detection_confidence;

        // Adjust based on inlier ratio
        confidence *= 0.5 + 0.5 * metrics.inlier_ratio;

        // Adjust based on consistency over time
        if self.quality_history.len() >= 5 {
            let recent_variance = self.compute_recent_variance(5);
            confidence *= 1.0 - recent_variance.min(0.5);
        }

        confidence.clamp(0.0, 1.0)
    }

    /// Compute detection rate
    fn compute_detection_rate(&self) -> f64 {
        if self.detection_attempts == 0 {
            0.0
        } else {
            self.successful_detections as f64 / self.detection_attempts as f64
        }
    }

    /// Compute detection stability
    fn compute_detection_stability(&self) -> f64 {
        if self.metrics_history.len() < 5 {
            return 0.5;
        }

        // Check consistency of detection counts
        let recent_detections: Vec<_> = self
            .metrics_history
            .iter()
            .rev()
            .take(10)
            .map(|m| m.num_inliers)
            .collect();

        let mean = recent_detections.iter().sum::<usize>() as f64 / recent_detections.len() as f64;
        let variance = recent_detections
            .iter()
            .map(|&d| (d as f64 - mean).powi(2))
            .sum::<f64>()
            / recent_detections.len() as f64;

        let std_dev = variance.sqrt();
        let cv = if mean > 0.0 { std_dev / mean } else { 1.0 };

        // Convert coefficient of variation to stability score
        (1.0 / (1.0 + cv)).clamp(0.0, 1.0)
    }

    /// Estimate false positive rate
    fn estimate_false_positive_rate(&self) -> f64 {
        if self.metrics_history.len() < 5 {
            return 0.0;
        }

        // Estimate based on outlier ratio and quality fluctuations
        let outlier_rates: Vec<_> = self
            .metrics_history
            .iter()
            .rev()
            .take(10)
            .map(|m| 1.0 - m.inlier_ratio)
            .collect();

        let quality_drops = self.count_quality_drops();

        let avg_outlier_rate = outlier_rates.iter().sum::<f64>() / outlier_rates.len() as f64;
        let drop_rate = quality_drops as f64 / self.quality_history.len().max(1) as f64;

        (avg_outlier_rate * 0.7 + drop_rate * 0.3).clamp(0.0, 1.0)
    }

    /// Compute convergence rate
    fn compute_convergence_rate(&self) -> f64 {
        if self.quality_history.len() < 3 {
            return 0.0;
        }

        // Calculate improvement rate over recent iterations
        let window = 5.min(self.quality_history.len());
        let recent_qualities: Vec<_> = self
            .quality_history
            .iter()
            .rev()
            .take(window)
            .map(|q| q.overall)
            .collect();

        let mut improvement_sum = 0.0;
        for i in 1..recent_qualities.len() {
            improvement_sum += recent_qualities[i - 1] - recent_qualities[i];
        }

        let avg_improvement = improvement_sum / (recent_qualities.len() - 1) as f64;

        // Convert to rate (positive is improving)
        (0.5 + avg_improvement * 5.0).clamp(0.0, 1.0)
    }

    /// Compute quality variance
    fn compute_quality_variance(&self) -> f64 {
        self.compute_recent_variance(10)
    }

    /// Compute improvement trend
    fn compute_improvement_trend(&self) -> f64 {
        if self.quality_history.len() < 10 {
            return 0.0;
        }

        // Compare recent average to older average
        let recent_window = 5;
        let old_window = 5;

        let recent_avg: f64 = self
            .quality_history
            .iter()
            .rev()
            .take(recent_window)
            .map(|q| q.overall)
            .sum::<f64>()
            / recent_window as f64;

        let old_avg: f64 = self
            .quality_history
            .iter()
            .rev()
            .skip(recent_window)
            .take(old_window)
            .map(|q| q.overall)
            .sum::<f64>()
            / old_window as f64;

        recent_avg - old_avg
    }

    /// Compute consistency score
    fn compute_consistency_score(&self) -> f64 {
        if self.metrics_history.len() < 5 {
            return 0.5;
        }

        // Check consistency of various metrics
        let inlier_consistency = self.compute_metric_consistency(|m| m.inlier_ratio);
        let error_consistency =
            self.compute_metric_consistency(|m| 1.0 / (1.0 + m.reprojection_error));
        let confidence_consistency = self.compute_metric_consistency(|m| m.detection_confidence);

        (inlier_consistency + error_consistency + confidence_consistency) / 3.0
    }

    /// Compute consistency of a specific metric
    fn compute_metric_consistency<F>(&self, extractor: F) -> f64
    where
        F: Fn(&CalibrationMetrics) -> f64,
    {
        let values: Vec<_> = self
            .metrics_history
            .iter()
            .rev()
            .take(10)
            .map(extractor)
            .collect();

        if values.len() < 2 {
            return 0.5;
        }

        let mean = values.iter().sum::<f64>() / values.len() as f64;
        let variance =
            values.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64;

        let std_dev = variance.sqrt();
        let cv = if mean > 0.0 { std_dev / mean } else { 1.0 };

        (1.0 / (1.0 + cv * 2.0)).clamp(0.0, 1.0)
    }

    /// Compute variance of recent quality scores
    fn compute_recent_variance(&self, window: usize) -> f64 {
        if self.quality_history.len() < window {
            return 0.0;
        }

        let recent_qualities: Vec<_> = self
            .quality_history
            .iter()
            .rev()
            .take(window)
            .map(|q| q.overall)
            .collect();

        let mean = recent_qualities.iter().sum::<f64>() / recent_qualities.len() as f64;
        let variance = recent_qualities
            .iter()
            .map(|&q| (q - mean).powi(2))
            .sum::<f64>()
            / recent_qualities.len() as f64;

        variance
    }

    /// Count significant quality drops
    fn count_quality_drops(&self) -> usize {
        if self.quality_history.len() < 2 {
            return 0;
        }

        let mut drops = 0;
        let threshold = 0.2; // 20% drop

        for i in 1..self.quality_history.len() {
            let prev = self.quality_history[i - 1].overall;
            let curr = self.quality_history[i].overall;

            if prev - curr > threshold {
                drops += 1;
            }
        }

        drops
    }

    /// Reset analyzer state
    pub fn reset(&mut self) {
        self.metrics_history.clear();
        self.quality_history.clear();
        self.detection_attempts = 0;
        self.successful_detections = 0;
    }
}

/// Confidence-based decision maker
pub struct ConfidenceDecisionMaker;

impl ConfidenceDecisionMaker {
    /// Decide if calibration should be accepted
    pub fn should_accept_calibration(confidence: &ConfidenceMetrics) -> bool {
        confidence.overall_confidence > 0.7
            && confidence.detection_stability > 0.6
            && confidence.false_positive_rate < 0.2
    }

    /// Decide if parameters need adjustment
    pub fn needs_parameter_adjustment(confidence: &ConfidenceMetrics) -> bool {
        confidence.overall_confidence < 0.5
            || confidence.detection_rate < 0.7
            || confidence.convergence_rate < 0.3
            || confidence.quality_variance > 0.3
    }

    /// Decide adjustment urgency
    pub fn adjustment_urgency(confidence: &ConfidenceMetrics) -> AdjustmentUrgency {
        if confidence.overall_confidence < 0.3 || confidence.detection_rate < 0.4 {
            AdjustmentUrgency::Critical
        } else if confidence.overall_confidence < 0.5 || confidence.detection_stability < 0.5 {
            AdjustmentUrgency::High
        } else if confidence.overall_confidence < 0.7 || confidence.convergence_rate < 0.5 {
            AdjustmentUrgency::Medium
        } else {
            AdjustmentUrgency::Low
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AdjustmentUrgency {
    Critical,
    High,
    Medium,
    Low,
}
