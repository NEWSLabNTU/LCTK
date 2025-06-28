//! Calibration convergence monitoring

use crate::CalibrationMetrics;
use nalgebra::{Isometry3, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Convergence status of calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConvergenceStatus {
    /// Whether calibration has converged
    pub is_converged: bool,
    /// Number of iterations/updates
    pub iterations: usize,
    /// Convergence rate (change per iteration)
    pub convergence_rate: f64,
    /// Estimated iterations to convergence
    pub estimated_iterations_remaining: Option<usize>,
    /// Convergence history
    pub history: ConvergenceHistory,
}

impl ConvergenceStatus {
    /// Get convergence score (0.0 to 1.0)
    pub fn convergence_score(&self) -> f64 {
        if self.is_converged {
            1.0
        } else {
            // Score based on convergence rate and iterations
            let rate_score = (-self.convergence_rate * 10.0).exp();
            let iteration_score = 1.0 - (1.0 / (1.0 + self.iterations as f64 / 10.0));
            (rate_score + iteration_score) / 2.0
        }
    }
}

/// History of convergence metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConvergenceHistory {
    /// Translation changes over iterations
    pub translation_changes: Vec<f64>,
    /// Rotation changes over iterations
    pub rotation_changes: Vec<f64>,
    /// Quality scores over iterations
    pub quality_scores: Vec<f64>,
    /// Timestamps
    pub timestamps: Vec<std::time::SystemTime>,
}

/// Monitor for tracking calibration convergence
pub struct ConvergenceMonitor {
    /// Convergence thresholds
    config: ConvergenceConfig,
    /// Previous transform
    previous_transform: Option<Isometry3<f64>>,
    /// History of transforms
    transform_history: VecDeque<(Isometry3<f64>, CalibrationMetrics)>,
    /// Maximum history size
    max_history: usize,
    /// Iteration counter
    iterations: usize,
}

/// Configuration for convergence monitoring
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConvergenceConfig {
    /// Translation change threshold for convergence (meters)
    pub translation_threshold: f64,
    /// Rotation change threshold for convergence (radians)
    pub rotation_threshold: f64,
    /// Minimum iterations before declaring convergence
    pub min_iterations: usize,
    /// Maximum iterations before timeout
    pub max_iterations: usize,
    /// Window size for convergence check
    pub convergence_window: usize,
    /// Quality improvement threshold
    pub quality_improvement_threshold: f64,
}

impl Default for ConvergenceConfig {
    fn default() -> Self {
        Self {
            translation_threshold: 0.001, // 1mm
            rotation_threshold: 0.001,    // ~0.06 degrees
            min_iterations: 5,
            max_iterations: 100,
            convergence_window: 5,
            quality_improvement_threshold: 0.01,
        }
    }
}

impl ConvergenceMonitor {
    /// Create a new convergence monitor
    pub fn new() -> Self {
        Self::with_config(ConvergenceConfig::default())
    }

    /// Create with custom configuration
    pub fn with_config(config: ConvergenceConfig) -> Self {
        Self {
            config,
            previous_transform: None,
            transform_history: VecDeque::new(),
            max_history: 50,
            iterations: 0,
        }
    }

    /// Update monitor with new calibration result
    pub fn update(&mut self, transform: &Isometry3<f64>, metrics: &CalibrationMetrics) {
        self.iterations += 1;

        // Add to history
        self.transform_history
            .push_back((*transform, metrics.clone()));
        if self.transform_history.len() > self.max_history {
            self.transform_history.pop_front();
        }

        self.previous_transform = Some(*transform);
    }

    /// Get current convergence status
    pub fn status(&self) -> ConvergenceStatus {
        let history = self.build_history();
        let is_converged = self.check_convergence(&history);
        let convergence_rate = self.calculate_convergence_rate(&history);
        let estimated_iterations_remaining = self.estimate_iterations_remaining(&history);

        ConvergenceStatus {
            is_converged,
            iterations: self.iterations,
            convergence_rate,
            estimated_iterations_remaining,
            history,
        }
    }

    /// Build convergence history
    fn build_history(&self) -> ConvergenceHistory {
        let mut translation_changes = Vec::new();
        let mut rotation_changes = Vec::new();
        let mut quality_scores = Vec::new();
        let mut timestamps = Vec::new();

        let current_time = std::time::SystemTime::now();
        let time_step = std::time::Duration::from_millis(100);

        for i in 1..self.transform_history.len() {
            let (prev_transform, _) = &self.transform_history[i - 1];
            let (curr_transform, curr_metrics) = &self.transform_history[i];

            let delta = prev_transform.inverse() * curr_transform;
            let translation_change = delta.translation.vector.norm();
            let rotation_change = delta.rotation.angle();

            translation_changes.push(translation_change);
            rotation_changes.push(rotation_change);
            quality_scores.push(curr_metrics.overall_score());
            timestamps
                .push(current_time - time_step * (self.transform_history.len() - i - 1) as u32);
        }

        ConvergenceHistory {
            translation_changes,
            rotation_changes,
            quality_scores,
            timestamps,
        }
    }

    /// Check if calibration has converged
    fn check_convergence(&self, history: &ConvergenceHistory) -> bool {
        if self.iterations < self.config.min_iterations {
            return false;
        }

        if self.iterations >= self.config.max_iterations {
            return true; // Force convergence at max iterations
        }

        let window = self
            .config
            .convergence_window
            .min(history.translation_changes.len());
        if window < self.config.convergence_window {
            return false; // Not enough history
        }

        // Check recent changes
        let recent_translation_changes: Vec<_> = history
            .translation_changes
            .iter()
            .rev()
            .take(window)
            .cloned()
            .collect();
        let recent_rotation_changes: Vec<_> = history
            .rotation_changes
            .iter()
            .rev()
            .take(window)
            .cloned()
            .collect();

        // All recent changes must be below threshold
        let translation_converged = recent_translation_changes
            .iter()
            .all(|&change| change < self.config.translation_threshold);
        let rotation_converged = recent_rotation_changes
            .iter()
            .all(|&change| change < self.config.rotation_threshold);

        // Check quality improvement
        let quality_stable = if history.quality_scores.len() >= window * 2 {
            let recent_quality: f64 = history
                .quality_scores
                .iter()
                .rev()
                .take(window)
                .sum::<f64>()
                / window as f64;
            let previous_quality: f64 = history
                .quality_scores
                .iter()
                .rev()
                .skip(window)
                .take(window)
                .sum::<f64>()
                / window as f64;

            (recent_quality - previous_quality).abs() < self.config.quality_improvement_threshold
        } else {
            true
        };

        translation_converged && rotation_converged && quality_stable
    }

    /// Calculate convergence rate
    fn calculate_convergence_rate(&self, history: &ConvergenceHistory) -> f64 {
        if history.translation_changes.len() < 2 {
            return 1.0;
        }

        // Average rate of change over recent iterations
        let window = 5.min(history.translation_changes.len());
        let recent_changes: Vec<_> = history
            .translation_changes
            .iter()
            .rev()
            .take(window)
            .cloned()
            .collect();

        if recent_changes.len() < 2 {
            return 1.0;
        }

        // Calculate average rate of decrease
        let mut rate_sum = 0.0;
        for i in 1..recent_changes.len() {
            let rate =
                (recent_changes[i] - recent_changes[i - 1]) / recent_changes[i - 1].max(1e-10);
            rate_sum += rate;
        }

        rate_sum / (recent_changes.len() - 1) as f64
    }

    /// Estimate iterations remaining to convergence
    fn estimate_iterations_remaining(&self, history: &ConvergenceHistory) -> Option<usize> {
        if history.translation_changes.is_empty() {
            return None;
        }

        let current_change = history.translation_changes.last()?;
        let rate = self.calculate_convergence_rate(history);

        if rate >= 0.0 {
            // Not converging
            return None;
        }

        // Estimate based on exponential decay
        let iterations_needed =
            (self.config.translation_threshold / current_change).ln() / rate.ln();

        Some(iterations_needed.abs() as usize)
    }

    /// Reset the monitor
    pub fn reset(&mut self) {
        self.previous_transform = None;
        self.transform_history.clear();
        self.iterations = 0;
    }
}

/// Adaptive convergence monitoring that adjusts thresholds
pub struct AdaptiveConvergenceMonitor {
    base_monitor: ConvergenceMonitor,
    threshold_adjuster: ThresholdAdjuster,
}

impl AdaptiveConvergenceMonitor {
    pub fn new() -> Self {
        Self {
            base_monitor: ConvergenceMonitor::new(),
            threshold_adjuster: ThresholdAdjuster::new(),
        }
    }

    /// Update with adaptive threshold adjustment
    pub fn update_adaptive(
        &mut self,
        transform: &Isometry3<f64>,
        metrics: &CalibrationMetrics,
    ) -> ConvergenceStatus {
        // Update base monitor
        self.base_monitor.update(transform, metrics);

        // Adjust thresholds based on current performance
        let adjusted_config = self.threshold_adjuster.adjust(
            &self.base_monitor.config,
            metrics,
            self.base_monitor.iterations,
        );
        self.base_monitor.config = adjusted_config;

        self.base_monitor.status()
    }
}

/// Threshold adjustment logic
struct ThresholdAdjuster {
    quality_history: VecDeque<f64>,
}

impl ThresholdAdjuster {
    fn new() -> Self {
        Self {
            quality_history: VecDeque::new(),
        }
    }

    fn adjust(
        &mut self,
        config: &ConvergenceConfig,
        metrics: &CalibrationMetrics,
        iterations: usize,
    ) -> ConvergenceConfig {
        let mut adjusted = config.clone();

        self.quality_history.push_back(metrics.overall_score());
        if self.quality_history.len() > 20 {
            self.quality_history.pop_front();
        }

        // Relax thresholds if quality is consistently high
        if self.quality_history.len() >= 10 {
            let avg_quality: f64 =
                self.quality_history.iter().sum::<f64>() / self.quality_history.len() as f64;

            if avg_quality > 0.9 {
                adjusted.translation_threshold *= 0.5;
                adjusted.rotation_threshold *= 0.5;
            } else if avg_quality < 0.5 && iterations > 20 {
                // Relax if struggling to converge
                adjusted.translation_threshold *= 2.0;
                adjusted.rotation_threshold *= 2.0;
            }
        }

        adjusted
    }
}
