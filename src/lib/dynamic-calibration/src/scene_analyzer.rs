//! Scene analysis for adaptive calibration

use anyhow::Result;
use calibration_quality::CalibrationMetrics;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Characteristics of the calibration scene
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneCharacteristics {
    /// Scene complexity (0.0 = simple, 1.0 = complex)
    pub complexity: f64,
    /// Noise level estimate (0.0 = clean, 1.0 = very noisy)
    pub noise_level: f64,
    /// Point cloud density (0.0 = sparse, 1.0 = dense)
    pub point_density: f64,
    /// Geometric structure regularity
    pub structure_regularity: f64,
    /// Occlusion level estimate
    pub occlusion_level: f64,
    /// Ambient light variation (for camera-based)
    pub ambient_light_variation: f64,
    /// Reflectivity variation
    pub reflectivity_variation: f64,
    /// Motion blur estimate
    pub motion_blur: f64,
    /// Environmental conditions score
    pub environmental_score: f64,
}

impl Default for SceneCharacteristics {
    fn default() -> Self {
        Self {
            complexity: 0.5,
            noise_level: 0.3,
            point_density: 0.7,
            structure_regularity: 0.7,
            occlusion_level: 0.2,
            ambient_light_variation: 0.3,
            reflectivity_variation: 0.3,
            motion_blur: 0.1,
            environmental_score: 0.7,
        }
    }
}

/// Analyzer for scene characteristics
pub struct SceneAnalyzer {
    /// History of scene characteristics
    history: VecDeque<SceneCharacteristics>,
    /// History of metrics for trend analysis
    metrics_history: VecDeque<CalibrationMetrics>,
    /// Maximum history size
    max_history: usize,
    /// Scene type detector
    scene_type: SceneType,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum SceneType {
    Unknown,
    Indoor,
    Outdoor,
    Industrial,
    Laboratory,
}

impl SceneAnalyzer {
    /// Create a new scene analyzer
    pub fn new() -> Self {
        Self {
            history: VecDeque::new(),
            metrics_history: VecDeque::new(),
            max_history: 30,
            scene_type: SceneType::Unknown,
        }
    }

    /// Analyze scene characteristics from calibration metrics
    pub fn analyze(&mut self, metrics: &CalibrationMetrics) -> Result<SceneCharacteristics> {
        // Update history
        self.metrics_history.push_back(metrics.clone());
        if self.metrics_history.len() > self.max_history {
            self.metrics_history.pop_front();
        }

        // Analyze various aspects
        let complexity = self.analyze_complexity(metrics);
        let noise_level = self.analyze_noise_level(metrics);
        let point_density = self.analyze_point_density(metrics);
        let structure_regularity = self.analyze_structure_regularity(metrics);
        let occlusion_level = self.analyze_occlusion_level(metrics);
        let ambient_light_variation = self.analyze_lighting_variation();
        let reflectivity_variation = self.analyze_reflectivity_variation();
        let motion_blur = self.analyze_motion_blur();
        let environmental_score = self.compute_environmental_score(metrics);

        // Update scene type
        self.update_scene_type(&SceneCharacteristics {
            complexity,
            noise_level,
            point_density,
            structure_regularity,
            occlusion_level,
            ambient_light_variation,
            reflectivity_variation,
            motion_blur,
            environmental_score,
        });

        let characteristics = SceneCharacteristics {
            complexity,
            noise_level,
            point_density,
            structure_regularity,
            occlusion_level,
            ambient_light_variation,
            reflectivity_variation,
            motion_blur,
            environmental_score,
        };

        // Update history
        self.history.push_back(characteristics.clone());
        if self.history.len() > self.max_history {
            self.history.pop_front();
        }

        Ok(characteristics)
    }

    /// Analyze scene complexity
    fn analyze_complexity(&self, metrics: &CalibrationMetrics) -> f64 {
        let mut complexity = 0.0;

        // Based on correspondence distribution
        if metrics.num_correspondences > 0 {
            let correspondence_ratio =
                metrics.num_inliers as f64 / metrics.num_correspondences as f64;
            complexity += (1.0 - correspondence_ratio) * 0.3;
        }

        // Based on geometric error distribution
        let error_variance = metrics.statistical_metrics.error_std_dev
            / (metrics.geometric_error.mean_translation_error + 0.001);
        complexity += error_variance.clamp(0.0, 1.0) * 0.3;

        // Based on outlier count
        let outlier_ratio = metrics.statistical_metrics.outlier_count as f64
            / metrics.num_correspondences.max(1) as f64;
        complexity += outlier_ratio * 0.4;

        complexity.clamp(0.0, 1.0)
    }

    /// Analyze noise level
    fn analyze_noise_level(&self, metrics: &CalibrationMetrics) -> f64 {
        let mut noise = 0.0;

        // Based on error statistics
        noise += (metrics.statistical_metrics.error_std_dev * 10.0).clamp(0.0, 1.0) * 0.4;

        // Based on outlier ratio
        let outlier_ratio = (1.0 - metrics.inlier_ratio).clamp(0.0, 1.0);
        noise += outlier_ratio * 0.3;

        // Based on high percentile errors
        let high_error_ratio = metrics.statistical_metrics.percentile_95_error
            / (metrics.statistical_metrics.median_error + 0.001);
        noise += (high_error_ratio / 10.0).clamp(0.0, 1.0) * 0.3;

        noise.clamp(0.0, 1.0)
    }

    /// Analyze point density
    fn analyze_point_density(&self, metrics: &CalibrationMetrics) -> f64 {
        // Estimate based on correspondence count
        let normalized_count = (metrics.num_correspondences as f64 / 100.0).clamp(0.0, 1.0);

        // Adjust based on inlier density
        let inlier_density = if metrics.num_correspondences > 0 {
            metrics.num_inliers as f64 / metrics.num_correspondences as f64
        } else {
            0.0
        };

        (normalized_count * 0.7 + inlier_density * 0.3).clamp(0.0, 1.0)
    }

    /// Analyze structure regularity
    fn analyze_structure_regularity(&self, metrics: &CalibrationMetrics) -> f64 {
        // Based on consistency score
        let consistency = metrics.consistency_score;

        // Based on error distribution uniformity
        let error_uniformity = 1.0
            - (metrics.statistical_metrics.error_std_dev
                / (metrics.geometric_error.mean_translation_error + 0.001))
                .clamp(0.0, 1.0);

        (consistency * 0.6 + error_uniformity * 0.4).clamp(0.0, 1.0)
    }

    /// Analyze occlusion level
    fn analyze_occlusion_level(&self, metrics: &CalibrationMetrics) -> f64 {
        // Estimate based on missing correspondences
        let expected_correspondences = 50.0; // Baseline expectation
        let correspondence_deficit =
            (expected_correspondences - metrics.num_correspondences as f64).max(0.0)
                / expected_correspondences;

        // Adjust based on spatial distribution (would need actual spatial data)
        let spatial_gaps = 1.0 - metrics.consistency_score;

        (correspondence_deficit * 0.7 + spatial_gaps * 0.3).clamp(0.0, 1.0)
    }

    /// Analyze lighting variation
    fn analyze_lighting_variation(&self) -> f64 {
        // This would require camera-specific metrics
        // For now, estimate based on detection confidence variation
        if self.metrics_history.len() < 5 {
            return 0.3; // Default
        }

        let confidences: Vec<_> = self
            .metrics_history
            .iter()
            .rev()
            .take(10)
            .map(|m| m.detection_confidence)
            .collect();

        let mean = confidences.iter().sum::<f64>() / confidences.len() as f64;
        let variance =
            confidences.iter().map(|&c| (c - mean).powi(2)).sum::<f64>() / confidences.len() as f64;

        variance.sqrt().clamp(0.0, 1.0)
    }

    /// Analyze reflectivity variation
    fn analyze_reflectivity_variation(&self) -> f64 {
        // Estimate based on error distribution patterns
        if let Some(latest) = self.metrics_history.back() {
            let error_range = latest.geometric_error.max_translation_error
                - latest.statistical_metrics.median_error;
            let normalized_range = (error_range * 5.0).clamp(0.0, 1.0);
            normalized_range
        } else {
            0.3 // Default
        }
    }

    /// Analyze motion blur
    fn analyze_motion_blur(&self) -> f64 {
        // Estimate based on temporal consistency
        if self.metrics_history.len() < 3 {
            return 0.1; // Default low blur
        }

        // Check for sudden quality drops that might indicate motion
        let mut blur_indicators = 0.0;
        let recent: Vec<_> = self.metrics_history.iter().rev().take(5).collect();

        for i in 1..recent.len() {
            let quality_drop = recent[i - 1].overall_score() - recent[i].overall_score();
            if quality_drop > 0.2 {
                blur_indicators += 1.0;
            }
        }

        (blur_indicators / recent.len() as f64).clamp(0.0, 1.0)
    }

    /// Compute environmental score
    fn compute_environmental_score(&self, metrics: &CalibrationMetrics) -> f64 {
        // Overall environmental conditions assessment
        let base_score = metrics.overall_score();

        // Penalize for high noise or low density
        let noise_penalty = self.analyze_noise_level(metrics) * 0.3;
        let density_bonus = self.analyze_point_density(metrics) * 0.3;

        (base_score - noise_penalty + density_bonus).clamp(0.0, 1.0)
    }

    /// Update scene type based on characteristics
    fn update_scene_type(&mut self, characteristics: &SceneCharacteristics) {
        if characteristics.structure_regularity > 0.8 && characteristics.noise_level < 0.3 {
            self.scene_type = SceneType::Laboratory;
        } else if characteristics.ambient_light_variation > 0.6
            && characteristics.environmental_score < 0.5
        {
            self.scene_type = SceneType::Outdoor;
        } else if characteristics.reflectivity_variation > 0.6 && characteristics.complexity > 0.7 {
            self.scene_type = SceneType::Industrial;
        } else if characteristics.occlusion_level < 0.3 && characteristics.point_density > 0.6 {
            self.scene_type = SceneType::Indoor;
        } else {
            self.scene_type = SceneType::Unknown;
        }
    }

    /// Get current scene type
    pub fn scene_type(&self) -> &str {
        match self.scene_type {
            SceneType::Unknown => "unknown",
            SceneType::Indoor => "indoor",
            SceneType::Outdoor => "outdoor",
            SceneType::Industrial => "industrial",
            SceneType::Laboratory => "laboratory",
        }
    }

    /// Get scene difficulty rating
    pub fn scene_difficulty(&self) -> SceneDifficulty {
        if let Some(latest) = self.history.back() {
            let difficulty_score = latest.complexity * 0.3
                + latest.noise_level * 0.3
                + (1.0 - latest.point_density) * 0.2
                + latest.occlusion_level * 0.2;

            if difficulty_score < 0.3 {
                SceneDifficulty::Easy
            } else if difficulty_score < 0.6 {
                SceneDifficulty::Medium
            } else if difficulty_score < 0.8 {
                SceneDifficulty::Hard
            } else {
                SceneDifficulty::Extreme
            }
        } else {
            SceneDifficulty::Medium
        }
    }

    /// Reset analyzer state
    pub fn reset(&mut self) {
        self.history.clear();
        self.metrics_history.clear();
        self.scene_type = SceneType::Unknown;
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SceneDifficulty {
    Easy,
    Medium,
    Hard,
    Extreme,
}

/// Scene-specific parameter recommendations
pub struct SceneBasedRecommendations;

impl SceneBasedRecommendations {
    /// Get recommended adjustments for scene type
    pub fn get_recommendations(
        scene_type: &str,
        characteristics: &SceneCharacteristics,
    ) -> ParameterRecommendations {
        match scene_type {
            "outdoor" => ParameterRecommendations {
                increase_outlier_threshold: true,
                increase_noise_filtering: true,
                reduce_min_inliers: characteristics.point_density < 0.5,
                increase_roi_size: true,
                enable_adaptive_downsampling: true,
            },
            "industrial" => ParameterRecommendations {
                increase_outlier_threshold: characteristics.reflectivity_variation > 0.6,
                increase_noise_filtering: false,
                reduce_min_inliers: false,
                increase_roi_size: false,
                enable_adaptive_downsampling: characteristics.complexity > 0.7,
            },
            "laboratory" => ParameterRecommendations {
                increase_outlier_threshold: false,
                increase_noise_filtering: false,
                reduce_min_inliers: false,
                increase_roi_size: false,
                enable_adaptive_downsampling: false,
            },
            _ => ParameterRecommendations::default(),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ParameterRecommendations {
    pub increase_outlier_threshold: bool,
    pub increase_noise_filtering: bool,
    pub reduce_min_inliers: bool,
    pub increase_roi_size: bool,
    pub enable_adaptive_downsampling: bool,
}
