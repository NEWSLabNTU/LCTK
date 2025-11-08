//! Hole pattern alignment using ICP
//!
//! This module aligns detected holes with the expected hole pattern
//! to improve detection accuracy and enable partial matching.

use super::{
    register_advanced, IcpRefinement, IcpStageConfig, PointCloud, RefinementResult,
    RegistrationSettings, SmallGicpConvergenceCriteria,
};
use crate::types::{DetectedHole, DetectionError};
use anyhow::Result;
use nalgebra::{Isometry3, Point3};

/// Hole pattern for matching
#[derive(Debug, Clone)]
pub struct HolePattern {
    /// Expected hole positions in board coordinates
    pub holes: Vec<HoleTemplate>,
    /// Minimum number of holes required for matching
    pub min_holes: usize,
}

/// Template for a single hole
#[derive(Debug, Clone)]
pub struct HoleTemplate {
    /// Position in board coordinates
    pub position: Point3<f64>,
    /// Expected radius
    pub radius: f64,
    /// Hole variant (for asymmetric patterns)
    pub variant: u8,
}

impl IcpRefinement {
    /// Align detected holes with expected pattern
    ///
    /// Uses point-to-point ICP to find the best alignment between
    /// detected holes and the expected pattern, enabling robust
    /// matching even with partial hole visibility.
    pub fn align_hole_pattern(
        &self,
        detected_holes: &[DetectedHole],
        expected_pattern: &HolePattern,
        initial_guess: Option<&Isometry3<f64>>,
        config: Option<&IcpStageConfig>,
    ) -> Result<RefinementResult> {
        let stage_config = config.unwrap_or(&self.config.hole_pattern_alignment);

        if !stage_config.enabled {
            return Ok(RefinementResult {
                transformation: initial_guess.cloned().unwrap_or_else(Isometry3::identity),
                fitness: 1.0,
                num_inliers: detected_holes.len() as i32,
                covariance: None,
                converged: true,
                iterations: 0,
            });
        }

        // Check if we have enough holes
        if detected_holes.len() < expected_pattern.min_holes {
            return Err(DetectionError::InsufficientData("Target holes empty".to_string()).into());
        }

        // Convert holes to point clouds
        let detected_points: Vec<Point3<f64>> = detected_holes.iter().map(|h| h.center).collect();

        let pattern_points: Vec<Point3<f64>> =
            expected_pattern.holes.iter().map(|h| h.position).collect();

        // Create point clouds
        let source = PointCloud::from_points(detected_points.clone());
        let target = PointCloud::from_points(pattern_points.clone());

        // For hole matching, we typically don't need preprocessing
        // as we have few points and want exact matching

        // Create settings
        let settings = RegistrationSettings {
            registration_type: self.convert_registration_type(&stage_config.registration_type),
            num_threads: self.config.num_threads,
            max_iterations: stage_config.max_iterations,
            convergence_criteria: SmallGicpConvergenceCriteria {
                rotation_epsilon: stage_config.convergence_criteria.rotation_epsilon,
                translation_epsilon: stage_config.convergence_criteria.translation_epsilon,
            },
            initial_guess: initial_guess.cloned(),
        };

        // Create robust kernel for outlier rejection
        let robust_kernel = self.create_robust_kernel(&stage_config.robust_kernel)?;

        // Build KdTree
        let target_tree = target.build_kdtree()?;

        // Perform registration
        let result = register_advanced(
            &target,
            &source,
            &target_tree,
            &settings,
            robust_kernel.as_ref(),
            None, // No DOF restriction for hole patterns
        )?;

        // Validate alignment quality
        let aligned_holes = apply_transform_to_holes(detected_holes, &result.transformation);
        let match_score = compute_pattern_match_score(&aligned_holes, expected_pattern);

        Ok(RefinementResult {
            transformation: result.transformation,
            fitness: match_score,
            num_inliers: result.num_inliers,
            covariance: None,
            converged: result.converged,
            iterations: result.iterations,
        })
    }

    /// Find correspondence between detected and expected holes
    pub fn find_hole_correspondences(
        &self,
        detected_holes: &[DetectedHole],
        expected_pattern: &HolePattern,
        transform: &Isometry3<f64>,
        max_distance: f64,
    ) -> Vec<(usize, usize)> {
        let mut correspondences = Vec::new();
        let transformed_holes = apply_transform_to_holes(detected_holes, transform);

        // For each detected hole, find closest pattern hole
        for (i, detected) in transformed_holes.iter().enumerate() {
            let mut best_j = None;
            let mut best_dist = max_distance;

            for (j, pattern) in expected_pattern.holes.iter().enumerate() {
                let dist = (detected.center - pattern.position).norm();
                if dist < best_dist {
                    best_dist = dist;
                    best_j = Some(j);
                }
            }

            if let Some(j) = best_j {
                correspondences.push((i, j));
            }
        }

        correspondences
    }
}

/// Apply transformation to detected holes
pub fn apply_transform_to_holes(
    holes: &[DetectedHole],
    transform: &Isometry3<f64>,
) -> Vec<DetectedHole> {
    holes
        .iter()
        .map(|hole| DetectedHole {
            center: transform * hole.center,
            radius: hole.radius,
            confidence: hole.confidence,
            id: hole.id.clone(),
        })
        .collect()
}

/// Compute pattern matching score
pub fn compute_pattern_match_score(aligned_holes: &[DetectedHole], pattern: &HolePattern) -> f64 {
    let max_distance = 0.05; // 5cm tolerance
    let mut matched = 0;
    let mut total_error = 0.0;

    for hole in aligned_holes {
        // Find closest pattern hole
        let mut min_dist = f64::MAX;
        for pattern_hole in &pattern.holes {
            let dist = (hole.center - pattern_hole.position).norm();
            min_dist = min_dist.min(dist);
        }

        if min_dist < max_distance {
            matched += 1;
            total_error += min_dist;
        }
    }

    if matched == 0 {
        return 0.0;
    }

    // Compute score based on matches and average error
    let match_ratio = matched as f64 / pattern.holes.len().max(aligned_holes.len()) as f64;
    let avg_error = total_error / matched as f64;
    let error_score = (1.0 - avg_error / max_distance).max(0.0);

    match_ratio * error_score
}

/// Create standard 3-hole asymmetric pattern
pub fn create_standard_hole_pattern(board_size: f64) -> HolePattern {
    let offset = board_size * 0.3;

    HolePattern {
        holes: vec![
            HoleTemplate {
                position: Point3::new(0.0, 0.0, 0.0),
                radius: 0.1,
                variant: 0, // Large center hole
            },
            HoleTemplate {
                position: Point3::new(offset, 0.0, 0.0),
                radius: 0.05,
                variant: 1, // Small hole
            },
            HoleTemplate {
                position: Point3::new(0.0, offset, 0.0),
                radius: 0.05,
                variant: 1, // Small hole
            },
        ],
        min_holes: 2, // Allow partial matching with 2/3 holes
    }
}

/// Simple hole pattern refinement using just point lists
pub fn refine_hole_pattern(
    detected_points: &[Point3<f64>],
    expected_points: &[Point3<f64>],
    refiner: &IcpRefinement,
) -> Result<RefinementResult> {
    if detected_points.len() < 2 || expected_points.len() < 2 {
        return Err(DetectionError::InsufficientData("Source holes empty".to_string()).into());
    }

    // Create HolePattern from expected points
    let pattern = HolePattern {
        holes: expected_points
            .iter()
            .map(|&p| HoleTemplate {
                position: p,
                radius: 0.05, // Default radius
                variant: 0,
            })
            .collect(),
        min_holes: 2,
    };

    // Create DetectedHole from detected points
    let detected_holes: Vec<DetectedHole> = detected_points
        .iter()
        .map(|&p| DetectedHole {
            center: p,
            radius: 0.05, // Default radius
            confidence: crate::types::DetectionConfidence::new(1.0),
            id: None,
        })
        .collect();

    // Use the full alignment method
    refiner.align_hole_pattern(&detected_holes, &pattern, None, None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::refinement::IcpRefinementConfig;

    #[test]
    fn test_pattern_match_score() {
        let pattern = create_standard_hole_pattern(1.0);

        // Perfect match
        let perfect_holes: Vec<DetectedHole> = pattern
            .holes
            .iter()
            .map(|h| DetectedHole {
                center: h.position,
                radius: h.radius,
                confidence: crate::types::DetectionConfidence::new(1.0),
                id: None,
            })
            .collect();

        let score = compute_pattern_match_score(&perfect_holes, &pattern);
        assert!(score > 0.99, "Perfect match should have high score");

        // Partial match (2/3 holes)
        let partial_holes = &perfect_holes[..2];
        let partial_score = compute_pattern_match_score(partial_holes, &pattern);
        assert!(
            partial_score > 0.5,
            "Partial match should have reasonable score"
        );
        assert!(
            partial_score < score,
            "Partial match should score lower than perfect"
        );
    }

    #[test]
    fn test_apply_transform_to_holes() {
        let holes = vec![DetectedHole {
            center: Point3::new(1.0, 0.0, 0.0),
            radius: 0.1,
            confidence: crate::types::DetectionConfidence::new(0.9),
            id: None,
        }];

        let transform = Isometry3::translation(1.0, 2.0, 3.0);
        let transformed = apply_transform_to_holes(&holes, &transform);

        assert_eq!(transformed[0].center, Point3::new(2.0, 2.0, 3.0));
        assert_eq!(transformed[0].radius, 0.1);
        assert_eq!(transformed[0].confidence.score(), 0.9);
    }

    #[test]
    fn test_hole_pattern_alignment_disabled() {
        use super::super::{IcpRefinementConfig, IcpStageConfig};

        let config = IcpRefinementConfig {
            hole_pattern_alignment: IcpStageConfig {
                enabled: false,
                ..Default::default()
            },
            ..Default::default()
        };

        let refiner = super::IcpRefinement::new(config);
        let detected_holes = vec![DetectedHole {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 0.1,
            confidence: crate::types::DetectionConfidence::new(0.9),
            id: None,
        }];

        let pattern = HolePattern {
            holes: vec![HoleTemplate {
                position: Point3::new(0.0, 0.0, 0.0),
                radius: 0.1,
                variant: 0,
            }],
            min_holes: 1,
        };

        let result = refiner
            .align_hole_pattern(&detected_holes, &pattern, None, None)
            .unwrap();

        // Should return identity transform when disabled
        assert_eq!(result.transformation, Isometry3::identity());
        assert_eq!(result.fitness, 1.0);
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn test_find_hole_correspondences() {
        let refiner = super::IcpRefinement::new(IcpRefinementConfig::default());

        let detected_holes = vec![
            DetectedHole {
                center: Point3::new(0.0, 0.0, 0.0),
                radius: 0.1,
                confidence: crate::types::DetectionConfidence::new(0.9),
                id: Some("hole1".to_string()),
            },
            DetectedHole {
                center: Point3::new(0.3, 0.0, 0.0),
                radius: 0.05,
                confidence: crate::types::DetectionConfidence::new(0.9),
                id: Some("hole2".to_string()),
            },
        ];

        let pattern = create_standard_hole_pattern(1.0);
        let transform = Isometry3::identity();

        let correspondences = refiner.find_hole_correspondences(
            &detected_holes,
            &pattern,
            &transform,
            0.1, // 10cm max distance
        );

        // Should find at least one correspondence
        assert!(!correspondences.is_empty());
        assert_eq!(correspondences[0].0, 0); // First detected hole
        assert_eq!(correspondences[0].1, 0); // First pattern hole
    }

    #[test]
    fn test_refine_hole_pattern_insufficient_data() {
        // Test with too few points
        let detected_points = vec![Point3::new(0.0, 0.0, 0.0)];
        let expected_points = vec![Point3::new(0.0, 0.0, 0.0)];
        let refiner = super::IcpRefinement::new(IcpRefinementConfig::default());

        let result = refine_hole_pattern(&detected_points, &expected_points, &refiner);
        assert!(result.is_err());
    }
}
