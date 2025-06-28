//! Square pose refinement after PCA fitting
//!
//! This module refines the initial PCA-based square orientation using
//! PlaneICP with DOF restrictions.

use super::{
    register_advanced, IcpRefinement, IcpStageConfig, KdTree, PointCloud, PreprocessorConfig,
    ProcessedPointCloud, RefinementResult, RegistrationSettings,
};
use crate::{
    diamond::DiamondSquare,
    types::{DetectedPlane, DetectionError},
};
use anyhow::Result;
use nalgebra::{Isometry3, Point3, Vector3};

impl IcpRefinement {
    /// Refine square pose after initial PCA fitting
    ///
    /// Uses PlaneICP with planar DOF restriction to improve the initial
    /// PCA-based square orientation estimation.
    pub fn refine_square_pose(
        &self,
        square_points: &[Point3<f64>],
        square_size: f64,
        initial_pose: &Isometry3<f64>,
        plane_normal: &Vector3<f64>,
        config: Option<&IcpStageConfig>,
    ) -> Result<RefinementResult> {
        let stage_config = config.unwrap_or(&self.config.square_pose_refinement);

        if !stage_config.enabled {
            return Ok(RefinementResult {
                transformation: *initial_pose,
                fitness: 1.0,
                num_inliers: square_points.len() as i32,
                covariance: None,
                converged: true,
                iterations: 0,
            });
        }

        // Generate ideal square template
        let template_points = generate_square_template(square_size, 0.02);

        // Transform template to initial pose for better convergence
        let transformed_template: Vec<Point3<f64>> =
            template_points.iter().map(|p| initial_pose * p).collect();

        // Create point clouds
        let source = PointCloud::from_points(square_points.to_vec());
        let target = PointCloud::from_points(transformed_template);

        // Preprocess with normal estimation for PlaneICP
        let preprocess_config = PreprocessorConfig {
            downsampling_resolution: stage_config.downsampling_resolution.unwrap_or(0.02),
            num_neighbors: stage_config.num_neighbors,
            num_threads: self.config.num_threads,
        };

        let source_processed = source.preprocess_points(&preprocess_config)?;
        let target_processed = target.preprocess_points(&preprocess_config)?;

        // Create settings for PlaneICP
        let settings = self.create_registration_settings(stage_config, &Isometry3::identity());

        // Use planar DOF restriction aligned with the detected plane
        let dof_restriction = self.create_dof_restriction(&stage_config.dof_restriction)?;

        // Create robust kernel for noisy data
        let robust_kernel = self.create_robust_kernel(&stage_config.robust_kernel)?;

        // Build KdTree
        let target_tree = target_processed.cloud.build_kdtree()?;

        // Perform registration
        let result = register_advanced(
            &target_processed.cloud,
            &source_processed.cloud,
            &target_tree,
            &settings,
            robust_kernel.as_ref(),
            dof_restriction.as_ref(),
        )?;

        // Combine initial pose with refinement
        let refined_transform = initial_pose * result.transformation;

        Ok(RefinementResult {
            transformation: refined_transform,
            fitness: result.fitness,
            num_inliers: result.num_inliers,
            covariance: None,
            converged: result.converged,
            iterations: result.iterations,
        })
    }
}

/// Generate ideal square template points
pub fn generate_square_template(size: f64, spacing: f64) -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    let half_size = size / 2.0;
    let n_points = (size / spacing) as i32;

    // Generate points on square boundary with higher density
    let boundary_spacing = spacing / 2.0;
    let n_boundary = (size / boundary_spacing) as i32;

    // Top and bottom edges
    for i in 0..=n_boundary {
        let x = -half_size + (i as f64) * boundary_spacing;
        points.push(Point3::new(x, -half_size, 0.0)); // Bottom
        points.push(Point3::new(x, half_size, 0.0)); // Top
    }

    // Left and right edges (skip corners to avoid duplicates)
    for i in 1..n_boundary {
        let y = -half_size + (i as f64) * boundary_spacing;
        points.push(Point3::new(-half_size, y, 0.0)); // Left
        points.push(Point3::new(half_size, y, 0.0)); // Right
    }

    // Add some interior points for robustness
    for i in 1..n_points {
        for j in 1..n_points {
            let x = -half_size + (i as f64) * spacing;
            let y = -half_size + (j as f64) * spacing;
            points.push(Point3::new(x, y, 0.0));
        }
    }

    points
}

/// Apply square pose refinement to detected square
pub fn refine_detected_square(
    square: &DiamondSquare,
    points: &[Point3<f64>],
    plane: &DetectedPlane,
    refiner: &IcpRefinement,
) -> Result<DiamondSquare> {
    let refinement =
        refiner.refine_square_pose(points, square.size, &square.pose, &plane.normal, None)?;

    Ok(DiamondSquare {
        center: refinement.transformation * Point3::origin(),
        size: square.size,
        pose: refinement.transformation,
        corners: square.corners,
        normal: square.normal,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_square_template() {
        let template = generate_square_template(1.0, 0.1);

        // Should have boundary and interior points
        assert!(!template.is_empty());

        // All points should be within square bounds
        for point in &template {
            assert!(point.x >= -0.5 && point.x <= 0.5);
            assert!(point.y >= -0.5 && point.y <= 0.5);
            assert_eq!(point.z, 0.0);
        }
    }
}
