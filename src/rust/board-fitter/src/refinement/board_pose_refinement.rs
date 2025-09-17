//! Final board pose refinement using GICP
//!
//! This module provides high-precision refinement of the complete board pose
//! using Generalized ICP with covariance estimation.

use super::{
    register_advanced, register_vgicp, GaussianVoxelMap, GaussianVoxelMapConfig, IcpRefinement,
    IcpStageConfig, PointCloud, PreprocessorConfig, RefinementResult, RegistrationTypeConfig,
};
use anyhow::Result;
use nalgebra::{Isometry3, Point3};

impl IcpRefinement {
    /// Refine the final board pose using GICP
    ///
    /// This is the most accurate refinement stage, using the complete board
    /// point cloud and an ideal board model to achieve sub-centimeter accuracy.
    pub fn refine_board_pose(
        &self,
        board_points: &[Point3<f64>],
        template_points: &[Point3<f64>],
        initial_transform: &Isometry3<f64>,
        config: Option<&IcpStageConfig>,
    ) -> Result<RefinementResult> {
        let stage_config = config.unwrap_or(&self.config.board_pose_refinement);

        if !stage_config.enabled {
            return Ok(RefinementResult {
                transformation: *initial_transform,
                fitness: 1.0,
                num_inliers: board_points.len() as i32,
                covariance: None,
                converged: true,
                iterations: 0,
            });
        }

        // Create point clouds
        let source = PointCloud::from_points(board_points.to_vec());
        let target = PointCloud::from_points(template_points.to_vec());

        // Preprocess if downsampling is requested
        let (source_processed, target_processed) =
            if let Some(resolution) = stage_config.downsampling_resolution {
                let preprocess_config = PreprocessorConfig {
                    downsampling_resolution: resolution,
                    num_neighbors: stage_config.num_neighbors,
                    num_threads: self.config.num_threads,
                };

                let source_prep = source.preprocess_points(&preprocess_config)?;
                let target_prep = target.preprocess_points(&preprocess_config)?;
                (source_prep.cloud, target_prep.cloud)
            } else {
                (source, target)
            };

        // Perform registration based on type
        let result = match &stage_config.registration_type {
            RegistrationTypeConfig::Vgicp { voxel_resolution } => {
                // Use VGICP for large point clouds
                let voxel_config = GaussianVoxelMapConfig {
                    voxel_resolution: *voxel_resolution,
                    num_threads: self.config.num_threads,
                };
                let voxelmap = GaussianVoxelMap::new(&target_processed, &voxel_config)?;

                let settings = self.create_registration_settings(stage_config, initial_transform);
                register_vgicp(&voxelmap, &source_processed, &settings)?
            }
            _ => {
                // Use standard registration for other types
                let settings = self.create_registration_settings(stage_config, initial_transform);
                let robust_kernel = self.create_robust_kernel(&stage_config.robust_kernel)?;
                let dof_restriction = self.create_dof_restriction(&stage_config.dof_restriction)?;

                // Build KdTree for target
                let target_tree = target_processed.build_kdtree()?;

                register_advanced(
                    &target_processed,
                    &source_processed,
                    &target_tree,
                    &settings,
                    robust_kernel.as_ref(),
                    dof_restriction.as_ref(),
                )?
            }
        };

        // Convert result
        Ok(self.convert_registration_result(result))
    }
}

/// Generate an ideal board template for refinement
pub fn generate_board_template(
    board_config: &board_fitter_config::SquareBoard,
    grid_spacing: f64,
) -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    let board_size = board_config.size.as_meters();

    // Generate grid of points on the board surface
    let grid_points = (board_size / grid_spacing) as i32;
    let half_size = board_size / 2.0;

    for i in 0..=grid_points {
        for j in 0..=grid_points {
            let x = -half_size + (i as f64) * grid_spacing;
            let y = -half_size + (j as f64) * grid_spacing;

            // Check if point is not inside a hole
            let mut in_hole = false;
            for hole in &board_config.holes {
                let dx = x - hole.position.x.as_meters();
                let dy = y - hole.position.y.as_meters();
                let radius = hole.radius.as_meters();
                if dx * dx + dy * dy < radius * radius {
                    in_hole = true;
                    break;
                }
            }

            if !in_hole {
                // Add point on board surface (z = 0 in board coordinates)
                points.push(Point3::new(x, y, 0.0));
            }
        }
    }

    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use board_fitter_config::{CircleHole, Point2D, SquareBoard};
    use measurements::Length;

    #[test]
    fn test_generate_board_template() {
        let mut board = SquareBoard::new(Length::from_meters(1.0));
        board.holes = vec![
            CircleHole {
                position: Point2D {
                    x: Length::from_meters(0.0),
                    y: Length::from_meters(0.0),
                },
                radius: Length::from_meters(0.1),
                id: Some("hole0".to_string()),
            },
            CircleHole {
                position: Point2D {
                    x: Length::from_meters(0.3),
                    y: Length::from_meters(0.0),
                },
                radius: Length::from_meters(0.05),
                id: Some("hole1".to_string()),
            },
            CircleHole {
                position: Point2D {
                    x: Length::from_meters(0.0),
                    y: Length::from_meters(0.3),
                },
                radius: Length::from_meters(0.05),
                id: Some("hole2".to_string()),
            },
        ];

        let template = generate_board_template(&board, 0.05);

        // Should have points but not in hole areas
        assert!(!template.is_empty());

        // No points should be inside holes
        for point in &template {
            for hole in &board.holes {
                let dx = point.x - hole.position.x.as_meters();
                let dy = point.y - hole.position.y.as_meters();
                let radius = hole.radius.as_meters();
                let dist_sq = dx * dx + dy * dy;
                assert!(dist_sq >= radius * radius);
            }
        }
    }

    #[test]
    fn test_refine_board_pose_disabled() {
        use super::super::{IcpRefinement, IcpRefinementConfig, IcpStageConfig};
        use nalgebra::Isometry3;

        let config = IcpRefinementConfig {
            board_pose_refinement: IcpStageConfig {
                enabled: false,
                ..Default::default()
            },
            ..Default::default()
        };

        let refiner = IcpRefinement::new(config);
        let board_points = vec![Point3::new(0.0, 0.0, 0.0)];
        let template_points = vec![Point3::new(0.0, 0.0, 0.0)];
        let initial_transform = Isometry3::identity();

        let result = refiner
            .refine_board_pose(&board_points, &template_points, &initial_transform, None)
            .unwrap();

        // Should return identity transform when disabled
        assert_eq!(result.transformation, initial_transform);
        assert_eq!(result.fitness, 1.0);
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn test_board_template_no_holes() {
        let board = SquareBoard::new(Length::from_meters(1.0));
        let template = generate_board_template(&board, 0.1);

        // Should have points covering the board
        assert!(!template.is_empty());

        // All points should be within board bounds
        for point in &template {
            assert!(point.x >= -0.5 && point.x <= 0.5);
            assert!(point.y >= -0.5 && point.y <= 0.5);
            assert_eq!(point.z, 0.0);
        }
    }
}
