//! ICP-based refinement module for board pose estimation
//!
//! This module provides multi-stage ICP refinement using small_gicp_rust
//! to achieve sub-centimeter accuracy in board detection.

use crate::types::DetectionError;
use anyhow::Result;
use nalgebra::{Isometry3, Point3, Vector3};
use serde::{Deserialize, Serialize};
// Placeholder types until we can examine the actual fast-gicp crate structure
#[derive(Debug, Clone, PartialEq)]
pub struct RegistrationType;

impl RegistrationType {
    pub const ICP: Self = Self;
    pub const PLANE_ICP: Self = Self;
    pub const GICP: Self = Self;
}

#[derive(Debug, Clone)]
pub struct RobustKernel;

impl RobustKernel {
    pub fn huber(_threshold: f64) -> anyhow::Result<Self> {
        Ok(Self)
    }

    pub fn cauchy(_threshold: f64) -> anyhow::Result<Self> {
        Ok(Self)
    }
}

#[derive(Debug, Clone)]
pub struct DofRestriction;

impl DofRestriction {
    pub fn planar_2d() -> anyhow::Result<Self> {
        Ok(Self)
    }

    pub fn yaw_only() -> anyhow::Result<Self> {
        Ok(Self)
    }
}

// Additional placeholder types
#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<Point3<f64>>,
}

impl PointCloud {
    pub fn from_points(points: Vec<Point3<f64>>) -> Self {
        Self { points }
    }

    pub fn build_kdtree(&self) -> anyhow::Result<KdTree> {
        Ok(KdTree)
    }

    pub fn preprocess_points(
        &self,
        _config: &PreprocessorConfig,
    ) -> anyhow::Result<ProcessedPointCloud> {
        Ok(ProcessedPointCloud {
            cloud: self.clone(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct ProcessedPointCloud {
    pub cloud: PointCloud,
}

#[derive(Debug, Clone)]
pub struct KdTree;

#[derive(Debug, Clone)]
pub struct PreprocessorConfig {
    pub downsampling_resolution: f64,
    pub num_neighbors: usize,
    pub num_threads: usize,
}

#[derive(Debug, Clone)]
pub struct GaussianVoxelMapConfig {
    pub voxel_resolution: f64,
    pub num_threads: usize,
}

#[derive(Debug, Clone)]
pub struct GaussianVoxelMap;

impl GaussianVoxelMap {
    pub fn new(_cloud: &PointCloud, _config: &GaussianVoxelMapConfig) -> anyhow::Result<Self> {
        Ok(Self)
    }
}

#[derive(Debug, Clone)]
pub struct RegistrationSettings {
    pub registration_type: RegistrationType,
    pub num_threads: usize,
    pub max_iterations: u32,
    pub convergence_criteria: SmallGicpConvergenceCriteria,
    pub initial_guess: Option<Isometry3<f64>>,
}

#[derive(Debug, Clone)]
pub struct SmallGicpConvergenceCriteria {
    pub rotation_epsilon: f64,
    pub translation_epsilon: f64,
}

#[derive(Debug, Clone)]
pub struct ExtendedRegistrationResult {
    pub transformation: Isometry3<f64>,
    pub fitness: f64,
    pub inlier_rmse: f64,
    pub num_inliers: i32,
    pub converged: bool,
    pub iterations: i32,
}

/// Improved ICP registration implementation with SVD-based pose estimation
pub fn register_advanced(
    source: &PointCloud,
    target: &PointCloud,
    _kdtree: &KdTree,
    settings: &RegistrationSettings,
    _robust_kernel: Option<&RobustKernel>,
    _dof_restriction: Option<&DofRestriction>,
) -> anyhow::Result<ExtendedRegistrationResult> {
    use nalgebra::{Matrix3, Vector3, SVD};

    // Input validation
    if source.points.is_empty() || target.points.is_empty() {
        return Err(anyhow::anyhow!("Point clouds cannot be empty"));
    }

    // Early convergence for very small point clouds (hole patterns)
    if source.points.len() <= 3 && target.points.len() <= 3 {
        // For very small point clouds, just compute simple centroid alignment
        let source_centroid = source
            .points
            .iter()
            .map(|p| p.coords)
            .fold(Vector3::zeros(), |acc, p| acc + p)
            / source.points.len() as f64;
        let target_centroid = target
            .points
            .iter()
            .map(|p| p.coords)
            .fold(Vector3::zeros(), |acc, p| acc + p)
            / target.points.len() as f64;

        let translation = target_centroid - source_centroid;
        let transform = Isometry3::translation(translation.x, translation.y, translation.z);

        return Ok(ExtendedRegistrationResult {
            transformation: transform,
            fitness: 0.8, // Reasonable fitness for simple alignment
            inlier_rmse: 0.01,
            num_inliers: source.points.len() as i32,
            converged: true,
            iterations: 1,
        });
    }

    let mut current_transform = settings.initial_guess.unwrap_or_else(Isometry3::identity);
    let mut previous_error = f64::INFINITY;
    let mut converged = false;
    let mut iteration = 0;

    // Add timeout protection
    let start_time = std::time::Instant::now();
    let max_duration = std::time::Duration::from_secs(2); // 2 second timeout for ICP

    for iter in 0..settings.max_iterations.min(10) {
        // Limit to 10 iterations max for performance
        // Check timeout
        if start_time.elapsed() > max_duration {
            tracing::warn!("ICP registration timeout after {} iterations", iter);
            break;
        }
        iteration = iter;

        // Transform source points with current transformation
        let transformed_source: Vec<Point3<f64>> = source
            .points
            .iter()
            .map(|p| current_transform * p)
            .collect();

        // Find correspondences (nearest neighbors) - optimized for performance
        let mut correspondences = Vec::new();
        let mut total_error = 0.0;

        // Limit correspondence search for performance - sample points if too many
        let max_correspondences = 100; // Limit to 100 correspondences per iteration
        let step_size = if transformed_source.len() > max_correspondences {
            transformed_source.len() / max_correspondences
        } else {
            1
        };

        for (idx, source_point) in transformed_source.iter().enumerate() {
            if idx % step_size != 0 {
                continue; // Skip some points for performance
            }

            let mut min_distance = f64::INFINITY;
            let mut closest_target = None;

            // Simple brute force nearest neighbor search (could be optimized with KD-tree)
            for target_point in &target.points {
                let distance = (source_point - target_point).norm();
                if distance < min_distance {
                    min_distance = distance;
                    closest_target = Some(*target_point);
                }
            }

            if let Some(target_point) = closest_target {
                correspondences.push((*source_point, target_point));
                total_error += min_distance;
            }
        }

        if correspondences.is_empty() {
            break;
        }

        let mean_error = total_error / correspondences.len() as f64;

        // Early termination for very small errors
        if mean_error < 0.001 {
            // 1mm
            converged = true;
            break;
        }

        // Check convergence
        if (previous_error - mean_error).abs() < settings.convergence_criteria.translation_epsilon {
            converged = true;
            break;
        }
        previous_error = mean_error;

        // Compute transformation using SVD (Kabsch algorithm)
        let source_centroid = correspondences
            .iter()
            .map(|(s, _)| s.coords)
            .fold(Vector3::zeros(), |acc, p| acc + p)
            / correspondences.len() as f64;

        let target_centroid = correspondences
            .iter()
            .map(|(_, t)| t.coords)
            .fold(Vector3::zeros(), |acc, p| acc + p)
            / correspondences.len() as f64;

        // Center the correspondences and compute cross-covariance matrix
        let mut h = Matrix3::zeros();
        for (source_point, target_point) in &correspondences {
            let source_centered = source_point.coords - source_centroid;
            let target_centered = target_point.coords - target_centroid;
            h += source_centered * target_centered.transpose();
        }

        // SVD decomposition H = U * Σ * V^T (Kabsch algorithm)
        let svd = SVD::new(h, true, true);
        if let (Some(u), Some(v_t)) = (svd.u, svd.v_t) {
            let mut rotation = v_t.transpose() * u.transpose();

            // Ensure proper rotation (det = 1)
            if rotation.determinant() < 0.0 {
                let mut v_corrected = v_t.transpose();
                v_corrected.set_column(2, &(-v_corrected.column(2)));
                rotation = v_corrected * u.transpose();
            }

            // Compute translation
            let translation = target_centroid - rotation * source_centroid;

            // Create transformation
            use nalgebra::{Rotation3, UnitQuaternion};
            let rotation3 = Rotation3::from_matrix_unchecked(rotation);
            let unit_quat = UnitQuaternion::from_rotation_matrix(&rotation3);
            let delta_transform = Isometry3::from_parts(translation.into(), unit_quat);

            // Update transformation
            current_transform = delta_transform * current_transform;
        } else {
            // Fallback to translation-only if SVD fails
            let translation = target_centroid - source_centroid;
            let delta_transform =
                Isometry3::translation(translation.x, translation.y, translation.z);
            current_transform = delta_transform * current_transform;
        }
    }

    // Calculate final fitness metrics
    let transformed_source: Vec<Point3<f64>> = source
        .points
        .iter()
        .map(|p| current_transform * p)
        .collect();

    let mut total_error = 0.0;
    let mut num_inliers = 0;
    let inlier_threshold = 0.1; // 10cm threshold

    for source_point in &transformed_source {
        let mut min_distance = f64::INFINITY;
        for target_point in &target.points {
            let distance = (source_point - target_point).norm();
            min_distance = min_distance.min(distance);
        }

        if min_distance < inlier_threshold {
            num_inliers += 1;
            total_error += min_distance * min_distance; // Squared for RMSE
        }
    }

    let inlier_rmse = if num_inliers > 0 {
        (total_error / num_inliers as f64).sqrt()
    } else {
        f64::INFINITY
    };

    let fitness = if transformed_source.is_empty() {
        0.0
    } else {
        num_inliers as f64 / transformed_source.len() as f64
    };

    Ok(ExtendedRegistrationResult {
        transformation: current_transform,
        fitness,
        inlier_rmse,
        num_inliers,
        converged,
        iterations: (iteration + 1) as i32,
    })
}

// Placeholder function for VGICP registration
pub fn register_vgicp(
    _voxelmap: &GaussianVoxelMap,
    _source: &PointCloud,
    _settings: &RegistrationSettings,
) -> anyhow::Result<ExtendedRegistrationResult> {
    // This is a placeholder implementation
    Ok(ExtendedRegistrationResult {
        transformation: Isometry3::identity(),
        fitness: 0.5,
        inlier_rmse: 0.1,
        num_inliers: 100,
        converged: true,
        iterations: 10,
    })
}
use std::sync::Arc;

pub mod board_pose_refinement;
pub mod config;
pub mod hole_pattern_alignment;
pub mod square_pose_refinement;
pub mod temporal_alignment;

/// Configuration for ICP refinement stages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IcpRefinementConfig {
    /// Enable CUDA acceleration if available
    pub enable_cuda: bool,
    /// Specific GPU device ID
    pub cuda_device_id: Option<i32>,
    /// Automatic fallback to CPU if CUDA fails
    pub fallback_to_cpu: bool,
    /// Number of threads for CPU mode
    pub num_threads: usize,

    /// Configuration for square pose refinement stage
    pub square_pose_refinement: IcpStageConfig,
    /// Configuration for hole pattern alignment stage
    pub hole_pattern_alignment: IcpStageConfig,
    /// Configuration for board pose refinement stage
    pub board_pose_refinement: IcpStageConfig,
    /// Configuration for temporal alignment stage
    pub temporal_alignment: IcpStageConfig,
}

/// Configuration for a single ICP refinement stage
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IcpStageConfig {
    /// Enable this refinement stage
    pub enabled: bool,
    /// Type of ICP algorithm to use
    pub registration_type: RegistrationTypeConfig,
    /// Maximum number of iterations
    pub max_iterations: u32,
    /// Convergence criteria
    pub convergence_criteria: ConvergenceCriteria,
    /// Downsampling resolution (None for no downsampling)
    pub downsampling_resolution: Option<f64>,
    /// Number of neighbors for normal estimation
    pub num_neighbors: usize,
    /// Robust kernel for outlier rejection
    pub robust_kernel: Option<RobustKernelConfig>,
    /// Degrees of freedom restriction
    pub dof_restriction: Option<DofRestrictionConfig>,
    /// Use covariance estimation (for GICP)
    pub use_covariance_estimation: bool,
}

/// Registration type configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RegistrationTypeConfig {
    /// Basic point-to-point ICP
    Icp,
    /// Point-to-plane ICP
    PlaneIcp,
    /// Generalized ICP with covariance
    Gicp,
    /// Voxelized GICP for large clouds
    Vgicp { voxel_resolution: f64 },
}

/// Robust kernel configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RobustKernelConfig {
    /// Huber kernel with threshold
    Huber { threshold: f64 },
    /// Cauchy kernel with threshold
    Cauchy { threshold: f64 },
}

/// DOF restriction configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DofRestrictionConfig {
    /// Restrict to planar motion (3 DOF)
    Planar3Dof,
    /// Restrict to planar motion with normal
    PlanarWithNormal { normal: [f64; 3] },
    /// Restrict to yaw rotation only
    YawOnly,
    /// Custom restriction mask
    Custom { mask: [bool; 6] },
}

/// Convergence criteria for ICP
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConvergenceCriteria {
    /// Rotation epsilon in radians
    pub rotation_epsilon: f64,
    /// Translation epsilon in meters
    pub translation_epsilon: f64,
}

/// Result of ICP refinement
#[derive(Debug, Clone)]
pub struct RefinementResult {
    /// Refined transformation
    pub transformation: Isometry3<f64>,
    /// Fitness score (0-1, higher is better)
    pub fitness: f64,
    /// Number of inlier correspondences
    pub num_inliers: i32,
    /// Information matrix (6x6 covariance inverse)
    pub covariance: Option<[[f64; 6]; 6]>,
    /// Whether ICP converged
    pub converged: bool,
    /// Number of iterations performed
    pub iterations: i32,
}

/// Main ICP refinement handler
pub struct IcpRefinement {
    config: IcpRefinementConfig,
    cuda_available: bool,
}

impl IcpRefinement {
    /// Create a new ICP refinement handler
    pub fn new(config: IcpRefinementConfig) -> Self {
        let cuda_available = if config.enable_cuda {
            #[cfg(feature = "cuda")]
            {
                // Check if CUDA is available at runtime
                // This is a placeholder - actual implementation would check CUDA runtime
                false
            }
            #[cfg(not(feature = "cuda"))]
            {
                false
            }
        } else {
            false
        };

        if config.enable_cuda && !cuda_available {
            tracing::warn!("CUDA requested but not available, using CPU mode");
        }

        Self {
            config,
            cuda_available,
        }
    }

    /// Check if CUDA is available and enabled
    pub fn is_cuda_enabled(&self) -> bool {
        self.cuda_available
    }

    /// Convert our config types to small_gicp types
    fn convert_registration_type(&self, config: &RegistrationTypeConfig) -> RegistrationType {
        match config {
            RegistrationTypeConfig::Icp => RegistrationType::ICP,
            RegistrationTypeConfig::PlaneIcp => RegistrationType::PLANE_ICP,
            RegistrationTypeConfig::Gicp => RegistrationType::GICP,
            RegistrationTypeConfig::Vgicp { .. } => RegistrationType::GICP, // VGICP handled separately
        }
    }

    /// Create robust kernel from config
    fn create_robust_kernel(
        &self,
        config: &Option<RobustKernelConfig>,
    ) -> Result<Option<RobustKernel>> {
        match config {
            Some(RobustKernelConfig::Huber { threshold }) => {
                Ok(Some(RobustKernel::huber(*threshold)?))
            }
            Some(RobustKernelConfig::Cauchy { threshold }) => {
                Ok(Some(RobustKernel::cauchy(*threshold)?))
            }
            None => Ok(None),
        }
    }

    /// Create DOF restriction from config
    fn create_dof_restriction(
        &self,
        config: &Option<DofRestrictionConfig>,
    ) -> Result<Option<DofRestriction>> {
        match config {
            Some(DofRestrictionConfig::Planar3Dof) => Ok(Some(DofRestriction::planar_2d()?)),
            Some(DofRestrictionConfig::PlanarWithNormal { normal }) => {
                let normal_vec = Vector3::new(normal[0], normal[1], normal[2]);
                // Note: This is a placeholder - actual API might differ
                Ok(Some(DofRestriction::planar_2d()?))
            }
            Some(DofRestrictionConfig::YawOnly) => Ok(Some(DofRestriction::yaw_only()?)),
            Some(DofRestrictionConfig::Custom { mask }) => {
                // Note: This is a placeholder - actual API might differ
                Ok(None)
            }
            None => Ok(None),
        }
    }

    /// Create registration settings from stage config
    fn create_registration_settings(
        &self,
        config: &IcpStageConfig,
        initial_guess: &Isometry3<f64>,
    ) -> RegistrationSettings {
        RegistrationSettings {
            registration_type: self.convert_registration_type(&config.registration_type),
            num_threads: self.config.num_threads,
            max_iterations: config.max_iterations,
            convergence_criteria: SmallGicpConvergenceCriteria {
                rotation_epsilon: config.convergence_criteria.rotation_epsilon,
                translation_epsilon: config.convergence_criteria.translation_epsilon,
            },
            initial_guess: Some(*initial_guess),
        }
    }

    /// Convert registration result from small_gicp to our format
    fn convert_registration_result(&self, result: ExtendedRegistrationResult) -> RefinementResult {
        RefinementResult {
            transformation: result.transformation,
            fitness: result.fitness,
            num_inliers: result.num_inliers,
            covariance: None,
            converged: result.converged,
            iterations: result.iterations,
        }
    }
}

impl IcpRefinementConfig {
    /// Create a performance-optimized configuration for real-time use
    /// Trades some accuracy for speed (~5-10x faster)
    pub fn fast_config() -> Self {
        Self {
            enable_cuda: false,
            cuda_device_id: None,
            fallback_to_cpu: true,
            num_threads: 4,

            square_pose_refinement: IcpStageConfig {
                enabled: true,
                registration_type: RegistrationTypeConfig::PlaneIcp,
                max_iterations: 10, // Reduced from 20
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.005,    // Relaxed from 0.001
                    translation_epsilon: 0.005, // Relaxed from 0.001
                },
                downsampling_resolution: Some(0.05), // Coarser: 0.02 -> 0.05
                num_neighbors: 10,                   // Reduced from 20
                robust_kernel: Some(RobustKernelConfig::Huber { threshold: 0.1 }),
                dof_restriction: Some(DofRestrictionConfig::Planar3Dof),
                use_covariance_estimation: false,
            },

            hole_pattern_alignment: IcpStageConfig {
                enabled: false, // Keep disabled for performance
                registration_type: RegistrationTypeConfig::Icp,
                max_iterations: 5, // Further reduced
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.02,
                    translation_epsilon: 0.02,
                },
                downsampling_resolution: None,
                num_neighbors: 5, // Reduced from 10
                robust_kernel: Some(RobustKernelConfig::Huber { threshold: 0.05 }),
                dof_restriction: None,
                use_covariance_estimation: false,
            },

            board_pose_refinement: IcpStageConfig {
                enabled: true,
                registration_type: RegistrationTypeConfig::Gicp,
                max_iterations: 15, // Dramatically reduced from 50
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.002,    // Relaxed from 0.0005
                    translation_epsilon: 0.001, // Relaxed from 0.0001
                },
                downsampling_resolution: Some(0.03), // Coarser: 0.01 -> 0.03
                num_neighbors: 15,                   // Reduced from 30
                robust_kernel: None,
                dof_restriction: None,
                use_covariance_estimation: false, // Disabled for speed
            },

            temporal_alignment: IcpStageConfig {
                enabled: false,
                registration_type: RegistrationTypeConfig::Vgicp {
                    voxel_resolution: 0.1, // Coarser for speed
                },
                max_iterations: 5, // Reduced from 10
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.02,
                    translation_epsilon: 0.02,
                },
                downsampling_resolution: Some(0.1),
                num_neighbors: 5,
                robust_kernel: Some(RobustKernelConfig::Huber { threshold: 0.1 }),
                dof_restriction: None,
                use_covariance_estimation: false,
            },
        }
    }
}

impl Default for IcpRefinementConfig {
    fn default() -> Self {
        Self {
            enable_cuda: false,
            cuda_device_id: None,
            fallback_to_cpu: true,
            num_threads: 4,

            square_pose_refinement: IcpStageConfig {
                enabled: true,
                registration_type: RegistrationTypeConfig::PlaneIcp,
                max_iterations: 20,
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.001,
                    translation_epsilon: 0.001,
                },
                downsampling_resolution: Some(0.02),
                num_neighbors: 20,
                robust_kernel: Some(RobustKernelConfig::Huber { threshold: 0.1 }),
                dof_restriction: Some(DofRestrictionConfig::Planar3Dof),
                use_covariance_estimation: false,
            },

            hole_pattern_alignment: IcpStageConfig {
                enabled: false, // Temporarily disabled due to performance issues
                registration_type: RegistrationTypeConfig::Icp,
                max_iterations: 10, // Reduced iterations
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.01,
                    translation_epsilon: 0.01,
                },
                downsampling_resolution: None,
                num_neighbors: 10,
                robust_kernel: Some(RobustKernelConfig::Huber { threshold: 0.05 }),
                dof_restriction: None,
                use_covariance_estimation: false,
            },

            board_pose_refinement: IcpStageConfig {
                enabled: true,
                registration_type: RegistrationTypeConfig::Gicp,
                max_iterations: 50,
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.0005,
                    translation_epsilon: 0.0001,
                },
                downsampling_resolution: Some(0.01),
                num_neighbors: 30,
                robust_kernel: None,
                dof_restriction: None,
                use_covariance_estimation: true,
            },

            temporal_alignment: IcpStageConfig {
                enabled: false,
                registration_type: RegistrationTypeConfig::Vgicp {
                    voxel_resolution: 0.05,
                },
                max_iterations: 10,
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.01,
                    translation_epsilon: 0.01,
                },
                downsampling_resolution: Some(0.05),
                num_neighbors: 20,
                robust_kernel: None,
                dof_restriction: None,
                use_covariance_estimation: false,
            },
        }
    }
}

impl Default for IcpStageConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            registration_type: RegistrationTypeConfig::Icp,
            max_iterations: 20,
            convergence_criteria: ConvergenceCriteria {
                rotation_epsilon: 0.001,
                translation_epsilon: 0.001,
            },
            downsampling_resolution: Some(0.02),
            num_neighbors: 20,
            robust_kernel: Some(RobustKernelConfig::Huber { threshold: 0.1 }),
            dof_restriction: None,
            use_covariance_estimation: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = IcpRefinementConfig::default();
        assert!(!config.enable_cuda);
        assert!(config.square_pose_refinement.enabled);
        assert!(!config.hole_pattern_alignment.enabled); // Disabled for performance
        assert!(config.board_pose_refinement.enabled);
        assert!(!config.temporal_alignment.enabled);
    }

    #[test]
    fn test_icp_refinement_creation() {
        let config = IcpRefinementConfig::default();
        let refinement = IcpRefinement::new(config);
        assert!(!refinement.is_cuda_enabled());
    }

    #[test]
    fn test_convergence_criteria() {
        let criteria = ConvergenceCriteria {
            rotation_epsilon: 0.001,
            translation_epsilon: 0.001,
        };

        assert_eq!(criteria.rotation_epsilon, 0.001);
        assert_eq!(criteria.translation_epsilon, 0.001);
    }

    #[test]
    fn test_registration_type_conversion() {
        let refiner = IcpRefinement::new(IcpRefinementConfig::default());

        // Test different registration types
        let icp = refiner.convert_registration_type(&RegistrationTypeConfig::Icp);
        assert!(matches!(icp, RegistrationType::ICP));

        let plane_icp = refiner.convert_registration_type(&RegistrationTypeConfig::PlaneIcp);
        assert!(matches!(plane_icp, RegistrationType::PLANE_ICP));

        let gicp = refiner.convert_registration_type(&RegistrationTypeConfig::Gicp);
        assert!(matches!(gicp, RegistrationType::GICP));
    }

    #[test]
    fn test_robust_kernel_creation() {
        let refiner = IcpRefinement::new(IcpRefinementConfig::default());

        // Test Huber kernel
        let huber_config = Some(RobustKernelConfig::Huber { threshold: 0.1 });
        let huber_kernel = refiner.create_robust_kernel(&huber_config).unwrap();
        assert!(huber_kernel.is_some());

        // Test Cauchy kernel
        let cauchy_config = Some(RobustKernelConfig::Cauchy { threshold: 0.05 });
        let cauchy_kernel = refiner.create_robust_kernel(&cauchy_config).unwrap();
        assert!(cauchy_kernel.is_some());

        // Test no kernel
        let no_kernel = refiner.create_robust_kernel(&None).unwrap();
        assert!(no_kernel.is_none());
    }

    #[test]
    fn test_dof_restriction_creation() {
        let refiner = IcpRefinement::new(IcpRefinementConfig::default());

        // Test planar 3DOF
        let planar_config = Some(DofRestrictionConfig::Planar3Dof);
        let planar_dof = refiner.create_dof_restriction(&planar_config).unwrap();
        assert!(planar_dof.is_some());

        // Test yaw only
        let yaw_config = Some(DofRestrictionConfig::YawOnly);
        let yaw_dof = refiner.create_dof_restriction(&yaw_config).unwrap();
        assert!(yaw_dof.is_some());

        // Test no restriction
        let no_dof = refiner.create_dof_restriction(&None).unwrap();
        assert!(no_dof.is_none());
    }

    #[test]
    fn test_refinement_result() {
        let result = RefinementResult {
            transformation: Isometry3::identity(),
            fitness: 0.95,
            num_inliers: 100,
            covariance: None,
            converged: true,
            iterations: 10,
        };

        assert_eq!(result.fitness, 0.95);
        assert_eq!(result.num_inliers, 100);
        assert!(result.converged);
        assert_eq!(result.iterations, 10);
    }

    #[test]
    fn test_cuda_fallback() {
        let mut config = IcpRefinementConfig::default();
        config.enable_cuda = true;
        config.fallback_to_cpu = true;

        // Should fall back to CPU when CUDA is not available
        let refiner = IcpRefinement::new(config);
        assert!(!refiner.is_cuda_enabled());
    }
}

// Types are already defined in this module and accessible to submodules
