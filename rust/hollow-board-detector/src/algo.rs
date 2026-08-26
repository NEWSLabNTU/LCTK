use crate::{
    config::Config,
    detection::{BoardIcpState, BoardModelParams, FitPlaneRansac, PlaneRansacData},
};
use ahash::AHashMap;
use anyhow::Result;
use arrsac::Arrsac;
use hollow_board_config::BoardModel;
use log::warn;
// `debug!` survives only in the parallel voxel-downsample path.
#[cfg(feature = "parallel")]
use log::debug;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use plane_estimator::PlaneEstimator;
use sample_consensus::Consensus;
use std::f64::{self};

#[cfg(feature = "parallel")]
use {dashmap::DashMap, rayon::prelude::*};

unzip_n::unzip_n!(2);

/// Voxel grid key for 3D space partitioning
type VoxelKey = (i32, i32, i32);

/// Compute voxel grid key for a point
#[inline]
pub fn compute_voxel_key(point: Point3<f64>, voxel_size: f64) -> VoxelKey {
    (
        (point.x / voxel_size).floor() as i32,
        (point.y / voxel_size).floor() as i32,
        (point.z / voxel_size).floor() as i32,
    )
}

/// Compute centroid of points
#[inline]
pub fn compute_centroid(points: &[Point3<f64>]) -> Point3<f64> {
    let n = points.len() as f64;
    let sum = points
        .iter()
        .fold(Point3::origin(), |acc, p| acc + p.coords);
    Point3::from(sum.coords / n)
}

/// Sequential voxel downsampling using AHashMap
fn voxel_downsample_sequential(
    points: &[Point3<f64>],
    voxel_size: f64,
    use_centroid: bool,
) -> Vec<Point3<f64>> {
    if points.is_empty() {
        return Vec::new();
    }

    let estimated_capacity = points.len() / 3;

    if use_centroid {
        // Collect points per voxel, compute centroids
        let mut voxel_map: AHashMap<VoxelKey, Vec<Point3<f64>>> =
            AHashMap::with_capacity(estimated_capacity);

        for &point in points {
            let key = compute_voxel_key(point, voxel_size);
            voxel_map.entry(key).or_default().push(point);
        }

        voxel_map
            .into_values()
            .map(|pts| compute_centroid(&pts))
            .collect()
    } else {
        // Keep first point in each voxel
        let mut voxel_map: AHashMap<VoxelKey, Point3<f64>> =
            AHashMap::with_capacity(estimated_capacity);

        for &point in points {
            let key = compute_voxel_key(point, voxel_size);
            voxel_map.entry(key).or_insert(point);
        }

        voxel_map.into_values().collect()
    }
}

/// Parallel voxel downsampling using DashMap + rayon
#[cfg(feature = "parallel")]
fn voxel_downsample_parallel(
    points: &[Point3<f64>],
    voxel_size: f64,
    use_centroid: bool,
) -> Vec<Point3<f64>> {
    if points.is_empty() {
        return Vec::new();
    }

    if use_centroid {
        // Parallel insertion into DashMap
        let voxel_map: DashMap<VoxelKey, Vec<Point3<f64>>> = DashMap::new();

        points.par_iter().for_each(|&point| {
            let key = compute_voxel_key(point, voxel_size);
            voxel_map.entry(key).or_default().push(point);
        });

        // Parallel centroid computation
        voxel_map
            .into_par_iter()
            .map(|(_, pts)| compute_centroid(&pts))
            .collect()
    } else {
        // Parallel first-point strategy
        let voxel_map = DashMap::new();

        points.par_iter().for_each(|&point| {
            let key = compute_voxel_key(point, voxel_size);
            voxel_map.entry(key).or_insert(point);
        });

        voxel_map.into_par_iter().map(|(_, pt)| pt).collect()
    }
}

/// Main voxel downsampling function with automatic parallel dispatch
///
/// Reduces point cloud density while preserving spatial distribution.
///
/// # Arguments
/// * `points` - Input point cloud
/// * `voxel_size` - Voxel grid resolution in meters (e.g., 0.02 = 2cm)
/// * `use_centroid` - true: average points in voxel, false: keep first point
/// * `parallel_threshold` - Use parallel processing if point count >= threshold
///
/// # Performance
/// - Sequential: O(N) time complexity with AHash (3-10x faster than SipHash)
/// - Parallel: 2-3x speedup for >50K points (requires 'parallel' feature)
/// - Pre-allocated HashMap reduces reallocation overhead
pub fn voxel_downsample(
    points: &[Point3<f64>],
    voxel_size: f64,
    use_centroid: bool,
    #[cfg_attr(not(feature = "parallel"), allow(unused_variables))] parallel_threshold: usize,
) -> Vec<Point3<f64>> {
    #[cfg(feature = "parallel")]
    {
        if points.len() >= parallel_threshold {
            debug!(
                "Using parallel voxel downsampling ({} points)",
                points.len()
            );
            return voxel_downsample_parallel(points, voxel_size, use_centroid);
        }
    }

    voxel_downsample_sequential(points, voxel_size, use_centroid)
}

/// Fits a plane in a point set using RANSAC algorithm.
pub fn fit_plane_ransac<'a>(
    board_detector: &Config,
    points: &'a [Point3<f64>],
) -> Result<Option<FitPlaneRansac<'a>>> {
    let Config {
        plane_ransac_inlier_threshold,
        plane_ransac_max_iterations,
        ..
    } = *board_detector;

    if points.len() < 3 {
        warn!(
            "RANSAC failed: Need at least 3 points, got {}",
            points.len()
        );
        return Ok(None);
    }

    let mut arrsac = Arrsac::new(plane_ransac_inlier_threshold, rand::thread_rng())
        .max_candidate_hypotheses(plane_ransac_max_iterations);
    let estimator = PlaneEstimator::new();

    let (mut plane_model, inlier_indices) = {
        match arrsac.model_inliers(&estimator, points.iter().cloned()) {
            Some(ret) => ret,
            None => {
                warn!("RANSAC failed: No valid plane found");
                return Ok(None);
            }
        }
    };

    {
        // M-03: orient the plane normal to point from the board toward the sensor
        // (the origin), i.e. along the viewing direction. The previous code forced
        // the normal onto +X, which only holds when the board sits in front of the
        // sensor along +X; for a sensor whose forward axis is not +X (or a board
        // mounted to the side) that flips the board frame into the wrong hemisphere.
        // Using the board centroid generalizes to any placement and reproduces the
        // old behavior for the common in-front (+X) rig.
        let centroid: Vector3<f64> = {
            let n = inlier_indices.len().max(1) as f64;
            let sum = inlier_indices
                .iter()
                .fold(Vector3::zeros(), |acc, &idx| acc + points[idx].coords);
            sum / n
        };
        let current_normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
        // Keep the normal pointing along the board direction (away from the sensor),
        // matching the original +X convention when the board is in front.
        if current_normal.dot(&centroid) < 0.0 {
            let flipped = nalgebra::Unit::new_normalize(-current_normal);
            plane_model.normal = flipped;
        }
    }

    let inlier_points: Vec<_> = inlier_indices.into_iter().map(|idx| &points[idx]).collect();

    let viz_msg = PlaneRansacData {
        plane_model: plane_model.clone(),
        inlier_points: inlier_points.iter().map(|point| **point).collect(),
    };

    Ok(Some(FitPlaneRansac {
        plane_model,
        inlier_points,
        ransac_data: viz_msg,
    }))
}

/// Board ICP iterator for step-by-step execution
pub struct BoardIcpIterator<'a> {
    board_detector_config: &'a Config,
    board_model_params: BoardModelParams,
    progress_callback: Option<&'a mut dyn FnMut(&BoardModel)>,
}

impl<'a> BoardIcpIterator<'a> {
    /// Create a new board ICP iterator
    pub fn new(
        board_detector_config: &'a Config,
        board_model_params: BoardModelParams,
        progress_callback: Option<&'a mut dyn FnMut(&BoardModel)>,
    ) -> Self {
        Self {
            board_detector_config,
            board_model_params,
            progress_callback,
        }
    }

    /// Create initial state from plane inlier points and initial pose
    pub fn initial_state(
        &self,
        initial_pose: Isometry3<f64>,
        initial_inlier_points: Vec<Point3<f64>>,
    ) -> BoardIcpState {
        BoardIcpState {
            iteration: 0,
            board_pose: initial_pose,
            inlier_points: initial_inlier_points,
            correspondences: Vec::new(),
            avg_loss: f64::INFINITY,
            previous_loss: None,
            total_correspondences: 0,
            good_correspondences: 0,
            termination_count: 0,
        }
    }

    /// Execute one ICP iteration step
    pub fn step(&mut self, current_state: &BoardIcpState) -> BoardIcpState {
        let board_model = BoardModel {
            pose: current_state.board_pose,
            board_shape: self.board_model_params.board_shape.clone(),
            marker_paper_size: self.board_model_params.marker_paper_size,
            marker_paper_placement: self.board_model_params.marker_paper_placement,
        };

        if let Some(cb) = self.progress_callback.as_mut() {
            cb(&board_model);
        }

        let correspondences = match board_model.find_correspondences(&current_state.inlier_points) {
            Some(corr) => corr,
            None => {
                return BoardIcpState {
                    iteration: current_state.iteration + 1,
                    correspondences: Vec::new(),
                    avg_loss: f64::INFINITY,
                    previous_loss: Some(current_state.avg_loss),
                    total_correspondences: 0,
                    good_correspondences: 0,
                    ..current_state.clone()
                };
            }
        };

        let total_correspondences = correspondences.len();

        #[cfg(feature = "parallel")]
        let correspondence_losses: Vec<_> = correspondences
            .par_iter()
            .map(|(input_point, corresponding_point)| (*input_point - corresponding_point).norm())
            .collect();

        #[cfg(not(feature = "parallel"))]
        let correspondence_losses: Vec<_> = correspondences
            .iter()
            .map(|(input_point, corresponding_point)| (*input_point - corresponding_point).norm())
            .collect();

        let avg_loss = correspondence_losses.iter().sum::<f64>() / correspondences.len() as f64;

        // Filter correspondences by outlier threshold
        let outlier_threshold = self.board_detector_config.icp_outlier_threshold;
        let good_correspondences: Vec<_> = correspondences
            .iter()
            .filter(|(input_point, corresponding_point)| {
                let loss = (**input_point - *corresponding_point).norm();
                loss <= outlier_threshold
            })
            .map(|(input_point, corresponding_point)| (**input_point, *corresponding_point))
            .collect();

        let good_correspondences_len = good_correspondences.len();

        if good_correspondences_len < 3 {
            return BoardIcpState {
                iteration: current_state.iteration + 1,
                correspondences: good_correspondences,
                avg_loss,
                previous_loss: Some(current_state.avg_loss),
                total_correspondences,
                good_correspondences: good_correspondences_len,
                ..current_state.clone()
            };
        }

        let good_correspondences_for_state: Vec<(Point3<f64>, Point3<f64>)> =
            good_correspondences.clone();
        let (good_corresponding_points, good_inlier_points): (Vec<Point3<f64>>, Vec<Point3<f64>>) =
            good_correspondences.into_iter().unzip();

        let align_pose: Isometry3<f64> =
            match Self::compute_kabsch_transform(&good_corresponding_points, &good_inlier_points) {
                Some(iso) => iso.inverse(),
                None => {
                    return BoardIcpState {
                        iteration: current_state.iteration + 1,
                        correspondences: good_correspondences_for_state,
                        avg_loss,
                        previous_loss: Some(current_state.avg_loss),
                        total_correspondences,
                        good_correspondences: good_correspondences_len,
                        ..current_state.clone()
                    };
                }
            };

        let new_pose = align_pose * current_state.board_pose;
        let damping_factor = self.board_detector_config.icp_damping_factor;

        let damped_translation = Translation3::from(
            current_state.board_pose.translation.vector
                + (new_pose.translation.vector - current_state.board_pose.translation.vector)
                    * damping_factor,
        );

        let damped_rotation = UnitQuaternion::slerp(
            &current_state.board_pose.rotation,
            &new_pose.rotation,
            damping_factor,
        );

        let applied_t =
            (damped_translation.vector - current_state.board_pose.translation.vector).norm();
        let applied_ang = damped_rotation
            .rotation_to(&current_state.board_pose.rotation)
            .angle();
        let pose_weight = applied_t + applied_ang;

        let termination_count =
            if pose_weight <= self.board_detector_config.icp_pose_weight_threshold {
                current_state.termination_count + 1
            } else {
                0
            };

        let damped_pose = Isometry3::from_parts(damped_translation, damped_rotation);

        BoardIcpState {
            iteration: current_state.iteration + 1,
            board_pose: damped_pose,
            inlier_points: current_state.inlier_points.clone(),
            correspondences: good_correspondences_for_state,
            avg_loss,
            previous_loss: Some(current_state.avg_loss),
            total_correspondences,
            good_correspondences: good_correspondences_len,
            termination_count,
        }
    }

    /// Check if algorithm should terminate
    pub fn should_terminate(&self, state: &BoardIcpState) -> bool {
        let config = self.board_detector_config;

        // Success criteria - algorithm should terminate
        if state.avg_loss < config.icp_rejection_threshold {
            return true;
        }

        if state.termination_count > 100 {
            return true;
        }

        // Failure criteria - algorithm should terminate
        if state.iteration >= config.max_icp_iterations {
            return true;
        }

        if state.inlier_points.len() < config.icp_min_inlier_points {
            return true;
        }

        if state.good_correspondences < 3 {
            return true;
        }

        if state.correspondences.is_empty() {
            return true;
        }

        false
    }

    /// Get termination reason
    pub fn termination_reason(&self, state: &BoardIcpState) -> String {
        let config = self.board_detector_config;

        // Success reasons (order matches should_terminate)
        if state.avg_loss < config.icp_rejection_threshold {
            "Converged (good fit)".to_string()
        } else if state.termination_count > 100 {
            "Converged (stable pose)".to_string()
        }
        // Failure reasons
        else if state.iteration >= config.max_icp_iterations {
            format!("Max iterations reached: {}", config.max_icp_iterations)
        } else if state.inlier_points.len() < config.icp_min_inlier_points {
            format!(
                "Insufficient inlier points: {} < {}",
                state.inlier_points.len(),
                config.icp_min_inlier_points
            )
        } else if state.good_correspondences < 3 {
            format!(
                "Insufficient points for Kabsch: {}",
                state.good_correspondences
            )
        } else if state.correspondences.is_empty() {
            "No correspondences found".to_string()
        } else {
            "Unknown".to_string()
        }
    }

    /// Helper to compute Kabsch transformation using nalgebra
    fn compute_kabsch_transform(
        input_points: &[Point3<f64>],
        target_points: &[Point3<f64>],
    ) -> Option<Isometry3<f64>> {
        if input_points.len() != target_points.len() || input_points.len() < 3 {
            return None;
        }

        // Compute centroids
        let input_centroid = Self::compute_centroid(input_points)?;
        let target_centroid = Self::compute_centroid(target_points)?;

        // Center the points
        let centered_input: Vec<Vector3<f64>> =
            input_points.iter().map(|p| p - input_centroid).collect();
        let centered_target: Vec<Vector3<f64>> =
            target_points.iter().map(|p| p - target_centroid).collect();

        // Create matrices
        let input_matrix = nalgebra::Matrix3xX::from_columns(&centered_input);
        let target_matrix = nalgebra::Matrix3xX::from_columns(&centered_target);

        // Compute covariance matrix H = sum(input_i * target_i^T)
        // With column-major matrices: input_matrix * target_matrix.transpose()
        let covariance = input_matrix * target_matrix.transpose();

        // SVD decomposition: H = U * S * V^T
        let svd = nalgebra::SVD::new(covariance, true, true);
        let u = svd.u?;
        let v_t = svd.v_t?;

        // Standard Kabsch algorithm: R = V * diag(1, 1, det(V * U^T)) * U^T
        // Since nalgebra SVD gives us V^T, we need to transpose it to get V
        let v = v_t.transpose();
        let u_t = u.transpose();

        // Compute the determinant to check for reflection
        let d = (v * u_t).determinant();
        let correction = nalgebra::Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d.signum()));
        let rotation_matrix = v * correction * u_t;

        // Convert to unit quaternion (convert dynamic matrix to fixed 3x3)
        let rotation_matrix_3x3 = rotation_matrix.fixed_view::<3, 3>(0, 0).into_owned();
        let rotation = UnitQuaternion::from_matrix(&rotation_matrix_3x3);

        let translation =
            Translation3::from(target_centroid.coords - rotation * input_centroid.coords);

        Some(Isometry3 {
            rotation,
            translation,
        })
    }

    /// Helper to compute centroid of points
    fn compute_centroid(points: &[Point3<f64>]) -> Option<Point3<f64>> {
        if points.is_empty() {
            return None;
        }

        let sum = points
            .iter()
            .fold(Vector3::zeros(), |acc, p| acc + p.coords);
        Some(Point3::from(sum / points.len() as f64))
    }
}
