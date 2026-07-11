use crate::{
    config::Config,
    detection::{
        BoardIcpState, BoardModelParams, FitBoardIcp, FitPlaneRansac, IcpData, IcpStatistics,
        PlaneRansacData,
    },
};
use ahash::AHashMap;
use anyhow::Result;
use arrsac::Arrsac;
use aruco_config::MultiArucoPattern;
use hollow_board_config::{BoardModel, BoardShape};
use log::{debug, warn};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use newslab_geom_algo::{self, centroid_of_points};
use plane_estimator::{PlaneEstimator, PlaneModel};
use sample_consensus::Consensus;
use std::{
    borrow::Borrow,
    f64::{self},
};

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
    parallel_threshold: usize,
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

/// Estimates the board pose from a point set using ICP algorithm.
#[allow(unused_assignments)]
pub fn fit_board_icp(
    board_detector: &Config,
    aruco_detector: &MultiArucoPattern,
    plane_model: &PlaneModel,
    plane_inlier_points: &[impl Borrow<Point3<f64>>],
    mut progress_cb: Option<&mut dyn FnMut(&BoardModel)>,
) -> Result<FitBoardIcp> {
    let Config {
        board_shape:
            BoardShape {
                board_width,
                hole_radius,
                hole_center_shift,
            },
        max_icp_iterations,
        icp_pose_weight_threshold,
        icp_rejection_threshold,
        icp_good_fit_threshold,
        icp_outlier_threshold,
        icp_damping_factor,
        icp_min_inlier_points,
        ..
    } = *board_detector;
    let marker_paper_size = aruco_detector.paper_size();

    let mut final_corresponding_points: Vec<Point3<f64>> = vec![];
    let mut final_icp_stats = IcpStatistics {
        iterations: 0,
        final_loss: 0.0,
        min_loss: f64::INFINITY,
        successful: false,
        initial_loss: 0.0,
        convergence_reason: "Not started".to_string(),
    };

    let (board_pose, icp_losses, viz_msg) = {
        let init_pose = {
            let inlier_centroid: Point3<f64> =
                centroid_of_points(plane_inlier_points.iter().map(|point| {
                    let point: [f64; 3] = (*point.borrow()).into();
                    point
                }))
                .unwrap()
                .into();

            let plane_normal = {
                let normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
                normal
            };

            let rotation = {
                let board_z_axis = Vector3::z_axis();

                let primary_rotation =
                    UnitQuaternion::rotation_between(&board_z_axis, &plane_normal)
                        .unwrap_or_else(UnitQuaternion::identity);

                let rotated_board_x = primary_rotation * Vector3::x_axis();

                let centroid_to_origin = Point3::origin() - inlier_centroid;
                let projected_to_origin =
                    centroid_to_origin - plane_normal * centroid_to_origin.dot(&plane_normal);

                if projected_to_origin.norm() > 1e-6 {
                    let target_direction = projected_to_origin.normalize();

                    let secondary_rotation =
                        UnitQuaternion::rotation_between(&rotated_board_x, &target_direction)
                            .unwrap_or_else(UnitQuaternion::identity);

                    secondary_rotation * primary_rotation
                } else {
                    primary_rotation
                }
            };

            Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation)
        };
        let init_inlier_points: Vec<&Point3<_>> = plane_inlier_points
            .iter()
            .map(|point| point.borrow())
            .collect();

        // Apply voxel downsampling if enabled
        let downsampled_points: Vec<Point3<f64>> = if board_detector.voxel_downsample_enabled {
            let owned_points: Vec<Point3<f64>> = init_inlier_points.iter().map(|&p| *p).collect();
            let original_count = owned_points.len();

            debug!("Voxel downsampling: {} points", original_count);
            let result = voxel_downsample(
                &owned_points,
                board_detector.voxel_downsample_size,
                board_detector.voxel_downsample_use_centroid,
                board_detector.voxel_parallel_threshold,
            );

            let reduction_pct = (1.0 - result.len() as f64 / original_count as f64) * 100.0;
            debug!(
                "  → {} points ({:.1}% reduction)",
                result.len(),
                reduction_pct
            );
            result
        } else {
            init_inlier_points.iter().map(|&p| *p).collect()
        };

        let (inlier_points, icp_losses, pose) = {
            let inlier_points = downsampled_points;
            let mut losses: Vec<f64> = vec![];
            let mut termination_count = 0;
            let mut pose = init_pose;
            let mut step = 0;
            let mut convergence_reason = String::new();
            let mut initial_loss_captured = false;
            let mut initial_loss = 0.0;

            loop {
                let board_model = BoardModel {
                    pose,
                    board_shape: BoardShape {
                        board_width,
                        hole_radius,
                        hole_center_shift,
                    },
                    marker_paper_size,
                };

                if let Some(cb) = progress_cb.as_mut() {
                    cb(&board_model);
                }

                let correspondings = match board_model.find_correspondences(&inlier_points) {
                    Some(corr) => corr,
                    None => {
                        convergence_reason = "No correspondences found".to_string();
                        break;
                    }
                };

                let correspondence_losses: Vec<_> = correspondings
                    .iter()
                    .map(|(input_point, corresponding_point)| {
                        (*input_point - corresponding_point).norm()
                    })
                    .collect();
                let avg_loss =
                    correspondence_losses.iter().sum::<f64>() / correspondings.len() as f64;

                // Filter correspondences by outlier threshold
                let good_correspondences: Vec<_> = correspondings
                    .iter()
                    .filter(|(input_point, corresponding_point)| {
                        let loss = (*input_point - corresponding_point).norm();
                        loss <= icp_outlier_threshold
                    })
                    .map(|(input_point, corresponding_point)| (*input_point, *corresponding_point))
                    .collect();

                let good_correspondences_len = good_correspondences.len();
                let (good_corresponding_points, good_inlier_points): (
                    Vec<Point3<f64>>,
                    Vec<Point3<f64>>,
                ) = good_correspondences.into_iter().unzip();

                if good_inlier_points.len() < 3 {
                    convergence_reason = format!(
                        "Insufficient points for Kabsch: {}",
                        good_inlier_points.len()
                    );
                    final_corresponding_points = good_corresponding_points;
                    break;
                }

                let align_pose: Isometry3<_> = {
                    match BoardIcpIterator::compute_kabsch_transform(
                        &good_corresponding_points,
                        &good_inlier_points,
                    ) {
                        Some(iso) => iso.inverse(),
                        None => {
                            convergence_reason = "Kabsch algorithm failed".to_string();
                            final_corresponding_points = good_corresponding_points;
                            break;
                        }
                    }
                };

                if !initial_loss_captured {
                    initial_loss = avg_loss;
                    initial_loss_captured = true;
                }

                losses.push(avg_loss);

                let damping_factor = icp_damping_factor;
                let new_pose = align_pose * pose;

                let damped_translation = Translation3::from(
                    pose.translation.vector
                        + (new_pose.translation.vector - pose.translation.vector) * damping_factor,
                );

                let damped_rotation =
                    UnitQuaternion::slerp(&pose.rotation, &new_pose.rotation, damping_factor);

                let applied_t = (damped_translation.vector - pose.translation.vector).norm();
                let applied_ang = damped_rotation.rotation_to(&pose.rotation).angle();
                let pose_weight = applied_t + applied_ang;

                if pose_weight <= icp_pose_weight_threshold {
                    termination_count += 1;
                } else {
                    termination_count = 0;
                }

                pose = Isometry3::from_parts(damped_translation, damped_rotation);

                step += 1;

                if good_correspondences_len < icp_min_inlier_points {
                    convergence_reason = format!(
                        "Insufficient good correspondences: {} < {}",
                        good_correspondences_len, icp_min_inlier_points
                    );
                    final_corresponding_points = good_corresponding_points;
                    break;
                }

                if *losses.last().unwrap() < icp_rejection_threshold || termination_count > 100 {
                    if termination_count > 100 {
                        debug!(
                            "ICP terminating: stable pose reached (termination_count = {})",
                            termination_count
                        );
                        eprintln!(
                            "[DEBUG] ICP terminating: stable pose reached (termination_count = {})",
                            termination_count
                        );
                    } else {
                        debug!(
                            "ICP terminating: loss below rejection threshold: {:.8}",
                            losses.last().unwrap()
                        );
                        eprintln!(
                            "[DEBUG] ICP terminating: loss below rejection threshold: {:.8}",
                            losses.last().unwrap()
                        );
                    }
                    debug!("  Pose weight threshold: {:.8}", icp_pose_weight_threshold);
                    debug!("  Good fit threshold: {:.8}", icp_good_fit_threshold);
                    debug!("  Rejection threshold: {:.8}", icp_rejection_threshold);
                    debug!("  Avg loss: {:.8}", *losses.last().unwrap());
                    debug!("  Total input points: {}", inlier_points.len());
                    debug!(
                        "  Good correspondences: {}/{}",
                        good_corresponding_points.len(),
                        correspondings.len()
                    );
                    debug!("  Pose: {:.8}", pose);
                    convergence_reason = if termination_count > 100 {
                        "Converged (stable pose)".to_string()
                    } else {
                        "Converged (good fit)".to_string()
                    };
                    final_corresponding_points = good_corresponding_points;
                    break;
                }

                if step == max_icp_iterations {
                    convergence_reason = format!("Max iterations reached: {}", max_icp_iterations);
                    final_corresponding_points = good_corresponding_points;
                    break;
                }
            }

            // Create ICP statistics
            let final_loss = losses.last().copied().unwrap_or(initial_loss);
            let min_loss = losses.iter().copied().fold(f64::INFINITY, f64::min);
            final_icp_stats = IcpStatistics {
                iterations: step,
                final_loss,
                min_loss,
                successful: !convergence_reason.contains("failed")
                    && !convergence_reason.contains("Insufficient"),
                initial_loss,
                convergence_reason,
            };

            (inlier_points, losses, pose)
        };

        let viz_msg = {
            let board_model = BoardModel {
                pose,
                board_shape: BoardShape {
                    board_width,
                    hole_radius,
                    hole_center_shift,
                },
                marker_paper_size,
            };

            let correspondences: Vec<_> = inlier_points
                .iter()
                .zip(final_corresponding_points.iter())
                .map(|(data_point, model_point)| (*data_point, *model_point))
                .collect();

            IcpData {
                correspondences,
                board_model,
            }
        };

        (pose, icp_losses, viz_msg)
    };

    let final_init_pose = {
        let inlier_centroid: Point3<f64> =
            centroid_of_points(plane_inlier_points.iter().map(|point| {
                let point: [f64; 3] = (*point.borrow()).into();
                point
            }))
            .unwrap()
            .into();

        let plane_normal = {
            let normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
            normal
        };

        let rotation = {
            if plane_inlier_points.len() >= 3 {
                let mean = inlier_centroid;

                let mut covariance = nalgebra::Matrix3::<f64>::zeros();
                for point in plane_inlier_points.iter() {
                    let p: Point3<f64> = *point.borrow();
                    let diff = p - mean;
                    covariance += diff * diff.transpose();
                }
                covariance /= plane_inlier_points.len() as f64;

                let eigen = covariance.symmetric_eigen();
                let eigenvalues = eigen.eigenvalues;
                let eigenvectors = eigen.eigenvectors;

                let mut eigen_pairs: Vec<(f64, Vector3<f64>)> = (0..3)
                    .map(|i| (eigenvalues[i], eigenvectors.column(i).into()))
                    .collect();
                eigen_pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

                let pc1 = eigen_pairs[0].1.normalize();
                let pc2 = eigen_pairs[1].1.normalize();
                let pc3 = eigen_pairs[2].1.normalize();

                let computed_normal = if pc3.dot(&plane_normal) > 0.0 {
                    pc3
                } else {
                    -pc3
                };

                let cross_product = pc1.cross(&pc2);
                let (x_axis, y_axis) = if cross_product.dot(&computed_normal) > 0.0 {
                    (pc1, pc2)
                } else {
                    (pc1, -pc2)
                };

                let rotation_matrix =
                    nalgebra::Matrix3::from_columns(&[x_axis, y_axis, computed_normal]);

                let pca_rotation = nalgebra::UnitQuaternion::from_matrix(&rotation_matrix);

                let adjustment_angle = -135.0_f64.to_radians();
                let normal_axis = nalgebra::Unit::new_normalize(computed_normal);
                let z_axis_rotation =
                    nalgebra::UnitQuaternion::from_axis_angle(&normal_axis, adjustment_angle);

                z_axis_rotation * pca_rotation
            } else {
                let board_z_axis = Vector3::z_axis();
                UnitQuaternion::rotation_between(&board_z_axis, &plane_normal)
                    .unwrap_or_else(UnitQuaternion::identity)
            }
        };

        Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation)
    };

    // Return result with success status already determined by ICP statistics
    // Success criteria:
    // 1. termination_count > 100 (stable pose)
    // 2. avg_loss < icp_rejection_threshold
    Ok(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
        successful: final_icp_stats.successful,
        initial_pose: final_init_pose,
        icp_stats: final_icp_stats.clone(),
    })
}

/// Estimates the board pose using the iterator API (new implementation)
///
/// This is the new implementation that uses BoardIcpIterator internally.
/// For step-by-step debugging, use BoardIcpIterator directly.
pub fn fit_board_icp_with_iterator<'a>(
    board_detector: &'a Config,
    aruco_detector: &MultiArucoPattern,
    plane_model: &PlaneModel,
    plane_inlier_points: &[impl Borrow<Point3<f64>>],
    progress_cb: Option<&'a mut dyn FnMut(&BoardModel)>,
) -> Result<FitBoardIcp> {
    let init_pose = {
        let inlier_centroid: Point3<f64> =
            centroid_of_points(plane_inlier_points.iter().map(|point| {
                let point: [f64; 3] = (*point.borrow()).into();
                point
            }))
            .unwrap()
            .into();

        let plane_normal = {
            let normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
            normal
        };

        let rotation = {
            if plane_inlier_points.len() >= 3 {
                let mean = inlier_centroid;
                let mut covariance = nalgebra::Matrix3::<f64>::zeros();
                for point in plane_inlier_points.iter() {
                    let p: Point3<f64> = *point.borrow();
                    let diff = p - mean;
                    covariance += diff * diff.transpose();
                }
                covariance /= plane_inlier_points.len() as f64;

                let eigen = covariance.symmetric_eigen();
                let eigenvalues = eigen.eigenvalues;
                let eigenvectors = eigen.eigenvectors;

                let mut eigen_pairs: Vec<(f64, Vector3<f64>)> = (0..3)
                    .map(|i| (eigenvalues[i], eigenvectors.column(i).into()))
                    .collect();
                eigen_pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

                let pc1 = eigen_pairs[0].1.normalize();
                let pc2 = eigen_pairs[1].1.normalize();
                let pc3 = eigen_pairs[2].1.normalize();

                let computed_normal = if pc3.dot(&plane_normal) > 0.0 {
                    pc3
                } else {
                    -pc3
                };

                let cross_product = pc1.cross(&pc2);
                let (x_axis, y_axis) = if cross_product.dot(&computed_normal) > 0.0 {
                    (pc1, pc2)
                } else {
                    (pc1, -pc2)
                };

                let rotation_matrix =
                    nalgebra::Matrix3::from_columns(&[x_axis, y_axis, computed_normal]);
                let pca_rotation = nalgebra::UnitQuaternion::from_matrix(&rotation_matrix);

                let adjustment_angle = -135.0_f64.to_radians();
                let normal_axis = nalgebra::Unit::new_normalize(computed_normal);
                let z_axis_rotation =
                    nalgebra::UnitQuaternion::from_axis_angle(&normal_axis, adjustment_angle);

                z_axis_rotation * pca_rotation
            } else {
                let board_z_axis = Vector3::z_axis();
                UnitQuaternion::rotation_between(&board_z_axis, &plane_normal)
                    .unwrap_or_else(UnitQuaternion::identity)
            }
        };

        Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation)
    };

    // Create iterator
    let board_model_params = BoardModelParams {
        board_shape: board_detector.board_shape.clone(),
        marker_paper_size: aruco_detector.paper_size(),
    };

    let mut iterator =
        BoardIcpIterator::new(board_detector, board_model_params.clone(), progress_cb);

    let init_inlier_points: Vec<Point3<f64>> =
        plane_inlier_points.iter().map(|p| *p.borrow()).collect();

    let mut state = iterator.initial_state(init_pose, init_inlier_points);
    let mut losses = vec![];
    let initial_loss = state.avg_loss;

    // Run to completion
    while !iterator.should_terminate(&state) {
        state = iterator.step(&state);
        losses.push(state.avg_loss);
    }

    // Determine success based on termination reason
    // Success criteria:
    // 1. "Converged (stable pose)" - termination_count > 100
    // 2. "Converged (good fit)" - avg_loss < icp_rejection_threshold
    let termination_reason = iterator.termination_reason(&state);
    let successful = termination_reason == "Converged (stable pose)"
        || termination_reason == "Converged (good fit)";

    // Debug output for termination
    debug!("ICP terminated: {}", termination_reason);
    eprintln!(
        "[DEBUG] ICP terminated after {} iterations: {}",
        state.iteration, termination_reason
    );
    eprintln!(
        "[DEBUG]   Final avg_loss: {:.8}, termination_count: {}",
        state.avg_loss, state.termination_count
    );
    eprintln!("[DEBUG]   Successful: {}", successful);

    let icp_stats = IcpStatistics {
        iterations: state.iteration,
        final_loss: state.avg_loss,
        min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
        successful,
        initial_loss,
        convergence_reason: termination_reason,
    };

    // Build visualization message
    let viz_msg = {
        let board_model = BoardModel {
            pose: state.board_pose,
            board_shape: board_model_params.board_shape.clone(),
            marker_paper_size: board_model_params.marker_paper_size,
        };

        IcpData {
            correspondences: state.correspondences.clone(),
            board_model,
        }
    };

    // Return result with success status determined by termination criteria only
    Ok(FitBoardIcp {
        board_pose: state.board_pose,
        icp_losses: losses,
        icp_data: viz_msg,
        successful,
        initial_pose: init_pose,
        icp_stats,
    })
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
