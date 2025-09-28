use crate::{
    config::Config,
    detection::{
        BoardIcpState, BoardModelParams, FitBoardIcp, FitPlaneRansac, IcpData, IcpStatistics,
        PlaneRansacData,
    },
};
use anyhow::Result;
use arrsac::Arrsac;
use aruco_config::MultiArucoPattern;
use hollow_board_config::{BoardModel, BoardShape};
use itertools::izip;
use log::{debug, warn};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use newslab_geom_algo::{self, centroid_of_points};
use noisy_float::prelude::*;
use plane_estimator::{PlaneEstimator, PlaneModel};
use sample_consensus::Consensus;
use std::{
    borrow::Borrow,
    f64::{self},
};

unzip_n::unzip_n!(2);

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

    // Check minimum points requirement
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
            Some(ret) => {
                debug!(
                    "RANSAC success: Found {} inliers out of {} points",
                    ret.1.len(),
                    points.len()
                );
                debug!("RANSAC plane normal: {:?}", ret.0.normal);
                debug!("RANSAC plane pose: {:?}", ret.0.pose());
                ret
            }
            None => {
                warn!("RANSAC failed: No valid plane found");
                return Ok(None);
            }
        }
    };

    // Force plane normal to point to the front (+X in world coordinates)
    {
        let desired_front = Vector3::x_axis();
        let current_normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
        debug!(
            "RANSAC: Original normal: {:?}, dot with +X: {:.6}",
            current_normal,
            current_normal.dot(&desired_front)
        );
        if current_normal.dot(&desired_front) < 0.0 {
            let flipped = nalgebra::Unit::new_normalize(-current_normal);
            plane_model.normal = flipped;
            debug!("RANSAC: Flipped normal to: {:?}", plane_model.normal);
        } else {
            debug!("RANSAC: Keeping original normal direction");
        }
    }

    let inlier_points: Vec<_> = inlier_indices.into_iter().map(|idx| &points[idx]).collect();

    let viz_msg = PlaneRansacData {
        plane_model: plane_model.clone(),
        inlier_points: inlier_points.iter().map(|point| **point).collect(),
    };

    debug!(
        "RANSAC result: {} inliers, plane normal: {:?}",
        inlier_points.len(),
        plane_model.normal
    );

    Ok(Some(FitPlaneRansac {
        plane_model,
        inlier_points,
        ransac_data: viz_msg,
    }))
}

/// Estimates the board pose from a point set using ICP algorithm.
pub fn fit_board_icp(
    board_detector: &Config,
    aruco_detector: &MultiArucoPattern,
    plane_model: &PlaneModel,
    plane_inlier_points: &[impl Borrow<Point3<f64>>],
    mut progress_cb: Option<&mut dyn FnMut(&BoardModel)>,
) -> Result<FitBoardIcp> {
    // find board by modified ICP algorithm

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
        icp_adaptive_threshold_multiplier,
        icp_adaptive_threshold_min,
        icp_adaptive_threshold_max,
        icp_damping_factor,
        icp_min_inlier_points,
        ..
    } = *board_detector;
    let marker_paper_size = aruco_detector.paper_size();

    // Declare variables for ICP outputs at function scope
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

            // Improved plane normal determination
            let plane_normal = {
                let normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);

                // The RANSAC plane normal from fit_plane_ransac is already forced to point toward +X (front)
                // We should use this directly as it represents the board facing direction
                let board_facing_normal = normal;

                debug!("INIT POSE: inlier_centroid={:.6}", inlier_centroid);
                debug!("INIT POSE: board_facing_normal={:.6}", board_facing_normal);
                debug!(
                    "INIT POSE: normal_magnitude={:.6}",
                    board_facing_normal.norm()
                );

                board_facing_normal
            };

            // Improved rotation calculation using direct normal alignment
            let rotation = {
                // The board's local +Z axis should align with the plane normal
                let board_z_axis = Vector3::z_axis();

                // Create rotation that aligns board +Z with plane normal
                let primary_rotation =
                    UnitQuaternion::rotation_between(&board_z_axis, &plane_normal).unwrap_or_else(
                        || {
                            debug!("INIT POSE: primary rotation failed, using identity");
                            UnitQuaternion::identity()
                        },
                    );

                // After primary rotation, determine board orientation in the plane
                // The board should be oriented so its local +X points roughly toward the sensor origin
                let rotated_board_x = primary_rotation * Vector3::x_axis();

                // Project the vector from centroid to origin onto the plane
                let centroid_to_origin = Point3::origin() - inlier_centroid;
                let projected_to_origin =
                    centroid_to_origin - plane_normal * centroid_to_origin.dot(&plane_normal);

                if projected_to_origin.norm() > 1e-6 {
                    let target_direction = projected_to_origin.normalize();

                    // Calculate rotation around plane normal to align board +X with target direction
                    let secondary_rotation =
                        UnitQuaternion::rotation_between(&rotated_board_x, &target_direction)
                            .unwrap_or_else(|| UnitQuaternion::identity());

                    let final_rotation = secondary_rotation * primary_rotation;

                    debug!("INIT POSE: primary_rotation={:.6}", primary_rotation);
                    debug!("INIT POSE: rotated_board_x={:.6}", rotated_board_x.as_ref());
                    debug!("INIT POSE: projected_to_origin={:.6}", projected_to_origin);
                    debug!("INIT POSE: target_direction={:.6}", target_direction);
                    debug!("INIT POSE: secondary_rotation={:.6}", secondary_rotation);
                    debug!("INIT POSE: final_rotation={:.6}", final_rotation);

                    final_rotation
                } else {
                    debug!("INIT POSE: no clear direction to origin, using primary rotation only");
                    primary_rotation
                }
            };

            let init_pose =
                Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation);
            debug!("INIT POSE: final_init_pose={:.6}", init_pose);

            // Validate the initialization
            let board_normal_after_rotation = rotation * Vector3::z_axis();
            let normal_alignment = board_normal_after_rotation.dot(&plane_normal);
            debug!("INIT POSE: normal_alignment_check={:.6}", normal_alignment);

            if normal_alignment < 0.9 {
                debug!("INIT POSE: WARNING - poor normal alignment, may need adjustment");
            }

            init_pose
        };
        let init_inlier_points: Vec<&Point3<_>> = plane_inlier_points
            .iter()
            .map(|point| point.borrow())
            .collect();

        let (inlier_points, icp_losses, pose) = {
            let mut inlier_points: Vec<Point3<f64>> =
                init_inlier_points.iter().map(|&p| *p).collect();
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

                // Use the board model's correspondence finding method for proper closest point calculation
                let correspondings = match board_model.find_correspondences(&inlier_points) {
                    Some(corr) => corr,
                    None => {
                        convergence_reason = "No correspondences found".to_string();
                        break;
                    }
                };

                // reject outliers
                let correspondence_losses: Vec<_> = correspondings
                    .iter()
                    .map(|(input_point, corresponding_point)| {
                        let loss = (*input_point - corresponding_point).norm();
                        loss
                    })
                    .collect();
                let avg_loss =
                    correspondence_losses.iter().sum::<f64>() / correspondings.len() as f64;

                // Improved outlier filtering with adaptive thresholds
                // Use a configurable threshold that adapts to the current loss
                let adaptive_threshold = (avg_loss * icp_adaptive_threshold_multiplier)
                    .max(icp_adaptive_threshold_min)
                    .min(icp_adaptive_threshold_max);

                // Alternative: use fixed outlier threshold for more stable filtering
                // let adaptive_threshold = icp_outlier_threshold;
                let good_correspondences: Vec<_> = correspondings
                    .iter()
                    .zip(correspondence_losses.iter())
                    .filter_map(|((input_point, corresponding_point), &loss)| {
                        if loss <= adaptive_threshold {
                            Some((*input_point, *corresponding_point))
                        } else {
                            None
                        }
                    })
                    .collect();

                let good_correspondences_len = good_correspondences.len();
                let (good_corresponding_points, good_inlier_points): (
                    Vec<Point3<f64>>,
                    Vec<Point3<f64>>,
                ) = good_correspondences.into_iter().unzip();

                // Centroids diagnostics (selected pairs)
                if good_inlier_points.len() >= 3 {
                    if let Some(in_centroid_arr) =
                        centroid_of_points(good_inlier_points.iter().map(|p| {
                            let a: [f64; 3] = (*p).into();
                            a
                        }))
                    {
                        let in_centroid: Point3<f64> = in_centroid_arr.into();
                    }
                    if let Some(mod_centroid_arr) =
                        centroid_of_points(good_corresponding_points.iter().map(|p| {
                            let a: [f64; 3] = (*p).into();
                            a
                        }))
                    {
                        let mod_centroid: Point3<f64> = mod_centroid_arr.into();
                    }
                }

                // Safety check: ensure we have at least 3 points for Kabsch
                if good_inlier_points.len() < 3 {
                    convergence_reason = format!(
                        "Insufficient points for Kabsch: {}",
                        good_inlier_points.len()
                    );
                    let stats = IcpStatistics {
                        iterations: step,
                        final_loss: losses.last().copied().unwrap_or(0.0),
                        min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
                        successful: false,
                        initial_loss: if initial_loss_captured {
                            initial_loss
                        } else {
                            0.0
                        },
                        convergence_reason: convergence_reason.clone(),
                    };
                    // Store final values in outer scope variables
                    final_corresponding_points = good_corresponding_points;
                    final_icp_stats = stats;
                    break;
                }

                // compute transformation
                let align_pose: Isometry3<_> = {
                    // Kabsch expects (input, target) pairs.
                    // Our correspondences are (data_point, model_point).
                    // To compute a transform that moves the model toward the data
                    // (so pose = align_pose * pose), we pass (model_point, data_point).
                    match BoardIcpIterator::compute_kabsch_transform(
                        &good_corresponding_points,
                        &good_inlier_points,
                    ) {
                        Some(iso) => iso,
                        None => {
                            convergence_reason = "Kabsch algorithm failed".to_string();
                            let stats = IcpStatistics {
                                iterations: step,
                                final_loss: losses.last().copied().unwrap_or(0.0),
                                min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
                                successful: false,
                                initial_loss: if initial_loss_captured {
                                    initial_loss
                                } else {
                                    0.0
                                },
                                convergence_reason: convergence_reason.clone(),
                            };
                            // Store final values in outer scope variables
                            final_corresponding_points = good_corresponding_points;
                            final_icp_stats = stats;
                            break;
                        }
                    }
                };

                // Capture initial loss for statistics
                if !initial_loss_captured {
                    initial_loss = avg_loss;
                    initial_loss_captured = true;
                }

                // update state
                losses.push(avg_loss);
                // Convert back to the expected format for the next iteration
                inlier_points = good_inlier_points;

                // Apply damping to prevent overshooting
                let damping_factor = icp_damping_factor;

                // Apply damping to the pose update (not the transformation itself)
                // This interpolates between the current pose and the new pose after applying the transformation
                // CRITICAL FIX: Use correct multiplication order like original wayside-portal
                let new_pose = align_pose * pose;

                // Diagnostics for pose delta before damping
                let delta_t = (new_pose.translation.vector - pose.translation.vector).norm();
                let delta_ang = new_pose.rotation.rotation_to(&pose.rotation).angle();

                // Debug step-by-step ICP progress
                debug!(
                    "ICP Step {}: loss={:.6}, inliers={}, correspondences={}, delta_t={:.6}, delta_ang={:.6}",
                    step, avg_loss, inlier_points.len(), good_correspondences_len, delta_t, delta_ang
                );

                // Damp the translation component
                let damped_translation = Translation3::from(
                    pose.translation.vector
                        + (new_pose.translation.vector - pose.translation.vector) * damping_factor,
                );

                // Damp the rotation component using spherical linear interpolation
                let damped_rotation =
                    UnitQuaternion::slerp(&pose.rotation, &new_pose.rotation, damping_factor);

                // Termination criteria based on the actually applied (damped) update
                {
                    let applied_t = (damped_translation.vector - pose.translation.vector).norm();
                    let applied_ang = damped_rotation.rotation_to(&pose.rotation).angle();
                    let pose_weight = applied_t + applied_ang;
                    if pose_weight <= icp_pose_weight_threshold {
                        termination_count += 1;
                    } else {
                        termination_count = 0;
                    }
                }

                pose = Isometry3::from_parts(damped_translation, damped_rotation);
                step += 1;

                // Check if we have enough inlier points to continue
                if inlier_points.len() < icp_min_inlier_points {
                    convergence_reason = format!(
                        "Insufficient inlier points: {} < {}",
                        inlier_points.len(),
                        icp_min_inlier_points
                    );
                    let stats = IcpStatistics {
                        iterations: step,
                        final_loss: losses.last().copied().unwrap_or(0.0),
                        min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
                        successful: false,
                        initial_loss: if initial_loss_captured {
                            initial_loss
                        } else {
                            0.0
                        },
                        convergence_reason: convergence_reason.clone(),
                    };
                    // Store final values in outer scope variables
                    final_corresponding_points = good_corresponding_points;
                    final_icp_stats = stats;
                    break;
                }

                if *losses.last().unwrap() < icp_good_fit_threshold || termination_count > 10 {
                    debug!(
                        "ICP terminating: loss is too small: {:.8}",
                        losses.last().unwrap()
                    );
                    debug!("  Pose weight threshold: {:.8}", icp_pose_weight_threshold);
                    debug!("  Good fit threshold: {:.8}", icp_good_fit_threshold);
                    debug!("  Rejection threshold: {:.8}", icp_rejection_threshold);
                    debug!("  Avg loss: {:.8}", *losses.last().unwrap());
                    debug!("  Inlier points: {}", inlier_points.len());
                    debug!(
                        "  Good corresponding points: {}",
                        good_corresponding_points.len()
                    );
                    debug!("  Pose: {:.8}", pose);
                    convergence_reason = if termination_count > 10 {
                        "Converged (stable pose)".to_string()
                    } else {
                        "Converged (good fit)".to_string()
                    };
                    let stats = IcpStatistics {
                        iterations: step,
                        final_loss: losses.last().copied().unwrap_or(0.0),
                        min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
                        successful: true, // Fixed: This should be true for normal convergence
                        initial_loss: if initial_loss_captured {
                            initial_loss
                        } else {
                            0.0
                        },
                        convergence_reason: convergence_reason.clone(),
                    };
                    // Store final values in outer scope variables
                    final_corresponding_points = good_corresponding_points;
                    final_icp_stats = stats;
                    break;
                }

                if step == max_icp_iterations {
                    convergence_reason = format!("Max iterations reached: {}", max_icp_iterations);
                    let stats = IcpStatistics {
                        iterations: step,
                        final_loss: losses.last().copied().unwrap_or(0.0),
                        min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
                        successful: false,
                        initial_loss: if initial_loss_captured {
                            initial_loss
                        } else {
                            0.0
                        },
                        convergence_reason: convergence_reason.clone(),
                    };
                    // Store final values in outer scope variables
                    final_corresponding_points = good_corresponding_points;
                    final_icp_stats = stats;
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

        // send to visualizer
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

            // Use the final corresponding points from ICP
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

    // Extract init_pose and create icp_stats for the return values
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
            // Use PCA to determine the correct in-plane orientation for diamond board
            if plane_inlier_points.len() >= 3 {
                // Compute PCA on plane inlier points to find principal directions
                let mean = inlier_centroid;

                // Compute covariance matrix of inlier points relative to centroid
                let mut covariance = nalgebra::Matrix3::<f64>::zeros();
                for point in plane_inlier_points.iter() {
                    let p: Point3<f64> = (*point.borrow()).into();
                    let diff = p - mean;
                    covariance += diff * diff.transpose();
                }
                covariance /= plane_inlier_points.len() as f64;

                // Compute eigendecomposition to get principal components
                let eigen = covariance.symmetric_eigen();
                let eigenvalues = eigen.eigenvalues;
                let eigenvectors = eigen.eigenvectors;

                // Sort eigenvalues and corresponding eigenvectors in descending order
                let mut eigen_pairs: Vec<(f64, Vector3<f64>)> = (0..3)
                    .map(|i| (eigenvalues[i], eigenvectors.column(i).into()))
                    .collect();
                eigen_pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

                // The two largest principal components should align with diamond edges
                // The smallest should align with the plane normal
                let pc1 = eigen_pairs[0].1.normalize(); // First principal component (largest variance)
                let pc2 = eigen_pairs[1].1.normalize(); // Second principal component
                let pc3 = eigen_pairs[2].1.normalize(); // Third principal component (should be normal)

                // Ensure the normal points in the correct direction
                let computed_normal = if pc3.dot(&plane_normal) > 0.0 {
                    pc3
                } else {
                    -pc3
                };

                // Ensure right-handed coordinate system: pc1 x pc2 should align with normal
                let cross_product = pc1.cross(&pc2);
                let (x_axis, y_axis) = if cross_product.dot(&computed_normal) > 0.0 {
                    (pc1, pc2)
                } else {
                    (pc1, -pc2) // Flip y-axis to maintain right-handed system
                };

                // Create rotation matrix from the orthonormal basis
                let rotation_matrix =
                    nalgebra::Matrix3::from_columns(&[x_axis, y_axis, computed_normal]);

                // Convert to unit quaternion
                // Note: from_matrix doesn't return Option, it directly returns UnitQuaternion
                let pca_rotation = nalgebra::UnitQuaternion::from_matrix(&rotation_matrix);

                // Apply additional -135° rotation around z-axis (board normal) to align coordinate system
                let adjustment_angle = -135.0_f64.to_radians();
                let normal_axis = nalgebra::Unit::new_normalize(computed_normal);
                let z_axis_rotation =
                    nalgebra::UnitQuaternion::from_axis_angle(&normal_axis, adjustment_angle);

                // Combine PCA rotation with the adjustment
                z_axis_rotation * pca_rotation
            } else {
                // Fallback for insufficient points
                let board_z_axis = Vector3::z_axis();
                UnitQuaternion::rotation_between(&board_z_axis, &plane_normal)
                    .unwrap_or_else(|| UnitQuaternion::identity())
            }
        };

        Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation)
    };

    // reject result if loss is too large
    {
        let min_icp_loss = icp_losses
            .iter()
            .copied()
            .map(r64)
            .min()
            .map(|loss| loss.raw());
        let min_icp_loss = match min_icp_loss {
            Some(loss) => loss,
            None => {
                return Ok(FitBoardIcp {
                    board_pose,
                    icp_losses,
                    icp_data: viz_msg,
                    successful: false,
                    initial_pose: final_init_pose,
                    icp_stats: final_icp_stats.clone(),
                })
            }
        };

        if min_icp_loss > icp_rejection_threshold {
            return Ok(FitBoardIcp {
                board_pose,
                icp_losses,
                icp_data: viz_msg,
                successful: false,
                initial_pose: final_init_pose,
                icp_stats: final_icp_stats.clone(),
            });
        }
    }

    // Save ICP results to CSV for 3D visualization
    let board_model = BoardModel {
        pose: board_pose,
        board_shape: BoardShape {
            board_width,
            hole_radius,
            hole_center_shift,
        },
        marker_paper_size,
    };

    let _final_loss = icp_losses
        .iter()
        .copied()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap_or(0.0);
    debug!(
        "FINAL ICP RESULT: pose={:.6}, loss={:.6}",
        board_pose, _final_loss
    );

    Ok(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
        successful: true,
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
    mut progress_cb: Option<&'a mut dyn FnMut(&BoardModel)>,
) -> Result<FitBoardIcp> {
    // Compute initial pose (extracted from original implementation)
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
                // Compute PCA on plane inlier points
                let mean = inlier_centroid;
                let mut covariance = nalgebra::Matrix3::<f64>::zeros();
                for point in plane_inlier_points.iter() {
                    let p: Point3<f64> = (*point.borrow()).into();
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
                    .unwrap_or_else(|| UnitQuaternion::identity())
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

    // Build result
    let successful = !iterator.termination_reason(&state).contains("failed")
        && !iterator.termination_reason(&state).contains("Insufficient");

    let icp_stats = IcpStatistics {
        iterations: state.iteration,
        final_loss: state.avg_loss,
        min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
        successful,
        initial_loss,
        convergence_reason: iterator.termination_reason(&state),
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

    // Check rejection threshold
    let min_icp_loss = losses.iter().copied().map(r64).min().map(|loss| loss.raw());

    if let Some(min_loss) = min_icp_loss {
        if min_loss > board_detector.icp_rejection_threshold {
            return Ok(FitBoardIcp {
                board_pose: state.board_pose,
                icp_losses: losses,
                icp_data: viz_msg,
                successful: false,
                initial_pose: init_pose,
                icp_stats,
            });
        }
    }

    Ok(FitBoardIcp {
        board_pose: state.board_pose,
        icp_losses: losses,
        icp_data: viz_msg,
        successful: true,
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
            adaptive_threshold: self.board_detector_config.icp_outlier_threshold,
            total_correspondences: 0,
            good_correspondences: 0,
            termination_count: 0,
        }
    }

    /// Execute one ICP iteration step
    pub fn step(&mut self, current_state: &BoardIcpState) -> BoardIcpState {
        debug!(
            "ICP Step {}: Starting iteration with pose: {:?}",
            current_state.iteration + 1,
            current_state.board_pose
        );

        // 1. Create board model with current pose
        let board_model = BoardModel {
            pose: current_state.board_pose,
            board_shape: self.board_model_params.board_shape.clone(),
            marker_paper_size: self.board_model_params.marker_paper_size,
        };

        // Trigger progress callback if provided
        if let Some(cb) = self.progress_callback.as_mut() {
            cb(&board_model);
        }

        // 2. Find correspondences using board model
        debug!(
            "ICP Step {}: Attempting to find correspondences for {} inlier points",
            current_state.iteration + 1,
            current_state.inlier_points.len()
        );

        let correspondences = match board_model.find_correspondences(&current_state.inlier_points) {
            Some(corr) => corr,
            None => {
                // No correspondences - return terminated state
                debug!(
                    "ICP Step {}: Board model find_correspondences returned None",
                    current_state.iteration + 1
                );
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
        debug!(
            "ICP Step {}: Found {} total correspondences",
            current_state.iteration + 1,
            total_correspondences
        );

        // 3. Compute losses for each correspondence
        let correspondence_losses: Vec<_> = correspondences
            .iter()
            .map(|(input_point, corresponding_point)| (*input_point - corresponding_point).norm())
            .collect();

        let avg_loss = correspondence_losses.iter().sum::<f64>() / correspondences.len() as f64;
        debug!(
            "ICP Step {}: Average correspondence loss: {:.4}m",
            current_state.iteration + 1,
            avg_loss
        );

        // 4. Filter outliers with adaptive threshold
        let adaptive_threshold = (avg_loss
            * self.board_detector_config.icp_adaptive_threshold_multiplier)
            .max(self.board_detector_config.icp_adaptive_threshold_min)
            .min(self.board_detector_config.icp_adaptive_threshold_max);

        debug!(
            "ICP Step {}: Adaptive threshold: {:.4}m (multiplier: {:.2}, min: {:.4}, max: {:.4})",
            current_state.iteration + 1,
            adaptive_threshold,
            self.board_detector_config.icp_adaptive_threshold_multiplier,
            self.board_detector_config.icp_adaptive_threshold_min,
            self.board_detector_config.icp_adaptive_threshold_max
        );

        let good_correspondences: Vec<_> = correspondences
            .iter()
            .zip(correspondence_losses.iter())
            .filter_map(|((input_point, corresponding_point), &loss)| {
                if loss <= adaptive_threshold {
                    Some((**input_point, *corresponding_point))
                } else {
                    None
                }
            })
            .collect();

        let good_correspondences_len = good_correspondences.len();
        debug!(
            "ICP Step {}: After outlier filtering: {} good correspondences (from {} total)",
            current_state.iteration + 1,
            good_correspondences_len,
            total_correspondences
        );

        // 5. Check if we have enough points for Kabsch
        if good_correspondences_len < 3 {
            debug!(
                "ICP Step {}: Insufficient correspondences for Kabsch algorithm (need ≥3, got {})",
                current_state.iteration + 1,
                good_correspondences_len
            );
            debug!(
                "ICP Step {}: Terminating iterations due to insufficient good correspondences",
                current_state.iteration + 1
            );
            return BoardIcpState {
                iteration: current_state.iteration + 1,
                correspondences: good_correspondences,
                avg_loss,
                previous_loss: Some(current_state.avg_loss),
                adaptive_threshold,
                total_correspondences,
                good_correspondences: good_correspondences_len,
                ..current_state.clone()
            };
        }

        // Clone good_correspondences before consuming
        let good_correspondences_for_state: Vec<(Point3<f64>, Point3<f64>)> =
            good_correspondences.clone();
        let (good_corresponding_points, good_inlier_points): (Vec<Point3<f64>>, Vec<Point3<f64>>) =
            good_correspondences.into_iter().unzip();

        // 6. Compute transformation using Kabsch
        let align_pose: Isometry3<f64> =
            match Self::compute_kabsch_transform(&good_corresponding_points, &good_inlier_points) {
                Some(iso) => iso,
                None => {
                    // Kabsch failed
                    return BoardIcpState {
                        iteration: current_state.iteration + 1,
                        correspondences: good_correspondences_for_state,
                        avg_loss,
                        previous_loss: Some(current_state.avg_loss),
                        adaptive_threshold,
                        total_correspondences,
                        good_correspondences: good_correspondences_len,
                        ..current_state.clone()
                    };
                }
            };

        // 7. Apply damping and update pose
        let new_pose = align_pose * current_state.board_pose;
        let damping_factor = self.board_detector_config.icp_damping_factor;

        // Debug logging
        let delta_t =
            (new_pose.translation.vector - current_state.board_pose.translation.vector).norm();
        let delta_ang = new_pose
            .rotation
            .rotation_to(&current_state.board_pose.rotation)
            .angle();
        debug!(
            "ICP Step {}: loss={:.6}, inliers={}, correspondences={}/{}, delta_t={:.6}, delta_ang={:.6}",
            current_state.iteration,
            avg_loss,
            good_inlier_points.len(),
            good_correspondences_len,
            total_correspondences,
            delta_t,
            delta_ang
        );

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

        // 8. Check termination criteria for pose convergence
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

        // 9. Return new state
        BoardIcpState {
            iteration: current_state.iteration + 1,
            board_pose: damped_pose,
            inlier_points: good_inlier_points,
            correspondences: good_correspondences_for_state,
            avg_loss,
            previous_loss: Some(current_state.avg_loss),
            adaptive_threshold,
            total_correspondences,
            good_correspondences: good_correspondences_len,
            termination_count,
        }
    }

    /// Check if algorithm should terminate
    pub fn should_terminate(&self, state: &BoardIcpState) -> bool {
        let config = self.board_detector_config;

        // Max iterations reached
        if state.iteration >= config.max_icp_iterations {
            return true;
        }

        // Good fit achieved
        if state.avg_loss < config.icp_good_fit_threshold {
            return true;
        }

        // Pose converged (stable for multiple iterations)
        if state.termination_count > 10 {
            return true;
        }

        // Insufficient inlier points
        if state.inlier_points.len() < config.icp_min_inlier_points {
            return true;
        }

        // Insufficient correspondences for Kabsch
        if state.good_correspondences < 3 {
            return true;
        }

        // No correspondences found
        if state.correspondences.is_empty() {
            return true;
        }

        false
    }

    /// Get termination reason
    pub fn termination_reason(&self, state: &BoardIcpState) -> String {
        let config = self.board_detector_config;

        if state.iteration >= config.max_icp_iterations {
            format!("Max iterations reached: {}", config.max_icp_iterations)
        } else if state.termination_count > 10 {
            "Converged (stable pose)".to_string()
        } else if state.avg_loss < config.icp_good_fit_threshold {
            "Converged (good fit)".to_string()
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
        let d = (&v * &u_t).determinant();
        let correction = nalgebra::Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d.signum()));
        let rotation_matrix = &v * correction * u_t;

        // Convert to unit quaternion (convert dynamic matrix to fixed 3x3)
        let rotation_matrix_3x3 = rotation_matrix.fixed_view::<3, 3>(0, 0).into_owned();
        let rotation = UnitQuaternion::from_matrix(&rotation_matrix_3x3);

        // Compute translation
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    /// Helper function to create test points
    fn create_test_points() -> Vec<Point3<f64>> {
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
        ]
    }

    /// Helper function to apply transformation to points
    fn transform_points(points: &[Point3<f64>], transform: &Isometry3<f64>) -> Vec<Point3<f64>> {
        points.iter().map(|p| transform * p).collect()
    }

    /// Helper function to check if two transformations are approximately equal
    fn transformations_approx_equal(t1: &Isometry3<f64>, t2: &Isometry3<f64>, eps: f64) -> bool {
        let translation_diff = (t1.translation.vector - t2.translation.vector).norm();
        let rotation_diff = t1.rotation.rotation_to(&t2.rotation).angle();

        translation_diff < eps && rotation_diff < eps
    }

    /// Helper function to check if points are approximately equal
    fn points_approx_equal(p1: &Point3<f64>, p2: &Point3<f64>, eps: f64) -> bool {
        (p1 - p2).norm() < eps
    }

    #[test]
    fn test_kabsch_identity_transformation() {
        // Test with identical point sets - should return identity transformation
        let input_points = create_test_points();
        let target_points = input_points.clone();

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let transform = result.unwrap();

        // Should be close to identity
        let identity = Isometry3::identity();
        assert!(transformations_approx_equal(&transform, &identity, EPSILON));
    }

    #[test]
    fn test_kabsch_pure_translation() {
        // Test with pure translation transformation
        let input_points = create_test_points();
        let translation = Translation3::new(2.0, 3.0, 1.5);
        let expected_transform = Isometry3::from_parts(translation, UnitQuaternion::identity());
        let target_points = transform_points(&input_points, &expected_transform);

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct by applying it to input points
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, EPSILON));
        }
    }

    #[test]
    fn test_kabsch_pure_rotation() {
        // Test with pure rotation around Z-axis
        let input_points = create_test_points();
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0);
        let expected_transform = Isometry3::from_parts(Translation3::identity(), rotation);
        let target_points = transform_points(&input_points, &expected_transform);

        println!("Input points: {:?}", input_points);
        println!("Target points: {:?}", target_points);
        println!("Expected transform: {:?}", expected_transform);

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        println!("Computed transform: {:?}", computed_transform);

        // Verify the transformation is correct
        for (i, (input, target)) in input_points.iter().zip(target_points.iter()).enumerate() {
            let transformed = computed_transform * input;
            let error = (transformed - target).norm();
            println!(
                "Point {}: input={:?}, target={:?}, transformed={:?}, error={:.2e}",
                i, input, target, transformed, error
            );
            assert!(
                points_approx_equal(&transformed, target, 1e-6),
                "Point {} failed: error={:.2e} > tolerance=1e-6",
                i,
                error
            );
        }
    }

    #[test]
    fn test_kabsch_rotation_and_translation() {
        // Test with combined rotation and translation
        let input_points = create_test_points();
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 6.0);
        let translation = Translation3::new(1.0, -2.0, 3.0);
        let expected_transform = Isometry3::from_parts(translation, rotation);
        let target_points = transform_points(&input_points, &expected_transform);

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, EPSILON));
        }
    }

    #[test]
    fn test_kabsch_arbitrary_rotation() {
        // Test with arbitrary rotation
        let input_points = create_test_points();
        let axis = nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0));
        let rotation = UnitQuaternion::from_axis_angle(&axis, PI / 3.0);
        let translation = Translation3::new(-1.5, 2.5, -0.5);
        let expected_transform = Isometry3::from_parts(translation, rotation);
        let target_points = transform_points(&input_points, &expected_transform);

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, 1e-12));
        }
    }

    #[test]
    fn test_kabsch_insufficient_points() {
        // Test with insufficient points (less than 3)
        let input_points = vec![Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let target_points = vec![Point3::new(2.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0)];

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);
        assert!(result.is_none());
    }

    #[test]
    fn test_kabsch_empty_points() {
        // Test with empty point sets
        let input_points: Vec<Point3<f64>> = vec![];
        let target_points: Vec<Point3<f64>> = vec![];

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);
        assert!(result.is_none());
    }

    #[test]
    fn test_kabsch_mismatched_sizes() {
        // Test with mismatched point set sizes
        let input_points = create_test_points();
        let target_points = vec![Point3::new(1.0, 0.0, 0.0)];

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);
        assert!(result.is_none());
    }

    #[test]
    fn test_kabsch_collinear_points() {
        // Test with collinear points (edge case)
        let input_points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ];
        let target_points = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
        ];

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        // Should still work for collinear points
        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, 1e-12));
        }
    }

    #[test]
    fn test_kabsch_planar_points() {
        // Test with planar points (2D points embedded in 3D)
        let input_points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];

        // Rotate the square by 45 degrees around Z and translate
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0);
        let translation = Translation3::new(2.0, 3.0, 1.0);
        let expected_transform = Isometry3::from_parts(translation, rotation);
        let target_points = transform_points(&input_points, &expected_transform);

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, 1e-12));
        }
    }

    #[test]
    fn test_kabsch_reflection_handling() {
        // Test reflection handling by creating a reflection transformation
        let input_points = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];

        // Create reflected points (mirror across YZ plane)
        let target_points = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, 1e-12));
        }

        // Check that the determinant is positive (proper rotation, not reflection)
        let det = computed_transform
            .rotation
            .to_rotation_matrix()
            .matrix()
            .determinant();
        assert!(
            det > 0.0,
            "Kabsch should produce proper rotations, not reflections"
        );
    }

    #[test]
    fn test_kabsch_numerical_stability() {
        // Test with very small transformations to check numerical stability
        let input_points = create_test_points();
        let small_rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1e-8);
        let small_translation = Translation3::new(1e-10, 1e-10, 1e-10);
        let small_transform = Isometry3::from_parts(small_translation, small_rotation);
        let target_points = transform_points(&input_points, &small_transform);

        let result = BoardIcpIterator::compute_kabsch_transform(&input_points, &target_points);

        assert!(result.is_some());
        let computed_transform = result.unwrap();

        // Verify the transformation is correct within reasonable tolerance
        for (input, target) in input_points.iter().zip(target_points.iter()) {
            let transformed = computed_transform * input;
            assert!(points_approx_equal(&transformed, target, 1e-8));
        }
    }

    #[test]
    fn test_compute_centroid() {
        // Test centroid computation
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
        ];

        let centroid = BoardIcpIterator::compute_centroid(&points);
        assert!(centroid.is_some());

        let expected_centroid = Point3::new(0.5, 0.5, 0.5);
        assert!(points_approx_equal(
            &centroid.unwrap(),
            &expected_centroid,
            EPSILON
        ));
    }

    #[test]
    fn test_compute_centroid_empty() {
        // Test centroid computation with empty points
        let points: Vec<Point3<f64>> = vec![];
        let centroid = BoardIcpIterator::compute_centroid(&points);
        assert!(centroid.is_none());
    }

    #[test]
    fn test_compute_centroid_single_point() {
        // Test centroid computation with single point
        let points = vec![Point3::new(1.0, 2.0, 3.0)];
        let centroid = BoardIcpIterator::compute_centroid(&points);
        assert!(centroid.is_some());
        assert!(points_approx_equal(&centroid.unwrap(), &points[0], EPSILON));
    }
}
