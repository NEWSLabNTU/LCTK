use crate::{
    config::Config,
    detection::{FitBoardIcp, FitPlaneRansac, IcpData, IcpStatistics, PlaneRansacData},
};
use anyhow::Result;
use arrsac::Arrsac;
use aruco_config::MultiArucoPattern;
use hollow_board_config::{BoardModel, BoardShape};
use itertools::izip;
use log::{debug, warn};
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use newslab_geom_algo::{self, centroid_of_points, kabsch, IJKW, XYZ};
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
            Some(ret) => ret,
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
        if current_normal.dot(&desired_front) < 0.0 {
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
                    let pairs = izip!(
                        good_corresponding_points
                            .iter()
                            .map(|&p| -> [f64; 3] { p.into() }),
                        good_inlier_points.iter().map(|&p| -> [f64; 3] { p.into() }),
                    );

                    match kabsch(pairs) {
                        Some((XYZ([x, y, z]), IJKW([i, j, k, w]))) => {
                            let iso = Isometry3 {
                                rotation: UnitQuaternion::from_quaternion(Quaternion::new(
                                    w, i, j, k,
                                )),
                                translation: Translation3::new(x, y, z),
                            };
                            iso
                        }
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
                let new_pose = pose * align_pose;

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
            let board_z_axis = Vector3::z_axis();
            let primary_rotation = UnitQuaternion::rotation_between(&board_z_axis, &plane_normal)
                .unwrap_or_else(|| UnitQuaternion::identity());
            primary_rotation
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
