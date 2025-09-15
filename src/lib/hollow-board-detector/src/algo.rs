use crate::{
    config::Config,
    detection::{FitBoardIcp, FitPlaneRansac, IcpData, PlaneRansacData},
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

    debug!("RANSAC: Starting plane fitting");
    debug!("  Input points: {}", points.len());
    debug!("  Inlier threshold: {}", plane_ransac_inlier_threshold);
    debug!("  Max iterations: {}", plane_ransac_max_iterations);

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

    let (plane_model, inlier_indices) = {
        match arrsac.model_inliers(&estimator, points.iter().cloned()) {
            Some(ret) => {
                debug!("RANSAC succeeded!");
                debug!("  Inliers found: {}", ret.1.len());
                debug!(
                    "  Inlier ratio: {:.2}%",
                    (ret.1.len() as f64 / points.len() as f64) * 100.0
                );
                ret
            }
            None => {
                warn!("RANSAC failed: No valid plane found");
                debug!("  Possible reasons:");
                debug!("    - Points are too noisy/scattered");
                debug!(
                    "    - Inlier threshold ({}) too strict",
                    plane_ransac_inlier_threshold
                );
                debug!(
                    "    - Not enough iterations ({})",
                    plane_ransac_max_iterations
                );
                debug!("    - Points don't form a plane");
                return Ok(None);
            }
        }
    };

    let inlier_points: Vec<_> = inlier_indices.into_iter().map(|idx| &points[idx]).collect();

    // Log plane model details
    debug!("Plane model found:");
    debug!(
        "  Normal: ({:.4}, {:.4}, {:.4})",
        plane_model.normal[0], plane_model.normal[1], plane_model.normal[2]
    );
    debug!(
        "  Center: ({:.4}, {:.4}, {:.4})",
        plane_model.center.x, plane_model.center.y, plane_model.center.z
    );

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
) -> Result<Option<FitBoardIcp>> {
    // find board by modified ICP algoirthm
    const GOOD_FIT_THRESHOLD: f64 = 0.015; // velodyne 32-MR
                                           // let good_fit_threshold = 0.1; // ouster os-1
    const OUTLIER_THRESHOLD: f64 = 0.1;

    debug!("ICP: Starting board fitting");
    debug!("  Plane inlier points: {}", plane_inlier_points.len());

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
        ..
    } = *board_detector;

    debug!("ICP Parameters:");
    debug!("  Max iterations: {}", max_icp_iterations);
    debug!("  Pose weight threshold: {}", icp_pose_weight_threshold);
    debug!("  Rejection threshold: {}", icp_rejection_threshold);
    let marker_paper_size = aruco_detector.paper_size();

    let (board_pose, icp_losses, viz_msg) = {
        let init_pose = {
            let inlier_centroid: Point3<f64> =
                centroid_of_points(plane_inlier_points.iter().map(|point| {
                    let point: [f64; 3] = (*point.borrow()).into();
                    point
                }))
                .unwrap()
                .into();

            // obtain the plane normal vector that points towards the origin
            let plane_normal = {
                let normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
                if (Point3::origin() - inlier_centroid).dot(&normal) < 0.0 {
                    -normal
                } else {
                    normal
                }
            };

            // Align the board's Z-axis with the plane normal
            // This is a much simpler and more direct approach
            let rotation = {
                let board_z_axis = Vector3::z_axis();
                UnitQuaternion::rotation_between(&board_z_axis, &plane_normal).unwrap_or_else(
                    || {
                        // If the vectors are parallel, use identity
                        UnitQuaternion::identity()
                    },
                )
            };

            Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation)
        };
        let init_inlier_points: Vec<&Point3<_>> = plane_inlier_points
            .iter()
            .map(|point| point.borrow())
            .collect();

        let (inlier_points, corresponding_points, icp_losses, pose) = {
            let mut inlier_points: Vec<Point3<f64>> =
                init_inlier_points.iter().map(|&p| *p).collect();
            let mut losses: Vec<f64> = vec![];
            let mut termination_count = 0;
            let mut pose = init_pose;
            let mut step = 0;

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

                if step == 0 || step % 10 == 0 {
                    // Show details for first step and every 10th step
                    debug!("ICP Step {}: Board model created", step);
                    debug!(
                        "  Board pose translation: ({:.4}, {:.4}, {:.4})",
                        pose.translation.x, pose.translation.y, pose.translation.z
                    );
                    debug!(
                        "  Board pose rotation: ({:.4}, {:.4}, {:.4}, {:.4})",
                        pose.rotation.i, pose.rotation.j, pose.rotation.k, pose.rotation.w
                    );
                    debug!("  Input inlier points: {}", inlier_points.len());
                }

                // Proper ICP correspondence finding: find closest points on board model
                let correspondings: Vec<(Point3<f64>, Point3<f64>)> = inlier_points
                    .iter()
                    .map(|input_point| {
                        // Transform input point to board coordinate system
                        let board_point = board_model.pose.inverse() * *input_point;

                        // Find closest point on board model (project onto board plane)
                        let board_center = Point3::origin(); // In board coordinates
                        let board_normal = Vector3::z_axis(); // Board normal in board coordinates
                        let vec_to_point: Vector3<f64> = board_point - board_center;
                        let distance_to_plane = vec_to_point.dot(&board_normal);
                        let projected_board_point =
                            board_point - board_normal.scale(distance_to_plane);

                        // Transform back to world coordinates
                        let corresponding_point = board_model.pose * projected_board_point;

                        (*input_point, corresponding_point)
                    })
                    .collect();

                if step == 0 || step % 10 == 0 {
                    // Show details for first step and every 10th step
                    debug!("Found {} correspondences", correspondings.len());
                    debug!("Correspondence details (showing first 5):");
                    for (i, (input_point, corresponding_point)) in
                        correspondings.iter().take(5).enumerate()
                    {
                        let distance = (input_point - corresponding_point).norm();
                        debug!("  {}: Input({:.4}, {:.4}, {:.4}) -> Corresponding({:.4}, {:.4}, {:.4}) | Distance: {:.6}",
                            i+1,
                            input_point.x, input_point.y, input_point.z,
                            corresponding_point.x, corresponding_point.y, corresponding_point.z,
                            distance
                        );
                    }
                    if correspondings.len() > 5 {
                        debug!(
                            "  ... and {} more correspondences",
                            correspondings.len() - 5
                        );
                    }
                }

                // reject outliers
                let correspondence_losses: Vec<_> = correspondings
                    .iter()
                    .map(|(input_point, corresponding_point)| {
                        let loss = (input_point - corresponding_point).norm();
                        loss
                    })
                    .collect();
                let avg_loss =
                    correspondence_losses.iter().sum::<f64>() / correspondings.len() as f64;

                if step == 0 || step % 10 == 0 {
                    // Show details for first step and every 10th step
                    let min_loss = correspondence_losses
                        .iter()
                        .fold(f64::INFINITY, |a, &b| a.min(b));
                    let max_loss = correspondence_losses
                        .iter()
                        .fold(f64::NEG_INFINITY, |a, &b| a.max(b));
                    debug!(
                        "Loss statistics: avg={:.6}, min={:.6}, max={:.6}",
                        avg_loss, min_loss, max_loss
                    );
                    debug!(
                        "Good fit threshold: {}, Outlier threshold: {}",
                        GOOD_FIT_THRESHOLD, OUTLIER_THRESHOLD
                    );
                }

                // Improved outlier filtering with adaptive thresholds
                // Use a more reasonable threshold that adapts to the current loss
                let adaptive_threshold = (avg_loss * 3.0).max(0.05).min(1.0); // Between 0.05 and 1.0

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

                if step == 0 || step % 10 == 0 {
                    debug!(
                        "Using adaptive threshold: {:.6} (avg_loss: {:.6})",
                        adaptive_threshold, avg_loss
                    );
                    debug!(
                        "{} points passed adaptive threshold",
                        good_correspondences.len()
                    );
                }

                let (good_inlier_points, good_corresponding_points): (
                    Vec<Point3<f64>>,
                    Vec<Point3<f64>>,
                ) = good_correspondences.into_iter().unzip();

                // Safety check: ensure we have at least 3 points for Kabsch
                if good_inlier_points.len() < 3 {
                    warn!(
                        "Not enough points for Kabsch ({} < 3), using identity transformation",
                        good_inlier_points.len()
                    );
                    let align_pose = Isometry3::identity();

                    // check termination criteria
                    termination_count = {
                        let pose_weight = {
                            let translation_weight = align_pose.translation.vector.norm();
                            let rotation_weight = align_pose
                                .rotation
                                .axis_angle()
                                .map(|(_, angle)| angle)
                                .unwrap_or(0.0);
                            translation_weight + rotation_weight
                        };

                        if step == 0 || step % 10 == 0 {
                            debug!("ICP Step {}: Pose weight analysis (identity)", step);
                            debug!(
                                "  Translation weight: {:.8}",
                                align_pose.translation.vector.norm()
                            );
                            debug!(
                                "  Rotation weight: {:.8}",
                                align_pose
                                    .rotation
                                    .axis_angle()
                                    .map(|(_, angle)| angle)
                                    .unwrap_or(0.0)
                            );
                            debug!("  Total pose weight: {:.8}", pose_weight);
                            debug!("  Threshold: {:.8}", icp_pose_weight_threshold);
                            debug!("  Avg loss: {:.8}", avg_loss);
                        }

                        if pose_weight <= icp_pose_weight_threshold {
                            termination_count + 1
                        } else {
                            0
                        }
                    };

                    // update state
                    losses.push(avg_loss);
                    // Keep the same points for next iteration
                    pose = pose * align_pose;
                    step += 1;

                    if step == 0 || step % 10 == 0 {
                        debug!("Termination count: {}/16", termination_count);
                        debug!("Step: {}/{}", step, max_icp_iterations);
                    }

                    if step == max_icp_iterations || termination_count > 16 {
                        debug!(
                            "ICP terminating: step={}, termination_count={}",
                            step, termination_count
                        );
                        break (inlier_points, good_corresponding_points, losses, pose);
                    }
                    continue;
                }

                // compute transformation
                let align_pose: Isometry3<_> = {
                    debug!(
                        "Computing transformation with {} points",
                        good_inlier_points.len()
                    );

                    let pairs = izip!(
                        good_inlier_points.iter().map(|&p| -> [f64; 3] { p.into() }),
                        good_corresponding_points
                            .iter()
                            .map(|&p| -> [f64; 3] { p.into() }),
                    );

                    match kabsch(pairs) {
                        Some((XYZ([x, y, z]), IJKW([i, j, k, w]))) => {
                            debug!("Kabsch succeeded: translation=({:.6}, {:.6}, {:.6}), rotation=({:.6}, {:.6}, {:.6}, {:.6})", x, y, z, i, j, k, w);
                            Isometry3 {
                                rotation: UnitQuaternion::from_quaternion(Quaternion::new(
                                    w, i, j, k,
                                )),
                                translation: Translation3::new(x, y, z),
                            }
                        }
                        None => {
                            warn!("Kabsch failed, using identity transformation");
                            Isometry3::identity()
                        }
                    }

                    // let align_translation = {
                    //     let input_centroid: Point3<f64> =
                    //         geom_algo::centroid_of_points(good_inlier_points.iter().map(|point| **point))
                    //             .unwrap();
                    //     let model_centroid: Point3<f64> =
                    //         geom_algo::centroid_of_points(good_corresponding_points.iter()).unwrap();
                    //     Translation3::from(input_centroid - model_centroid)
                    // };

                    // let align_quaternion = {
                    //     let input_target_pairs = good_corresponding_points
                    //         .iter()
                    //         .map(|point| align_translation * point)
                    //         .zip(good_inlier_points.iter().copied());

                    //     geom_algo::fit_rotation(input_target_pairs).unwrap()
                    // };
                    // align_quaternion * align_translation
                };

                // check termination criteria
                termination_count = {
                    let pose_weight = {
                        let translation_weight = align_pose.translation.vector.norm();
                        let rotation_weight = align_pose
                            .rotation
                            .axis_angle()
                            .map(|(_, angle)| angle)
                            .unwrap_or(0.0);
                        translation_weight + rotation_weight
                    };

                    if step == 0 || step % 10 == 0 {
                        // Show details for first step and every 10th step
                        debug!("ICP Step {}: Pose weight analysis", step);
                        debug!(
                            "  Translation weight: {:.8}",
                            align_pose.translation.vector.norm()
                        );
                        debug!(
                            "  Rotation weight: {:.8}",
                            align_pose
                                .rotation
                                .axis_angle()
                                .map(|(_, angle)| angle)
                                .unwrap_or(0.0)
                        );
                        debug!("  Total pose weight: {:.8}", pose_weight);
                        debug!("  Threshold: {:.8}", icp_pose_weight_threshold);
                        debug!("  Avg loss: {:.8}", avg_loss);
                    }

                    if pose_weight <= icp_pose_weight_threshold {
                        termination_count + 1
                    } else {
                        0
                    }
                };

                // update state
                losses.push(avg_loss);
                // Convert back to the expected format for the next iteration
                inlier_points = good_inlier_points;

                // Apply damping to prevent overshooting
                let damping_factor = 0.3; // Reduce the step size

                // Simple damping: interpolate between current pose and new pose
                let damped_translation =
                    Translation3::from(align_pose.translation.vector * damping_factor);
                let damped_rotation = UnitQuaternion::slerp(
                    &UnitQuaternion::identity(),
                    &align_pose.rotation,
                    damping_factor,
                );
                let damped_align_pose = Isometry3::from_parts(damped_translation, damped_rotation);

                pose = pose * damped_align_pose;
                step += 1;

                if step == 0 || step % 10 == 0 {
                    // Show details for first step and every 10th step
                    debug!("Termination count: {}/16", termination_count);
                    debug!("Step: {}/{}", step, max_icp_iterations);
                }

                if step == max_icp_iterations || termination_count > 16 {
                    debug!(
                        "ICP terminating: step={}, termination_count={}",
                        step, termination_count
                    );
                    break (inlier_points, good_corresponding_points, losses, pose);
                }
            }
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

            let correspondences: Vec<_> = izip!(
                inlier_points.iter().map(|point| (*point).to_owned()),
                corresponding_points.iter().map(|point| point.to_owned())
            )
            .collect();

            IcpData {
                correspondences,
                board_model,
            }
        };

        (pose, icp_losses, viz_msg)
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
            None => return Ok(None),
        };

        if min_icp_loss > icp_rejection_threshold {
            return Ok(None);
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
    debug!("ICP completed successfully! Loss: {:.6}", _final_loss);

    Ok(Some(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
    }))
}
