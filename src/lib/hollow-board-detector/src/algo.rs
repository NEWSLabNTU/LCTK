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
                ret
            }
            None => {
                warn!("RANSAC failed: No valid plane found");
                return Ok(None);
            }
        }
    };

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
) -> Result<FitBoardIcp> {
    // find board by modified ICP algoirthm
    const GOOD_FIT_THRESHOLD: f64 = 0.015; // velodyne 32-MR
                                           // let good_fit_threshold = 0.1; // ouster os-1
    const OUTLIER_THRESHOLD: f64 = 0.1;

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

                let (good_inlier_points, good_corresponding_points): (
                    Vec<Point3<f64>>,
                    Vec<Point3<f64>>,
                ) = good_correspondences.into_iter().unzip();

                // Safety check: ensure we have at least 3 points for Kabsch
                if good_inlier_points.len() < 3 {
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

                    if step == max_icp_iterations || termination_count > 100 {
                        break (inlier_points, good_corresponding_points, losses, pose);
                    }
                    continue;
                }

                // compute transformation
                let align_pose: Isometry3<_> = {
                    let pairs = izip!(
                        good_inlier_points.iter().map(|&p| -> [f64; 3] { p.into() }),
                        good_corresponding_points
                            .iter()
                            .map(|&p| -> [f64; 3] { p.into() }),
                    );

                    match kabsch(pairs) {
                        Some((XYZ([x, y, z]), IJKW([i, j, k, w]))) => {
                            Isometry3 {
                                rotation: UnitQuaternion::from_quaternion(Quaternion::new(
                                    w, i, j, k,
                                )),
                                translation: Translation3::new(x, y, z),
                            }
                        }
                        None => {
                            Isometry3::identity()
                        }
                    }
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
                let damping_factor = 0.05; // Reduce the step size

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
                
                // Removed premature break on small inlier count; rely on thresholds/iterations
                if *losses.last().unwrap() < icp_rejection_threshold {
                    debug!("🏆 ICP terminating: loss is too small: {:.8}", losses.last().unwrap());
                    debug!("  Pose weight threshold: {:.8}", icp_pose_weight_threshold);
                    debug!("  Rejection threshold: {:.8}", icp_rejection_threshold);
                    debug!("  Avg loss: {:.8}", *losses.last().unwrap());
                    debug!("  Inlier points: {}", inlier_points.len());
                    debug!(
                        "  Good corresponding points: {}",
                        good_corresponding_points.len()
                    );
                    debug!("  Pose: {:.8}", pose);
                    break (inlier_points, good_corresponding_points, losses, pose);
                }

                if step == max_icp_iterations || termination_count > 100 {
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
            None => return Ok(FitBoardIcp {
                board_pose,
                icp_losses,
                icp_data: viz_msg,
                successful: false,
            }),
        };

        if min_icp_loss > icp_rejection_threshold {
            return Ok(FitBoardIcp {
                board_pose,
                icp_losses,
                icp_data: viz_msg,
                successful: false,
            });
        }
    }

    let _final_loss = icp_losses
        .iter()
        .copied()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap_or(0.0);
    debug!("ICP completed successfully! Loss: {:.6}", _final_loss);

    Ok(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
        successful: true,
    })
}
