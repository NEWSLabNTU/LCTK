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

            // obtain the plane normal vector that points towards the origin (like lib.rs)
            let plane_normal = {
                let normal: Vector3<f64> = nalgebra::convert(*plane_model.normal);
                if (Point3::origin() - inlier_centroid).dot(&normal) < 0.0 {
                    -normal
                } else {
                    normal
                }
            };

            // Let the XY-plane projections of board normal and plane normal overlap
            // (follow the rotation initialization logic from lib.rs)
            let rotation = {
                // Lift the board's +Z to point toward +X after two fixed rotations
                let lifting_rotation =
                    UnitQuaternion::from_euler_angles(0.0, -f64::consts::FRAC_PI_2, 0.0)
                        * UnitQuaternion::from_euler_angles(0.0, 0.0, -f64::consts::FRAC_PI_4);
                let lifted_normal = lifting_rotation * Vector3::z_axis();

                // Align the lifted normal to the plane normal projected on XY plane
                let planar_rotation = {
                    let planar_plane_normal = Vector3::new(plane_normal.x, plane_normal.y, 0.0);
                    UnitQuaternion::rotation_between(&lifted_normal, &planar_plane_normal)
                        .unwrap_or_else(|| {
                            if lifted_normal.dot(&planar_plane_normal) >= 0.0 {
                                UnitQuaternion::identity()
                            } else {
                                UnitQuaternion::from_euler_angles(0.0, 0.0, f64::consts::PI)
                            }
                        })
                };

                planar_rotation * lifting_rotation
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

                if let Some(cb) = progress_cb.as_mut() {
                    cb(&board_model);
                }

                // Use the board model's correspondence finding method
                let correspondings = match board_model.find_correspondences(&inlier_points) {
                    Some(corr) => corr,
                    None => {
                        debug!("ICP step {}: No correspondences found", step);
                        break (inlier_points, vec![], losses, pose);
                    }
                };

                // reject outliers (mirror logic from lib.rs)
                let (good_inlier_points, good_corresponding_points, avg_loss) = {
                    let losses: Vec<_> = correspondings
                        .iter()
                        .map(|(input_point, corresponding_point)| {
                            (*input_point - corresponding_point).norm()
                        })
                        .collect();
                    let avg_loss = losses.iter().sum::<f64>() / correspondings.len() as f64;

                    debug!("ICP step {}: avg_loss = {:.6}, correspondences = {}", step, avg_loss, correspondings.len());
                    debug!("  -> Loss progression: {:?}", losses);

                    let good_correspondences: Vec<_> = if avg_loss <= GOOD_FIT_THRESHOLD {
                        izip!(correspondings.into_iter(), losses.into_iter())
                            .filter_map(|((inlier_point, corresponding_point), loss)| {
                                (loss < OUTLIER_THRESHOLD)
                                    .then(|| (inlier_point, corresponding_point))
                            })
                            .collect()
                    } else {
                        correspondings
                    };

                    let good_correspondences_len = good_correspondences.len();
                    let (good_inlier_points, good_corresponding_points): (
                        Vec<Point3<f64>>,
                        Vec<Point3<f64>>,
                    ) = good_correspondences.into_iter().unzip();

                    debug!("  -> good_correspondences = {}, good_inlier_points = {}", good_correspondences_len, good_inlier_points.len());

                    (good_inlier_points, good_corresponding_points, avg_loss)
                };

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

                // compute transformation (mirror lib.rs approach)
                let align_pose: Isometry3<_> = {
                    let align_translation = {
                        let input_centroid: Point3<f64> =
                            centroid_of_points(good_inlier_points.iter().map(|point| {
                                let point: [f64; 3] = (*point).into();
                                point
                            }))
                            .unwrap()
                            .into();
                        let model_centroid: Point3<f64> =
                            centroid_of_points(good_corresponding_points.iter().map(|point| {
                                let point: [f64; 3] = (*point).into();
                                point
                            }))
                            .unwrap()
                            .into();
                        Translation3::from(input_centroid - model_centroid)
                    };

                    let align_quaternion = {
                        let input_target_pairs = good_corresponding_points
                            .iter()
                            .map(|point| align_translation * point)
                            .zip(good_inlier_points.iter().copied());

                        let pairs = izip!(
                            input_target_pairs.clone().map(|(_, p)| -> [f64; 3] { p.into() }),
                            input_target_pairs.map(|(p, _)| -> [f64; 3] { p.into() }),
                        );

                        match kabsch(pairs) {
                            Some((XYZ([_x, _y, _z]), IJKW([i, j, k, w]))) => UnitQuaternion::from_quaternion(Quaternion::new(w, i, j, k)),
                            None => {
                                debug!("ICP step {}: Failed to fit rotation, using identity", step);
                                UnitQuaternion::identity()
                            }
                        }
                    };

                    Isometry3::from_parts(align_translation, align_quaternion)
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
                inlier_points = good_inlier_points;

                debug!("  -> align_pose translation: {:.6}, rotation angle: {:.6}", 
                       align_pose.translation.vector.norm(),
                       align_pose.rotation.axis_angle().map(|(_, angle)| angle).unwrap_or(0.0));
                debug!("  -> pose_weight: {:.8}, termination_count: {}", 
                       align_pose.translation.vector.norm() + align_pose.rotation.axis_angle().map(|(_, angle)| angle).unwrap_or(0.0),
                       termination_count);

                // Apply pose update (mirror lib.rs: align_pose * pose)
                pose = align_pose * pose;
                step += 1;

                debug!("  -> new pose translation: {:.6}, rotation angle: {:.6}", 
                       pose.translation.vector.norm(),
                       pose.rotation.axis_angle().map(|(_, angle)| angle).unwrap_or(0.0));
                debug!("  -> Updated loss: {:.8}, inlier_count: {}", avg_loss, inlier_points.len());
                
                // Removed premature break on small inlier count; rely on thresholds/iterations
                if *losses.last().unwrap() < icp_rejection_threshold {
                    debug!(
                        "🏆 ICP terminating: loss is too small: {:.8}",
                        losses.last().unwrap()
                    );
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
