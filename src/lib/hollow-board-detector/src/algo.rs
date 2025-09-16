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

                let rot = planar_rotation * lifting_rotation;
                // Debug initialization details
                {
                    debug!(
                        "Init pose -> centroid: [{:.6}, {:.6}, {:.6}], plane_normal: [{:.6}, {:.6}, {:.6}]",
                        inlier_centroid.x,
                        inlier_centroid.y,
                        inlier_centroid.z,
                        plane_normal.x,
                        plane_normal.y,
                        plane_normal.z
                    );
                    let axis_ang = rot
                        .axis_angle()
                        .map(|(axis, ang)| (axis.into_inner(), ang));
                    if let Some((axis, ang)) = axis_ang {
                        debug!(
                            "Init rotation axis: [{:.6}, {:.6}, {:.6}], angle: {:.6}",
                            axis.x, axis.y, axis.z, ang
                        );
                    } else {
                        debug!("Init rotation is identity (no axis-angle)");
                    }
                }
                rot
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

                // Use the board model's correspondence finding method for proper closest point calculation
                let correspondings = match board_model.find_correspondences(&inlier_points) {
                    Some(corr) => corr,
                    None => {
                        debug!("ICP step {}: No correspondences found", step);
                        break (inlier_points, vec![], losses, pose);
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

                debug!("ICP step {}: avg_loss = {:.6}, correspondences = {}", step, avg_loss, correspondings.len());
                debug!("  -> Loss progression: {:?}", losses);

                // Improved outlier filtering with adaptive thresholds
                // Use a more reasonable threshold that adapts to the current loss
                let adaptive_threshold = (avg_loss * 2.0).max(0.01).min(1.0);
                debug!(
                    "  -> adaptive_threshold={:.6} (from avg_loss={:.6})",
                    adaptive_threshold, avg_loss
                );

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

                debug!("  -> good_correspondences = {}, good_inlier_points = {}", good_correspondences_len, good_inlier_points.len());

                // Centroids diagnostics (selected pairs)
                if good_inlier_points.len() >= 3 {
                    if let Some(in_centroid_arr) = centroid_of_points(good_inlier_points.iter().map(|p| {
                        let a: [f64; 3] = (*p).into();
                        a
                    })) {
                        let in_centroid: Point3<f64> = in_centroid_arr.into();
                        debug!(
                            "  -> inlier centroid: [{:.6}, {:.6}, {:.6}]",
                            in_centroid.x, in_centroid.y, in_centroid.z
                        );
                    }
                    if let Some(mod_centroid_arr) = centroid_of_points(good_corresponding_points.iter().map(|p| {
                        let a: [f64; 3] = (*p).into();
                        a
                    })) {
                        let mod_centroid: Point3<f64> = mod_centroid_arr.into();
                        debug!(
                            "  -> model centroid:  [{:.6}, {:.6}, {:.6}]",
                            mod_centroid.x, mod_centroid.y, mod_centroid.z
                        );
                    }
                }

                // Safety check: ensure we have at least 3 points for Kabsch
                if good_inlier_points.len() < 3 {
                    debug!("ICP step {}: Too few inlier points ({}), terminating", step, good_inlier_points.len());
                    break (inlier_points, good_corresponding_points, losses, pose);
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
                        good_inlier_points
                            .iter()
                            .map(|&p| -> [f64; 3] { p.into() }),
                    );

                    match kabsch(pairs) {
                        Some((XYZ([x, y, z]), IJKW([i, j, k, w]))) => {
                            let iso = Isometry3 {
                                rotation: UnitQuaternion::from_quaternion(Quaternion::new(
                                    w, i, j, k,
                                )),
                                translation: Translation3::new(x, y, z),
                            };
                            let aa = iso.rotation.axis_angle().map(|(ax, ang)| (ax.into_inner(), ang));
                            if let Some((ax, ang)) = aa {
                                debug!(
                                    "  -> align_pose: t=[{:.6}, {:.6}, {:.6}], axis=[{:.6}, {:.6}, {:.6}], angle={:.6}",
                                    iso.translation.vector.x,
                                    iso.translation.vector.y,
                                    iso.translation.vector.z,
                                    ax.x, ax.y, ax.z, ang
                                );
                            } else {
                                debug!(
                                    "  -> align_pose: t=[{:.6}, {:.6}, {:.6}], identity rotation",
                                    iso.translation.vector.x,
                                    iso.translation.vector.y,
                                    iso.translation.vector.z
                                );
                            }
                            iso
                        }
                        None => {
                            debug!("ICP step {}: Failed to fit transformation, terminating", step);
                            break (inlier_points, good_corresponding_points, losses, pose);
                        }
                    }
                };

                // update state
                losses.push(avg_loss);
                // Convert back to the expected format for the next iteration
                inlier_points = good_inlier_points;

                debug!("  -> align_pose translation: {:.6}, rotation angle: {:.6}", 
                       align_pose.translation.vector.norm(),
                       align_pose.rotation.axis_angle().map(|(_, angle)| angle).unwrap_or(0.0));

                // Apply damping to prevent overshooting
                let damping_factor = 0.3; // More reasonable damping factor for faster convergence

                // Apply damping to the pose update (not the transformation itself)
                // This interpolates between the current pose and the new pose after applying the transformation
                let new_pose = pose * align_pose;

                // Diagnostics for pose delta before damping
                let delta_t = (new_pose.translation.vector - pose.translation.vector).norm();
                let delta_ang = new_pose
                    .rotation
                    .rotation_to(&pose.rotation)
                    .angle();
                debug!(
                    "  -> pose delta pre-damp: dT={:.6}, dAng={:.6}, damping_factor={:.3}",
                    delta_t, delta_ang, damping_factor
                );

                // Damp the translation component
                let damped_translation = Translation3::from(
                    pose.translation.vector +
                    (new_pose.translation.vector - pose.translation.vector) * damping_factor
                );

                // Damp the rotation component using spherical linear interpolation
                let damped_rotation = UnitQuaternion::slerp(
                    &pose.rotation,
                    &new_pose.rotation,
                    damping_factor,
                );

                // Termination criteria based on the actually applied (damped) update
                {
                    let applied_t = (damped_translation.vector - pose.translation.vector).norm();
                    let applied_ang = damped_rotation
                        .rotation_to(&pose.rotation)
                        .angle();
                    let pose_weight = applied_t + applied_ang;
                    if pose_weight <= icp_pose_weight_threshold {
                        termination_count += 1;
                    } else {
                        termination_count = 0;
                    }
                    debug!(
                        "  -> applied pose_weight: {:.8}, termination_count: {}",
                        pose_weight, termination_count
                    );
                }

                pose = Isometry3::from_parts(damped_translation, damped_rotation);
                step += 1;

                debug!("  -> new pose translation: {:.?}, rotation angle: {:.?}", 
                       pose.translation.vector,
                       pose.rotation.axis_angle().map(|(_, angle)| angle).unwrap_or(0.0));
                debug!("  -> Updated loss: {:.8}, inlier_count: {}", avg_loss, inlier_points.len());
                
                // Check if we have enough inlier points to continue
                if inlier_points.len() < 2000 {
                    debug!("ICP terminating: too few inlier points: {}", inlier_points.len());
                    break (inlier_points, good_corresponding_points, losses, pose);
                }
                
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

    Ok(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
        successful: true,
    })
}
