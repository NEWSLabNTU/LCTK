use crate::{
    config::Config,
    detection::{FitBoardIcp, FitPlaneRansac, IcpData, PlaneRansacData},
};
use anyhow::Result;
use approx::abs_diff_eq;
use arrsac::Arrsac;
use aruco_config::multi_aruco::MultiArucoPattern;
use hollow_board_config::{BoardModel, BoardShape};
use itertools::izip;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use newslab_geom_algo::{self, centroid_of_points, kabsch, IJKW, XYZ};
use noisy_float::prelude::*;
use plane_estimator::{PlaneEstimator, PlaneModel};
use sample_consensus::Consensus;
use std::{
    borrow::Borrow,
    f64::{
        self,
        consts::{FRAC_PI_2, FRAC_PI_4, PI},
    },
};

unzip_n::unzip_n!(2);

const EPS_F64: f64 = 1e-4;

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

    let mut arrsac = Arrsac::new(plane_ransac_inlier_threshold, rand::thread_rng())
        .max_candidate_hypotheses(plane_ransac_max_iterations);
    let estimator = PlaneEstimator::new();
    let (plane_model, inlier_indices) = {
        match arrsac.model_inliers(&estimator, points.iter().cloned()) {
            Some(ret) => ret,
            None => return Ok(None),
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
) -> Result<Option<FitBoardIcp>> {
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

            // let the xy-plane projections of board normal and plane normal overlap
            // it decreases the chance of falling into local minimum
            let rotation = {
                let lifting_rotation = UnitQuaternion::from_euler_angles(0.0, -FRAC_PI_2, 0.0)
                    * UnitQuaternion::from_euler_angles(0.0, 0.0, -FRAC_PI_4);
                let lifted_normal = lifting_rotation * Vector3::z_axis();
                debug_assert!(abs_diff_eq!(
                    (*lifted_normal + *Vector3::x_axis()).norm(),
                    0.0,
                    epsilon = EPS_F64
                ));

                let planar_rotation = {
                    let planar_plane_normal = Vector3::new(plane_normal.x, plane_normal.y, 0.0);
                    UnitQuaternion::rotation_between(&lifted_normal, &planar_plane_normal)
                        .unwrap_or_else(|| {
                            if lifted_normal.dot(&planar_plane_normal) >= 0.0 {
                                UnitQuaternion::identity()
                            } else {
                                UnitQuaternion::from_euler_angles(0.0, 0.0, PI)
                            }
                        })
                };
                planar_rotation * lifting_rotation
            };

            Isometry3::from_parts(Translation3::identity(), rotation)
        };
        let init_inlier_points: Vec<&Point3<_>> = plane_inlier_points
            .iter()
            .map(|point| point.borrow())
            .collect();

        let (inlier_points, corresponding_points, icp_losses, pose) = {
            let mut inlier_points = init_inlier_points;
            let mut losses = vec![];
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

                let correspondings = board_model.find_correspondences(inlier_points);
                let correspondings = match correspondings {
                    Some(corr) => corr,
                    None => return Ok(None),
                };
                debug_assert!({
                    correspondings
                        .iter()
                        .all(|(_data_point, corresponding_point)| {
                            let center = board_model.board_center();

                            abs_diff_eq!(
                                board_model
                                    .board_z_axis()
                                    .dot(&(corresponding_point - center)),
                                0.0,
                                epsilon = EPS_F64
                            )
                        })
                });

                // reject outliers
                let (good_inlier_points, good_corresponding_points, avg_loss) = {
                    let losses: Vec<_> = correspondings
                        .iter()
                        .map(|(input_point, corresponding_point)| {
                            let loss = (input_point.borrow() - corresponding_point).norm();
                            loss
                        })
                        .collect();
                    let avg_loss = losses.iter().sum::<f64>() / correspondings.len() as f64;

                    let good_correspondences: Vec<_> = if avg_loss <= GOOD_FIT_THRESHOLD {
                        izip!(correspondings, losses.iter().cloned())
                            .filter_map(|((inlier_point, corresponding_point), loss)| {
                                (loss < OUTLIER_THRESHOLD)
                                    .then_some((inlier_point, corresponding_point))
                            })
                            .collect()
                    } else {
                        correspondings
                    };

                    let (good_inlier_points, good_corresponding_points) =
                        good_correspondences.into_iter().unzip_n_vec();

                    (good_inlier_points, good_corresponding_points, avg_loss)
                };

                // compute transformation
                let align_pose: Isometry3<_> = {
                    // let lhs = good_inlier_points.into_iter().map(<[f64; 3]>::from);
                    let pairs = izip!(
                        good_inlier_points
                            .iter()
                            .map(|&&p| -> [f64; 3] { p.into() }),
                        good_corresponding_points
                            .iter()
                            .map(|&p| -> [f64; 3] { p.into() }),
                    );

                    match kabsch(pairs) {
                        Some((XYZ([x, y, z]), IJKW([i, j, k, w]))) => Isometry3 {
                            rotation: UnitQuaternion::from_quaternion(Quaternion::new(w, i, j, k)),
                            translation: Translation3::new(x, y, z),
                        },
                        None => Isometry3::identity(),
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
                    if pose_weight <= icp_pose_weight_threshold {
                        termination_count + 1
                    } else {
                        0
                    }
                };

                // update state
                losses.push(avg_loss);
                inlier_points = good_inlier_points;
                pose = align_pose * pose;
                step += 1;

                if step == max_icp_iterations || termination_count > 16 {
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

    Ok(Some(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
    }))
}
