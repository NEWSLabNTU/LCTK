use crate::{
    config::Config,
    detection::{FitBoardIcp, FitPlaneRansac, IcpData, PlaneRansacData},
};
use anyhow::Result;
use arrsac::Arrsac;
use aruco_config::MultiArucoPattern;
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
    },
    fs::File,
    io::Write,
};

/// Helper function to save 3D points to CSV for visualization
fn save_points_to_csv_3d(points: &[impl Borrow<Point3<f64>>], filename: &str) -> std::io::Result<()> {
    let mut file = File::create(filename)?;
    writeln!(file, "x,y,z")?;
    
    for point in points {
        let p = point.borrow();
        writeln!(file, "{},{},{}", p.x, p.y, p.z)?;
    }
    
    println!("  💾 Saved {} points to {}", points.len(), filename);
    Ok(())
}

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

    println!("🔍 RANSAC Debug: Starting plane fitting");
    println!("  📊 Input points: {}", points.len());
    println!("  🎯 Inlier threshold: {}", plane_ransac_inlier_threshold);
    println!("  🔄 Max iterations: {}", plane_ransac_max_iterations);

    // Check minimum points requirement
    if points.len() < 3 {
        println!("  ❌ RANSAC failed: Need at least 3 points, got {}", points.len());
        return Ok(None);
    }

    let mut arrsac = Arrsac::new(plane_ransac_inlier_threshold, rand::thread_rng())
        .max_candidate_hypotheses(plane_ransac_max_iterations);
    let estimator = PlaneEstimator::new();
    
    let (plane_model, inlier_indices) = {
        match arrsac.model_inliers(&estimator, points.iter().cloned()) {
            Some(ret) => {
                println!("  ✅ RANSAC succeeded!");
                println!("    📈 Inliers found: {}", ret.1.len());
                println!("    📊 Inlier ratio: {:.2}%", (ret.1.len() as f64 / points.len() as f64) * 100.0);
                ret
            },
            None => {
                println!("  ❌ RANSAC failed: No valid plane found");
                println!("    Possible reasons:");
                println!("    - Points are too noisy/scattered");
                println!("    - Inlier threshold ({}) too strict", plane_ransac_inlier_threshold);
                println!("    - Not enough iterations ({})", plane_ransac_max_iterations);
                println!("    - Points don't form a plane");
                return Ok(None);
            }
        }
    };

    let inlier_points: Vec<_> = inlier_indices.into_iter().map(|idx| &points[idx]).collect();

    // Log plane model details
    println!("  🎯 Plane model found:");
    println!("    Normal: ({:.4}, {:.4}, {:.4})", 
             plane_model.normal[0], plane_model.normal[1], plane_model.normal[2]);
    println!("    Center: ({:.4}, {:.4}, {:.4})", 
             plane_model.center.x, plane_model.center.y, plane_model.center.z);

    // Save RANSAC inliers to CSV for 3D visualization
    if let Err(e) = save_points_to_csv_3d(&inlier_points, "ransac_plane_inliers.csv") {
        println!("  ⚠️ Failed to save RANSAC inliers: {}", e);
    }

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

    println!("🔧 ICP Debug: Starting board fitting");
    println!("  📊 Plane inlier points: {}", plane_inlier_points.len());

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
    
    println!("  ⚙️ ICP Parameters:");
    println!("    Max iterations: {}", max_icp_iterations);
    println!("    Pose weight threshold: {}", icp_pose_weight_threshold);
    println!("    Rejection threshold: {}", icp_rejection_threshold);
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
                UnitQuaternion::rotation_between(&board_z_axis, &plane_normal)
                    .unwrap_or_else(|| {
                        // If the vectors are parallel, use identity
                        UnitQuaternion::identity()
                    })
            };

            Isometry3::from_parts(Translation3::from(inlier_centroid.coords), rotation)
        };
        let init_inlier_points: Vec<&Point3<_>> = plane_inlier_points
            .iter()
            .map(|point| point.borrow())
            .collect();

        let (inlier_points, corresponding_points, icp_losses, pose) = {
            let mut inlier_points: Vec<Point3<f64>> = init_inlier_points.iter().map(|&p| *p).collect();
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
                
                if step == 0 || step % 10 == 0 { // Show details for first step and every 10th step
                    println!("  🔧 ICP Step {}: Board model created", step);
                    println!("    📍 Board pose translation: ({:.4}, {:.4}, {:.4})",
                             pose.translation.x, pose.translation.y, pose.translation.z);
                    println!("    🔄 Board pose rotation: ({:.4}, {:.4}, {:.4}, {:.4})",
                             pose.rotation.i, pose.rotation.j, pose.rotation.k, pose.rotation.w);
                    println!("    📊 Input inlier points: {}", inlier_points.len());
                }


                // Simple correspondence finding: project points onto board plane
                let correspondings: Vec<(Point3<f64>, Point3<f64>)> = inlier_points
                    .iter()
                    .map(|input_point| {
                        // Project point onto board plane
                        let board_center = board_model.board_center();
                        let board_normal = board_model.board_z_axis();
                        let vec_to_point: Vector3<f64> = *input_point - board_center;
                        let distance_to_plane = vec_to_point.dot(&board_normal);
                        let projected_point = *input_point - board_normal.scale(distance_to_plane);
                        
                        (*input_point, projected_point)
                    })
                    .collect();

                if step == 0 || step % 10 == 0 { // Show details for first step and every 10th step
                    println!("    ✅ Found {} correspondences", correspondings.len());
                    println!("    📊 Correspondence details (showing first 5):");
                    for (i, (input_point, corresponding_point)) in correspondings.iter().take(5).enumerate() {
                        let distance = (input_point - corresponding_point).norm();
                        println!("      {}: Input({:.4}, {:.4}, {:.4}) -> Corresponding({:.4}, {:.4}, {:.4}) | Distance: {:.6}",
                            i+1,
                            input_point.x, input_point.y, input_point.z,
                            corresponding_point.x, corresponding_point.y, corresponding_point.z,
                            distance
                        );
                    }
                    if correspondings.len() > 5 {
                        println!("      ... and {} more correspondences", correspondings.len() - 5);
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
                let avg_loss = correspondence_losses.iter().sum::<f64>() / correspondings.len() as f64;
                
                if step == 0 || step % 10 == 0 { // Show details for first step and every 10th step
                    let min_loss = correspondence_losses.iter().fold(f64::INFINITY, |a, &b| a.min(b));
                    let max_loss = correspondence_losses.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
                    println!("    📈 Loss statistics: avg={:.6}, min={:.6}, max={:.6}", avg_loss, min_loss, max_loss);
                    println!("    🎯 Good fit threshold: {}, Outlier threshold: {}", GOOD_FIT_THRESHOLD, OUTLIER_THRESHOLD);
                }

                let good_correspondences: Vec<_> = correspondings
                    .iter()
                    .zip(losses.iter())
                    .filter_map(|((input_point, corresponding_point), &loss)| {
                        if loss <= OUTLIER_THRESHOLD {
                            Some((*input_point, *corresponding_point))
                        } else {
                            None
                        }
                    })
                    .collect();

                let (good_inlier_points, good_corresponding_points): (Vec<Point3<f64>>, Vec<Point3<f64>>) = good_correspondences.into_iter().unzip();

                // compute transformation
                let align_pose: Isometry3<_> = {
                    // let lhs = good_inlier_points.into_iter().map(<[f64; 3]>::from);
                    let pairs = izip!(
                        good_inlier_points
                            .iter()
                            .map(|&p| -> [f64; 3] { p.into() }),
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
                    
                    if step == 0 || step % 10 == 0 { // Show details for first step and every 10th step
                        println!("    🔄 ICP Step {}: Pose weight analysis", step);
                        println!("      📏 Translation weight: {:.8}", align_pose.translation.vector.norm());
                        println!("      🔄 Rotation weight: {:.8}", align_pose.rotation.axis_angle().map(|(_, angle)| angle).unwrap_or(0.0));
                        println!("      ⚖️ Total pose weight: {:.8}", pose_weight);
                        println!("      🎯 Threshold: {:.8}", icp_pose_weight_threshold);
                        println!("      📊 Avg loss: {:.8}", avg_loss);
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
                pose = pose * align_pose;
                step += 1;

                if step == 0 || step % 10 == 0 { // Show details for first step and every 10th step
                    println!("    📊 Termination count: {}/16", termination_count);
                    println!("    🔢 Step: {}/{}", step, max_icp_iterations);
                }
                
                if step == max_icp_iterations || termination_count > 16 {
                    println!("    🛑 ICP terminating: step={}, termination_count={}", step, termination_count);
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
    
    // Save board corners
    let board_width_f64: f64 = board_width.as_meters();
    let board_corners = vec![
        board_model.bottom_corner(),
        board_model.top_corner(),
        board_model.top_corner() + board_model.board_x_axis().as_ref() * board_width_f64,
        board_model.bottom_corner() + board_model.board_x_axis().as_ref() * board_width_f64,
    ];
    
    if let Err(e) = save_points_to_csv_3d(&board_corners, "icp_board_corners.csv") {
        println!("  ⚠️ Failed to save ICP board corners: {}", e);
    }
    
    // Save board center and pose information
    let board_info = vec![
        board_model.board_center(),
        board_model.board_center() + board_model.board_x_axis().as_ref() * 0.1, // X axis indicator
        board_model.board_center() + board_model.board_y_axis().as_ref() * 0.1, // Y axis indicator  
        board_model.board_center() + board_model.board_z_axis().as_ref() * 0.1, // Z axis indicator
    ];
    
    if let Err(e) = save_points_to_csv_3d(&board_info, "icp_board_pose.csv") {
        println!("  ⚠️ Failed to save ICP board pose: {}", e);
    }

    let final_loss = icp_losses.iter().copied().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(0.0);
    println!("  🎯 ICP completed successfully! Loss: {:.6}", final_loss);

    Ok(Some(FitBoardIcp {
        board_pose,
        icp_losses,
        icp_data: viz_msg,
    }))
}
