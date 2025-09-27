use crate::{
    algo::{fit_board_icp, fit_board_icp_with_iterator, fit_plane_ransac, BoardIcpIterator},
    config::Config,
    detection::{BoardIcpState, BoardModelParams, FitBoardIcp, FitPlaneRansac},
    Detection,
};
use anyhow::Result;
use aruco_config::MultiArucoPattern;
use hollow_board_config::{BoardModel, BoardShape};
use na::coordinates::XYZ;
use nalgebra as na;
use noisy_float::prelude::*;
use std::f64::{self, consts::FRAC_PI_2};

#[derive(Debug, Clone)]
pub struct Detector {
    config: Config,
    aruco_pattern: MultiArucoPattern,
}

impl Detector {
    pub fn new(config: Config, aruco_pattern: MultiArucoPattern) -> Self {
        Self {
            config,
            aruco_pattern,
        }
    }

    /// Get a reference to the detector configuration
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Get a reference to the ArUco pattern
    pub fn aruco_pattern(&self) -> &MultiArucoPattern {
        &self.aruco_pattern
    }

    pub fn detect(&self, points: &[na::Point3<f64>]) -> Result<Option<Detection>> {
        let Config {
            board_shape:
                BoardShape {
                    board_width,
                    hole_radius,
                    hole_center_shift,
                },
            ..
        } = self.config;
        let marker_paper_size = self.aruco_pattern.paper_size();

        // fit plane using AARSAC (a derivation of RANSAC)
        let FitPlaneRansac {
            plane_model,
            inlier_points: plane_inlier_points,
            ransac_data: plane_ransac_data,
        } = match fit_plane_ransac(&self.config, points)? {
            Some(ret) => ret,
            None => return Ok(None),
        };

        // fit board using custom ICP (now returns FitBoardIcp directly)
        let FitBoardIcp {
            board_pose,
            icp_losses,
            icp_data,
            successful,
            initial_pose,
            icp_stats,
        } = fit_board_icp(
            &self.config,
            &self.aruco_pattern,
            &plane_model,
            &plane_inlier_points,
            None,
        )?;

        if !successful {
            return Ok(None);
        }

        let board_model = BoardModel {
            pose: board_pose,
            board_shape: BoardShape {
                board_width,
                hole_radius,
                hole_center_shift,
            },
            marker_paper_size,
        };

        // Correct the board pose to ensure it stands right.
        let pose = {
            let board_normal = board_model.board_z_axis();

            let corners = [
                board_model.bottom_corner(),
                board_model.left_corner(),
                board_model.top_corner(),
                board_model.right_corner(),
            ];
            let (lowest_index, _) = corners
                .iter()
                .enumerate()
                .min_by_key(|(_, point)| r64(point.z))
                .unwrap();

            // Compute the rotation using the by 90deg * index.
            let fixup_rotation = {
                let angle = FRAC_PI_2 * lowest_index as f64;
                na::UnitQuaternion::from_axis_angle(&board_normal, angle)
            };

            // Use the position of the lowest point as the new translation.
            let fixup_translation = {
                let lowest_point = &corners[lowest_index];
                let XYZ { x, y, z } = **lowest_point;
                na::Translation3::new(x, y, z)
            };

            // Construct the corrected pose
            fixup_translation * fixup_rotation * board_model.pose.rotation
        };

        let detection = Detection {
            board_model: BoardModel {
                pose,
                ..board_model
            },
            plane_ransac_data,
            icp_data,
            icp_losses,
            initial_pose,
            icp_stats,
        };

        Ok(Some(detection))
    }

    pub fn detect_with_progress<F>(
        &self,
        points: &[na::Point3<f64>],
        mut progress: F,
    ) -> Result<Option<Detection>>
    where
        F: FnMut(&BoardModel),
    {
        let Config {
            board_shape:
                BoardShape {
                    board_width,
                    hole_radius,
                    hole_center_shift,
                },
            ..
        } = self.config;
        let marker_paper_size = self.aruco_pattern.paper_size();

        let FitPlaneRansac {
            plane_model,
            inlier_points: plane_inlier_points,
            ransac_data: plane_ransac_data,
        } = match fit_plane_ransac(&self.config, points)? {
            Some(ret) => ret,
            None => return Ok(None),
        };

        let FitBoardIcp {
            board_pose,
            icp_losses,
            icp_data,
            successful,
            initial_pose,
            icp_stats,
        } = fit_board_icp(
            &self.config,
            &self.aruco_pattern,
            &plane_model,
            &plane_inlier_points,
            Some(&mut progress),
        )?;

        if !successful {
            return Ok(None);
        }

        let board_model = BoardModel {
            pose: board_pose,
            board_shape: BoardShape {
                board_width,
                hole_radius,
                hole_center_shift,
            },
            marker_paper_size,
        };

        let pose = {
            let board_normal = board_model.board_z_axis();

            let corners = [
                board_model.bottom_corner(),
                board_model.left_corner(),
                board_model.top_corner(),
                board_model.right_corner(),
            ];
            let (lowest_index, _) = corners
                .iter()
                .enumerate()
                .min_by_key(|(_, point)| r64(point.z))
                .unwrap();

            let fixup_rotation = {
                let angle = FRAC_PI_2 * lowest_index as f64;
                na::UnitQuaternion::from_axis_angle(&board_normal, angle)
            };

            let fixup_translation = {
                let lowest_point = &corners[lowest_index];
                let XYZ { x, y, z } = **lowest_point;
                na::Translation3::new(x, y, z)
            };

            fixup_translation * fixup_rotation * board_model.pose.rotation
        };

        let detection = Detection {
            board_model: BoardModel {
                pose,
                ..board_model
            },
            plane_ransac_data,
            icp_data,
            icp_losses,
            initial_pose,
            icp_stats,
        };

        Ok(Some(detection))
    }

    pub fn detect_with_iterator<F>(
        &self,
        points: &[na::Point3<f64>],
        mut iteration_callback: F,
    ) -> Result<Option<Detection>>
    where
        F: FnMut(&BoardIcpState, &BoardModel),
    {
        let Config {
            board_shape:
                BoardShape {
                    board_width,
                    hole_radius,
                    hole_center_shift,
                },
            ..
        } = self.config;
        let marker_paper_size = self.aruco_pattern.paper_size();

        let FitPlaneRansac {
            plane_model,
            inlier_points: plane_inlier_points,
            ransac_data: plane_ransac_data,
        } = match fit_plane_ransac(&self.config, points)? {
            Some(ret) => ret,
            None => return Ok(None),
        };

        let board_model_params = BoardModelParams {
            board_shape: BoardShape {
                board_width,
                hole_radius,
                hole_center_shift,
            },
            marker_paper_size,
        };

        let (board_pose, icp_losses, icp_data, successful, initial_pose, icp_stats) = {
            use crate::algo::fit_board_icp;
            use std::borrow::Borrow;

            let init_pose = fit_board_icp(
                &self.config,
                &self.aruco_pattern,
                &plane_model,
                &plane_inlier_points,
                None,
            )?
            .initial_pose;

            let mut iterator =
                BoardIcpIterator::new(&self.config, board_model_params.clone(), None);

            let init_inlier_points: Vec<na::Point3<f64>> =
                plane_inlier_points.iter().map(|p| *p.borrow()).collect();

            let mut state = iterator.initial_state(init_pose, init_inlier_points);
            let mut losses = vec![];
            let initial_loss = state.avg_loss;

            while !iterator.should_terminate(&state) {
                let current_board_model = BoardModel {
                    pose: state.board_pose,
                    board_shape: board_model_params.board_shape.clone(),
                    marker_paper_size: board_model_params.marker_paper_size,
                };

                iteration_callback(&state, &current_board_model);

                state = iterator.step(&state);
                losses.push(state.avg_loss);
            }

            let successful = !iterator.termination_reason(&state).contains("failed")
                && !iterator.termination_reason(&state).contains("Insufficient");

            let icp_stats = crate::detection::IcpStatistics {
                iterations: state.iteration,
                final_loss: state.avg_loss,
                min_loss: losses.iter().copied().fold(f64::INFINITY, f64::min),
                successful,
                initial_loss,
                convergence_reason: iterator.termination_reason(&state),
            };

            let icp_data = crate::detection::IcpData {
                correspondences: state.correspondences,
                board_model: BoardModel {
                    pose: state.board_pose,
                    board_shape: board_model_params.board_shape.clone(),
                    marker_paper_size: board_model_params.marker_paper_size,
                },
            };

            (
                state.board_pose,
                losses,
                icp_data,
                successful,
                init_pose,
                icp_stats,
            )
        };

        if !successful {
            return Ok(None);
        }

        let board_model = BoardModel {
            pose: board_pose,
            board_shape: BoardShape {
                board_width,
                hole_radius,
                hole_center_shift,
            },
            marker_paper_size,
        };

        let pose = {
            let board_normal = board_model.board_z_axis();

            let corners = [
                board_model.bottom_corner(),
                board_model.left_corner(),
                board_model.top_corner(),
                board_model.right_corner(),
            ];
            let (lowest_index, _) = corners
                .iter()
                .enumerate()
                .min_by_key(|(_, point)| r64(point.z))
                .unwrap();

            let fixup_rotation = {
                let angle = FRAC_PI_2 * lowest_index as f64;
                na::UnitQuaternion::from_axis_angle(&board_normal, angle)
            };

            let fixup_translation = {
                let lowest_point = &corners[lowest_index];
                let XYZ { x, y, z } = **lowest_point;
                na::Translation3::new(x, y, z)
            };

            fixup_translation * fixup_rotation * board_model.pose.rotation
        };

        let detection = Detection {
            board_model: BoardModel {
                pose,
                ..board_model
            },
            plane_ransac_data,
            icp_data,
            icp_losses,
            initial_pose,
            icp_stats,
        };

        Ok(Some(detection))
    }
}
