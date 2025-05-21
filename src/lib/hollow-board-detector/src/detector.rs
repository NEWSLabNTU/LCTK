use crate::{
    algo::{fit_board_icp, fit_plane_ransac},
    config::Config,
    detection::{FitBoardIcp, FitPlaneRansac},
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
        } = {
            let ret = fit_plane_ransac(&self.config, points)?;
            match ret {
                Some(ret) => ret,
                None => return Ok(None),
            }
        };

        // fit board using custom ICP
        let FitBoardIcp {
            board_pose,
            icp_losses,
            icp_data,
        } = {
            let opt = fit_board_icp(
                &self.config,
                &self.aruco_pattern,
                &plane_model,
                &plane_inlier_points,
            )?;

            match opt {
                Some(ret) => ret,
                None => return Ok(None),
            }
        };

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
            fixup_translation * fixup_rotation * &board_model.pose.rotation
        };

        let detection = Detection {
            board_model: BoardModel {
                pose,
                ..board_model
            },
            plane_ransac_data,
            icp_data,
            icp_losses,
        };

        Ok(Some(detection))
    }
}
