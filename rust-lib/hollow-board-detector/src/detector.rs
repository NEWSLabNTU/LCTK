use crate::{
    algo::{fit_board_icp, fit_plane_ransac},
    config::Config,
    detection::{FitBoardIcp, FitPlaneRansac},
    BoardDetection,
};
use anyhow::Result;
use aruco_config::multi_aruco::MultiArucoPattern;
use hollow_board_config::{BoardModel, BoardShape};
use nalgebra as na;
use std::f64;

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

    pub fn detect(&self, points: &[na::Point3<f64>]) -> Result<Option<BoardDetection>> {
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

        let detection = BoardDetection {
            board_model: BoardModel {
                pose: board_pose,
                board_shape: BoardShape {
                    board_width,
                    hole_radius,
                    hole_center_shift,
                },
                marker_paper_size,
            },
            plane_ransac_data,
            icp_data,
            icp_losses,
        };

        Ok(Some(detection))
    }
}
