use cv_convert::{prelude::*, OpenCvPose};
use log::warn;
use nalgebra as na;
use opencv::{
    calib3d,
    core::{Mat, Point2d, Point3d, Vector, CV_64FC1},
    prelude::*,
};
use serde::{Deserialize, Serialize};
use serde_types::CameraIntrinsics;

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Hash,
    strum::EnumString,
    strum::Display,
    Serialize,
    Deserialize,
)]
pub enum PnpMethod {
    ITERATIVE,
    EPNP,
    IPPE,
    SQPNP,
}

impl PnpMethod {
    pub fn opencv_flag(&self) -> i32 {
        use calib3d::*;

        match self {
            Self::ITERATIVE => SOLVEPNP_ITERATIVE,
            Self::EPNP => SOLVEPNP_EPNP,
            Self::IPPE => SOLVEPNP_IPPE,
            Self::SQPNP => SOLVEPNP_SQPNP,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PnpSolver {
    camera_matrix: Mat,
    distortion_coefs: Mat,
    method: i32, // OpenCV flag
}

// HACK: workaround that Mat-typed fields are not Sync
unsafe impl Sync for PnpSolver {}

impl PnpSolver {
    pub fn new(intrinsics: &CameraIntrinsics, method: PnpMethod) -> Self {
        let camera_matrix = Mat::from(&intrinsics.camera_matrix);
        let distortion_coefs = Mat::from(&intrinsics.distortion_coefs);

        if method == PnpMethod::IPPE {
            warn!("By using IPPE method in PnP solver, object points must be coplanar.");
        }

        Self {
            camera_matrix,
            distortion_coefs,
            method: method.opencv_flag(),
        }
    }

    pub fn solve<I>(&self, pairs: I) -> Option<na::Isometry3<f64>>
    where
        I: IntoIterator<Item = (Point3d, Point2d)>,
    {
        let (object_points, image_points): (Vector<Point3d>, Vector<Point2d>) =
            pairs.into_iter().unzip();

        // check empty input because calib3d::solve_pnp() panics on empty input
        if object_points.is_empty() {
            return None;
        }

        let mut rvec = Mat::zeros(3, 1, CV_64FC1).unwrap().to_mat().unwrap();
        let mut tvec = Mat::zeros(3, 1, CV_64FC1).unwrap().to_mat().unwrap();
        let solved = calib3d::solve_pnp(
            &object_points,
            &image_points,
            &self.camera_matrix,
            &self.distortion_coefs,
            &mut rvec,
            &mut tvec,
            false,
            self.method,
        )
        .unwrap();

        if !solved {
            return None;
        }

        let transform: na::Isometry3<f64> = OpenCvPose { rvec, tvec }.try_to_cv().unwrap();

        Some(transform)
    }
}
