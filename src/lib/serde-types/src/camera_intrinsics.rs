use crate::{CameraMatrix, DistortionCoefs};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub camera_matrix: CameraMatrix,
    pub distortion_coefs: DistortionCoefs,
}

impl CameraIntrinsics {
    pub fn identity() -> Self {
        Self {
            camera_matrix: CameraMatrix::identity(),
            distortion_coefs: DistortionCoefs::zeros(),
        }
    }
}

impl Default for CameraIntrinsics {
    fn default() -> Self {
        Self::identity()
    }
}

// #[cfg(feature = "with-nalgebra")]
// impl From<&CameraIntrinsic> for opencv_ros_camera::RosOpenCvIntrinsics<f64> {
//     fn from(from: &CameraIntrinsic) -> Self {
//         let CameraIntrinsic {
//             camera_matrix,
//             distortion_coefs,
//         } = from;

//         opencv_ros_camera::RosOpenCvIntrinsics::from_params_with_distortion(
//             camera_matrix.fx().raw(),
//             0.0, // skew
//             camera_matrix.fy().raw(),
//             camera_matrix.cx().raw(),
//             camera_matrix.cy().raw(),
//             distortion_coefs.into(),
//         )
//     }
// }

// #[cfg(feature = "with-nalgebra")]
// impl From<CameraIntrinsic> for opencv_ros_camera::RosOpenCvIntrinsics<f64> {
//     fn from(from: CameraIntrinsic) -> Self {
//         (&from).into()
//     }
// }
