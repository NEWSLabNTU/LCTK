use noisy_float::prelude::*;
use serde::{Deserialize, Serialize};

/// The camera distortion coefficients in `[k1, k2, p1, p2, k3]` format.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct DistortionCoefs(pub [R64; 5]);

impl DistortionCoefs {
    pub fn zeros() -> Self {
        DistortionCoefs([r64(0.0), r64(0.0), r64(0.0), r64(0.0), r64(0.0)])
    }

    pub fn k1(&self) -> R64 {
        self.0[0]
    }

    pub fn k2(&self) -> R64 {
        self.0[1]
    }

    pub fn p1(&self) -> R64 {
        self.0[2]
    }

    pub fn p2(&self) -> R64 {
        self.0[3]
    }

    pub fn k3(&self) -> R64 {
        self.0[4]
    }
}

impl Default for DistortionCoefs {
    fn default() -> Self {
        Self::zeros()
    }
}

#[cfg(feature = "with-nalgebra")]
mod with_nalgebra {
    use crate::DistortionCoefs;
    use nalgebra as na;

    // impl From<&DistortionCoefs> for opencv_ros_camera::Distortion<f64> {
    //     fn from(from: &DistortionCoefs) -> Self {
    //         let coefs: [f64; 5] = unsafe { mem::transmute(from.0) };
    //         let slice: &[f64] = coefs.as_ref();
    //         opencv_ros_camera::Distortion::from_opencv_vec(na::Vector5::from_column_slice(
    //             slice,
    //         ))
    //     }
    // }

    // impl From<DistortionCoefs> for opencv_ros_camera::Distortion<f64> {
    //     fn from(from: DistortionCoefs) -> Self {
    //         (&from).into()
    //     }
    // }

    impl From<&DistortionCoefs> for na::Vector5<f64> {
        fn from(from: &DistortionCoefs) -> Self {
            na::Vector5::from_iterator(from.0.iter().map(|val| val.raw()))
        }
    }
}

#[cfg(feature = "with-opencv")]
mod with_opencv {
    use crate::DistortionCoefs;
    use opencv::core as core_cv;

    impl From<&DistortionCoefs> for core_cv::Mat {
        fn from(from: &DistortionCoefs) -> Self {
            core_cv::Mat::from_exact_iter(from.0.iter().map(|val| val.raw())).unwrap()
        }
    }

    impl From<DistortionCoefs> for core_cv::Mat {
        fn from(from: DistortionCoefs) -> Self {
            (&from).into()
        }
    }
}
