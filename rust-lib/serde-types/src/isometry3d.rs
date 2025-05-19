use noisy_float::prelude::*;
use serde::{Deserialize, Serialize};

/// The 3D rotation the encodes into `[i, j, k, w]` format.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct UnitQuaternion(pub [R64; 4]);

/// The 3D translation that encodes into `[x, y, z]` format.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Translation3D(pub [R64; 3]);

/// The 3D isometry transformation, a combination of translation and rotation.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Isometry3D {
    pub rotation: UnitQuaternion,
    pub translation: Translation3D,
}

#[cfg(all(feature = "with-opencv", feature = "with-nalgebra"))]
mod with_opencv_nalgebra {
    use crate::Isometry3D;
    use nalgebra as na;
    use opencv::core as core_cv;

    impl From<&Isometry3D> for cv_convert::OpenCvPose<core_cv::Mat> {
        fn from(from: &Isometry3D) -> Self {
            use cv_convert::TryIntoCv;
            na::Isometry3::<f64>::from(from).try_into_cv().unwrap()
        }
    }

    impl From<Isometry3D> for cv_convert::OpenCvPose<core_cv::Mat> {
        fn from(from: Isometry3D) -> Self {
            (&from).into()
        }
    }
}

#[cfg(feature = "with-nalgebra")]
mod with_nalgebra {
    use crate::{Isometry3D, Translation3D, UnitQuaternion};
    use nalgebra as na;
    use noisy_float::prelude::*;

    impl From<&UnitQuaternion> for na::UnitQuaternion<f64> {
        fn from(from: &UnitQuaternion) -> Self {
            let [i, j, k, w] = from.0;
            na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                w.raw(),
                i.raw(),
                j.raw(),
                k.raw(),
            ))
        }
    }

    impl From<UnitQuaternion> for na::UnitQuaternion<f64> {
        fn from(from: UnitQuaternion) -> Self {
            (&from).into()
        }
    }

    impl From<&na::UnitQuaternion<f64>> for UnitQuaternion {
        fn from(from: &na::UnitQuaternion<f64>) -> Self {
            Self([r64(from.i), r64(from.j), r64(from.k), r64(from.w)])
        }
    }

    impl From<na::UnitQuaternion<f64>> for UnitQuaternion {
        fn from(from: na::UnitQuaternion<f64>) -> Self {
            (&from).into()
        }
    }

    impl From<&Translation3D> for na::Translation3<f64> {
        fn from(from: &Translation3D) -> Self {
            let [x, y, z] = from.0;
            na::Translation3::new(x.raw(), y.raw(), z.raw())
        }
    }

    impl From<Translation3D> for na::Translation3<f64> {
        fn from(from: Translation3D) -> Self {
            (&from).into()
        }
    }

    impl From<&na::Translation3<f64>> for Translation3D {
        fn from(from: &na::Translation3<f64>) -> Self {
            Self([r64(from.x), r64(from.y), r64(from.z)])
        }
    }

    impl From<na::Translation3<f64>> for Translation3D {
        fn from(from: na::Translation3<f64>) -> Self {
            (&from).into()
        }
    }

    impl From<&Isometry3D> for na::Isometry3<f64> {
        fn from(from: &Isometry3D) -> Self {
            let Isometry3D {
                rotation,
                translation,
            } = from;

            na::Isometry3 {
                rotation: rotation.into(),
                translation: translation.into(),
            }
        }
    }

    impl From<Isometry3D> for na::Isometry3<f64> {
        fn from(from: Isometry3D) -> Self {
            (&from).into()
        }
    }

    impl From<&na::Isometry3<f64>> for Isometry3D {
        fn from(from: &na::Isometry3<f64>) -> Self {
            let na::Isometry3 {
                rotation,
                translation,
            } = from;

            Self {
                rotation: rotation.into(),
                translation: translation.into(),
            }
        }
    }

    impl From<na::Isometry3<f64>> for Isometry3D {
        fn from(from: na::Isometry3<f64>) -> Self {
            (&from).into()
        }
    }
}
