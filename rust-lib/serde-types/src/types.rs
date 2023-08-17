use anyhow::{anyhow, ensure, Error, Result};
use approx::abs_diff_eq;
use noisy_float::prelude::*;
use serde::{de::Error as _, ser::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use std::{
    borrow::Cow, convert::TryFrom, fmt, fmt::Display, num::NonZeroUsize, ops::RangeInclusive,
    str::FromStr,
};

#[cfg(feature = "with-nalgebra")]
use nalgebra as na;

#[cfg(feature = "with-opencv")]
use opencv::{core as core_cv, prelude::*};

pub use ident::*;
mod ident {
    use std::{borrow::Borrow, ops::Deref};

    use super::*;

    /// Identifier that consists of ASCII alphanumeric and '-', '_' characters.
    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    pub struct Ident(String);

    impl Ident {
        /// Create an identifier from a string.
        pub fn try_new<'a, S>(name: S) -> Result<Self>
        where
            S: Into<Cow<'a, str>>,
        {
            let name = name.into();
            let ok = !name.is_empty()
                && name
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || "-_".contains(c));
            ensure!(
                ok,
                "the name must consist of ASCII alphabets, numbers, '-' and '_', but get {}",
                name
            );

            Ok(Self(name.into_owned()))
        }

        pub fn new<'a, S>(name: S) -> Self
        where
            S: Into<Cow<'a, str>>,
        {
            Self::try_new(name).unwrap()
        }

        pub fn as_str(&self) -> &str {
            self.0.as_str()
        }
    }

    impl FromStr for Ident {
        type Err = Error;

        fn from_str(name: &str) -> Result<Self, Self::Err> {
            Ident::try_new(name)
        }
    }

    impl Display for Ident {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            self.0.fmt(f)
        }
    }

    impl AsRef<str> for Ident {
        fn as_ref(&self) -> &str {
            &self.0
        }
    }

    impl Borrow<str> for Ident {
        fn borrow(&self) -> &str {
            &self.0
        }
    }

    impl Deref for Ident {
        type Target = str;

        fn deref(&self) -> &Self::Target {
            &self.0
        }
    }

    impl Serialize for Ident {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            self.to_string().serialize(serializer)
        }
    }

    impl<'a> Deserialize<'a> for Ident {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'a>,
        {
            let name = String::deserialize(deserializer)?;
            Ident::try_new(name).map_err(|err| D::Error::custom(format!("{}", err)))
        }
    }
}

pub use device_path_v1::*;
mod device_path_v1 {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct DevicePathV1 {
        pub host: Ident,
        pub device: Ident,
    }

    impl DevicePathV1 {
        pub fn try_new<'a>(
            host: impl Into<Cow<'a, str>>,
            device: impl Into<Cow<'a, str>>,
        ) -> Result<Self> {
            let host = host.into();
            let device = device.into();

            Ok(Self {
                host: Ident::try_new(host)?,
                device: Ident::try_new(device)?,
            })
        }

        pub fn new<'a>(host: impl Into<Cow<'a, str>>, device: impl Into<Cow<'a, str>>) -> Self {
            Self::try_new(host, device).unwrap()
        }

        pub fn to_v2(&self) -> DevicePathV2 {
            self.clone().into()
        }
    }

    impl Display for DevicePathV1 {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
            write!(f, "{}.{}", self.host, self.device)
        }
    }

    impl FromStr for DevicePathV1 {
        type Err = Error;

        fn from_str(text: &str) -> Result<Self, Self::Err> {
            let (host, device) = (move || {
                let mut tokens = text.split('.');
                let host = tokens.next()?;
                let device = tokens.next()?;

                let no_3rd_token = tokens.next().is_none();
                let is_non_empty = !host.is_empty() && !device.is_empty();
                let is_valid = no_3rd_token && is_non_empty;

                if !is_valid {
                    return None;
                }

                Some((host, device))
            })()
            .ok_or_else(|| {
                anyhow!(
                    "expect device path in this format '<host>.<device>', but get '{}'",
                    text
                )
            })?;

            Self::try_new(host, device)
        }
    }

    impl From<DevicePathV2> for DevicePathV1 {
        fn from(from: DevicePathV2) -> Self {
            let DevicePathV2 { host, device } = from;
            Self { host, device }
        }
    }
}

pub use into_device_path::*;
mod into_device_path {
    use super::*;

    pub trait IntoDevicePath {
        fn into_device_path(self) -> DevicePathV2;
    }

    impl IntoDevicePath for &str {
        fn into_device_path(self) -> DevicePathV2 {
            self.parse()
                .unwrap_or_else(|text| panic!("'{}' is not a valid device", text))
        }
    }

    impl IntoDevicePath for String {
        fn into_device_path(self) -> DevicePathV2 {
            self.as_str().into_device_path()
        }
    }

    impl IntoDevicePath for &String {
        fn into_device_path(self) -> DevicePathV2 {
            self.as_str().into_device_path()
        }
    }

    impl IntoDevicePath for DevicePathV2 {
        fn into_device_path(self) -> DevicePathV2 {
            self
        }
    }

    impl IntoDevicePath for &DevicePathV2 {
        fn into_device_path(self) -> DevicePathV2 {
            self.clone()
        }
    }

    impl IntoDevicePath for DevicePathV1 {
        fn into_device_path(self) -> DevicePathV2 {
            self.into()
        }
    }

    impl IntoDevicePath for &DevicePathV1 {
        fn into_device_path(self) -> DevicePathV2 {
            self.clone().into()
        }
    }
}

pub use device_path_v2::*;
mod device_path_v2 {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    pub struct DevicePathV2 {
        pub(super) host: Ident,
        pub(super) device: Ident,
    }

    impl DevicePathV2 {
        pub fn try_new<'a>(
            host: impl Into<Cow<'a, str>>,
            device: impl Into<Cow<'a, str>>,
        ) -> Result<Self> {
            let host = host.into();
            let device = device.into();

            Ok(Self {
                host: Ident::try_new(host)?,
                device: Ident::try_new(device)?,
            })
        }

        pub fn new<'a>(host: impl Into<Cow<'a, str>>, device: impl Into<Cow<'a, str>>) -> Self {
            Self::try_new(host, device).unwrap()
        }

        /// Get a reference to the device path's host.
        pub fn host(&self) -> &str {
            self.host.as_ref()
        }

        /// Get a reference to the device path's device.
        pub fn device(&self) -> &str {
            self.device.as_ref()
        }

        pub fn to_v1(&self) -> DevicePathV1 {
            self.clone().into()
        }
    }

    impl Display for DevicePathV2 {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
            write!(f, "{}.{}", self.host, self.device)
        }
    }

    impl FromStr for DevicePathV2 {
        type Err = Error;

        fn from_str(text: &str) -> Result<Self, Self::Err> {
            let (host, device) = (move || {
                let mut tokens = text.split('.');
                let host = tokens.next()?;
                let device = tokens.next()?;

                let no_3rd_token = tokens.next().is_none();
                let is_non_empty = !host.is_empty() && !device.is_empty();
                let is_valid = no_3rd_token && is_non_empty;

                if !is_valid {
                    return None;
                }

                Some((host, device))
            })()
            .ok_or_else(|| {
                anyhow!(
                    "expect device path in this format '<host>.<device>', but get '{}'",
                    text
                )
            })?;

            Self::try_new(host, device)
        }
    }

    impl Serialize for DevicePathV2 {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            self.to_string().serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for DevicePathV2 {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            let path = Self::from_str(&text).map_err(|err| D::Error::custom(format!("{}", err)))?;
            Ok(path)
        }
    }

    impl From<DevicePathV1> for DevicePathV2 {
        fn from(from: DevicePathV1) -> Self {
            let DevicePathV1 { host, device } = from;
            Self { host, device }
        }
    }
}

pub type DevicePath = DevicePathV2;

pub use device_tuple::*;
mod device_tuple {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    pub struct DeviceTuple {
        pub src: DevicePath,
        pub tgt: DevicePath,
    }

    impl Display for DeviceTuple {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "{} -> {}", self.src, self.tgt)
        }
    }

    impl FromStr for DeviceTuple {
        type Err = Error;

        fn from_str(text: &str) -> Result<Self, Self::Err> {
            let (first, second) = (move || {
                let mut tokens = text.split(',');
                let first = tokens.next()?;
                let second = tokens.next()?;
                tokens.next().is_none().then_some((first, second))
            })()
            .ok_or_else(|| anyhow!("invalid device tuple '{}'", text))?;

            Ok(Self {
                src: DevicePath::from_str(first)?,
                tgt: DevicePath::from_str(second)?,
            })
        }
    }

    impl Serialize for DeviceTuple {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            self.to_string().serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for DeviceTuple {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            Self::from_str(&text).map_err(|err| D::Error::custom(format!("{}", err)))
        }
    }
}

pub use into_device_tuple::*;
mod into_device_tuple {
    use super::*;

    pub trait IntoDeviceTuple {
        fn into_device_tuple(self) -> DeviceTuple;
    }

    impl IntoDeviceTuple for DeviceTuple {
        fn into_device_tuple(self) -> DeviceTuple {
            self
        }
    }

    impl IntoDeviceTuple for &DeviceTuple {
        fn into_device_tuple(self) -> DeviceTuple {
            self.to_owned()
        }
    }

    impl<P1, P2> IntoDeviceTuple for (P1, P2)
    where
        P1: IntoDevicePath,
        P2: IntoDevicePath,
    {
        fn into_device_tuple(self) -> DeviceTuple {
            let (src, tgt) = self;
            DeviceTuple {
                src: src.into_device_path(),
                tgt: tgt.into_device_path(),
            }
        }
    }
}

pub use size_2d::*;
mod size_2d {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct Size2D {
        pub h: NonZeroUsize,
        pub w: NonZeroUsize,
    }

    impl Size2D {
        pub fn h(&self) -> usize {
            self.h.get()
        }

        pub fn w(&self) -> usize {
            self.w.get()
        }
    }

    #[cfg(feature = "with-opencv")]
    pub use with_opencv::*;

    #[cfg(feature = "with-opencv")]
    mod with_opencv {
        use super::*;

        impl From<&Size2D> for core_cv::Size {
            fn from(from: &Size2D) -> Self {
                core_cv::Size {
                    width: from.w() as i32,
                    height: from.h() as i32,
                }
            }
        }

        impl From<Size2D> for core_cv::Size {
            fn from(from: Size2D) -> Self {
                (&from).into()
            }
        }
    }
}

pub use camera_intrinsics::*;
mod camera_intrinsics {
    use super::*;

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
}

pub use isometry3d::*;
mod isometry3d {
    use super::*;

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
    pub use with_opencv_nalgebra::*;

    #[cfg(all(feature = "with-opencv", feature = "with-nalgebra"))]
    mod with_opencv_nalgebra {
        use super::*;

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
    pub use with_nalgebra::*;

    #[cfg(feature = "with-nalgebra")]
    mod with_nalgebra {
        use super::*;

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
}

pub use distortion_coefs::*;
mod distortion_coefs {
    use super::*;

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
    pub use with_nalgebra::*;

    #[cfg(feature = "with-nalgebra")]
    mod with_nalgebra {
        use super::*;

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
    impl From<&DistortionCoefs> for core_cv::Mat {
        fn from(from: &DistortionCoefs) -> Self {
            core_cv::Mat::from_exact_iter(from.0.iter().map(|val| val.raw())).unwrap()
        }
    }

    #[cfg(feature = "with-opencv")]
    impl From<DistortionCoefs> for core_cv::Mat {
        fn from(from: DistortionCoefs) -> Self {
            (&from).into()
        }
    }
}

pub use camera_matrix::*;
mod camera_matrix {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(try_from = "CameraMatrixUnchecked", into = "CameraMatrixUnchecked")]
    pub struct CameraMatrix(pub [[R64; 3]; 3]);

    impl Default for CameraMatrix {
        fn default() -> Self {
            Self::identity()
        }
    }

    impl CameraMatrix {
        pub fn identity() -> Self {
            CameraMatrix([
                [r64(1.0), r64(0.0), r64(0.0)],
                [r64(0.0), r64(1.0), r64(0.0)],
                [r64(0.0), r64(0.0), r64(1.0)],
            ])
        }

        pub fn fx(&self) -> R64 {
            self.0[0][0]
        }

        pub fn fy(&self) -> R64 {
            self.0[1][1]
        }

        pub fn cx(&self) -> R64 {
            self.0[0][2]
        }

        pub fn cy(&self) -> R64 {
            self.0[1][2]
        }
    }

    #[cfg(feature = "with-nalgebra")]
    impl From<&CameraMatrix> for na::Matrix3<f64> {
        fn from(from: &CameraMatrix) -> Self {
            use slice_of_array::prelude::*;
            let values: Vec<_> = from.0.flat().iter().map(|val| val.raw()).collect();
            Self::from_row_slice(&values)
        }
    }

    #[cfg(feature = "with-opencv")]
    impl From<&CameraMatrix> for core_cv::Mat {
        fn from(from: &CameraMatrix) -> Self {
            use slice_of_array::prelude::*;
            Self::from_exact_iter(from.0.flat().iter().map(|val| val.raw()))
                .unwrap()
                .reshape(1, 3)
                .unwrap()
        }
    }

    #[cfg(feature = "with-opencv")]
    impl From<CameraMatrix> for core_cv::Mat {
        fn from(from: CameraMatrix) -> Self {
            (&from).into()
        }
    }

    impl From<CameraMatrix> for CameraMatrixUnchecked {
        fn from(from: CameraMatrix) -> Self {
            Self(from.0)
        }
    }

    impl TryFrom<CameraMatrixUnchecked> for CameraMatrix {
        type Error = Error;

        fn try_from(from: CameraMatrixUnchecked) -> Result<Self, Self::Error> {
            let mat = from.0;
            ensure!(
                mat[0][1] == 0.0
                    && mat[1][0] == 0.0
                    && mat[2][0] == 0.0
                    && mat[2][1] == 0.0
                    && abs_diff_eq!(mat[2][2].raw(), 1.0)
            );
            Ok(Self(mat))
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(transparent)]
    pub struct CameraMatrixUnchecked(pub [[R64; 3]; 3]);
}

pub use twd97::*;
mod twd97 {
    use super::*;

    const LONGITUDE_RANGE: RangeInclusive<f64> = -461216.18..=1932367.55;
    const LATITUDE_RANGE: RangeInclusive<f64> = 509174.11..=2985577.33;

    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    pub struct Twd97 {
        pub lon: R64,
        pub lat: R64,
    }

    impl Twd97 {
        fn check(&self) -> Result<(), String> {
            let Self { lon, lat } = *self;
            let is_lon_ok = LONGITUDE_RANGE.contains(&lon.raw());
            let is_lat_ok = LATITUDE_RANGE.contains(&lat.raw());

            let reason = match (is_lon_ok, is_lat_ok) {
                (true, true) => return Ok(()),
                (false, true) => {
                    format!(
                        "invalid longitude {}, it must be in range of {:?}",
                        lon, LONGITUDE_RANGE
                    )
                }
                (true, false) => {
                    format!(
                        "invalid latitude {}, it must be in range of {:?}",
                        lat, LATITUDE_RANGE
                    )
                }
                (false, false) => {
                    format!(
                        "invalid coordinate lon={} lat={}, their respective range are {:?} and {:?}",
                        lon, lat, LONGITUDE_RANGE, LATITUDE_RANGE
                    )
                }
            };

            Err(reason)
        }
    }

    impl Serialize for Twd97 {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            let Self { lon, lat } = *self;

            self.check().map_err(S::Error::custom)?;

            Twd97Unchecked { lon, lat }.serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for Twd97 {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let Twd97Unchecked { lon, lat } = Twd97Unchecked::deserialize(deserializer)?;

            let lonlat = Twd97 { lon, lat };

            lonlat.check().map_err(D::Error::custom)?;
            Ok(lonlat)
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    struct Twd97Unchecked {
        pub lon: R64,
        pub lat: R64,
    }
}

pub use fraction::*;
mod fraction {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct Fraction {
        pub num: usize,
        pub deno: NonZeroUsize,
    }

    impl Fraction {
        pub fn reduce(&self) -> Self {
            let gcd = gcd::binary_usize(self.num, self.deno.get());
            Self {
                num: self.num / gcd,
                deno: NonZeroUsize::new(self.deno.get() / gcd).unwrap(),
            }
        }

        pub fn to_f64(&self) -> f64 {
            self.num as f64 / self.deno.get() as f64
        }

        pub fn recip(&self) -> Option<Self> {
            Some(Self {
                num: self.deno.get(),
                deno: NonZeroUsize::new(self.num)?,
            })
        }
    }

    impl FromStr for Fraction {
        type Err = anyhow::Error;

        fn from_str(text: &str) -> Result<Self, Self::Err> {
            let mut tokens = text.split('/');

            let err = || {
                anyhow!(
                    "Invalid fraction string '{}'.
It must be in 'num/deno' format.",
                    text
                )
            };

            let num = tokens.next().ok_or_else(err)?.parse().map_err(|_| err())?;
            let deno = tokens.next().ok_or_else(err)?.parse().map_err(|_| err())?;

            if tokens.next().is_some() {
                return Err(err());
            }

            Ok(Self { num, deno })
        }
    }

    impl Display for Fraction {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "{}/{}", self.num, self.deno)
        }
    }

    impl Serialize for Fraction {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            format!("{}", self).serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for Fraction {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            text.parse().map_err(D::Error::custom)
        }
    }
}

pub use non_zero_fraction::*;
mod non_zero_fraction {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct NonZeroFraction {
        pub num: NonZeroUsize,
        pub deno: NonZeroUsize,
    }

    impl NonZeroFraction {
        pub fn reduce(&self) -> Self {
            let gcd = gcd::binary_usize(self.num.get(), self.deno.get());
            Self {
                num: NonZeroUsize::new(self.num.get() / gcd).unwrap(),
                deno: NonZeroUsize::new(self.deno.get() / gcd).unwrap(),
            }
        }

        pub fn to_f64(&self) -> f64 {
            self.num.get() as f64 / self.deno.get() as f64
        }

        pub fn recip(&self) -> Self {
            Self {
                num: self.deno,
                deno: self.num,
            }
        }
    }

    impl FromStr for NonZeroFraction {
        type Err = anyhow::Error;

        fn from_str(text: &str) -> Result<Self, Self::Err> {
            let mut tokens = text.split('/');

            let err = || {
                anyhow!(
                    "Invalid fraction string '{}'.
It must be in 'num/deno' format.",
                    text
                )
            };

            let num = tokens.next().ok_or_else(err)?.parse().map_err(|_| err())?;
            let deno = tokens.next().ok_or_else(err)?.parse().map_err(|_| err())?;

            if tokens.next().is_some() {
                return Err(err());
            }

            Ok(Self { num, deno })
        }
    }

    impl Display for NonZeroFraction {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "{}/{}", self.num, self.deno)
        }
    }

    impl Serialize for NonZeroFraction {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            format!("{}", self).serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for NonZeroFraction {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            text.parse().map_err(D::Error::custom)
        }
    }
}

pub use mrpt_calibration::MrptCalibration;
pub mod mrpt_calibration {
    use crate::{CameraIntrinsics, CameraMatrix, DistortionCoefs};
    use anyhow::{ensure, Result};
    #[cfg(all(feature = "with-opencv", feature = "with-nalgebra"))]
    use cv_convert::{OpenCvPose, TryIntoCv};
    #[cfg(feature = "with-nalgebra")]
    use nalgebra as na;
    use noisy_float::prelude::*;
    #[cfg(feature = "with-opencv")]
    use opencv::prelude::*;
    use serde::{de::Error as _, Deserialize, Deserializer};
    use std::mem;

    /// The type defines the calibration parameter file generated by MRPT
    /// camera-calib.
    #[derive(Debug, Clone, Deserialize)]
    pub struct MrptCalibration {
        pub camera_name: String,
        pub focal_length_meters: R64,
        pub image_height: usize,
        pub image_width: usize,
        pub distortion_model: DistortionModel,
        pub distortion_coefficients: Matrix,
        pub camera_matrix: Matrix,
        pub projection_matrix: Matrix,
        pub rectification_matrix: Matrix,
    }

    impl MrptCalibration {
        pub fn intrinsic_params(&self) -> Result<CameraIntrinsics> {
            use slice_of_array::prelude::*;

            let Self {
                camera_matrix,
                distortion_coefficients,
                ..
            } = self;

            ensure!(camera_matrix.rows == 3 && camera_matrix.cols == 3);
            ensure!(distortion_coefficients.rows == 1 && distortion_coefficients.cols >= 5);
            {
                let is_all_zero = distortion_coefficients
                    .data
                    .get(5..)
                    .into_iter()
                    .flatten()
                    .all(|&val| val == 0.0);
                ensure!(is_all_zero);
            }

            let camera_matrix = CameraMatrix({
                let array: &[[R64; 3]] = camera_matrix.data().nest();
                array.try_into().unwrap()
            });
            let distortion_coefs =
                DistortionCoefs(distortion_coefficients.data()[0..5].try_into().unwrap());

            Ok(CameraIntrinsics {
                camera_matrix,
                distortion_coefs,
            })
        }
    }

    #[derive(Debug, Clone)]
    pub struct Matrix {
        rows: usize,
        cols: usize,
        data: Vec<R64>,
    }

    impl Matrix {
        /// Converts the matrix to a OpenCV Mat.
        #[cfg(feature = "with-opencv")]
        pub fn to_opencv(&self) -> Mat {
            let mat = Mat::from_slice(self.data_f64()).unwrap();
            mat.reshape(1, self.rows as i32).unwrap()
        }

        #[cfg(feature = "with-nalgebra")]
        pub fn to_na(&self) -> na::DMatrix<f64> {
            na::DMatrix::from_row_slice(self.rows, self.cols, self.data_f64())
        }

        /// Get the matrix's rows.
        pub fn rows(&self) -> usize {
            self.rows
        }

        /// Get the matrix's cols.
        pub fn cols(&self) -> usize {
            self.cols
        }

        /// Get a R64 slice to the matrix's data.
        pub fn data(&self) -> &[R64] {
            self.data.as_ref()
        }

        /// Get a f64 slice to the matrix's data.
        pub fn data_f64(&self) -> &[f64] {
            unsafe { mem::transmute(self.data()) }
        }
    }

    impl<'de> Deserialize<'de> for Matrix {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let UncheckedMatrix { rows, cols, data } = UncheckedMatrix::deserialize(deserializer)?;
            if rows * cols != data.len() {
                return Err(D::Error::custom(format!(
                    "data size ({}) does not match rows ({}) and cols ({})",
                    data.len(),
                    rows,
                    cols
                )));
            }
            Ok(Self { rows, cols, data })
        }
    }

    #[derive(Debug, Clone, Deserialize)]
    pub struct UncheckedMatrix {
        rows: usize,
        cols: usize,
        data: Vec<R64>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
    #[serde(rename_all = "snake_case")]
    pub enum DistortionModel {
        PlumbBob,
    }

    #[derive(Debug, Clone, Deserialize)]
    #[serde(tag = "type", rename_all = "snake_case")]
    pub enum ExtrinsicsData {
        Quaternion(ExtrinsicsTransform),
        Matrix(ExtrinsicsMatrix),
    }

    impl ExtrinsicsData {
        #[cfg(feature = "with-nalgebra")]
        pub fn to_na(&self) -> na::Isometry3<f64> {
            match self {
                Self::Quaternion(me) => me.to_na(),
                Self::Matrix(me) => me.to_na(),
            }
        }

        #[cfg(all(feature = "with-opencv", feature = "with-nalgebra"))]
        pub fn to_opencv(&self) -> Result<OpenCvPose<Mat>> {
            let pose = self.to_na().try_into_cv()?;
            Ok(pose)
        }
    }

    #[derive(Debug, Clone, Deserialize)]
    pub struct ExtrinsicsTransform {
        pub rot_wijk: [R64; 4],
        pub trans_xyz: [R64; 3],
    }

    impl ExtrinsicsTransform {
        #[cfg(feature = "with-nalgebra")]
        pub fn to_na(&self) -> na::Isometry3<f64> {
            let Self {
                rot_wijk,
                trans_xyz,
            } = *self;
            let [w, i, j, k]: [f64; 4] = unsafe { mem::transmute(rot_wijk) };
            let [x, y, z]: [f64; 3] = unsafe { mem::transmute(trans_xyz) };

            let rotation = na::UnitQuaternion::from_quaternion(na::Quaternion::new(w, i, j, k));
            let translation = na::Translation3::new(x, y, z);

            na::Isometry3 {
                rotation,
                translation,
            }
        }
    }

    #[derive(Debug, Clone, Deserialize)]
    pub struct ExtrinsicsMatrix {
        pub rot: [[R64; 3]; 3],
        pub trans: [R64; 3],
    }

    impl ExtrinsicsMatrix {
        #[cfg(feature = "with-nalgebra")]
        pub fn to_na(&self) -> na::Isometry3<f64> {
            use slice_of_array::prelude::*;

            let rotation = {
                let slice: &[R64] = self.rot.flat();
                let slice: &[f64] = unsafe { mem::transmute(slice) };
                let mat = na::Matrix3::from_row_slice(slice);
                na::UnitQuaternion::from_matrix(&mat)
            };
            let translation = {
                let [x, y, z]: [f64; 3] = unsafe { mem::transmute(self.trans) };
                na::Translation3::new(x, y, z)
            };
            na::Isometry3 {
                rotation,
                translation,
            }
        }
    }
}
