use crate::infra_v2::{
    CameraIntrinsics, CameraProfile, CoordinateTransform, DevicePose, LidarProfile,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Param {
    CoordinateTransform(CoordinateTransform),
    CameraIntrinsics(CameraIntrinsics),
    DevicePose(DevicePose),
    CameraProfile(CameraProfile),
    LidarProfile(Box<LidarProfile>),
}

impl From<CoordinateTransform> for Param {
    fn from(from: CoordinateTransform) -> Self {
        Param::CoordinateTransform(from)
    }
}

impl From<CameraIntrinsics> for Param {
    fn from(from: CameraIntrinsics) -> Self {
        Param::CameraIntrinsics(from)
    }
}

impl From<DevicePose> for Param {
    fn from(from: DevicePose) -> Self {
        Param::DevicePose(from)
    }
}

impl From<CameraProfile> for Param {
    fn from(from: CameraProfile) -> Self {
        Param::CameraProfile(from)
    }
}

impl From<LidarProfile> for Param {
    fn from(from: LidarProfile) -> Self {
        Param::LidarProfile(Box::new(from))
    }
}
