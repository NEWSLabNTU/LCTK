use crate::common::*;
use common_types::{CameraIntrinsics as Intrinsics, DevicePathV2};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub device: DevicePathV2,
    pub params: Intrinsics,
}
