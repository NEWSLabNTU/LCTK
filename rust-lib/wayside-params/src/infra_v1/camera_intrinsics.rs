use crate::common::*;
use common_types::{CameraIntrinsics, DevicePathV1};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraIntrinsicsConfig {
    #[serde(flatten)]
    pub device: DevicePathV1,
    #[serde(flatten)]
    pub intrinsic: CameraIntrinsics,
}
