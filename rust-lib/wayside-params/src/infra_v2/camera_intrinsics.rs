use common_types::{CameraIntrinsics as Intrinsics, DevicePathV2};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub device: DevicePathV2,
    pub params: Intrinsics,
}
