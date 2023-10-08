use common_types::DevicePathV2;
use serde::{Deserialize, Serialize};

pub type CameraKind = crate::infra_v1::CameraProfile;

/// The camera device configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraProfile {
    pub device: DevicePathV2,
    pub profile: CameraKind,
}
