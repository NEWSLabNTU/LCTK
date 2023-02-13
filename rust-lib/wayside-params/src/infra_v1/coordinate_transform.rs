use crate::common::*;
use common_types::serde_types::{DevicePathV1, Isometry3D};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CoordinateTransform {
    pub from: DevicePathV1,
    pub to: DevicePathV1,
    #[serde(flatten)]
    pub transform: Isometry3D,
}
