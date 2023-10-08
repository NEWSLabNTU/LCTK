use common_types::serde_types::{DevicePathV2, Isometry3D};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CoordinateTransform {
    pub src: DevicePathV2,
    pub tgt: DevicePathV2,
    pub transform: Isometry3D,
}
