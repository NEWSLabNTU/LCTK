// use crate::common::*;

// pub mod serde_device_path_v1 {
//     use super::*;
//     use common_types::{DevicePath, DevicePathV1};

//     pub fn serialize<S>(device: &DevicePath, serializer: S) -> Result<S::Ok, S::Error>
//     where
//         S: Serializer,
//     {
//         device.to_v1().serialize(serializer)
//     }

//     pub fn deserialize<'de, D>(deserializer: D) -> Result<DevicePath, D::Error>
//     where
//         D: Deserializer<'de>,
//     {
//         Ok(DevicePathV1::deserialize(deserializer)?.into())
//     }
// }
