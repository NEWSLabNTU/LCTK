use super::{
    super::{
        CameraIntrinsics, CameraProfile, CoordinateTransform, DevicePose, LidarProfile, Param,
    },
    ParamList,
};
#[cfg(feature = "with-nalgebra")]
use crate::utils::CoordTransformMap;
use anyhow::{Context, Result};
use common_types::{DevicePathV2, DeviceTuple};
use derivative::Derivative;
use indexmap::IndexMap;
use serde_loader::Json5Path;
use std::path::Path;

#[derive(Debug, Clone, Eq, Derivative)]
#[derivative(Hash, PartialEq)]
pub struct InfraV2 {
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub coordinate_transform: IndexMap<DeviceTuple, CoordinateTransform>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub camera_intrinsics: IndexMap<DevicePathV2, CameraIntrinsics>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub device_pose: IndexMap<DevicePathV2, DevicePose>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub camera_profile: IndexMap<DevicePathV2, CameraProfile>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub lidar_profile: IndexMap<DevicePathV2, LidarProfile>,
}

impl InfraV2 {
    pub fn open<P: AsRef<Path>>(file: P) -> Result<Self> {
        let file = file.as_ref();
        let main_file: ParamList = Json5Path::open_and_take(file)
            .with_context(|| format!("unable to open file '{}'", file.display()))?;

        let output = main_file.into_params().fold(
            Self {
                coordinate_transform: IndexMap::new(),
                camera_intrinsics: IndexMap::new(),
                device_pose: IndexMap::new(),
                camera_profile: IndexMap::new(),
                lidar_profile: IndexMap::new(),
            },
            |mut infra, param| {
                insert_param(&mut infra, param);
                infra
            },
        );

        Ok(output)
    }

    #[cfg(feature = "with-nalgebra")]
    pub fn to_coord_transform_map(&self) -> Result<CoordTransformMap> {
        CoordTransformMap::new(self.coordinate_transform.iter().map(|(_, param)| {
            let param = param.clone();
            (
                param.src,
                param.tgt,
                nalgebra::Isometry3::from(param.transform),
            )
        }))
    }
}

fn insert_param(infra: &mut InfraV2, param: Param) {
    use Param as P;

    match param {
        P::CoordinateTransform(param) => {
            let (src, tgt) = (param.src.clone(), param.tgt.clone());
            infra
                .coordinate_transform
                .insert(DeviceTuple { src, tgt }, param);
        }
        P::CameraIntrinsics(param) => {
            let key = param.device.clone();
            infra.camera_intrinsics.insert(key, param);
        }
        P::DevicePose(param) => {
            let key = param.device.clone();
            infra.device_pose.insert(key, param);
        }
        P::CameraProfile(param) => {
            let key = param.device.clone();
            infra.camera_profile.insert(key, param);
        }
        P::LidarProfile(param) => {
            let key = param.device.clone();
            infra.lidar_profile.insert(key, *param);
        }
    }
}
