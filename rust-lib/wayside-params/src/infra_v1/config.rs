use super::{CameraConfig, CameraIntrinsicsConfig, CoordinateTransform, DevicePose, LidarConfig};
#[cfg(feature = "with-nalgebra")]
use crate::utils::CoordTransformMap;
use anyhow::{Context, Result};
use common_types::DevicePathV2;
use derivative::Derivative;
use indexmap::IndexMap;
use itertools::Itertools;
use log::warn;
use serde::{Deserialize, Serialize};
use serde_loader::Json5Path;
use std::path::Path;

#[derive(Debug, Clone, Eq, Derivative)]
#[derivative(Hash, PartialEq)]
pub struct InfraV1 {
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub coordinate_transform: IndexMap<(DevicePathV2, DevicePathV2), CoordinateTransform>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub camera_intrinsic: IndexMap<DevicePathV2, CameraIntrinsicsConfig>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub device_pose: IndexMap<DevicePathV2, DevicePose>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub camera_config: IndexMap<DevicePathV2, CameraConfig>,
    #[derivative(Hash(hash_with = "common_types::hash_index_map"))]
    pub lidar_config: IndexMap<DevicePathV2, LidarConfig>,
}

impl InfraV1 {
    pub fn load(dir: impl AsRef<Path>) -> Result<Self> {
        let dir = dir.as_ref();

        let params: Vec<_> = dir
            .read_dir()
            .with_context(|| format!("cannot open directory '{}'", dir.display()))?
            .map(|entry| -> Result<_> {
                let entry = entry?;

                let file_type = entry.file_type()?;
                if !(file_type.is_file() || file_type.is_symlink()) {
                    return Ok(None);
                }

                let path = entry.path();
                let is_json_file = path
                    .extension()
                    .and_then(|ext| {
                        let is_json = ext.to_str()? == "json";
                        Some(is_json)
                    })
                    .unwrap_or(false);

                if !is_json_file {
                    warn!("skip '{}' because it's not a .json file", path.display());
                    return Ok(None);
                }

                let params: ParameterConfig = Json5Path::open_and_take(&path)
                    .with_context(|| format!("unable to open file '{}'", path.display()))?;
                Ok(Some(params))
            })
            .filter_map(|params| params.transpose())
            .try_collect()?;

        let init = Self {
            coordinate_transform: IndexMap::new(),
            camera_intrinsic: IndexMap::new(),
            device_pose: IndexMap::new(),
            camera_config: IndexMap::new(),
            lidar_config: IndexMap::new(),
        };

        let output = params.into_iter().fold(init, |mut infra, param| {
            match param {
                ParameterConfig::CoordinateTransform(param) => {
                    let (p1, p2) = (param.from.clone(), param.to.clone());
                    infra
                        .coordinate_transform
                        .insert((p1.into(), p2.into()), param);
                }
                ParameterConfig::CameraIntrinsic(param) => {
                    let key = param.device.clone();
                    infra.camera_intrinsic.insert(key.into(), param);
                }
                ParameterConfig::DevicePose(param) => {
                    let key = param.device.clone();
                    infra.device_pose.insert(key.into(), param);
                }
                ParameterConfig::CameraProfile(param) => {
                    let key = param.device.clone();
                    infra.camera_config.insert(key.into(), param);
                }
                ParameterConfig::LidarProfile(param) => {
                    let key = param.device.clone();
                    infra.lidar_config.insert(key.into(), *param);
                }
            }

            infra
        });

        Ok(output)
    }

    #[cfg(feature = "with-nalgebra")]
    pub fn to_coord_transform_map(&self) -> Result<CoordTransformMap> {
        CoordTransformMap::new(self.coordinate_transform.iter().map(|((src, tgt), param)| {
            (
                src.clone(),
                tgt.clone(),
                nalgebra::Isometry3::from(&param.transform),
            )
        }))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ParameterConfig {
    CoordinateTransform(CoordinateTransform),
    CameraIntrinsic(CameraIntrinsicsConfig),
    DevicePose(DevicePose),
    CameraProfile(CameraConfig),
    LidarProfile(Box<LidarConfig>),
}

impl From<CoordinateTransform> for ParameterConfig {
    fn from(from: CoordinateTransform) -> Self {
        ParameterConfig::CoordinateTransform(from)
    }
}

impl From<CameraIntrinsicsConfig> for ParameterConfig {
    fn from(from: CameraIntrinsicsConfig) -> Self {
        ParameterConfig::CameraIntrinsic(from)
    }
}

impl From<DevicePose> for ParameterConfig {
    fn from(from: DevicePose) -> Self {
        ParameterConfig::DevicePose(from)
    }
}

impl From<CameraConfig> for ParameterConfig {
    fn from(from: CameraConfig) -> Self {
        ParameterConfig::CameraProfile(from)
    }
}

impl From<LidarConfig> for ParameterConfig {
    fn from(from: LidarConfig) -> Self {
        ParameterConfig::LidarProfile(Box::new(from))
    }
}
