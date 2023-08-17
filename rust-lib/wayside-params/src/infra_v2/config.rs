use super::{CameraIntrinsics, CameraProfile, CoordinateTransform, DevicePose, LidarProfile};
use crate::common::*;
#[cfg(feature = "with-nalgebra")]
use crate::utils::CoordTransformMap;
use common_types::{DevicePathV2, DeviceTuple};
use serde_loader::Json5Path;

pub use main::*;
mod main {

    use super::*;

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
}

pub use file::*;
mod file {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(transparent)]
    pub struct ParamList {
        pub params: Vec<ParamOrImport>,
    }

    impl ParamList {
        pub fn into_params(self) -> impl Iterator<Item = Param> + Send {
            self.params
                .into_iter()
                .flat_map(|params| params.into_params())
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(tag = "type", rename_all = "snake_case")]
    pub enum ParamOrImport {
        CoordinateTransform(CoordinateTransform),
        CameraIntrinsics(CameraIntrinsics),
        DevicePose(DevicePose),
        CameraProfile(CameraProfile),
        LidarProfile(Box<LidarProfile>),
        Import(Box<Import>),
    }

    impl ParamOrImport {
        pub fn into_params(self) -> Box<dyn Iterator<Item = Param> + Send> {
            let param: Param = match self {
                Self::Import(import) => return Box::new(import.into_params()),
                Self::CoordinateTransform(param) => param.into(),
                Self::CameraIntrinsics(param) => param.into(),
                Self::DevicePose(param) => param.into(),
                Self::CameraProfile(param) => param.into(),
                Self::LidarProfile(param) => (*param).into(),
            };
            Box::new(iter::once(param))
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct Import {
        pub path: Json5Path<ParamList>,
    }

    impl Import {
        pub fn into_params(self) -> impl Iterator<Item = Param> + Send {
            self.path.take().into_params()
        }
    }
}

pub use param::*;
mod param {
    use super::*;

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
}
