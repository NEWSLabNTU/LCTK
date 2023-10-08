use std::iter;

use super::Param;
use crate::infra_v2::{
    CameraIntrinsics, CameraProfile, CoordinateTransform, DevicePose, LidarProfile,
};
use serde::{Deserialize, Serialize};
use serde_loader::Json5Path;

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
