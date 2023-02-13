mod common;
pub mod infra_v1;
pub mod infra_v2;

mod convert;
mod utils;
#[cfg(feature = "with-nalgebra")]
pub use utils::CoordTransformMap;

use crate::common::*;
use serde_loader::{AbsPathBuf, Json5Path};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ParamConfig {
    InfraV1(InfraV1Config),
    InfraV2(InfraV2Config),
}

impl ParamConfig {
    pub fn open(file: impl AsRef<Path>) -> Result<Self> {
        Ok(Json5Path::open_and_take(file)?)
    }

    pub fn to_infra_v1(&self) -> Result<infra_v1::InfraV1> {
        match self {
            Self::InfraV1(config) => Ok(config.load()?),
            _ => bail!("unable to convert to infra-v1 format"),
        }
    }

    pub fn to_infra_v2(&self) -> Result<infra_v2::InfraV2> {
        let params = match self {
            Self::InfraV1(config) => config.load()?.into(),
            Self::InfraV2(config) => config.load()?,
        };
        Ok(params)
    }
}

impl From<InfraV1Config> for ParamConfig {
    fn from(v: InfraV1Config) -> Self {
        ParamConfig::InfraV1(v)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct InfraV1Config {
    pub dir: AbsPathBuf,
}

impl InfraV1Config {
    pub fn load(&self) -> Result<infra_v1::InfraV1> {
        infra_v1::InfraV1::load(&self.dir)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct InfraV2Config {
    pub main_file: AbsPathBuf,
}

impl InfraV2Config {
    pub fn load(&self) -> Result<infra_v2::InfraV2> {
        infra_v2::InfraV2::open(&self.main_file)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_config() -> Result<()> {
        glob::glob(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/*/params.json",
        ))?
        .try_for_each(|file| -> Result<_> {
            let _ = ParamConfig::open(file?)?.to_infra_v2()?;
            Ok(())
        })?;

        Ok(())
    }
}
