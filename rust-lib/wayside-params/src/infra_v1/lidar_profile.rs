use crate::common::*;
use common_types::DevicePathV1;
use serde_loader::{AbsPathBuf, JsonPrettyPath};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LidarConfig {
    #[serde(flatten)]
    pub device: DevicePathV1,
    pub profile: LidarProfile,
}

impl LidarConfig {
    pub fn params(&self) -> LidarParams {
        self.profile.params()
    }
}

/// The generic LiDAR configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum LidarProfile {
    OusterOs1(Box<OusterOs1Profile>),
    Velodyne(VelodyneProfile),
    PcapFile(PcapFileProfile),
}

impl LidarProfile {
    pub fn time_offset(&self) -> chrono::Duration {
        match self {
            Self::OusterOs1(profile) => profile.time_offset,
            Self::Velodyne(profile) => profile.time_offset,
            Self::PcapFile(profile) => profile.time_offset,
        }
    }

    pub fn as_ouster_os1(&self) -> Option<&OusterOs1Profile> {
        if let Self::OusterOs1(v) = self {
            Some(&*v)
        } else {
            None
        }
    }

    pub fn as_velodyne(&self) -> Option<&VelodyneProfile> {
        if let Self::Velodyne(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_pcap_file(&self) -> Option<&PcapFileProfile> {
        if let Self::PcapFile(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PcapFileProfile> for LidarProfile {
    fn from(v: PcapFileProfile) -> Self {
        LidarProfile::PcapFile(v)
    }
}

impl From<VelodyneProfile> for LidarProfile {
    fn from(v: VelodyneProfile) -> Self {
        LidarProfile::Velodyne(v)
    }
}

impl From<OusterOs1Profile> for LidarProfile {
    fn from(v: OusterOs1Profile) -> Self {
        LidarProfile::OusterOs1(Box::new(v))
    }
}

impl LidarProfile {
    pub fn params(&self) -> LidarParams {
        match self {
            LidarProfile::OusterOs1(profile) => profile.params.clone().into(),
            LidarProfile::Velodyne(profile) => profile.params.clone().into(),
            LidarProfile::PcapFile(profile) => profile.lidar.clone(),
        }
    }
}

/// The Ouster-OS1 LiDAR configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OusterOs1Profile {
    #[serde(flatten)]
    pub params: OusterOs1Params,
    pub host_address: SocketAddr,
    pub remote_address: SocketAddr,
    #[serde(
        with = "common_types::serde_chrono_duration",
        default = "common_types::zero_chrono_duration"
    )]
    pub time_offset: chrono::Duration,
}

/// The Velodyne LiDAR configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VelodyneProfile {
    #[serde(flatten)]
    pub params: VelodyneParams,
    pub host_address: SocketAddr,
    pub remote_address: Option<IpAddr>,
    #[serde(
        with = "common_types::serde_chrono_duration",
        default = "common_types::zero_chrono_duration"
    )]
    pub time_offset: chrono::Duration,
}

/// The PCAP file as LiDAR device configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PcapFileProfile {
    pub file: AbsPathBuf,
    pub lidar: LidarParams,
    #[serde(
        with = "common_types::serde_chrono_duration",
        default = "common_types::zero_chrono_duration"
    )]
    pub time_offset: chrono::Duration,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum LidarParams {
    Velodyne(VelodyneParams),
    OusterOs1(Box<OusterOs1Params>),
}

impl From<OusterOs1Params> for LidarParams {
    fn from(v: OusterOs1Params) -> Self {
        LidarParams::OusterOs1(Box::new(v))
    }
}

impl From<VelodyneParams> for LidarParams {
    fn from(v: VelodyneParams) -> Self {
        LidarParams::Velodyne(v)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OusterOs1Params {
    pub config_file: JsonPrettyPath<ouster_lidar::Config>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VelodyneParams {
    pub model: VelodyneModel,
    pub return_mode: VelodyneReturnMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VelodyneModel {
    Vlp16,
    PuckHiRes,
    Vlp32C,
}

impl VelodyneModel {
    pub fn num_channels(&self) -> u32 {
        match self {
            VelodyneModel::Vlp16 => 16,
            VelodyneModel::PuckHiRes => 16,
            VelodyneModel::Vlp32C => 32,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VelodyneReturnMode {
    Strongest,
    Last,
    Dual,
}
