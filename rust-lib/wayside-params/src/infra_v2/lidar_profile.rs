use crate::common::*;
use common_types::DevicePathV2;
use serde_loader::{AbsPathBuf, JsonPrettyPath};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LidarProfile {
    pub device: DevicePathV2,
    pub profile: LidarKind,
}

/// The generic LiDAR configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum LidarKind {
    Ouster(Box<ouster::Profile>),
    Velodyne(Box<velodyne::Profile>),
}

impl LidarKind {
    pub fn time_offset(&self) -> chrono::Duration {
        match self {
            Self::Ouster(profile) => profile.time_offset,
            Self::Velodyne(profile) => profile.time_offset,
        }
    }

    pub fn as_ouster_os1(&self) -> Option<&ouster::Profile> {
        if let Self::Ouster(v) = self {
            Some(&*v)
        } else {
            None
        }
    }

    pub fn as_velodyne(&self) -> Option<&velodyne::Profile> {
        if let Self::Velodyne(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<velodyne::Profile> for LidarKind {
    fn from(v: velodyne::Profile) -> Self {
        LidarKind::Velodyne(Box::new(v))
    }
}

impl From<ouster::Profile> for LidarKind {
    fn from(v: ouster::Profile) -> Self {
        LidarKind::Ouster(Box::new(v))
    }
}

pub mod velodyne {
    use super::*;

    /// The Velodyne LiDAR configuration.
    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct Profile {
        pub model: Model,
        pub return_mode: ReturnMode,
        #[serde(
            with = "common_types::serde_chrono_duration",
            default = "common_types::zero_chrono_duration"
        )]
        pub time_offset: chrono::Duration,
        pub source: Source,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(tag = "type", rename_all = "snake_case")]
    pub enum Source {
        Network(NetworkConnection),
        PcapFile(PcapFile),
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct NetworkConnection {
        pub host_address: SocketAddr,
        pub remote_address: Option<IpAddr>,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct PcapFile {
        pub file: AbsPathBuf,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub enum Model {
        #[serde(rename = "vlp-16")]
        Vlp16,
        #[serde(rename = "puck-hi-res")]
        PuckHiRes,
        #[serde(rename = "vlp-32c")]
        Vlp32C,
    }

    impl Model {
        pub fn num_channels(&self) -> u32 {
            match self {
                Self::Vlp16 => 16,
                Self::PuckHiRes => 16,
                Self::Vlp32C => 32,
            }
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(rename_all = "snake_case")]
    pub enum ReturnMode {
        Strongest,
        Last,
        Dual,
    }
}

pub mod ouster {
    use super::*;

    /// The Ouster LiDAR configuration.
    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct Profile {
        pub model: Model,
        pub config_file: JsonPrettyPath<ouster_lidar::Config>,
        #[serde(
            with = "common_types::serde_chrono_duration",
            default = "common_types::zero_chrono_duration"
        )]
        pub time_offset: chrono::Duration,
        pub source: Source,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub enum Model {
        #[serde(rename = "os-1")]
        Os1,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(tag = "type", rename_all = "snake_case")]
    pub enum Source {
        Network(NetworkConnection),
        PcapFile(PcapFile),
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct NetworkConnection {
        pub host_address: SocketAddr,
        pub remote_address: SocketAddr,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct PcapFile {
        pub file: AbsPathBuf,
    }
}
