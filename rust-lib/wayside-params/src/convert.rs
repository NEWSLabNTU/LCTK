use common_types::DeviceTuple;

use crate::{infra_v1, infra_v2};

impl From<infra_v1::InfraV1> for infra_v2::InfraV2 {
    fn from(v1: infra_v1::InfraV1) -> Self {
        let infra_v1::InfraV1 {
            coordinate_transform,
            camera_intrinsic: camera_intrinsics,
            device_pose,
            camera_config: camera_profile,
            lidar_config: lidar_profile,
        } = v1;

        Self {
            coordinate_transform: coordinate_transform
                .into_iter()
                .map(|((src, tgt), param)| (DeviceTuple { src, tgt }, param.into()))
                .collect(),
            camera_intrinsics: camera_intrinsics
                .into_iter()
                .map(|(device, param)| (device, param.into()))
                .collect(),
            device_pose: device_pose
                .into_iter()
                .map(|(device, param)| (device, param.into()))
                .collect(),
            camera_profile: camera_profile
                .into_iter()
                .map(|(device, param)| (device, param.into()))
                .collect(),
            lidar_profile: lidar_profile
                .into_iter()
                .map(|(device, param)| (device, param.into()))
                .collect(),
        }
    }
}

impl From<infra_v1::CoordinateTransform> for infra_v2::CoordinateTransform {
    fn from(v1: infra_v1::CoordinateTransform) -> Self {
        let infra_v1::CoordinateTransform {
            from: src,
            to: tgt,
            transform,
        } = v1;

        Self {
            src: src.into(),
            tgt: tgt.into(),
            transform,
        }
    }
}

impl From<infra_v1::CameraIntrinsicsConfig> for infra_v2::CameraIntrinsics {
    fn from(v1: infra_v1::CameraIntrinsicsConfig) -> Self {
        let infra_v1::CameraIntrinsicsConfig { device, intrinsic } = v1;

        Self {
            device: device.into(),
            params: intrinsic,
        }
    }
}

impl From<infra_v1::DevicePose> for infra_v2::DevicePose {
    fn from(v1: infra_v1::DevicePose) -> Self {
        let infra_v1::DevicePose {
            device,
            roll_degrees,
            elevation_degrees,
            elevation_m,
        } = v1;

        Self {
            device: device.into(),
            roll_degrees,
            elevation_degrees,
            elevation_m,
        }
    }
}

impl From<infra_v1::CameraConfig> for infra_v2::CameraProfile {
    fn from(v1: infra_v1::CameraConfig) -> Self {
        let infra_v1::CameraConfig { device, profile } = v1;

        Self {
            device: device.into(),
            profile,
        }
    }
}

impl From<infra_v1::LidarConfig> for infra_v2::LidarProfile {
    fn from(v1: infra_v1::LidarConfig) -> Self {
        let infra_v1::LidarConfig { device, profile } = v1;

        Self {
            device: device.into(),
            profile: profile.into(),
        }
    }
}

impl From<infra_v1::LidarProfile> for infra_v2::LidarKind {
    fn from(v1: infra_v1::LidarProfile) -> Self {
        use infra_v1::LidarProfile as L1;

        match v1 {
            L1::OusterOs1(param) => infra_v2::ouster::Profile::from(*param).into(),
            L1::Velodyne(param) => infra_v2::velodyne::Profile::from(param).into(),
            L1::PcapFile(param) => param.into(),
        }
    }
}

impl From<infra_v1::OusterOs1Profile> for infra_v2::ouster::Profile {
    fn from(v1: infra_v1::OusterOs1Profile) -> Self {
        use infra_v2::ouster;

        let infra_v1::OusterOs1Profile {
            params: infra_v1::OusterOs1Params { config_file },
            host_address,
            remote_address,
            time_offset,
        } = v1;

        Self {
            model: ouster::Model::Os1,
            config_file,
            time_offset,
            source: ouster::Source::Network(ouster::NetworkConnection {
                host_address,
                remote_address,
            }),
        }
    }
}

impl From<infra_v1::VelodyneProfile> for infra_v2::velodyne::Profile {
    fn from(v1: infra_v1::VelodyneProfile) -> Self {
        use infra_v2::velodyne;

        let infra_v1::VelodyneProfile {
            params: infra_v1::VelodyneParams { model, return_mode },
            host_address,
            remote_address,
            time_offset,
        } = v1;

        Self {
            model: model.into(),
            return_mode: return_mode.into(),
            time_offset,
            source: velodyne::Source::Network(velodyne::NetworkConnection {
                host_address,
                remote_address,
            }),
        }
    }
}

impl From<infra_v1::VelodyneModel> for infra_v2::velodyne::Model {
    fn from(v1: infra_v1::VelodyneModel) -> Self {
        use infra_v1::VelodyneModel as M1;
        use infra_v2::velodyne::Model as M2;

        match v1 {
            M1::Vlp16 => M2::Vlp16,
            M1::PuckHiRes => M2::PuckHiRes,
            M1::Vlp32C => M2::Vlp32C,
        }
    }
}

impl From<infra_v1::VelodyneReturnMode> for infra_v2::velodyne::ReturnMode {
    fn from(v1: infra_v1::VelodyneReturnMode) -> Self {
        use infra_v1::VelodyneReturnMode as M1;
        use infra_v2::velodyne::ReturnMode as M2;

        match v1 {
            M1::Strongest => M2::Strongest,
            M1::Last => M2::Last,
            M1::Dual => M2::Dual,
        }
    }
}

impl From<infra_v1::PcapFileProfile> for infra_v2::LidarKind {
    fn from(v1: infra_v1::PcapFileProfile) -> Self {
        use infra_v1::{LidarParams as L, PcapFileProfile as P1};
        use infra_v2::{ouster, velodyne};

        let P1 {
            file,
            lidar,
            time_offset,
        } = v1;

        match lidar {
            L::Velodyne(infra_v1::VelodyneParams { model, return_mode }) => velodyne::Profile {
                model: model.into(),
                return_mode: return_mode.into(),
                time_offset,
                source: velodyne::Source::PcapFile(velodyne::PcapFile { file }),
            }
            .into(),
            L::OusterOs1(param) => {
                let infra_v1::OusterOs1Params { config_file } = *param;

                ouster::Profile {
                    model: ouster::Model::Os1,
                    config_file,
                    time_offset,
                    source: ouster::Source::PcapFile(ouster::PcapFile { file }),
                }
                .into()
            }
        }
    }
}
