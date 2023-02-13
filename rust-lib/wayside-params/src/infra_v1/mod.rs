pub use config::*;
mod config;

pub use device_pose::*;
mod device_pose;

pub use camera_intrinsics::*;
mod camera_intrinsics;

pub use coordinate_transform::*;
mod coordinate_transform;

pub use camera_profile::*;
mod camera_profile;

pub use lidar_profile::*;
mod lidar_profile;

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result;

    #[test]
    fn infra_v1_test() -> Result<()> {
        glob::glob(&format!(
            "{}/../config/params/*/",
            env!("CARGO_MANIFEST_DIR")
        ))?
        .try_for_each(|dir| -> Result<_> {
            let _ = InfraV1::load(dir?)?;
            Ok(())
        })?;

        Ok(())
    }
}
