use crate::common::*;
use common_types::DevicePathV1;
#[cfg(feature = "with-nalgebra")]
use nalgebra as na;

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DevicePose {
    #[serde(flatten)]
    pub device: DevicePathV1,
    pub roll_degrees: R64,
    pub elevation_degrees: R64,
    pub elevation_m: R64,
}

impl DevicePose {
    #[cfg(feature = "with-nalgebra")]
    pub fn pose(&self) -> na::Isometry3<f64> {
        let rot = na::UnitQuaternion::from_euler_angles(
            self.roll_degrees.raw().to_radians(),
            -self.elevation_degrees.raw().to_radians(),
            0.0,
        );
        let trans = na::Translation3::new(0.0, 0.0, self.elevation_m.raw());
        na::Isometry3::from_parts(trans, rot)
    }

    pub fn roll_radians(&self) -> f64 {
        self.roll_degrees.raw().to_radians()
    }

    pub fn elevation_radians(&self) -> f64 {
        self.elevation_degrees.raw().to_radians()
    }
}
