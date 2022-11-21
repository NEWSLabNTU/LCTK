use crate::common::*;

/// The trait defines the general point features that the implemented schema should provide.
pub trait PcdPoint {
    /// Gives the x value in Cartesian coordinate.
    fn x(&self) -> f64;
    /// Gives the y value in Cartesian coordinate.
    fn y(&self) -> f64;
    /// Gives the z value in Cartesian coordinate.
    fn z(&self) -> f64;
    /// Gives the distance to origin point in Cartesian coordinate.
    fn distance(&self) -> f64;
    /// Gives the azimuth angle in spherical coordinate.
    ///
    /// The angle measures in radians and in counter-clockwise direction.
    fn azimuthal_angle(&self) -> f64;
    /// Gives the vertical angle in spherical coordinate.
    ///
    /// The value is zero at equator, and is positive towards north pole.
    fn vertical_angle(&self) -> f64;
    /// Gives the polar angle in spherical coordinate.
    ///
    /// The value is zero at the north pole, and is positive towards south pole.
    fn polar_angle(&self) -> f64;
    /// Gives the intensity from sensor. It gives `None` if it lacks the data.
    fn intensity(&self) -> Option<f64>;
    /// Gives the laser index on sensor. It gives `None` if it lacks the data.
    fn laser_id(&self) -> Option<u32>;
    /// Gives the timestamp in milliseconds. It gives `None` if it lacks the data.
    fn timestamp_ns(&self) -> Option<u32>;
}

pub use libpcl::*;
mod libpcl {
    use super::*;

    /// The standard schema used by libpcl.
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize, PcdSerialize, PcdDeserialize)]
    pub struct LibpclPoint {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub rgb: u32,
    }

    impl PcdPoint for LibpclPoint {
        fn x(&self) -> f64 {
            self.x as f64
        }

        fn y(&self) -> f64 {
            self.y as f64
        }

        fn z(&self) -> f64 {
            self.z as f64
        }

        fn distance(&self) -> f64 {
            (self.x().powi(2) + self.y().powi(2) + self.z().powi(2)).sqrt()
        }

        fn azimuthal_angle(&self) -> f64 {
            self.y().atan2(self.x())
        }

        fn vertical_angle(&self) -> f64 {
            -self.polar_angle() + f64::consts::FRAC_PI_2
        }

        fn polar_angle(&self) -> f64 {
            (self.x().powi(2) + self.y().powi(2)).sqrt().atan2(self.z())
        }

        fn intensity(&self) -> Option<f64> {
            None
        }

        fn laser_id(&self) -> Option<u32> {
            None
        }

        fn timestamp_ns(&self) -> Option<u32> {
            None
        }
    }
}

pub use libpcl_ext::*;
mod libpcl_ext {
    use super::*;

    #[derive(Debug, Clone, PartialEq, PcdSerialize)]
    pub struct LibpclExtPoint {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub intensity: f32,
        pub timestamp_ms: f64,
    }

    impl PcdPoint for LibpclExtPoint {
        fn x(&self) -> f64 {
            self.x as f64
        }

        fn y(&self) -> f64 {
            self.y as f64
        }

        fn z(&self) -> f64 {
            self.z as f64
        }

        fn distance(&self) -> f64 {
            (self.x().powi(2) + self.y().powi(2) + self.z().powi(2)).sqrt()
        }

        fn azimuthal_angle(&self) -> f64 {
            self.y().atan2(self.x())
        }

        fn vertical_angle(&self) -> f64 {
            -self.polar_angle() + f64::consts::FRAC_PI_2
        }

        fn polar_angle(&self) -> f64 {
            (self.x().powi(2) + self.y().powi(2)).sqrt().atan2(self.z())
        }

        fn intensity(&self) -> Option<f64> {
            Some(self.intensity as f64)
        }

        fn laser_id(&self) -> Option<u32> {
            None
        }

        fn timestamp_ns(&self) -> Option<u32> {
            Some(self.timestamp_ms as u32 * 1000)
        }
    }
}

pub use spherical::*;
mod spherical {
    use super::*;

    /// The schema that uses spherical coordinates.
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize, PcdSerialize, PcdDeserialize)]
    pub struct SphericalPoint {
        pub distance: f64,
        pub azimuthal_angle: f64,
        pub polar_angle: f64,
        pub intensity: f64,
        pub laser_id: u32,
        pub timestamp_ns: u32,
    }

    impl PcdPoint for SphericalPoint {
        fn x(&self) -> f64 {
            self.distance * self.polar_angle.sin() * self.azimuthal_angle.cos()
        }

        fn y(&self) -> f64 {
            self.distance * self.polar_angle.sin() * self.azimuthal_angle.sin()
        }

        fn z(&self) -> f64 {
            self.distance * self.polar_angle.cos()
        }

        fn distance(&self) -> f64 {
            self.distance
        }

        fn azimuthal_angle(&self) -> f64 {
            self.azimuthal_angle
        }

        fn vertical_angle(&self) -> f64 {
            -self.polar_angle() + f64::consts::FRAC_PI_2
        }

        fn polar_angle(&self) -> f64 {
            self.polar_angle
        }

        fn intensity(&self) -> Option<f64> {
            self.intensity.into()
        }

        fn laser_id(&self) -> Option<u32> {
            self.laser_id.into()
        }

        fn timestamp_ns(&self) -> Option<u32> {
            self.timestamp_ns.into()
        }
    }
}

pub use newslab_v1::*;
mod newslab_v1 {
    use super::*;

    /// The custom schema used by NEWSLAB.
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize, PcdSerialize, PcdDeserialize)]
    pub struct NewslabV1Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub distance: f64,
        pub azimuthal_angle: f64,
        pub vertical_angle: f64,
        pub intensity: f64,
        pub laser_id: u32,
        pub timestamp_ns: u32,
    }

    impl PcdPoint for NewslabV1Point {
        fn x(&self) -> f64 {
            self.x
        }

        fn y(&self) -> f64 {
            self.y
        }

        fn z(&self) -> f64 {
            self.z
        }

        fn distance(&self) -> f64 {
            self.distance
        }

        fn azimuthal_angle(&self) -> f64 {
            self.azimuthal_angle
        }

        fn vertical_angle(&self) -> f64 {
            self.vertical_angle
        }

        fn polar_angle(&self) -> f64 {
            -self.vertical_angle + f64::consts::FRAC_PI_2
        }

        fn intensity(&self) -> Option<f64> {
            self.intensity.into()
        }

        fn laser_id(&self) -> Option<u32> {
            self.laser_id.into()
        }

        fn timestamp_ns(&self) -> Option<u32> {
            self.timestamp_ns.into()
        }
    }
}
