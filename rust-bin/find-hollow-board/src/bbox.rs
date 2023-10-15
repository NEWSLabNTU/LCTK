use kiss3d::nalgebra as na30;
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BBox {
    pub pose: na::Isometry3<f64>,
    pub size_xyz: [f64; 3],
}

impl Default for BBox {
    fn default() -> Self {
        Self {
            pose: na::Isometry3::identity(),
            size_xyz: [1.4, 0.4, 1.4],
        }
    }
}

impl BBox {
    pub fn contains_point(&self, pt: &na::Point3<f64>) -> bool {
        let pt = self.pose.inverse() * pt;
        let [sx, sy, sz] = self.size_xyz;

        let in_range = |size: f64, val: f64| {
            let half = size / 2.0;
            (-half..=half).contains(&val)
        };

        in_range(sx, pt.x) && in_range(sy, pt.y) && in_range(sz, pt.z)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BBox30 {
    pub pose: na30::Isometry3<f64>,
    pub size_xyz: [f64; 3],
}

impl Default for BBox30 {
    fn default() -> Self {
        Self {
            pose: na30::Isometry3::identity(),
            size_xyz: [1.4, 0.4, 1.4],
        }
    }
}

impl BBox30 {
    pub fn contains_point(&self, pt: &na30::Point3<f64>) -> bool {
        let pt = self.pose.inverse() * pt;
        let [sx, sy, sz] = self.size_xyz;

        let in_range = |size: f64, val: f64| {
            let half = size / 2.0;
            (-half..=half).contains(&val)
        };

        in_range(sx, pt.x) && in_range(sy, pt.y) && in_range(sz, pt.z)
    }
}

impl From<BBox> for BBox30 {
    fn from(src: BBox) -> Self {
        let BBox { pose, size_xyz } = src;
        let na::Isometry3 {
            rotation,
            translation,
        } = pose;
        let na::coordinates::XYZ { x, y, z } = *translation;
        let na::coordinates::IJKW { i, j, k, w } = **rotation;
        let pose = na30::Isometry3 {
            rotation: na30::UnitQuaternion::from_quaternion(na30::Quaternion::new(w, i, j, k)),
            translation: na30::Translation3::new(x, y, z),
        };
        Self { pose, size_xyz }
    }
}

impl From<BBox30> for BBox {
    fn from(src: BBox30) -> Self {
        let BBox30 { pose, size_xyz } = src;
        let na30::Isometry3 {
            rotation,
            translation,
        } = pose;
        let na30::coordinates::XYZ { x, y, z } = *translation;
        let na30::coordinates::IJKW { i, j, k, w } = **rotation;
        let pose = na::Isometry3 {
            rotation: na::UnitQuaternion::from_quaternion(na::Quaternion::new(w, i, j, k)),
            translation: na::Translation3::new(x, y, z),
        };
        Self { pose, size_xyz }
    }
}
