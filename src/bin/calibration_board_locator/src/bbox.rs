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
