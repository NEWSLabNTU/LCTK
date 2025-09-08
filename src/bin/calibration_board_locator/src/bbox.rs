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
            pose: na::Isometry3::new(na::Vector3::new(2.5, 0.0, 0.0), na::Vector3::zeros()),
            size_xyz: [1.0, 3.0, 2.0], // x_range: 2~3 (1), y_range: -1.5~1.5 (3), z_range: -1~1 (2)
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
