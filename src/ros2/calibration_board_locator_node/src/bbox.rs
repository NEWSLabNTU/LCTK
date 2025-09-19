use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BBoxConfig {
    pub pose: PoseConfig,
    pub size_xyz: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseConfig {
    pub translation: [f64; 3],
    pub rotation: [f64; 4], // quaternion [w, x, y, z]
    #[serde(default)]
    pub euler_angles: Option<[f64; 3]>, // [roll, pitch, yaw] in radians
}

#[derive(Debug, Clone)]
pub struct BBox {
    pub pose: na::Isometry3<f64>,
    pub size_xyz: [f64; 3],
}
impl Default for BBoxConfig {
    fn default() -> Self {
        Self {
            pose: PoseConfig {
                translation: [2.5, 0.0, 0.0],
                rotation: [1.0, 0.0, 0.0, 0.0], // identity quaternion
                euler_angles: Some([0.0, 0.0, 0.0]), // no rotation
            },
            size_xyz: [1.0, 3.0, 2.0], // x_range: 2~3 (1), y_range: -1.5~1.5 (3), z_range: -1~1 (2)
        }
    }
}

impl From<BBoxConfig> for BBox {
    fn from(config: BBoxConfig) -> Self {
        let translation = na::Vector3::new(
            config.pose.translation[0],
            config.pose.translation[1],
            config.pose.translation[2],
        );

        // If euler angles are provided, use them; otherwise use quaternion
        let rotation = if let Some(euler) = config.pose.euler_angles {
            na::UnitQuaternion::from_euler_angles(euler[0], euler[1], euler[2])
        } else {
            na::UnitQuaternion::new_normalize(na::Quaternion::new(
                config.pose.rotation[0], // w
                config.pose.rotation[1], // x
                config.pose.rotation[2], // y
                config.pose.rotation[3], // z
            ))
        };

        Self {
            pose: na::Isometry3::from_parts(translation.into(), rotation),
            size_xyz: config.size_xyz,
        }
    }
}

impl From<&BBox> for BBoxConfig {
    fn from(bbox: &BBox) -> Self {
        let translation = bbox.pose.translation.vector;
        let quaternion = bbox.pose.rotation.quaternion();
        let euler = bbox.pose.rotation.euler_angles();

        Self {
            pose: PoseConfig {
                translation: [translation.x, translation.y, translation.z],
                rotation: [quaternion.w, quaternion.i, quaternion.j, quaternion.k],
                euler_angles: Some([euler.0, euler.1, euler.2]),
            },
            size_xyz: bbox.size_xyz,
        }
    }
}

impl Default for BBox {
    fn default() -> Self {
        BBoxConfig::default().into()
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

    pub fn update_from_parameters(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        size_x: f64,
        size_y: f64,
        size_z: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) {
        let translation = na::Vector3::new(x, y, z);
        let rotation = na::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        self.pose = na::Isometry3::from_parts(translation.into(), rotation);
        self.size_xyz = [size_x, size_y, size_z];
    }

    pub fn get_euler_angles(&self) -> [f64; 3] {
        let euler = self.pose.rotation.euler_angles();
        [euler.0, euler.1, euler.2] // [roll, pitch, yaw]
    }

    pub fn get_translation(&self) -> [f64; 3] {
        let t = self.pose.translation.vector;
        [t.x, t.y, t.z]
    }

    pub fn to_config(&self) -> BBoxConfig {
        self.into()
    }
}
