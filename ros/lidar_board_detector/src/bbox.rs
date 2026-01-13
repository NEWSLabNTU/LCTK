use anyhow::{anyhow, Result};
use geometry_msgs::msg::{Point, Pose, Quaternion};
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::{fs, path::Path};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BBox {
    pub pose: na::Isometry3<f64>,
    pub size_xyz: [f64; 3],
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

    /// Convert BBox to ROS geometry_msgs::Pose
    pub fn to_ros_pose(&self) -> Pose {
        let translation = &self.pose.translation;
        let rotation = &self.pose.rotation;

        Pose {
            position: Point {
                x: translation.x,
                y: translation.y,
                z: translation.z,
            },
            orientation: Quaternion {
                x: rotation.i,
                y: rotation.j,
                z: rotation.k,
                w: rotation.w,
            },
        }
    }

    /// Create BBox from ROS geometry_msgs::Pose and size
    pub fn from_ros_pose(pose: &Pose, size_xyz: [f64; 3]) -> Result<Self> {
        // Validate size parameters
        for (i, &size) in size_xyz.iter().enumerate() {
            if size <= 0.0 {
                return Err(anyhow!(
                    "Size parameter {} must be positive, got {}",
                    ['x', 'y', 'z'][i],
                    size
                ));
            }
        }

        // Validate and normalize quaternion
        let quat = &pose.orientation;
        let quat_norm =
            (quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w).sqrt();

        if quat_norm < 1e-6 {
            return Err(anyhow!("Invalid quaternion with near-zero norm"));
        }

        // Normalize quaternion
        let normalized_quat = na::UnitQuaternion::new_normalize(na::Quaternion::new(
            quat.w / quat_norm,
            quat.x / quat_norm,
            quat.y / quat_norm,
            quat.z / quat_norm,
        ));

        let translation = na::Translation3::new(pose.position.x, pose.position.y, pose.position.z);

        let isometry = na::Isometry3::from_parts(translation, normalized_quat);

        Ok(BBox {
            pose: isometry,
            size_xyz,
        })
    }

    /// Convert BBox to pretty-printed JSON5 string
    pub fn to_json5_string(&self) -> Result<String> {
        // Create a serializable representation with separate translation and rotation
        let translation = &self.pose.translation;
        let quaternion = self.pose.rotation.quaternion();

        let json_repr = serde_json::json!({
            "pose": {
                "translation": [translation.x, translation.y, translation.z],
                "rotation": [quaternion.w, quaternion.i, quaternion.j, quaternion.k]
            },
            "size_xyz": self.size_xyz
        });

        // Convert to pretty-printed JSON first, then format as JSON5 with comments
        let _json_str = serde_json::to_string_pretty(&json_repr)?;

        // Add JSON5 formatting and comments
        let json5_str = format!(
            r#"{{
    // Bounding box configuration for filtering point cloud data
    // The pose defines the position and orientation of the box center
    "pose": {{
        // Translation from origin (x, y, z in meters)
        "translation": [{:.1}, {:.1}, {:.1}],
        // Rotation as quaternion (w, x, y, z)
        "rotation": [{:.6}, {:.6}, {:.6}, {:.6}]
    }},
    // Size of the bounding box in meters (x, y, z)
    "size_xyz": [{:.1}, {:.1}, {:.1}]
}}"#,
            translation.x,
            translation.y,
            translation.z,
            quaternion.w,
            quaternion.i,
            quaternion.j,
            quaternion.k,
            self.size_xyz[0],
            self.size_xyz[1],
            self.size_xyz[2]
        );

        Ok(json5_str)
    }

    /// Load BBox from JSON5 string
    pub fn from_json5_string(json5_str: &str) -> Result<Self> {
        #[derive(Deserialize)]
        struct BBoxJson {
            pose: PoseJson,
            size_xyz: [f64; 3],
        }

        #[derive(Deserialize)]
        struct PoseJson {
            translation: [f64; 3],
            rotation: [f64; 4], // [w, x, y, z]
        }

        let bbox_json: BBoxJson = json5::from_str(json5_str)?;

        // Validate size parameters
        for (i, &size) in bbox_json.size_xyz.iter().enumerate() {
            if size <= 0.0 {
                return Err(anyhow!(
                    "Size parameter {} must be positive, got {}",
                    ['x', 'y', 'z'][i],
                    size
                ));
            }
        }

        // Create translation
        let translation = na::Translation3::new(
            bbox_json.pose.translation[0],
            bbox_json.pose.translation[1],
            bbox_json.pose.translation[2],
        );

        // Create and validate quaternion
        let [w, x, y, z] = bbox_json.pose.rotation;
        let quat_norm = (x * x + y * y + z * z + w * w).sqrt();

        if quat_norm < 1e-6 {
            return Err(anyhow!("Invalid quaternion with near-zero norm"));
        }

        let normalized_quat = na::UnitQuaternion::new_normalize(na::Quaternion::new(w, x, y, z));
        let isometry = na::Isometry3::from_parts(translation, normalized_quat);

        Ok(BBox {
            pose: isometry,
            size_xyz: bbox_json.size_xyz,
        })
    }

    /// Save BBox to JSON5 file with atomic write
    pub fn save_to_file<P: AsRef<Path>>(&self, file_path: P) -> Result<()> {
        let path = file_path.as_ref();
        let json5_content = self.to_json5_string()?;

        // Atomic write: write to temporary file, then rename
        let temp_path = path.with_extension("tmp");

        // Create parent directory if it doesn't exist
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        // Write to temporary file
        fs::write(&temp_path, json5_content)?;

        // Atomic rename
        fs::rename(&temp_path, path)?;

        Ok(())
    }

    /// Load BBox from JSON5 file
    pub fn load_from_file<P: AsRef<Path>>(file_path: P) -> Result<Self> {
        let content = fs::read_to_string(file_path)?;
        Self::from_json5_string(&content)
    }
}
