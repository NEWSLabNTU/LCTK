use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Crop box used to isolate the calibration board from the raw cloud.
///
/// # On-disk format (`config/board/bbox*.json5`)
///
/// The file is parsed straight into this struct by `serde`; there is no
/// hand-written parser. That means the `rotation` array is deserialized by
/// nalgebra's `UnitQuaternion` impl, which is transparent down to
/// `Quaternion`'s backing `Vector4` — whose component order is
/// **`[x, y, z, w]` (i, j, k, w), scalar LAST**:
///
/// ```json5
/// {
///     "pose": {
///         "translation": [x, y, z],       // meters
///         "rotation": [x, y, z, w],       // scalar-LAST, and must already be unit norm
///     },
///     "size_xyz": [sx, sy, sz],           // meters, full extents (box is centered on the pose)
/// }
/// ```
///
/// Two traps live here, and both have bitten this file before:
///
/// 1. `na::Quaternion::new(w, i, j, k)` and every other nalgebra *constructor*
///    take the scalar **first**, the opposite of the storage/serde order above.
///    A config written scalar-first parses successfully and silently means a
///    different rotation. The live ROS-parameter path in `main.rs`
///    (`bbox_rotation_w/_x/_y/_z` fed to `Quaternion::new`) is scalar-first
///    because it goes through the constructor; this file's array is not.
/// 2. `Unit`'s `Deserialize` does **not** normalize — it wraps the value as-is.
///    A non-unit array yields a `UnitQuaternion` that is not a unit quaternion,
///    which turns `pose.inverse()` in [`BBox::contains_point`] into a scaling.
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
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Local stand-in for `approx`, which is not a dependency of this package.
    fn assert_close(actual: f64, expected: f64, eps: f64, what: &str) {
        assert!(
            (actual - expected).abs() < eps,
            "{what}: expected {expected} +/- {eps}, got {actual}"
        );
    }

    fn parse(json5_str: &str) -> BBox {
        json5::from_str(json5_str).expect("shipped-format bbox config must parse")
    }

    /// Pins the scalar-LAST order of the `rotation` array against the
    /// scalar-first constructor order. A 90° yaw is deliberately asymmetric:
    /// under the wrong reading the array below is a 90° *roll* instead, and a
    /// rotation-invariant assertion (norms, the box being a box) would not
    /// notice.
    #[test]
    fn rotation_array_is_scalar_last() {
        let h = std::f64::consts::FRAC_1_SQRT_2; // sin(45°) == cos(45°)
        let bbox = parse(&format!(
            r#"{{ "pose": {{ "translation": [0, 0, 0], "rotation": [0, 0, {h}, {h}] }},
                  "size_xyz": [1, 1, 1] }}"#
        ));

        // +90° about Z sends +X to +Y. Under a scalar-first reading the same
        // array would be +90° about Y, which sends +X to -Z.
        let mapped = bbox.pose.rotation * na::Vector3::x();
        assert_close(mapped.x, 0.0, 1e-12, "x");
        assert_close(mapped.y, 1.0, 1e-12, "y");
        assert_close(mapped.z, 0.0, 1e-12, "z");
    }

    /// `Unit`'s `Deserialize` does not normalize, so a shipped file that is not
    /// unit norm silently corrupts `contains_point`.
    #[test]
    fn shipped_configs_parse_as_unit_quaternions() {
        const SHIPPED: [(&str, &str); 6] = [
            (
                "bbox.json5",
                include_str!("../../lctk_launch/config/board/bbox.json5"),
            ),
            (
                "bbox_v1.json5",
                include_str!("../../lctk_launch/config/board/bbox_v1.json5"),
            ),
            (
                "bbox-seyond.json5",
                include_str!("../../lctk_launch/config/board/bbox-seyond.json5"),
            ),
            (
                "bbox-vlp.json5",
                include_str!("../../lctk_launch/config/board/bbox-vlp.json5"),
            ),
            (
                "bbox_2_lidar_seyond.json5",
                include_str!("../../lctk_launch/config/board/bbox_2_lidar_seyond.json5"),
            ),
            (
                "bbox_2_lidar_vlp32.json5",
                include_str!("../../lctk_launch/config/board/bbox_2_lidar_vlp32.json5"),
            ),
        ];

        for (name, text) in SHIPPED {
            let bbox = parse(text);
            let norm = bbox.pose.rotation.quaternion().norm();
            // 1e-6, not float epsilon: the shipped quaternions are hand-rounded
            // to seven decimals, so exact unit norm is not achievable. This
            // catches a genuinely unnormalized array, which `Unit` would accept.
            assert_close(norm, 1.0, 1e-6, name);
            assert!(
                bbox.size_xyz.iter().all(|&s| s > 0.0),
                "{name}: sizes must be positive"
            );
        }
    }

    /// The two-LiDAR presets mean "no rotation". Written scalar-first as
    /// `[1, 0, 0, 0]` they parsed as a 180° roll instead — harmless only
    /// because that maps a centered box onto itself. Pin the intent.
    #[test]
    fn two_lidar_presets_are_unrotated() {
        for text in [
            include_str!("../../lctk_launch/config/board/bbox_2_lidar_seyond.json5"),
            include_str!("../../lctk_launch/config/board/bbox_2_lidar_vlp32.json5"),
        ] {
            let rotation = parse(text).pose.rotation;
            assert_close(rotation.angle(), 0.0, 1e-12, "rotation angle");
        }
    }

    #[test]
    fn contains_point_respects_pose_and_extents() {
        let bbox = parse(
            r#"{ "pose": { "translation": [10, 0, 0], "rotation": [0, 0, 0, 1] },
                 "size_xyz": [2, 4, 6] }"#,
        );

        assert!(bbox.contains_point(&na::Point3::new(10.0, 0.0, 0.0)));
        // Half-extents are measured from the center, so ±1 in x is the edge.
        assert!(bbox.contains_point(&na::Point3::new(11.0, 2.0, 3.0)));
        assert!(!bbox.contains_point(&na::Point3::new(11.1, 0.0, 0.0)));
        assert!(!bbox.contains_point(&na::Point3::new(0.0, 0.0, 0.0)));
    }
}
