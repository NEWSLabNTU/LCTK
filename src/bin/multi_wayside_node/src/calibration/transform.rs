use crate::types::BoardDetection;
use eyre::Result;
use nalgebra::Isometry3;

/// Trait for computing calibration transforms
pub trait CalibrationSolver: Send + Sync {
    fn compute_transform(
        &self,
        detection1: &BoardDetection,
        detection2: &BoardDetection,
        same_face_mode: bool,
    ) -> Result<Isometry3<f64>>;
}

/// Default implementation of CalibrationSolver
pub struct DefaultCalibrationSolver;

impl CalibrationSolver for DefaultCalibrationSolver {
    fn compute_transform(
        &self,
        detection1: &BoardDetection,
        detection2: &BoardDetection,
        same_face_mode: bool,
    ) -> Result<Isometry3<f64>> {
        // Implementation based on original multi_wayside logic
        compute_lidar_to_lidar_transform(&detection1.pose, &detection2.pose, same_face_mode)
    }
}

/// Compute transformation between two LiDAR coordinate frames
/// based on detections of the same calibration board
pub fn compute_lidar_to_lidar_transform(
    pose1: &Isometry3<f64>,
    pose2: &Isometry3<f64>,
    same_face_mode: bool,
) -> Result<Isometry3<f64>> {
    if same_face_mode {
        // Both LiDARs see the same face of the board
        // Transform from LiDAR1 to LiDAR2: T_2to1 = T_board1 * T_board2^(-1)
        Ok(pose1 * pose2.inverse())
    } else {
        // LiDARs see opposite faces of the board
        // Need to account for 180-degree rotation around the board's Y-axis
        let board_flip = Isometry3::rotation(nalgebra::Vector3::y() * std::f64::consts::PI);
        Ok(pose1 * board_flip * pose2.inverse())
    }
}

/// Apply VLP16 coordinate system bug fix
pub fn apply_vlp16_coordinate_fix(transform: Isometry3<f64>) -> Isometry3<f64> {
    // VLP16 has a known coordinate system issue - this is a placeholder
    // for the specific correction that would be applied
    // TODO: Implement actual VLP16 coordinate correction if needed
    transform
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn test_same_face_transform() {
        // Create two poses representing the same board seen from different LiDARs
        let pose1 =
            Isometry3::from_parts(Translation3::new(1.0, 0.0, 0.0), UnitQuaternion::identity());

        let pose2 =
            Isometry3::from_parts(Translation3::new(2.0, 0.0, 0.0), UnitQuaternion::identity());

        let transform = compute_lidar_to_lidar_transform(&pose1, &pose2, true).unwrap();

        // The transform should place LiDAR2's origin relative to LiDAR1
        let expected_translation = pose1.translation.vector - pose2.translation.vector;
        let actual_translation = transform.translation.vector;

        assert!((expected_translation - actual_translation).norm() < 1e-6);
    }

    #[test]
    fn test_opposite_face_transform() {
        let pose1 = Isometry3::identity();
        let pose2 = Isometry3::identity();

        let transform = compute_lidar_to_lidar_transform(&pose1, &pose2, false).unwrap();

        // With opposite faces, should include 180-degree rotation
        let rotation_angle = transform.rotation.angle();
        assert!((rotation_angle - std::f64::consts::PI).abs() < 1e-6);
    }

    #[test]
    fn test_calibration_solver() {
        let solver = DefaultCalibrationSolver;

        let detection1 = BoardDetection {
            pose: Isometry3::identity(),
            confidence: 0.8,
            inlier_count: 100,
            timestamp: std::time::SystemTime::now(),
        };

        let detection2 = BoardDetection {
            pose: Isometry3::from_parts(
                Translation3::new(1.0, 0.0, 0.0),
                UnitQuaternion::identity(),
            ),
            confidence: 0.8,
            inlier_count: 100,
            timestamp: std::time::SystemTime::now(),
        };

        let transform = solver
            .compute_transform(&detection1, &detection2, true)
            .unwrap();
        assert!(transform.translation.vector.norm() > 0.0);
    }
}
