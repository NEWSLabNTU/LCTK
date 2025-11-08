use nalgebra::Isometry3;

/// Trait for validating calibration results
pub trait CalibrationValidator: Send + Sync {
    fn validate_transform(&self, transform: &Isometry3<f64>) -> CalibrationQuality;
    fn is_acceptable(&self, quality: &CalibrationQuality) -> bool;
}

/// Quality assessment of calibration result
#[derive(Debug, Clone)]
pub struct CalibrationQuality {
    pub translation_magnitude: f64,
    pub rotation_angle: f64,
    pub is_reasonable: bool,
    pub confidence_score: f64,
    pub warnings: Vec<String>,
}

/// Default implementation of CalibrationValidator
pub struct DefaultCalibrationValidator {
    max_translation: f64,
    max_rotation: f64,
    min_confidence: f64,
}

impl DefaultCalibrationValidator {
    pub fn new() -> Self {
        Self {
            max_translation: 10.0,                    // 10 meters
            max_rotation: std::f64::consts::PI / 2.0, // 90 degrees
            min_confidence: 0.5,
        }
    }

    pub fn with_limits(max_translation: f64, max_rotation: f64, min_confidence: f64) -> Self {
        Self {
            max_translation,
            max_rotation,
            min_confidence,
        }
    }
}

impl CalibrationValidator for DefaultCalibrationValidator {
    fn validate_transform(&self, transform: &Isometry3<f64>) -> CalibrationQuality {
        let translation_magnitude = transform.translation.vector.norm();
        let rotation_angle = transform.rotation.angle();

        let mut warnings = Vec::new();
        let mut confidence_score = 1.0;

        // Check translation magnitude
        if translation_magnitude > self.max_translation {
            warnings.push(format!(
                "Large translation: {:.2}m > {:.2}m",
                translation_magnitude, self.max_translation
            ));
            confidence_score *= 0.5;
        }

        // Check rotation angle
        if rotation_angle > self.max_rotation {
            warnings.push(format!(
                "Large rotation: {:.1}° > {:.1}°",
                rotation_angle.to_degrees(),
                self.max_rotation.to_degrees()
            ));
            confidence_score *= 0.5;
        }

        // Check for very small transforms (might indicate poor detection)
        if translation_magnitude < 0.1 && rotation_angle < 0.1 {
            warnings.push("Very small transform - check detection quality".to_string());
            confidence_score *= 0.7;
        }

        // Check for unrealistic transforms
        let is_reasonable = translation_magnitude <= self.max_translation * 2.0
            && rotation_angle <= self.max_rotation * 2.0;

        if !is_reasonable {
            warnings.push("Transform appears unrealistic".to_string());
            confidence_score = 0.0;
        }

        CalibrationQuality {
            translation_magnitude,
            rotation_angle,
            is_reasonable,
            confidence_score,
            warnings,
        }
    }

    fn is_acceptable(&self, quality: &CalibrationQuality) -> bool {
        quality.is_reasonable && quality.confidence_score >= self.min_confidence
    }
}

/// Calculate relative pose error between two transforms
pub fn calculate_pose_error(
    transform1: &Isometry3<f64>,
    transform2: &Isometry3<f64>,
) -> (f64, f64) {
    let relative_transform = transform1.inverse() * transform2;
    let translation_error = relative_transform.translation.vector.norm();
    let rotation_error = relative_transform.rotation.angle();
    (translation_error, rotation_error)
}

/// Check if a transform represents a reasonable LiDAR-to-LiDAR calibration
pub fn is_reasonable_lidar_transform(transform: &Isometry3<f64>) -> bool {
    let translation = transform.translation.vector.norm();
    let rotation = transform.rotation.angle();

    // Reasonable bounds for LiDAR-to-LiDAR calibration
    translation <= 20.0 && rotation <= std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    #[test]
    fn test_reasonable_transform() {
        let validator = DefaultCalibrationValidator::new();

        // Reasonable transform
        let transform = Isometry3::from_parts(
            Translation3::new(2.0, 1.0, 0.5),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.1),
        );

        let quality = validator.validate_transform(&transform);
        assert!(quality.is_reasonable);
        assert!(validator.is_acceptable(&quality));
        assert!(quality.confidence_score > 0.8);
    }

    #[test]
    fn test_unreasonable_transform() {
        let validator = DefaultCalibrationValidator::new();

        // Unreasonable transform (too large)
        let transform = Isometry3::from_parts(
            Translation3::new(50.0, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI),
        );

        let quality = validator.validate_transform(&transform);
        assert!(!quality.warnings.is_empty());
        assert!(quality.confidence_score < 0.5);
    }

    #[test]
    fn test_small_transform() {
        let validator = DefaultCalibrationValidator::new();

        // Very small transform
        let transform = Isometry3::from_parts(
            Translation3::new(0.01, 0.01, 0.01),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.01),
        );

        let quality = validator.validate_transform(&transform);
        assert!(quality.warnings.iter().any(|w| w.contains("Very small")));
    }

    #[test]
    fn test_pose_error_calculation() {
        let transform1 =
            Isometry3::from_parts(Translation3::new(1.0, 0.0, 0.0), UnitQuaternion::identity());

        let transform2 = Isometry3::from_parts(
            Translation3::new(1.1, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.1),
        );

        let (trans_error, rot_error) = calculate_pose_error(&transform1, &transform2);
        assert!((trans_error - 0.1).abs() < 1e-6);
        assert!((rot_error - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_reasonable_lidar_transform() {
        // Reasonable transform
        let transform1 = Isometry3::from_parts(
            Translation3::new(5.0, 2.0, 1.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5),
        );
        assert!(is_reasonable_lidar_transform(&transform1));

        // Unreasonable transform
        let transform2 = Isometry3::from_parts(
            Translation3::new(100.0, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        assert!(!is_reasonable_lidar_transform(&transform2));
    }
}
