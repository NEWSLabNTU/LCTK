use crate::types::{LidarPoint, RoiBounds};
use nalgebra::Point3;

/// Trait for cropping point clouds based on ROI
pub trait RoiCropper: Send + Sync {
    fn crop_lidar_points(&self, points: &[LidarPoint], bounds: &RoiBounds) -> Vec<LidarPoint>;
    fn crop_nalgebra_points(&self, points: &[Point3<f64>], bounds: &RoiBounds) -> Vec<Point3<f64>>;
}

/// Default implementation of RoiCropper
pub struct DefaultRoiCropper;

impl RoiCropper for DefaultRoiCropper {
    fn crop_lidar_points(&self, points: &[LidarPoint], bounds: &RoiBounds) -> Vec<LidarPoint> {
        points
            .iter()
            .filter(|point| {
                point.x >= bounds.min_x as f32
                    && point.x <= bounds.max_x as f32
                    && point.y >= bounds.min_y as f32
                    && point.y <= bounds.max_y as f32
                    && point.z >= bounds.min_z as f32
                    && point.z <= bounds.max_z as f32
            })
            .cloned()
            .collect()
    }

    fn crop_nalgebra_points(&self, points: &[Point3<f64>], bounds: &RoiBounds) -> Vec<Point3<f64>> {
        crate::types::apply_roi_crop(points, bounds)
    }
}

/// Helper function to create ROI bounds from center and size
pub fn bounds_from_center_size(
    center_x: f64,
    center_y: f64,
    center_z: f64,
    size_x: f64,
    size_y: f64,
    size_z: f64,
) -> RoiBounds {
    RoiBounds {
        min_x: center_x - size_x / 2.0,
        max_x: center_x + size_x / 2.0,
        min_y: center_y - size_y / 2.0,
        max_y: center_y + size_y / 2.0,
        min_z: center_z - size_z / 2.0,
        max_z: center_z + size_z / 2.0,
    }
}

/// Helper function to get center and size from ROI bounds
pub fn center_size_from_bounds(bounds: &RoiBounds) -> ((f64, f64, f64), (f64, f64, f64)) {
    let center_x = (bounds.min_x + bounds.max_x) / 2.0;
    let center_y = (bounds.min_y + bounds.max_y) / 2.0;
    let center_z = (bounds.min_z + bounds.max_z) / 2.0;

    let size_x = bounds.max_x - bounds.min_x;
    let size_y = bounds.max_y - bounds.min_y;
    let size_z = bounds.max_z - bounds.min_z;

    ((center_x, center_y, center_z), (size_x, size_y, size_z))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crop_lidar_points() {
        let cropper = DefaultRoiCropper;
        let bounds = RoiBounds {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: -1.0,
            max_z: 1.0,
        };

        let points = vec![
            LidarPoint {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // inside
            LidarPoint {
                x: 2.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // outside
            LidarPoint {
                x: 0.5,
                y: 0.5,
                z: 0.5,
                intensity: 0.0,
            }, // inside
            LidarPoint {
                x: -2.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // outside
        ];

        let cropped = cropper.crop_lidar_points(&points, &bounds);
        assert_eq!(cropped.len(), 2);
        assert_eq!(cropped[0].x, 0.0);
        assert_eq!(cropped[1].x, 0.5);
    }

    #[test]
    fn test_bounds_from_center_size() {
        let bounds = bounds_from_center_size(2.0, 0.0, 0.0, 4.0, 4.0, 2.0);

        assert_eq!(bounds.min_x, 0.0);
        assert_eq!(bounds.max_x, 4.0);
        assert_eq!(bounds.min_y, -2.0);
        assert_eq!(bounds.max_y, 2.0);
        assert_eq!(bounds.min_z, -1.0);
        assert_eq!(bounds.max_z, 1.0);
    }

    #[test]
    fn test_center_size_from_bounds() {
        let bounds = RoiBounds {
            min_x: 0.0,
            max_x: 4.0,
            min_y: -2.0,
            max_y: 2.0,
            min_z: -1.0,
            max_z: 1.0,
        };

        let ((cx, cy, cz), (sx, sy, sz)) = center_size_from_bounds(&bounds);

        assert_eq!(cx, 2.0);
        assert_eq!(cy, 0.0);
        assert_eq!(cz, 0.0);
        assert_eq!(sx, 4.0);
        assert_eq!(sy, 4.0);
        assert_eq!(sz, 2.0);
    }

    #[test]
    fn test_bounds_conversion_roundtrip() {
        let original_bounds = RoiBounds {
            min_x: -3.0,
            max_x: 5.0,
            min_y: -2.0,
            max_y: 4.0,
            min_z: -1.0,
            max_z: 3.0,
        };

        let ((cx, cy, cz), (sx, sy, sz)) = center_size_from_bounds(&original_bounds);
        let reconstructed_bounds = bounds_from_center_size(cx, cy, cz, sx, sy, sz);

        assert!((reconstructed_bounds.min_x - original_bounds.min_x).abs() < 1e-6);
        assert!((reconstructed_bounds.max_x - original_bounds.max_x).abs() < 1e-6);
        assert!((reconstructed_bounds.min_y - original_bounds.min_y).abs() < 1e-6);
        assert!((reconstructed_bounds.max_y - original_bounds.max_y).abs() < 1e-6);
        assert!((reconstructed_bounds.min_z - original_bounds.min_z).abs() < 1e-6);
        assert!((reconstructed_bounds.max_z - original_bounds.max_z).abs() < 1e-6);
    }
}
