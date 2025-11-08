use approx::assert_relative_eq;
use hollow_board_detector::algo::{compute_centroid, compute_voxel_key, voxel_downsample};
use nalgebra::Point3;

#[test]
fn test_empty_point_cloud() {
    let points: Vec<Point3<f64>> = vec![];
    let result = voxel_downsample(&points, 0.02, true, 50_000);
    assert_eq!(result.len(), 0, "Empty input should return empty output");
}

#[test]
fn test_single_point() {
    let points = vec![Point3::new(1.0, 2.0, 3.0)];
    let result = voxel_downsample(&points, 0.02, true, 50_000);
    assert_eq!(result.len(), 1);
    assert_relative_eq!(result[0], points[0], epsilon = 1e-10);
}

#[test]
fn test_points_in_same_voxel_centroid() {
    // Four points in the same 2cm voxel
    let points = vec![
        Point3::new(0.000, 0.000, 0.000),
        Point3::new(0.005, 0.005, 0.005),
        Point3::new(0.010, 0.010, 0.010),
        Point3::new(0.015, 0.015, 0.015),
    ];

    let result = voxel_downsample(&points, 0.02, true, 50_000);
    assert_eq!(
        result.len(),
        1,
        "All points in same voxel should yield 1 point"
    );

    // Expected centroid: (0.0075, 0.0075, 0.0075)
    assert_relative_eq!(result[0].x, 0.0075, epsilon = 1e-10);
    assert_relative_eq!(result[0].y, 0.0075, epsilon = 1e-10);
    assert_relative_eq!(result[0].z, 0.0075, epsilon = 1e-10);
}

#[test]
fn test_points_in_same_voxel_first_point() {
    let points = vec![
        Point3::new(0.000, 0.000, 0.000),
        Point3::new(0.005, 0.005, 0.005),
        Point3::new(0.010, 0.010, 0.010),
    ];

    let result = voxel_downsample(&points, 0.02, false, 50_000);
    assert_eq!(result.len(), 1);
    // First point strategy keeps first point
    assert_relative_eq!(result[0], points[0], epsilon = 1e-10);
}

#[test]
fn test_points_in_different_voxels() {
    // Points in different 2cm voxels
    let points = vec![
        Point3::new(0.00, 0.00, 0.00), // Voxel (0,0,0)
        Point3::new(0.03, 0.03, 0.03), // Voxel (1,1,1)
        Point3::new(0.06, 0.06, 0.06), // Voxel (3,3,3)
    ];

    let result = voxel_downsample(&points, 0.02, true, 50_000);
    assert_eq!(
        result.len(),
        3,
        "Points in different voxels should be preserved"
    );
}

#[test]
fn test_voxel_key_computation() {
    let voxel_size = 0.02;

    // Points in same voxel should have same key
    let key1 = compute_voxel_key(Point3::new(0.000, 0.000, 0.000), voxel_size);
    let key2 = compute_voxel_key(Point3::new(0.015, 0.015, 0.015), voxel_size);
    assert_eq!(key1, key2, "Points in same voxel should have same key");

    // Points in different voxels should have different keys
    let key3 = compute_voxel_key(Point3::new(0.025, 0.025, 0.025), voxel_size);
    assert_ne!(
        key1, key3,
        "Points in different voxels should have different keys"
    );
}

#[test]
fn test_centroid_computation() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
    ];

    let centroid = compute_centroid(&points);
    assert_relative_eq!(centroid.x, 0.25, epsilon = 1e-10);
    assert_relative_eq!(centroid.y, 0.25, epsilon = 1e-10);
    assert_relative_eq!(centroid.z, 0.25, epsilon = 1e-10);
}

#[test]
fn test_negative_coordinates() {
    // Test points in same voxel with negative coordinates
    // Voxel -26 contains range [-0.52, -0.50)
    // -0.52 / 0.02 = -26, floor(-26) = -26
    // -0.51 / 0.02 = -25.5, floor(-25.5) = -26
    let points_same_voxel = vec![
        Point3::new(-0.52, -0.52, -0.52),
        Point3::new(-0.51, -0.51, -0.51),
    ];
    let result = voxel_downsample(&points_same_voxel, 0.02, true, 50_000);
    assert_eq!(
        result.len(),
        1,
        "Negative coordinates in same voxel should work correctly"
    );

    // Test points in different voxels with negative coordinates
    // -0.52 is in voxel -26, -0.54 is in voxel -27
    let points_different_voxels = vec![
        Point3::new(-0.52, -0.52, -0.52),
        Point3::new(-0.54, -0.54, -0.54),
    ];
    let result2 = voxel_downsample(&points_different_voxels, 0.02, true, 50_000);
    assert_eq!(
        result2.len(),
        2,
        "Points in different voxels should remain separate"
    );
}

#[test]
fn test_reduction_ratio() {
    // Create a dense grid of points
    let mut points = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            for k in 0..10 {
                points.push(Point3::new(
                    i as f64 * 0.005,
                    j as f64 * 0.005,
                    k as f64 * 0.005,
                ));
            }
        }
    }
    assert_eq!(points.len(), 1000);

    let result = voxel_downsample(&points, 0.02, true, 50_000);

    // 10x10x10 grid with 5mm spacing, 20mm voxels
    // Should reduce to roughly 3x3x3 = 27 voxels
    assert!(result.len() < 50, "Should significantly reduce point count");
    assert!(
        result.len() > 20,
        "Should preserve reasonable number of points"
    );
}
