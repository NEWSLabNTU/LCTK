use hollow_board_detector::algo::voxel_downsample;
use nalgebra::{Point3, Translation3};
use proptest::prelude::*;

proptest! {
    #[test]
    fn prop_output_size_bounded(
        num_points in 1usize..1000,
        voxel_size in 0.01f64..0.1f64
    ) {
        let points: Vec<Point3<f64>> = (0..num_points)
            .map(|i| Point3::new(i as f64 * 0.001, 0.0, 0.0))
            .collect();

        let result = voxel_downsample(&points, voxel_size, true, 50_000);

        // Output size must be <= input size
        prop_assert!(result.len() <= points.len());
        // Output must be non-empty if input is non-empty
        prop_assert!(!result.is_empty());
    }

    #[test]
    fn prop_deterministic(
        seed in 0u64..1000,
        voxel_size in 0.01f64..0.1f64
    ) {
        use rand::{SeedableRng, rngs::StdRng, Rng};
        let mut rng = StdRng::seed_from_u64(seed);

        let points: Vec<Point3<f64>> = (0..100)
            .map(|_| Point3::new(
                rng.gen::<f64>() * 10.0,
                rng.gen::<f64>() * 10.0,
                rng.gen::<f64>() * 10.0,
            ))
            .collect();

        let mut result1 = voxel_downsample(&points, voxel_size, true, 50_000);
        let mut result2 = voxel_downsample(&points, voxel_size, true, 50_000);

        // Same input should always produce same output (but order may vary)
        prop_assert_eq!(result1.len(), result2.len());

        // Sort results for comparison (HashMap doesn't guarantee order)
        result1.sort_by(|a, b| {
            a.x.partial_cmp(&b.x)
                .unwrap()
                .then(a.y.partial_cmp(&b.y).unwrap())
                .then(a.z.partial_cmp(&b.z).unwrap())
        });
        result2.sort_by(|a, b| {
            a.x.partial_cmp(&b.x)
                .unwrap()
                .then(a.y.partial_cmp(&b.y).unwrap())
                .then(a.z.partial_cmp(&b.z).unwrap())
        });

        for (p1, p2) in result1.iter().zip(result2.iter()) {
            prop_assert!((p1 - p2).norm() < 1e-10);
        }
    }

    #[test]
    fn prop_translation_invariance(
        dx in -10.0f64..10.0,
        dy in -10.0f64..10.0,
        dz in -10.0f64..10.0,
        voxel_size in 0.01f64..0.1f64
    ) {
        // Create test points
        let points: Vec<Point3<f64>> = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(0.2, 0.2, 0.2),
        ];

        // Translate points
        let translation = Translation3::new(dx, dy, dz);
        let translated: Vec<Point3<f64>> = points.iter()
            .map(|p| translation.transform_point(p))
            .collect();

        let result1 = voxel_downsample(&points, voxel_size, true, 50_000);
        let result2 = voxel_downsample(&translated, voxel_size, true, 50_000);

        // Translation should not affect point count
        prop_assert_eq!(result1.len(), result2.len());
    }

    #[test]
    fn prop_scale_relationship(
        voxel_size in 0.01f64..0.05f64
    ) {
        let points: Vec<Point3<f64>> = (0..100)
            .map(|i| Point3::new(i as f64 * 0.01, 0.0, 0.0))
            .collect();

        let result_small = voxel_downsample(&points, voxel_size, true, 50_000);
        let result_large = voxel_downsample(&points, voxel_size * 2.0, true, 50_000);

        // Larger voxels should result in fewer or equal points
        prop_assert!(result_large.len() <= result_small.len());
    }
}
