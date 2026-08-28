mod common;
use board_cluster_detector::geometry::*;
use nalgebra::{Point3, Translation3};
use proptest::prelude::*;

#[test]
fn finite_only_drops_non_finite() {
    let pts = vec![
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(f64::NAN, 0.0, 0.0),
        Point3::new(0.0, f64::INFINITY, 0.0),
        Point3::new(4.0, 5.0, 6.0),
    ];
    let out = finite_only(&pts);
    assert_eq!(out.len(), 2);
    assert_eq!(out[0], Point3::new(1.0, 2.0, 3.0));
    assert_eq!(out[1], Point3::new(4.0, 5.0, 6.0));
}

#[test]
fn fit_plane_on_xy_plane_gives_z_normal() {
    // z ~ 0 patch -> normal || +-z, projection preserves x,y extent
    let mut pts = vec![];
    for i in 0..10 {
        for j in 0..10 {
            pts.push(Point3::new(i as f64 * 0.1, j as f64 * 0.1, 0.0));
        }
    }
    let plane = fit_plane(&pts);
    assert!(plane.normal.z.abs() > 0.999, "normal={:?}", plane.normal);
    let coords = project_to_plane(&pts, &plane);
    assert!((extent_2d(&coords) - 0.9).abs() < 1e-6);
    assert!(plane_rms(&pts, &plane) < 1e-9);
}

#[test]
fn unproject_round_trips_project_to_plane() {
    let mut pts = vec![];
    for i in 0..10 {
        for j in 0..10 {
            pts.push(Point3::new(i as f64 * 0.1, j as f64 * 0.1, 0.0));
        }
    }
    let plane = fit_plane(&pts);
    let coords = project_to_plane(&pts, &plane);
    let round_tripped = unproject(&coords, &plane);
    assert_eq!(round_tripped.len(), pts.len());
    for (orig, back) in pts.iter().zip(round_tripped.iter()) {
        assert!((orig.x - back.x).abs() < 1e-9, "x: {orig:?} vs {back:?}");
        assert!((orig.y - back.y).abs() < 1e-9, "y: {orig:?} vs {back:?}");
        assert!((orig.z - back.z).abs() < 1e-9, "z: {orig:?} vs {back:?}");
    }
}

#[test]
fn voxel_downsample_collapses_points_in_same_voxel() {
    let pts = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.01, 0.01, 0.01)];
    let out = voxel_downsample(&pts, 0.03);
    assert_eq!(out.len(), 1);
    assert!((out[0].x - 0.005).abs() < 1e-12);
    assert!((out[0].y - 0.005).abs() < 1e-12);
    assert!((out[0].z - 0.005).abs() < 1e-12);
}

#[test]
fn voxel_downsample_keeps_points_in_different_voxels() {
    let pts = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)];
    let out = voxel_downsample(&pts, 0.03);
    assert_eq!(out.len(), 2);
}

// Ported from the now-deleted `rust/hollow-board-detector/tests/test_voxel_downsample.rs`
// (Phase 8 packet W5-E2). This crate's `voxel_downsample` is the live bbox-free-path
// implementation; the old crate's `algo::voxel_downsample` it originally targeted is dead.

#[test]
fn voxel_downsample_empty_input_returns_empty() {
    let points: Vec<Point3<f64>> = vec![];
    let result = voxel_downsample(&points, 0.02);
    assert_eq!(result.len(), 0, "Empty input should return empty output");
}

#[test]
fn voxel_downsample_single_point_unchanged() {
    let points = vec![Point3::new(1.0, 2.0, 3.0)];
    let result = voxel_downsample(&points, 0.02);
    assert_eq!(result.len(), 1);
    assert!((result[0] - points[0]).norm() < 1e-10);
}

#[test]
fn voxel_downsample_negative_coordinates_floor_semantics() {
    // Voxel -26 contains range [-0.52, -0.50):
    // -0.52 / 0.02 = -26,   floor(-26)   = -26
    // -0.51 / 0.02 = -25.5, floor(-25.5) = -26
    // So both points fall in the same negative voxel -- a classic sign-bug guard for
    // `floor()`-based keys, where a naive truncating cast would put -25.5 in voxel -25
    // instead of -26.
    let points_same_voxel = vec![
        Point3::new(-0.52, -0.52, -0.52),
        Point3::new(-0.51, -0.51, -0.51),
    ];
    let result = voxel_downsample(&points_same_voxel, 0.02);
    assert_eq!(
        result.len(),
        1,
        "Negative coordinates in same voxel should work correctly"
    );

    // -0.52 is in voxel -26, -0.54 is in voxel -27: adjacent but distinct voxels.
    let points_different_voxels = vec![
        Point3::new(-0.52, -0.52, -0.52),
        Point3::new(-0.54, -0.54, -0.54),
    ];
    let result2 = voxel_downsample(&points_different_voxels, 0.02);
    assert_eq!(
        result2.len(),
        2,
        "Points in different voxels should remain separate"
    );
}

#[test]
fn voxel_downsample_reduction_ratio_on_dense_grid() {
    // Dense 10x10x10 grid with 5mm spacing, downsampled with 20mm voxels.
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

    let result = voxel_downsample(&points, 0.02);

    // Should reduce to roughly 3x3x3 = 27 voxels.
    assert!(result.len() < 50, "Should significantly reduce point count");
    assert!(
        result.len() > 20,
        "Should preserve reasonable number of points"
    );
}

// Property-based coverage, ported from the now-deleted
// `rust/hollow-board-detector/tests/test_voxel_properties.rs`. There was previously no
// property-based coverage of either live `voxel_downsample`; this crate's centroid-only
// implementation is the natural home since all four properties were exercised against
// `use_centroid = true` originally.
proptest! {
    #[test]
    fn prop_output_size_bounded(
        num_points in 1usize..1000,
        voxel_size in 0.01f64..0.1f64
    ) {
        let points: Vec<Point3<f64>> = (0..num_points)
            .map(|i| Point3::new(i as f64 * 0.001, 0.0, 0.0))
            .collect();

        let result = voxel_downsample(&points, voxel_size);

        // Output size must be <= input size.
        prop_assert!(result.len() <= points.len());
        // Output must be non-empty if input is non-empty.
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

        let mut result1 = voxel_downsample(&points, voxel_size);
        let mut result2 = voxel_downsample(&points, voxel_size);

        // Same input should always produce same output. Note: unlike the original
        // AHashMap-backed implementation this property described, this crate's
        // `voxel_downsample` is BTreeMap-keyed, so the two calls are already emitted
        // in identical order without the sort below -- this test no longer catches
        // an ordering bug the way it originally did. It is kept (with the sort, and
        // an explicit value comparison rather than just a length check) as a guard
        // against a future swap back to an unordered map.
        prop_assert_eq!(result1.len(), result2.len());

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
        // NOTE: this property does not hold for a `floor()`-keyed grid in general --
        // translating a cloud can shift points across voxel boundaries and change how
        // many distinct voxels they occupy. It holds here only because these three
        // fixture points are spaced 0.1 apart while `voxel_size` is drawn from
        // 0.01..0.1 (strictly less than the spacing), which guarantees each point
        // keeps its own voxel along every axis regardless of translation. This is the
        // literal property the original test asserted (equal *counts*, not equal
        // point sets) -- ported as-is, not strengthened or weakened.
        let points: Vec<Point3<f64>> = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(0.2, 0.2, 0.2),
        ];

        // Translate points.
        let translation = Translation3::new(dx, dy, dz);
        let translated: Vec<Point3<f64>> = points.iter()
            .map(|p| translation.transform_point(p))
            .collect();

        let result1 = voxel_downsample(&points, voxel_size);
        let result2 = voxel_downsample(&translated, voxel_size);

        // Translation should not affect point count (for this fixture / voxel_size range).
        prop_assert_eq!(result1.len(), result2.len());
    }

    #[test]
    fn prop_scale_relationship(
        voxel_size in 0.01f64..0.05f64
    ) {
        let points: Vec<Point3<f64>> = (0..100)
            .map(|i| Point3::new(i as f64 * 0.01, 0.0, 0.0))
            .collect();

        let result_small = voxel_downsample(&points, voxel_size);
        let result_large = voxel_downsample(&points, voxel_size * 2.0);

        // Larger voxels should result in fewer or equal points.
        prop_assert!(result_large.len() <= result_small.len());
    }
}
