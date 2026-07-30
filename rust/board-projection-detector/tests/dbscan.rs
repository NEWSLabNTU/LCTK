use board_projection_detector::dbscan::*;
use nalgebra::Point3;

#[test]
fn two_separated_blobs_get_two_labels() {
    let mut pts = vec![];
    for i in 0..40 {
        pts.push(Point3::new(
            0.0 + (i % 5) as f64 * 0.01,
            (i / 5) as f64 * 0.01,
            0.0,
        ));
    }
    for i in 0..40 {
        pts.push(Point3::new(
            5.0 + (i % 5) as f64 * 0.01,
            (i / 5) as f64 * 0.01,
            0.0,
        ));
    }
    let labels = dbscan(&pts, 0.05, 5);
    let uniq: std::collections::BTreeSet<_> = labels.iter().filter(|&&l| l >= 0).collect();
    assert_eq!(uniq.len(), 2);
    assert_ne!(labels[0], labels[79]);
}

#[test]
fn anisotropic_scaling_compresses_z_with_range() {
    // far point: z scaled DOWN so ring gaps merge; near point ~unchanged
    let near = Point3::new(0.5, 0.0, 1.0);
    let far = Point3::new(20.0, 0.0, 1.0);
    let out = anisotropic_scaled(&[near, far], 0.15, 3.0);
    assert!((out[0].z - 1.0).abs() < 1e-6, "near z unchanged");
    assert!(out[1].z < 0.5, "far z compressed, got {}", out[1].z);
}
