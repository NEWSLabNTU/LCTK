#![allow(deprecated)] // Legacy fixture/config contract.

use board_cluster_detector::{
    config::{production_tuning, TargetDetectionParams, TargetSide},
    scorer::*,
};

#[test]
fn min_area_rect_of_axis_square() {
    let sq = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let r = min_area_rect(&sq).unwrap();
    assert!((r.center[0] - 0.5).abs() < 1e-9 && (r.center[1] - 0.5).abs() < 1e-9);
    assert!((r.size[0] - 1.0).abs() < 1e-6 && (r.size[1] - 1.0).abs() < 1e-6);
}

#[test]
fn min_area_rect_of_rotated_square() {
    // 1x1 square rotated 30 degrees, still area ~1, min side ~1
    let th = 30f64.to_radians();
    let (c, s) = (th.cos(), th.sin());
    let base = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let sq: Vec<_> = base
        .iter()
        .map(|p| [p[0] * c - p[1] * s, p[0] * s + p[1] * c])
        .collect();
    let r = min_area_rect(&sq).unwrap();
    assert!(
        (r.size[0] * r.size[1] - 1.0).abs() < 1e-3,
        "area {:?}",
        r.size
    );
}

#[test]
fn seed_center_falls_back_to_centroid_when_wrong_size() {
    let tuning = production_tuning([0.0, 0.0, 1.0], 30);
    let params = TargetDetectionParams::new(TargetSide::metres(1.0).unwrap(), &tuning);
    // a 0.1 m blob -> fails size gate -> centroid fallback
    let blob = vec![[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]];
    let c = seed_center(&blob, &params);
    assert!((c[0] - 0.05).abs() < 1e-9 && (c[1] - 0.05).abs() < 1e-9);
}
