mod common;
use board_cluster_detector::geometry::*;
use nalgebra::Point3;

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
