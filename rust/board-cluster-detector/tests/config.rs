#![allow(deprecated)] // Exercises the migration facade intentionally.

use board_cluster_detector::config::{
    production_config, production_tuning, DetectorTuning, ForegroundMethod,
};
use std::str::FromStr;

#[test]
fn production_config_matches_python_preset() {
    let c = production_config(1.0, [0.0, 0.0, 1.0], 30);
    assert_eq!(c.side_m(), 1.0);
    assert_eq!(c.up_axis, [0.0, 0.0, 1.0]);
    assert_eq!(c.cluster_min_points, 30);
    assert_eq!(c.stance_floor, 0.9);
    assert!(c.isolation);
    assert_eq!(c.flatness_rms_max, 0.045);
    assert_eq!(c.square_icp_residual_max, 0.45);
    assert_eq!(c.side_tol, 0.20);
    assert_eq!(c.cell_m, 0.02);
    assert_eq!(c.vertical_gap_deg, 3.0);
    assert_eq!(c.isolation_max_density, 0.3);
}

#[test]
fn production_tuning_reuses_serde_defaults_except_documented_overrides() {
    let defaults: DetectorTuning = json5::from_str("{}").unwrap();
    let production = production_tuning([0.0, 0.0, 1.0], 30);

    assert_eq!(production.cluster_eps, defaults.cluster_eps);
    assert_eq!(production.side_tol, defaults.side_tol);
    assert_eq!(production.cell_m, defaults.cell_m);
    assert_eq!(production.vertical_gap_deg, defaults.vertical_gap_deg);
    assert_eq!(
        production.square_icp_residual_max,
        defaults.square_icp_residual_max
    );
    assert_eq!(
        production.isolation_max_density,
        defaults.isolation_max_density
    );
    assert_eq!(production.strip_plane_dist, defaults.strip_plane_dist);
    assert_eq!(
        production.strip_plane_min_frac,
        defaults.strip_plane_min_frac
    );
    assert_eq!(
        production.merge_seed_min_points,
        defaults.merge_seed_min_points
    );
    assert_eq!(production.merge_offset_tol, defaults.merge_offset_tol);
    assert_eq!(production.merge_dist_factor, defaults.merge_dist_factor);
    assert_eq!(production.patch_min_points, defaults.patch_min_points);
    assert_eq!(
        production.patch_extent_lo_frac,
        defaults.patch_extent_lo_frac
    );
    assert_eq!(
        production.patch_extent_hi_diag_frac,
        defaults.patch_extent_hi_diag_frac
    );
    assert_eq!(
        production.isolation_coplanar_tol,
        defaults.isolation_coplanar_tol
    );
    assert_eq!(production.isolation_band_lo, defaults.isolation_band_lo);
    assert_eq!(production.isolation_band_hi, defaults.isolation_band_hi);

    assert_eq!(production.flatness_rms_max, 0.045);
    assert_eq!(production.stance_floor, 0.9);
    assert!(production.isolation);
}

#[test]
fn foreground_method_from_str() {
    assert!(matches!(
        ForegroundMethod::from_str("plane_strip"),
        Ok(ForegroundMethod::PlaneStrip)
    ));
    assert!(matches!(
        ForegroundMethod::from_str("background_subtraction"),
        Ok(ForegroundMethod::BackgroundSubtraction)
    ));
    assert!(ForegroundMethod::from_str("bogus").is_err());
}
