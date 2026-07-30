use board_projection_detector::config::{production_config, ForegroundMethod};
use std::str::FromStr;

#[test]
fn production_config_matches_python_preset() {
    let c = production_config(1.0, [0.0, 0.0, 1.0], 30);
    assert_eq!(c.side_m, 1.0);
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
