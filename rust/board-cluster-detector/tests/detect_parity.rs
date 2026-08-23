//! End-to-end parity: full `detect()` pipeline vs the Python golden vectors.
#![allow(deprecated)] // Legacy decision-parity contract.

mod common;

use approx::assert_relative_eq;
use board_cluster_detector::{
    config::{production_config, production_tuning, BoardConfig, TargetSide},
    detector::{detect, detect_for_target},
};

#[test]
fn real_one_metre_fixture_has_equivalent_neutral_and_compatibility_evidence() {
    let fixture = common::load_all()
        .into_iter()
        .find(|fixture| fixture.name == "ds5_f0034_ba")
        .expect("curated real 1 m fixture");
    let (method, background) = common::method_and_background(&fixture);
    let mut tuning = production_tuning(fixture.golden.up_axis, fixture.golden.cluster_min_points);
    // Remove only the compatibility-only orientation gate. This makes the
    // facade's decision boundary identical to neutral clustering while still
    // exercising the actual deprecated delegation path.
    tuning.stance_floor = 0.0;
    let board = BoardConfig::new(1.0, tuning.clone());

    let neutral = detect_for_target(
        &fixture.input,
        TargetSide::metres(1.0).unwrap(),
        &tuning,
        method,
        fixture.golden.voxel,
        background.as_ref(),
    );
    let compatibility = detect(
        &fixture.input,
        &board,
        method,
        fixture.golden.voxel,
        background.as_ref(),
    );

    assert_eq!(neutral.n_candidates, compatibility.n_candidates);
    assert_eq!(neutral.reject, compatibility.reject);
    assert_eq!(neutral.foreground_points, compatibility.foreground_points);
    let neutral_observation = neutral.observation.expect("neutral real-fixture evidence");
    let compatibility_observation = compatibility
        .observation
        .expect("compatibility real-fixture evidence");
    assert_eq!(neutral_observation.points, compatibility_observation.points);
    assert_relative_eq!(
        neutral_observation.plane.center,
        compatibility_observation.plane.center,
        epsilon = f64::EPSILON
    );
    assert_relative_eq!(
        neutral_observation.plane.normal,
        compatibility_observation.plane.normal,
        epsilon = f64::EPSILON
    );
    assert_relative_eq!(
        neutral_observation.plane.u,
        compatibility_observation.plane.u,
        epsilon = f64::EPSILON
    );
    assert_relative_eq!(
        neutral_observation.plane.v,
        compatibility_observation.plane.v,
        epsilon = f64::EPSILON
    );
    assert_relative_eq!(
        neutral_observation.square_fit.residual,
        compatibility_observation.square_fit.residual,
        epsilon = f64::EPSILON
    );
    assert_eq!(
        neutral_observation.square_fit.corners_2d,
        compatibility_observation.square_fit.corners_2d
    );
}

/// Per-frame detect/no-detect regression guard (NOT bit-exact parity — see
/// `common::KNOWN_PER_FRAME_MISMATCHES`). Asserts every per-frame divergence
/// from the Python golden is one of the documented, RNG-driven known mismatches;
/// a new divergence (a fixture that starts mismatching, or a known one that
/// changes class) fails the test. For frames that both match AND detect, also
/// checks the fitted board center is within 2 cm of the golden centroid.
#[test]
fn per_frame_detection_decision_matches_python() {
    let mut unexpected = vec![];
    for f in common::load_all() {
        let board = production_config(1.0, f.golden.up_axis, f.golden.cluster_min_points);
        let (method, bg) = common::method_and_background(&f);
        let out = detect(&f.input, &board, method, f.golden.voxel, bg.as_ref());
        if out.detection.is_some() != f.golden.detected {
            if !common::KNOWN_PER_FRAME_MISMATCHES.contains(&f.name.as_str()) {
                unexpected.push(format!(
                    "{}: rust={} python={}",
                    f.name,
                    out.detection.is_some(),
                    f.golden.detected
                ));
            }
            continue;
        }
        if let (Some(d), Some(sc)) = (&out.detection, &f.golden.selected_centroid) {
            let c = d.center;
            let dist =
                ((c.x - sc[0]).powi(2) + (c.y - sc[1]).powi(2) + (c.z - sc[2]).powi(2)).sqrt();
            assert!(dist < 0.02, "{}: centroid off {dist:.3} m", f.name);
        }
    }
    assert!(
        unexpected.is_empty(),
        "NEW per-frame divergence outside the documented allowlist (regression): {unexpected:?}"
    );
}

#[test]
fn recall_precision_parity_per_dataset() {
    common::assert_recall_precision_parity(&common::load_all(), |f| {
        let board = production_config(1.0, f.golden.up_axis, f.golden.cluster_min_points);
        let (m, bg) = common::method_and_background(f);
        detect(&f.input, &board, m, f.golden.voxel, bg.as_ref())
            .detection
            .is_some()
    });
}
