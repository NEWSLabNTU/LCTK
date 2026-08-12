//! End-to-end parity: full `detect()` pipeline vs the Python golden vectors.
mod common;

use board_cluster_detector::{config::production_config, detector::detect};

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
