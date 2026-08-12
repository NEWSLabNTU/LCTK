mod common;
use board_cluster_detector::{candidates::*, config::production_config};
use nalgebra::Point3;

#[test]
fn plausible_patch_accepts_board_rejects_small() {
    let board = production_config(1.0, [0.0, 0.0, 1.0], 30);
    // a ~1 m flat square patch in the x=2 plane
    let mut patch = vec![];
    for i in 0..40 {
        for j in 0..40 {
            patch.push(Point3::new(
                2.0,
                -0.5 + i as f64 * 0.025,
                -0.5 + j as f64 * 0.025,
            ));
        }
    }
    assert!(plausible_board_patch(&patch, &board).is_some());
    let tiny: Vec<_> = patch.iter().take(10).copied().collect();
    assert!(plausible_board_patch(&tiny, &board).is_none());
}

#[test]
fn candidate_parity_against_python() {
    for f in common::load_all() {
        common::assert_candidate_parity(&f);
    }
}
