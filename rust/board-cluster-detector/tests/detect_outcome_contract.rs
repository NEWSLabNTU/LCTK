//! Self-contained characterization of the public bbox-free detector outcome
//! before target-side observations are introduced.

use approx::assert_relative_eq;
use board_cluster_detector::{
    background::BackgroundModel,
    config::{production_config, ForegroundMethod},
    detector::{detect, RejectReason},
};
use nalgebra::Point3;

/// Dense 1 m square in the x=2 plane, rotated 45 degrees in y-z so one
/// diagonal is aligned with sensor up. Filling the face approximates the
/// LiDAR evidence from the legacy hollow plate without external fixtures.
fn diamond_square_points() -> Vec<Point3<f64>> {
    let inv_sqrt_2 = 1.0 / 2.0_f64.sqrt();
    let samples_per_side = 41;
    let mut points = Vec::with_capacity(samples_per_side * samples_per_side);

    for row in 0..samples_per_side {
        for column in 0..samples_per_side {
            let u = -0.5 + row as f64 / (samples_per_side - 1) as f64;
            let v = -0.5 + column as f64 / (samples_per_side - 1) as f64;
            points.push(Point3::new(2.0, (u - v) * inv_sqrt_2, (u + v) * inv_sqrt_2));
        }
    }

    points
}

fn empty_background() -> BackgroundModel {
    let mut background = BackgroundModel::new(0.01, 0, 1);
    background.finalize();
    background
}

#[test]
fn accepted_one_metre_diamond_exposes_selected_patch_plane_and_pose() {
    let points = diamond_square_points();
    let board = production_config(1.0, [0.0, 0.0, 1.0], 20);
    let background = empty_background();

    let outcome = detect(
        &points,
        &board,
        ForegroundMethod::BackgroundSubtraction,
        0.01,
        Some(&background),
    );

    let detection = outcome
        .detection
        .expect("synthetic 1 m diamond must pass current bbox-free gates");
    let selected_points = outcome
        .selected_points
        .expect("an accepted outcome exposes the selected raw patch");
    let selected_plane = outcome
        .selected_plane
        .expect("an accepted outcome exposes the selected patch plane");

    assert_eq!(outcome.n_candidates, 1);
    assert_eq!(selected_points.len(), outcome.foreground_points.len());
    assert!(selected_points.len() >= board.patch_min_points);
    assert!(outcome.reject.is_none());
    assert!(outcome.reject_detail.is_none());
    assert!(outcome.rejected_cluster.is_empty());

    assert_relative_eq!(selected_plane.center.x, 2.0, epsilon = 1e-12);
    assert_relative_eq!(detection.center.x, 2.0, epsilon = 1e-12);
    for index in 0..4 {
        let next = (index + 1) % 4;
        assert_relative_eq!(
            (detection.corners_3d[next] - detection.corners_3d[index]).norm(),
            1.0,
            epsilon = 1e-9
        );
    }
}

#[test]
fn post_candidate_rejection_exposes_rejected_cluster_but_no_selection() {
    let points = diamond_square_points();
    let mut board = production_config(1.0, [0.0, 0.0, 1.0], 20);
    // No physical stance can exceed 1.0, so this forces rejection after the
    // candidate and 1 m square-fit stages without changing their evidence.
    board.stance_floor = 1.01;
    let background = empty_background();

    let outcome = detect(
        &points,
        &board,
        ForegroundMethod::BackgroundSubtraction,
        0.01,
        Some(&background),
    );

    assert_eq!(outcome.n_candidates, 1);
    assert!(outcome.detection.is_none());
    assert!(outcome.selected_points.is_none());
    assert!(outcome.selected_plane.is_none());
    assert_eq!(outcome.reject, Some(RejectReason::Stance));

    let detail = outcome
        .reject_detail
        .expect("post-candidate rejection exposes its measured gate evidence");
    assert!(detail.measured <= 1.0);
    assert_relative_eq!(detail.threshold, 1.01, epsilon = f64::EPSILON);
    assert_eq!(
        outcome.rejected_cluster.len(),
        outcome.foreground_points.len()
    );
    assert!(!outcome.rejected_cluster.is_empty());
}
