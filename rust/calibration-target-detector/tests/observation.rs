use approx::assert_relative_eq;
use board_cluster_detector::{
    background::BackgroundModel,
    config::{production_tuning, ForegroundMethod, TargetSide},
    detector::{detect_for_target, SquarePlaneObservation},
    geometry::{fit_plane, project_to_plane, voxel_downsample, PlaneModel},
    square_fit::{fit_fixed_square, SquareFit},
};
use calibration_target_detector::TargetSquarePlaneObservation;
use nalgebra::{Point3, Vector3};

fn evidence(normal: Vector3<f64>) -> SquarePlaneObservation {
    let side = 2.0;
    let half = side / 2.0;
    SquarePlaneObservation {
        points: Vec::new(),
        plane: PlaneModel {
            center: Point3::new(0.0, 0.0, 3.0),
            normal,
            u: Vector3::x(),
            v: Vector3::y(),
        },
        square_fit: SquareFit {
            center: [0.25, -0.5],
            theta: 0.0,
            residual: 0.0,
            corners_2d: [
                [0.25 + half, -0.5 + half],
                [0.25 - half, -0.5 + half],
                [0.25 - half, -0.5 - half],
                [0.25 + half, -0.5 - half],
            ],
        },
    }
}

fn observation(sensor_up: Vector3<f64>) -> TargetSquarePlaneObservation {
    TargetSquarePlaneObservation::from_square_plane(&evidence(Vector3::z()), sensor_up).unwrap()
}

fn diamond_square_points(side_m: f64) -> Vec<Point3<f64>> {
    let inv_sqrt_2 = std::f64::consts::FRAC_1_SQRT_2;
    let samples_per_side = 41;
    let mut points = Vec::with_capacity(samples_per_side * samples_per_side);
    for row in 0..samples_per_side {
        for column in 0..samples_per_side {
            let u = side_m * (-0.5 + row as f64 / (samples_per_side - 1) as f64);
            let v = side_m * (-0.5 + column as f64 / (samples_per_side - 1) as f64);
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
fn normal_is_oriented_toward_sensor_regardless_of_plane_svd_sign() {
    for normal in [Vector3::z(), -Vector3::z()] {
        let result =
            TargetSquarePlaneObservation::from_square_plane(&evidence(normal), Vector3::y())
                .unwrap();
        assert_relative_eq!(
            result.sensor_facing_normal.into_inner(),
            -Vector3::z(),
            epsilon = 1e-12
        );
    }
}

#[test]
fn center_and_corners_preserve_square_fit_coordinate_order() {
    let result = observation(Vector3::y());
    assert_relative_eq!(result.center, Point3::new(0.25, -0.5, 3.0), epsilon = 1e-12);
    assert_relative_eq!(
        result.fitted_corners[0],
        Point3::new(1.25, 0.5, 3.0),
        epsilon = 1e-12
    );
    assert_relative_eq!(
        result.fitted_corners[1],
        Point3::new(-0.75, 0.5, 3.0),
        epsilon = 1e-12
    );
    assert_relative_eq!(
        result.fitted_corners[2],
        Point3::new(-0.75, -1.5, 3.0),
        epsilon = 1e-12
    );
    assert_relative_eq!(
        result.fitted_corners[3],
        Point3::new(1.25, -1.5, 3.0),
        epsilon = 1e-12
    );
    for candidate in &result.board_up_candidates {
        assert_eq!(
            candidate.corner,
            result.fitted_corners[candidate.corner_index]
        );
        assert_relative_eq!(
            candidate.board_up.into_inner(),
            (candidate.corner - result.center).normalize(),
            epsilon = 1e-12
        );
    }
}

#[test]
fn candidate_axes_are_right_handed() {
    let result = observation(Vector3::y());
    for candidate in &result.board_up_candidates {
        assert_relative_eq!(
            candidate.x_axis.cross(&candidate.board_up),
            candidate.z_axis.into_inner(),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            candidate.x_axis.dot(&candidate.board_up),
            0.0,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            candidate.board_up.dot(&candidate.z_axis),
            0.0,
            epsilon = 1e-12
        );
    }
}

#[test]
fn alignment_scores_match_corner_up_angles() {
    let base = observation(Vector3::y());
    let up = base.board_up_candidates[0].board_up.into_inner();
    let orthogonal = base.board_up_candidates[1].board_up.into_inner();
    for (angle_degrees, expected) in [
        (0.0, 1.0),
        (22.5, 0.923_879_532_511_286_7),
        (30.0, 0.866_025_403_784_438_6),
    ] as [(f64, f64); 3]
    {
        let angle = angle_degrees.to_radians();
        let sensor_up = up * angle.cos() + orthogonal * angle.sin();
        let result = observation(sensor_up);
        assert_relative_eq!(result.orientation.best_alignment, expected, epsilon = 1e-12);
        assert_eq!(result.orientation.best_candidate_index, 0);
        assert!(!result.orientation.ambiguous);
    }
}

#[test]
fn edge_aligned_sensor_up_is_ambiguous_at_point_707() {
    let base = observation(Vector3::y());
    let up = base.board_up_candidates[0].board_up.into_inner();
    let orthogonal = base.board_up_candidates[1].board_up.into_inner();
    let result = observation((up + orthogonal).normalize());
    assert_relative_eq!(
        result.orientation.best_alignment,
        std::f64::consts::FRAC_1_SQRT_2,
        epsilon = 1e-12
    );
    assert_relative_eq!(
        result.orientation.second_best_alignment,
        std::f64::consts::FRAC_1_SQRT_2,
        epsilon = 1e-12
    );
    assert_relative_eq!(result.orientation.alignment_gap, 0.0, epsilon = 1e-12);
    assert_eq!(result.orientation.best_candidate_index, 0);
    assert!(result.orientation.ambiguous);
}

#[test]
fn bbox_and_bbox_free_handoffs_have_same_observation_semantics() {
    let side_m = 0.6;
    let points = diamond_square_points(side_m);
    let sensor_up = Vector3::new(0.0, 1.0, 0.0);

    // Bbox-style handoff: selected/cropped points are fitted directly, then
    // enter through the explicit plane + known-square constructor.  ROS wiring
    // of this path is intentionally deferred to W4-A.
    let downsampled = voxel_downsample(&points, 0.01);
    let bbox_plane = fit_plane(&downsampled);
    let bbox_coords = project_to_plane(&downsampled, &bbox_plane);
    let bbox_fit = fit_fixed_square(&bbox_coords, side_m, None, None).unwrap();
    let bbox = TargetSquarePlaneObservation::from_fitted_square(&bbox_plane, &bbox_fit, sensor_up)
        .unwrap();

    // Bbox-free handoff: exercise W2-B's complete target-side detector.
    let background = empty_background();
    let outcome = detect_for_target(
        &points,
        TargetSide::metres(side_m).unwrap(),
        &production_tuning([0.0, 0.0, 1.0], 20),
        ForegroundMethod::BackgroundSubtraction,
        0.01,
        Some(&background),
    );
    let bbox_free_evidence = outcome.observation.unwrap();
    let bbox_free =
        TargetSquarePlaneObservation::from_square_plane(&bbox_free_evidence, sensor_up).unwrap();

    assert_relative_eq!(bbox.center, bbox_free.center, epsilon = 1e-9);
    assert_relative_eq!(
        bbox.sensor_facing_normal.into_inner(),
        bbox_free.sensor_facing_normal.into_inner(),
        epsilon = 1e-9
    );
    for (left, right) in bbox.fitted_corners.iter().zip(&bbox_free.fitted_corners) {
        assert_relative_eq!(left, right, epsilon = 1e-9);
    }
    assert_eq!(bbox.orientation, bbox_free.orientation);
    for (left, right) in bbox
        .board_up_candidates
        .iter()
        .zip(&bbox_free.board_up_candidates)
    {
        assert_relative_eq!(
            left.board_up.into_inner(),
            right.board_up.into_inner(),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            left.sensor_up_alignment,
            right.sensor_up_alignment,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            left.x_axis.into_inner(),
            right.x_axis.into_inner(),
            epsilon = 1e-9
        );
    }
}

#[test]
fn invalid_sensor_frame_inputs_are_rejected() {
    let invalid_up =
        TargetSquarePlaneObservation::from_square_plane(&evidence(Vector3::z()), Vector3::zeros());
    assert!(invalid_up.unwrap_err().to_string().contains("sensor_up"));
    let invalid_normal =
        TargetSquarePlaneObservation::from_square_plane(&evidence(Vector3::zeros()), Vector3::y());
    assert!(invalid_normal
        .unwrap_err()
        .to_string()
        .contains("plane.normal"));
}
