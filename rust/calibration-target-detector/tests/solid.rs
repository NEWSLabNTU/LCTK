use board_cluster_detector::{
    detector::SquarePlaneObservation, geometry::PlaneModel, square_fit::SquareFit,
};
use calibration_target::ValidatedTarget;
use calibration_target_detector::{
    SolidRefinementTuning, TargetDetectionDiagnostics, TargetPoseEstimate, TargetPoseEstimator,
    TargetPoseEstimatorTuning, TargetRejectReason, TargetSquarePlaneObservation,
};
use nalgebra::{Point3, Vector3};

const SOLID: &str = r#"{
  schema_version: 1, target_id: "solid_600_aruco_1", revision: 1,
  board_frame_convention: "corner_aligned_plate_center_v1",
  plate: { side: "600mm", surface: { kind: "solid" } },
  fiducial: { kind: "square_aruco_grid", dictionary: "DICT_5X5_1000", marker_ids: [1],
    paper_side: "600mm", paper_center: { toward_left_corner: "0mm", toward_top_corner: "0mm" },
    outer_border: "60mm", cells_per_side: 1, marker_fill_ratio: 1.0, border_bits: 1 },
  lidar_orientation_reference: { kind: "mounting_up", local_axis: "+y" },
}"#;

fn target() -> ValidatedTarget {
    ValidatedTarget::parse_json5(SOLID.as_bytes()).unwrap()
}

fn solid_tuning() -> SolidRefinementTuning {
    SolidRefinementTuning::new(0.015, 8, 1, 3, 4, 2)
}

fn estimator(tuning: SolidRefinementTuning) -> TargetPoseEstimator {
    TargetPoseEstimator::new(&target(), TargetPoseEstimatorTuning::for_solid(tuning)).unwrap()
}

fn observation(sensor_up: Vector3<f64>) -> TargetSquarePlaneObservation {
    observation_with_square_fit(sensor_up, [0.0, 0.0], 0.3)
}

fn observation_with_square_fit(
    sensor_up: Vector3<f64>,
    center: [f64; 2],
    half: f64,
) -> TargetSquarePlaneObservation {
    let evidence = SquarePlaneObservation {
        points: Vec::new(),
        plane: PlaneModel {
            center: Point3::new(0.0, 0.0, 3.0),
            normal: Vector3::z(),
            u: Vector3::x(),
            v: Vector3::y(),
        },
        square_fit: SquareFit {
            center,
            theta: 0.0,
            residual: 0.0,
            corners_2d: [
                [center[0] + half, center[1] + half],
                [center[0] - half, center[1] + half],
                [center[0] - half, center[1] - half],
                [center[0] + half, center[1] - half],
            ],
        },
    };
    TargetSquarePlaneObservation::from_square_plane(&evidence, sensor_up).unwrap()
}

fn sensor_up_at_angle_from_first_corner(degrees: f64) -> Vector3<f64> {
    let radians = (45.0 + degrees).to_radians();
    Vector3::new(radians.cos(), radians.sin(), 0.0)
}

fn point_on_edge(
    observation: &TargetSquarePlaneObservation,
    edge: usize,
    fraction: f64,
) -> Point3<f64> {
    let start = observation.fitted_corners[edge];
    start + (observation.fitted_corners[(edge + 1) % 4] - start) * fraction
}

fn edge_points(
    observation: &TargetSquarePlaneObservation,
    edges: &[usize],
    fractions: &[f64],
) -> Vec<Point3<f64>> {
    edges
        .iter()
        .flat_map(|&edge| {
            fractions
                .iter()
                .map(move |&fraction| point_on_edge(observation, edge, fraction))
        })
        .collect()
}

fn detected(estimate: TargetPoseEstimate) -> calibration_target_detector::TargetDetection {
    match estimate {
        TargetPoseEstimate::Detected(detection) => *detection,
        TargetPoseEstimate::Rejected(rejection) => panic!("expected detection, got {rejection:?}"),
    }
}

fn rejected(estimate: TargetPoseEstimate) -> calibration_target_detector::TargetRejection {
    match estimate {
        TargetPoseEstimate::Rejected(rejection) => *rejection,
        TargetPoseEstimate::Detected(detection) => panic!("expected rejection, got {detection:?}"),
    }
}

#[test]
fn facade_accepts_solid_without_exposing_a_solid_estimator() {
    let estimator = estimator(solid_tuning());
    let observation = observation(Vector3::new(1.0, 1.0, 0.0).normalize());
    let expected_quadrant = observation.orientation.best_candidate_index;
    let detection = detected(estimator.estimate(
        observation.clone(),
        edge_points(&observation, &[0, 1, 2, 3], &[0.25, 0.75]),
    ));
    assert_eq!(detection.selected_quadrant, expected_quadrant);
    assert_eq!(detection.target_identity.target_id, "solid_600_aruco_1");
    assert!(matches!(
        detection.diagnostics,
        TargetDetectionDiagnostics::Solid(_)
    ));
}

#[test]
fn facade_returns_structured_solid_rejections() {
    let rejection = rejected(
        estimator(solid_tuning()).estimate(observation(Vector3::new(1.0, 0.0, 0.0)), Vec::new()),
    );
    assert!(matches!(
        rejection.reason,
        TargetRejectReason::BoardUpAlignment { .. }
    ));
}

#[test]
fn exact_square_accepts_and_keeps_fitted_pose() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let detection = detected(estimator(solid_tuning()).estimate(
        observation.clone(),
        edge_points(&observation, &[0, 1, 2, 3], &[0.2, 0.8]),
    ));
    assert_eq!(detection.selected_quadrant, 0);
    assert_eq!(detection.pose.translation.vector, observation.center.coords);
}

#[test]
fn noisy_square_and_outliers_accept_without_changing_quadrant() {
    let observation = observation_with_square_fit(
        sensor_up_at_angle_from_first_corner(0.0),
        [0.04, -0.03],
        0.296,
    );
    let mut points = edge_points(&observation, &[0, 1, 2, 3], &[0.2, 0.8]);
    for (index, point) in points.iter_mut().enumerate() {
        point.z += if index % 2 == 0 { 0.008 } else { -0.008 };
    }
    points.extend([
        Point3::new(4.0, -3.0, 0.0),
        Point3::new(-2.0, 1.0, 5.0),
        Point3::new(f64::NAN, 0.0, 0.0),
    ]);
    let detection = detected(estimator(solid_tuning()).estimate(observation.clone(), points));
    assert_eq!(
        detection.selected_quadrant,
        observation.orientation.best_candidate_index
    );
}

#[test]
fn interior_only_returns_insufficient_outer_edge_evidence() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let rejection = rejected(
        estimator(solid_tuning()).estimate(observation, vec![Point3::new(0.0, 0.0, 3.0); 24]),
    );
    let TargetRejectReason::InsufficientOuterEdgeEvidence { evidence } = rejection.reason else {
        panic!("expected outer-edge rejection")
    };
    assert_eq!(evidence.edge_point_count, 0);
    assert_eq!(evidence.edge_point_counts, [0; 4]);
    assert_eq!(evidence.covered_edge_count, 0);
    assert_eq!(evidence.occupied_longitudinal_bins, [0; 4]);
    assert!((evidence.board_up_alignment - 1.0).abs() < 1e-12);
    assert_eq!(evidence.minimum_edge_points, 8);
    assert_eq!(evidence.minimum_covered_edges, 3);
    assert!(evidence.weak_in_plane_center);
    assert!(evidence.weak_yaw);
}

#[test]
fn orientation_gate_accepts_22_point_5_degrees_and_rejects_30_degrees() {
    let pass = observation(sensor_up_at_angle_from_first_corner(22.5));
    detected(
        estimator(solid_tuning())
            .estimate(pass.clone(), edge_points(&pass, &[0, 1, 2, 3], &[0.2, 0.8])),
    );
    let fail = observation(sensor_up_at_angle_from_first_corner(30.0));
    let rejection = rejected(
        estimator(solid_tuning())
            .estimate(fail.clone(), edge_points(&fail, &[0, 1, 2, 3], &[0.2, 0.8])),
    );
    assert!(matches!(
        rejection.reason,
        TargetRejectReason::BoardUpAlignment { .. }
    ));
}

#[test]
fn repeated_single_edge_and_corner_evidence_reject() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let rejection = rejected(estimator(solid_tuning()).estimate(
        observation.clone(),
        vec![point_on_edge(&observation, 0, 0.5); 16],
    ));
    assert!(matches!(
        rejection.reason,
        TargetRejectReason::InsufficientOuterEdgeEvidence { .. }
    ));
    let rejection = rejected(
        estimator(solid_tuning())
            .estimate(observation.clone(), vec![observation.fitted_corners[0]; 16]),
    );
    assert!(matches!(
        rejection.reason,
        TargetRejectReason::InsufficientOuterEdgeEvidence { .. }
    ));
}

#[test]
fn clustered_two_corner_evidence_rejects() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let mut points = Vec::new();
    for fraction in [0.01, 0.02, 0.03, 0.04] {
        points.push(point_on_edge(&observation, 0, fraction));
        points.push(point_on_edge(&observation, 1, 1.0 - fraction));
    }
    let rejection = rejected(estimator(solid_tuning()).estimate(observation, points));
    assert!(matches!(
        rejection.reason,
        TargetRejectReason::InsufficientOuterEdgeEvidence { .. }
    ));
}

#[test]
fn three_edges_are_accepted_but_report_weak_in_plane_observability() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let detection = detected(estimator(solid_tuning()).estimate(
        observation.clone(),
        edge_points(&observation, &[0, 1, 2], &[0.15, 0.5, 0.85]),
    ));
    let TargetDetectionDiagnostics::Solid(diagnostics) = detection.diagnostics else {
        panic!("expected solid diagnostics");
    };
    assert_eq!(diagnostics.covered_edge_count, 3);
    assert!(diagnostics.weak_in_plane_center);
    assert!(diagnostics.weak_yaw);
}

#[test]
fn clustered_fourth_edge_remains_weak_in_diagnostics() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let mut points = edge_points(&observation, &[0, 1, 2], &[0.2, 0.8]);
    points.extend([point_on_edge(&observation, 3, 0.5); 3]);
    let detection = detected(estimator(solid_tuning()).estimate(observation, points));
    let TargetDetectionDiagnostics::Solid(diagnostics) = detection.diagnostics else {
        panic!("expected solid diagnostics");
    };
    assert_eq!(diagnostics.covered_edge_count, 3);
    assert_eq!(diagnostics.edge_point_counts[3], 3);
    assert_eq!(diagnostics.occupied_longitudinal_bins[3], 1);
    assert!(diagnostics.weak_in_plane_center);
    assert!(diagnostics.weak_yaw);
}

#[test]
fn independent_bins_on_all_edges_report_strong_coverage() {
    let observation = observation(sensor_up_at_angle_from_first_corner(0.0));
    let detection = detected(estimator(solid_tuning()).estimate(
        observation.clone(),
        edge_points(&observation, &[0, 1, 2, 3], &[0.2, 0.8]),
    ));
    let TargetDetectionDiagnostics::Solid(diagnostics) = detection.diagnostics else {
        panic!("expected solid diagnostics");
    };
    assert_eq!(diagnostics.covered_edge_count, 4);
    assert_eq!(diagnostics.occupied_longitudinal_bins, [2; 4]);
    assert!(!diagnostics.weak_in_plane_center);
    assert!(!diagnostics.weak_yaw);
}

#[test]
fn invalid_solid_tuning_is_rejected_at_facade_construction() {
    for invalid in [
        SolidRefinementTuning::new(0.0, 8, 1, 3, 4, 2),
        SolidRefinementTuning::new(0.015, 0, 1, 3, 4, 2),
        SolidRefinementTuning::new(0.015, 8, 0, 3, 4, 2),
        SolidRefinementTuning::new(0.015, 8, 1, 2, 4, 2),
        SolidRefinementTuning::new(0.015, 8, 1, 3, 1, 2),
        SolidRefinementTuning::new(0.015, 8, 1, 3, 4, 5),
    ] {
        assert!(
            TargetPoseEstimator::new(&target(), TargetPoseEstimatorTuning::for_solid(invalid))
                .is_err()
        );
    }
}
