use board_cluster_detector::{geometry::PlaneModel, square_fit::SquareFit};
use calibration_target::{Surface, ValidatedTarget};
use calibration_target_detector::{
    PerforatedIcpConfig, TargetDetectionDiagnostics, TargetPoseEstimate, TargetPoseEstimator,
    TargetPoseEstimatorTuning, TargetRejectReason, TargetSquarePlaneObservation,
};
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

const HOLLOW: &[u8] = include_bytes!("../../../fixtures/targets/hollow_1000_aruco_4_v1.json5");

fn target() -> ValidatedTarget {
    ValidatedTarget::parse_json5(HOLLOW).unwrap()
}

fn tuning() -> PerforatedIcpConfig {
    PerforatedIcpConfig::new(40, 0.2, 0.5, 1e-8, 1e-8, 0.005, 3, 0.0001, 1, 1e-5)
}

fn observation() -> TargetSquarePlaneObservation {
    let plane = PlaneModel {
        center: Point3::new(0.0, 0.0, 3.0),
        normal: Vector3::z(),
        u: Vector3::x(),
        v: Vector3::y(),
    };
    let half = 1.0 / std::f64::consts::SQRT_2;
    let square = SquareFit {
        center: [0.0, 0.0],
        theta: 0.0,
        residual: 0.0,
        corners_2d: [[half, 0.0], [0.0, half], [-half, 0.0], [0.0, -half]],
    };
    TargetSquarePlaneObservation::from_fitted_square(&plane, &square, Vector3::y()).unwrap()
}

fn samples(target: &ValidatedTarget, pose: Isometry3<f64>) -> Vec<Point3<f64>> {
    let Surface::Perforated { circular_cutouts } = &target.plate().surface else {
        unreachable!()
    };
    let mut local = Vec::new();
    for xi in -16..=16 {
        for yi in -16..=16 {
            let x = xi as f64 * 0.04;
            let y = yi as f64 * 0.04;
            if x.abs() + y.abs() > target.half_diagonal_m() - 0.01 {
                continue;
            }
            if circular_cutouts.iter().any(|cutout| {
                let dx = x - cutout.x_um as f64 / 1e6;
                let dy = y - cutout.y_um as f64 / 1e6;
                dx.hypot(dy) < cutout.radius_um as f64 / 1e6 + 0.002
            }) {
                continue;
            }
            local.push(Point3::new(x, y, 0.0));
        }
    }
    for cutout in circular_cutouts {
        let (x, y, radius) = (
            cutout.x_um as f64 / 1e6,
            cutout.y_um as f64 / 1e6,
            cutout.radius_um as f64 / 1e6,
        );
        for sample in 0..32 {
            let angle = sample as f64 * std::f64::consts::TAU / 32.0;
            local.push(Point3::new(
                x + radius * angle.cos(),
                y + radius * angle.sin(),
                0.0,
            ));
        }
    }
    local
        .into_iter()
        .map(|point| pose.transform_point(&point))
        .collect()
}

#[test]
fn facade_accepts_perforated_target_with_cutout_diagnostics() {
    let target = target();
    let observation = observation();
    let expected = 2;
    let candidate = &observation.board_up_candidates[expected];
    let pose = Isometry3::from_parts(
        Translation3::from(observation.center.coords),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
            Matrix3::from_columns(&[
                candidate.x_axis.into_inner(),
                candidate.board_up.into_inner(),
                candidate.z_axis.into_inner(),
            ]),
        )),
    );
    let estimator =
        TargetPoseEstimator::new(&target, TargetPoseEstimatorTuning::for_perforated(tuning()))
            .unwrap();
    let TargetPoseEstimate::Detected(detection) =
        estimator.estimate(observation, samples(&target, pose))
    else {
        panic!("expected detection")
    };
    assert_eq!(detection.selected_quadrant, expected);
    assert!(matches!(
        detection.diagnostics,
        TargetDetectionDiagnostics::CutoutIcp(_)
    ));
}

#[test]
fn facade_rejects_ambiguous_perforated_evidence_structurally() {
    let target = target();
    let estimator =
        TargetPoseEstimator::new(&target, TargetPoseEstimatorTuning::for_perforated(tuning()))
            .unwrap();
    let TargetPoseEstimate::Rejected(rejection) = estimator.estimate(observation(), Vec::new())
    else {
        panic!("expected rejection")
    };
    assert!(matches!(
        rejection.reason,
        TargetRejectReason::PerforatedIcpFailure { .. }
            | TargetRejectReason::AmbiguousCutoutEvidence { .. }
    ));
}
