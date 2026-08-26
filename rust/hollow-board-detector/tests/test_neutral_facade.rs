use calibration_target::ValidatedTarget;
use hollow_board_detector::{PerforatedIcpConfig, TargetPoseEstimator, TargetPoseEstimatorTuning};

const HOLLOW: &[u8] = include_bytes!("../../../fixtures/targets/hollow_1000_aruco_4_v1.json5");

#[test]
fn legacy_crate_reexports_the_neutral_estimator_facade() {
    let target = ValidatedTarget::parse_json5(HOLLOW).unwrap();
    let tuning = PerforatedIcpConfig::new(40, 0.2, 0.5, 1e-8, 1e-8, 0.005, 3, 0.0001, 1, 1e-5);
    let estimator =
        TargetPoseEstimator::new(&target, TargetPoseEstimatorTuning::for_perforated(tuning))
            .unwrap();
    assert_eq!(estimator.target_identity().target_id, "hollow_1000_aruco_4");
}
