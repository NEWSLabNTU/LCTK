use hollow_board_config::BoardShape;
use hollow_board_detector::{algo::BoardIcpIterator, config::Config, detection::BoardModelParams};
use measurements::Length;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};

fn create_test_config() -> Config {
    Config {
        max_icp_iterations: 100,
        icp_pose_weight_threshold: 1e-6,
        icp_rejection_threshold: 0.1,
        plane_ransac_max_iterations: 1000,
        plane_ransac_inlier_threshold: 0.01,
        skip_ransac: false,
        icp_good_fit_threshold: 0.05,
        icp_outlier_threshold: 0.1,
        icp_damping_factor: 0.5,
        icp_min_inlier_points: 3,
        board_shape: BoardShape {
            board_width: Length::from_meters(0.5),
            hole_radius: Length::from_meters(0.05),
            hole_center_shift: Length::from_meters(0.0),
        },
    }
}

fn create_test_board_params() -> BoardModelParams {
    BoardModelParams {
        board_shape: BoardShape {
            board_width: Length::from_meters(0.5),
            hole_radius: Length::from_meters(0.05),
            hole_center_shift: Length::from_meters(0.0),
        },
        marker_paper_size: Length::from_meters(0.1),
    }
}

fn create_test_points() -> Vec<Point3<f64>> {
    vec![
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(0.1, 0.0, 1.0),
        Point3::new(0.0, 0.1, 1.0),
        Point3::new(0.1, 0.1, 1.0),
    ]
}

#[test]
fn test_iterator_state_initialization() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let state = iterator.initial_state(initial_pose, initial_points.clone());

    assert_eq!(state.iteration, 0);
    assert_eq!(state.board_pose, initial_pose);
    assert_eq!(state.inlier_points.len(), initial_points.len());
    assert_eq!(state.correspondences.len(), 0);
    assert_eq!(state.avg_loss, f64::INFINITY);
    assert_eq!(state.previous_loss, None);
    assert_eq!(state.termination_count, 0);
}

#[test]
fn test_iterator_step_increments_iteration() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let state = iterator.initial_state(initial_pose, initial_points);
    let next_state = iterator.step(&state);

    assert_eq!(next_state.iteration, state.iteration + 1);
}

#[test]
fn test_iterator_stateless_property() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let state = iterator.initial_state(initial_pose, initial_points);

    let next_state_1 = iterator.step(&state);
    let next_state_2 = iterator.step(&state);

    assert_eq!(next_state_1.iteration, next_state_2.iteration);
    assert_eq!(next_state_1.board_pose, next_state_2.board_pose);
    assert_eq!(next_state_1.avg_loss, next_state_2.avg_loss);
}

#[test]
fn test_iterator_termination_max_iterations() {
    let mut config = create_test_config();
    config.max_icp_iterations = 10;

    let board_params = create_test_board_params();
    let iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let mut state = iterator.initial_state(initial_pose, initial_points);
    state.iteration = 10;

    assert!(iterator.should_terminate(&state));
    assert!(iterator
        .termination_reason(&state)
        .contains("Max iterations"));
}

#[test]
fn test_iterator_termination_convergence() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let mut state = iterator.initial_state(initial_pose, initial_points);
    state.termination_count = 11;

    assert!(iterator.should_terminate(&state));
    assert!(iterator.termination_reason(&state).contains("Converged"));
}

#[test]
fn test_iterator_handles_no_correspondences() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::from_parts(
        Translation3::new(100.0, 100.0, 100.0),
        UnitQuaternion::identity(),
    );
    let initial_points = create_test_points();

    let state = iterator.initial_state(initial_pose, initial_points);
    let next_state = iterator.step(&state);

    assert_eq!(next_state.correspondences.len(), 0);
    assert_eq!(next_state.board_pose, state.board_pose);
}

#[test]
fn test_iterator_handles_insufficient_points() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let insufficient_points = vec![Point3::new(0.0, 0.0, 1.0), Point3::new(0.1, 0.0, 1.0)];

    let state = iterator.initial_state(initial_pose, insufficient_points);
    let next_state = iterator.step(&state);

    assert_eq!(next_state.good_correspondences, 0);
}

#[test]
fn test_iterator_correspondence_filtering() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let state = iterator.initial_state(initial_pose, initial_points);
    let next_state = iterator.step(&state);

    assert!(next_state.good_correspondences <= next_state.total_correspondences);
}

#[test]
fn test_iterator_damping_applied() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let translation = Translation3::new(0.1, 0.1, 0.0);
    let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1);
    let initial_pose = Isometry3::from_parts(translation, rotation);

    let initial_points = create_test_points();

    let state = iterator.initial_state(initial_pose, initial_points);
    let next_state = iterator.step(&state);

    let translation_delta =
        (next_state.board_pose.translation.vector - state.board_pose.translation.vector).norm();

    assert!(translation_delta < 0.1 * config.icp_damping_factor + 0.01);
}

#[test]
fn test_iterator_termination_reason() {
    let config = create_test_config();
    let board_params = create_test_board_params();
    let iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let initial_points = create_test_points();

    let mut state_converged = iterator.initial_state(initial_pose.clone(), initial_points.clone());
    state_converged.termination_count = 11;

    let reason_converged = iterator.termination_reason(&state_converged);
    assert!(reason_converged.contains("Converged"));

    let mut state_max_iter = iterator.initial_state(initial_pose, initial_points);
    state_max_iter.iteration = config.max_icp_iterations;

    let reason_max_iter = iterator.termination_reason(&state_max_iter);
    assert!(reason_max_iter.contains("Max iterations"));

    let state_in_progress = iterator.initial_state(Isometry3::identity(), create_test_points());
    let reason_in_progress = iterator.termination_reason(&state_in_progress);
    assert!(reason_in_progress.contains("Unknown") || reason_in_progress.contains("Insufficient"));
}
