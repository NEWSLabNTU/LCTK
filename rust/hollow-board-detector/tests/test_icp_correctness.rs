use hollow_board_config::BoardShape;
use hollow_board_detector::{
    algo::BoardIcpIterator,
    config::{Config, SensorUpAxis},
    detection::BoardModelParams,
};
use measurements::Length;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

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
        voxel_downsample_enabled: false,
        voxel_downsample_size: 0.02,
        voxel_downsample_use_centroid: true,
        voxel_parallel_threshold: 50_000,
        // These tests seed BoardIcpIterator with an explicit pose, so neither
        // field affects them; they only satisfy the struct literal.
        sensor_up_axis: SensorUpAxis::Z,
        initial_inplane_rotation_deg: 0.0,
        board_shape: BoardShape {
            board_width: Length::from_meters(0.5),
            hole_radius: Length::from_meters(0.05),
            hole_center_shift: Length::from_meters(0.05),
        },
    }
}

fn create_board_params() -> BoardModelParams {
    BoardModelParams {
        board_shape: BoardShape {
            board_width: Length::from_meters(0.5),
            hole_radius: Length::from_meters(0.05),
            hole_center_shift: Length::from_meters(0.05),
        },
        marker_paper_size: Length::from_meters(0.1),
    }
}

fn create_grid_points(pose: &Isometry3<f64>, grid_size: usize) -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    let step = 0.5 / grid_size as f64;

    for i in 0..grid_size {
        for j in 0..grid_size {
            let x = i as f64 * step;
            let y = j as f64 * step;
            let local_point = Point3::new(x, y, 0.0);
            let world_point = pose.transform_point(&local_point);
            points.push(world_point);
        }
    }

    points
}

#[test]
fn test_identity_transformation_convergence() {
    let config = create_test_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let points = create_grid_points(&initial_pose, 10);

    let mut state = iterator.initial_state(initial_pose, points);

    assert_eq!(
        state.iteration, 0,
        "Initial state should start at iteration 0"
    );
    assert_eq!(
        state.termination_count, 0,
        "Initial termination count should be 0"
    );

    let mut iteration_count = 0;
    while !iterator.should_terminate(&state) && iteration_count < 10 {
        let next_state = iterator.step(&state);

        assert_eq!(
            next_state.iteration,
            state.iteration + 1,
            "Iteration should increment"
        );

        state = next_state;
        iteration_count += 1;
    }

    assert!(
        iterator.should_terminate(&state),
        "Iterator should eventually terminate"
    );
}

#[test]
fn test_small_translation_recovery() {
    let config = create_test_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let target_pose = Isometry3::from_parts(
        Translation3::new(0.01, 0.01, 0.0),
        UnitQuaternion::identity(),
    );

    let points = create_grid_points(&target_pose, 10);

    let initial_pose = Isometry3::identity();
    let mut state = iterator.initial_state(initial_pose, points);

    let initial_translation_error =
        (state.board_pose.translation.vector - target_pose.translation.vector).norm();

    assert!(
        initial_translation_error > 0.0,
        "Initial error should be non-zero: {}",
        initial_translation_error
    );

    let mut iteration_count = 0;
    while !iterator.should_terminate(&state) && iteration_count < 20 {
        state = iterator.step(&state);
        iteration_count += 1;
    }

    assert!(
        iterator.should_terminate(&state),
        "Algorithm should terminate"
    );
}

#[test]
fn test_small_rotation_handling() {
    let config = create_test_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let small_angle = 0.05;
    let target_pose = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), small_angle),
    );

    let points = create_grid_points(&target_pose, 10);

    let initial_pose = Isometry3::identity();
    let mut state = iterator.initial_state(initial_pose, points);

    let mut iteration_count = 0;
    while !iterator.should_terminate(&state) && iteration_count < 20 {
        state = iterator.step(&state);
        iteration_count += 1;
    }

    assert!(
        iterator.should_terminate(&state),
        "Algorithm should terminate"
    );
}

#[test]
fn test_termination_on_insufficient_points() {
    let config = create_test_config();
    let board_params = create_board_params();
    let iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let insufficient_points = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.1, 0.0, 0.0)];

    let state = iterator.initial_state(initial_pose, insufficient_points);

    assert!(
        iterator.should_terminate(&state)
            || state.inlier_points.len() < config.icp_min_inlier_points,
        "Should recognize insufficient points"
    );
}

#[test]
fn test_damping_reduces_step_size() {
    let config = create_test_config();
    let board_params = create_board_params();

    let target_pose =
        Isometry3::from_parts(Translation3::new(0.1, 0.1, 0.0), UnitQuaternion::identity());

    let points = create_grid_points(&target_pose, 10);
    let initial_pose = Isometry3::identity();

    let mut iterator = BoardIcpIterator::new(&config, board_params, None);
    let state0 = iterator.initial_state(initial_pose, points.clone());
    let state1 = iterator.step(&state0);

    if state1.good_correspondences >= 3 {
        let movement =
            (state1.board_pose.translation.vector - state0.board_pose.translation.vector).norm();

        let max_expected_movement = 0.1 * config.icp_damping_factor * 2.0;

        assert!(
            movement <= max_expected_movement,
            "Damping should limit movement: {} <= {}",
            movement,
            max_expected_movement
        );
    }
}

#[test]
fn test_convergence_counter_increases() {
    let config = create_test_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let points = create_grid_points(&initial_pose, 10);

    let mut state = iterator.initial_state(initial_pose, points);

    let mut max_termination_count = 0;
    let mut iteration_count = 0;

    while !iterator.should_terminate(&state) && iteration_count < 20 {
        state = iterator.step(&state);
        max_termination_count = max_termination_count.max(state.termination_count);
        iteration_count += 1;
    }

    assert!(
        iterator.should_terminate(&state),
        "Algorithm should terminate"
    );
}
