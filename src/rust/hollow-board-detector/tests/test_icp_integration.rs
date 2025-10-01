use aruco_config::{ArucoDictionary, MultiArucoPattern};
use hollow_board_config::BoardShape;
use hollow_board_detector::{
    algo::{fit_board_icp_with_iterator, BoardIcpIterator},
    config::Config,
    detection::BoardModelParams,
};
use measurements::Length;
use nalgebra::{Point3, Unit, Vector3};
use noisy_float::types::R64;
use plane_estimator::PlaneModel;

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
        icp_min_inlier_points: 10,
        board_shape: BoardShape {
            board_width: Length::from_meters(0.5),
            hole_radius: Length::from_meters(0.05),
            hole_center_shift: Length::from_meters(0.05),
        },
    }
}

fn create_test_plane() -> PlaneModel {
    PlaneModel {
        center: Point3::new(0.0, 0.0, 1.0),
        normal: Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
    }
}

fn create_test_plane_points() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            let x = -0.25 + 0.025 * i as f64;
            let y = -0.25 + 0.025 * j as f64;
            points.push(Point3::new(x, y, 1.0));
        }
    }
    points
}

fn create_test_aruco_pattern() -> MultiArucoPattern {
    MultiArucoPattern {
        marker_ids: vec![0, 1, 2, 3],
        dictionary: ArucoDictionary::DICT_4X4_50,
        board_size: Length::from_meters(0.1),
        board_border_size: Length::from_meters(0.0125),
        marker_square_size_ratio: R64::new(0.8),
        num_squares_per_side: 2,
        border_bits: 1,
    }
}

#[test]
fn test_iterator_api_produces_consistent_results() {
    let config = create_test_config();
    let aruco_pattern = create_test_aruco_pattern();
    let plane_model = create_test_plane();
    let plane_points = create_test_plane_points();

    let result1 =
        fit_board_icp_with_iterator(&config, &aruco_pattern, &plane_model, &plane_points, None);

    let result2 =
        fit_board_icp_with_iterator(&config, &aruco_pattern, &plane_model, &plane_points, None);

    assert!(result1.is_ok());
    assert!(result2.is_ok());

    let fit1 = result1.unwrap();
    let fit2 = result2.unwrap();

    assert_eq!(
        fit1.successful, fit2.successful,
        "Success status should match"
    );
    assert_eq!(
        fit1.icp_stats.iterations, fit2.icp_stats.iterations,
        "Iterations should match"
    );

    assert!(
        (fit1.board_pose.translation.vector - fit2.board_pose.translation.vector).norm() < 1e-9,
        "Translation should match: {:?} vs {:?}",
        fit1.board_pose.translation,
        fit2.board_pose.translation
    );

    if fit1.icp_stats.final_loss.is_finite() && fit2.icp_stats.final_loss.is_finite() {
        assert!(
            (fit1.icp_stats.final_loss - fit2.icp_stats.final_loss).abs() < 1e-9,
            "Final loss should match: {} vs {}",
            fit1.icp_stats.final_loss,
            fit2.icp_stats.final_loss
        );
    }
}

#[test]
fn test_manual_iterator_loop_matches_wrapper() {
    let config = create_test_config();
    let aruco_pattern = create_test_aruco_pattern();
    let plane_model = create_test_plane();
    let plane_points = create_test_plane_points();

    let wrapper_result =
        fit_board_icp_with_iterator(&config, &aruco_pattern, &plane_model, &plane_points, None)
            .unwrap();

    let board_params = BoardModelParams {
        board_shape: config.board_shape.clone(),
        marker_paper_size: aruco_pattern.paper_size(),
    };

    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let init_pose = wrapper_result.initial_pose;
    let init_points = plane_points.clone();

    let mut state = iterator.initial_state(init_pose, init_points);
    let mut iteration_count = 0;

    while !iterator.should_terminate(&state) && iteration_count < config.max_icp_iterations {
        state = iterator.step(&state);
        iteration_count += 1;
    }

    assert_eq!(
        state.iteration, wrapper_result.icp_stats.iterations,
        "Manual loop should match wrapper iterations: {} vs {}",
        state.iteration, wrapper_result.icp_stats.iterations
    );

    if state.avg_loss.is_finite() && wrapper_result.icp_stats.final_loss.is_finite() {
        assert!(
            (state.avg_loss - wrapper_result.icp_stats.final_loss).abs() < 1e-9,
            "Manual loop final loss should match wrapper: {} vs {}",
            state.avg_loss,
            wrapper_result.icp_stats.final_loss
        );
    } else {
        assert_eq!(
            state.avg_loss.is_finite(),
            wrapper_result.icp_stats.final_loss.is_finite(),
            "Both should have same finiteness: manual={} vs wrapper={}",
            state.avg_loss,
            wrapper_result.icp_stats.final_loss
        );
    }
}
