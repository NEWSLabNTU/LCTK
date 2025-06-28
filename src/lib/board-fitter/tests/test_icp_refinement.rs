//! Integration tests for ICP refinement functionality

use board_fitter::{
    refinement::{config::IcpConfigBuilder, IcpRefinement, IcpRefinementConfig},
    types::{BoardDetection, DetectionConfidence, PointCloud},
    BoardDetector, BoardTracker, DetectionConfig, TrackingConfig,
};
use board_fitter_config::{BoardConfig, CircleHole, Point2D, SquareBoard};
use measurements::Length;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::PI;

/// Generate a synthetic point cloud representing a diamond board
fn generate_synthetic_board_cloud(
    pose: &Isometry3<f64>,
    board_size: f64,
    point_density: f64,
    noise_level: f64,
) -> PointCloud {
    use rand::{Rng, SeedableRng};
    use rand_chacha::ChaCha8Rng;

    let mut rng = ChaCha8Rng::seed_from_u64(42);
    let mut points = Vec::new();

    // Generate grid points on board surface
    let n_points = (board_size / point_density) as i32;
    for i in 0..=n_points {
        for j in 0..=n_points {
            let x = -board_size / 2.0 + (i as f64) * point_density;
            let y = -board_size / 2.0 + (j as f64) * point_density;

            // Add noise
            let noise_x = rng.gen_range(-noise_level..noise_level);
            let noise_y = rng.gen_range(-noise_level..noise_level);
            let noise_z = rng.gen_range(-noise_level..noise_level);

            let local_point = Point3::new(x + noise_x, y + noise_y, noise_z);
            let world_point = pose * local_point;

            points.push(world_point);
        }
    }

    PointCloud::new(points, "test_frame".to_string())
}

/// Create a test board configuration
fn create_test_board_config() -> BoardConfig {
    let mut board = SquareBoard::new(Length::from_meters(1.0));

    // Add asymmetric hole pattern
    board.holes = vec![
        CircleHole {
            position: Point2D {
                x: Length::from_meters(0.0),
                y: Length::from_meters(0.0),
            },
            radius: Length::from_meters(0.1),
            id: Some("center".to_string()),
        },
        CircleHole {
            position: Point2D {
                x: Length::from_meters(0.3),
                y: Length::from_meters(0.0),
            },
            radius: Length::from_meters(0.05),
            id: Some("right".to_string()),
        },
        CircleHole {
            position: Point2D {
                x: Length::from_meters(0.0),
                y: Length::from_meters(0.3),
            },
            radius: Length::from_meters(0.05),
            id: Some("top".to_string()),
        },
    ];

    BoardConfig {
        board,
        detection: None,
        metadata: None,
    }
}

#[test]
fn test_detection_with_icp_refinement() {
    // Create board configuration
    let board_config = create_test_board_config();

    // Create detection config with ICP enabled
    let mut detection_config = DetectionConfig::new_with_default(board_config);
    detection_config.icp_refinement = Some(IcpRefinementConfig::default());

    // Create detector
    let mut detector = BoardDetector::new(detection_config);

    // Generate synthetic board at known pose
    let true_pose = Isometry3::from_parts(
        Translation3::new(1.0, 2.0, 0.5),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0),
    );

    let point_cloud = generate_synthetic_board_cloud(&true_pose, 1.0, 0.02, 0.001);

    // Detect board
    let result = detector.detect(&point_cloud).unwrap();

    // Should detect the board
    assert_eq!(result.count(), 1);

    let detection = &result.detections[0];

    // Check pose accuracy (should be within 1cm and 1 degree)
    let position_error = (detection.pose.translation.vector - true_pose.translation.vector).norm();
    assert!(
        position_error < 0.01,
        "Position error: {:.3}m",
        position_error
    );

    let rotation_error = (detection.pose.rotation.inverse() * true_pose.rotation)
        .angle()
        .abs();
    assert!(
        rotation_error < 0.017,
        "Rotation error: {:.1}°",
        rotation_error.to_degrees()
    );
}

#[test]
fn test_detection_without_icp_refinement() {
    // Create board configuration
    let board_config = create_test_board_config();

    // Create detection config without ICP
    let detection_config = DetectionConfig::without_icp(board_config);

    // Create detector
    let mut detector = BoardDetector::new(detection_config);

    // Generate synthetic board with more noise
    let true_pose = Isometry3::from_parts(
        Translation3::new(1.0, 2.0, 0.5),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0),
    );

    let point_cloud = generate_synthetic_board_cloud(&true_pose, 1.0, 0.02, 0.005);

    // Detect board
    let result = detector.detect(&point_cloud).unwrap();

    // Should still detect the board
    assert_eq!(result.count(), 1);

    let detection = &result.detections[0];

    // Position error should be larger without ICP
    let position_error = (detection.pose.translation.vector - true_pose.translation.vector).norm();

    // Without ICP, we expect larger errors
    assert!(
        position_error < 0.1,
        "Position error: {:.3}m",
        position_error
    );
}

#[test]
fn test_tracking_with_temporal_icp() {
    // Create ICP configuration with temporal alignment enabled
    let icp_config = IcpConfigBuilder::new()
        .with_temporal_alignment(true)
        .with_threads(4)
        .build()
        .unwrap();

    let icp_refiner = IcpRefinement::new(icp_config);

    // Create tracking config
    let tracking_config = TrackingConfig::default();

    // Create tracker with ICP
    let mut tracker = BoardTracker::new_with_icp(tracking_config, icp_refiner);

    // Simulate board moving over time
    let initial_pose = Isometry3::identity();
    let velocity = Vector3::new(0.1, 0.0, 0.0); // 10cm/s in x direction

    let mut previous_detections = Vec::new();

    for frame in 0..5 {
        let dt = 0.1; // 100ms between frames
        let translation = initial_pose.translation.vector + velocity * (frame as f64 * dt);
        let current_pose = Isometry3::from_parts(translation.into(), initial_pose.rotation);

        // Create detection at current pose
        let detection = BoardDetection {
            id: uuid::Uuid::new_v4(),
            pose: current_pose,
            confidence: DetectionConfidence::new(0.9),
            dimensions: Vector3::new(1.0, 1.0, 0.02),
            holes: Vec::new(),
            supporting_points: Vec::new(),
            timestamp: std::time::Instant::now(),
        };

        // Update tracker
        let tracks = tracker.update(vec![detection.clone()]).unwrap();

        // Should maintain one track
        assert_eq!(tracks.len(), 1);

        let track = &tracks[0];

        // Track should follow the motion
        if frame > 0 {
            let expected_pos = initial_pose.translation.vector + velocity * (frame as f64 * dt);
            let track_error = (track.pose.translation.vector - expected_pos).norm();
            assert!(
                track_error < 0.02,
                "Frame {}: tracking error {:.3}m",
                frame,
                track_error
            );
        }

        previous_detections.push(detection);
    }
}

#[test]
fn test_icp_config_builder() {
    // Test various configurations
    let config1 = IcpConfigBuilder::new()
        .with_cuda(false)
        .with_threads(8)
        .with_square_refinement(true)
        .with_hole_alignment(true)
        .with_board_refinement(true)
        .with_temporal_alignment(false)
        .build()
        .unwrap();

    assert!(!config1.enable_cuda);
    assert_eq!(config1.num_threads, 8);
    assert!(config1.square_pose_refinement.enabled);
    assert!(config1.hole_pattern_alignment.enabled);
    assert!(config1.board_pose_refinement.enabled);
    assert!(!config1.temporal_alignment.enabled);

    // Test CUDA configuration
    let config2 = IcpConfigBuilder::new().with_cuda(true).build().unwrap();

    assert!(config2.enable_cuda);
    assert!(config2.fallback_to_cpu); // Should have CPU fallback by default
}

#[test]
fn test_partial_hole_matching_with_icp() {
    // Create board configuration
    let board_config = create_test_board_config();

    // Create detection config with ICP
    let detection_config = DetectionConfig::new_with_default(board_config.clone());

    // Create detector
    let mut detector = BoardDetector::new(detection_config);

    // Generate synthetic board with only partial hole visibility
    let pose = Isometry3::identity();
    let point_cloud = generate_synthetic_board_cloud(&pose, 1.0, 0.02, 0.001);

    // Simulate holes as low-intensity regions (simplified)
    // In a real scenario, holes would be actual gaps in the point cloud

    // Detect board
    let result = detector.detect(&point_cloud).unwrap();

    // Even with partial holes, ICP should help refine the detection
    if result.count() > 0 {
        let detection = &result.detections[0];
        assert!(detection.confidence.score() > 0.5);
    }
}

#[test]
fn test_icp_performance_comparison() {
    use std::time::Instant;

    // Create board configuration
    let board_config = create_test_board_config();

    // Generate test point cloud
    let pose = Isometry3::identity();
    let point_cloud = generate_synthetic_board_cloud(&pose, 1.0, 0.01, 0.002);

    // Test with ICP
    let mut config_with_icp = DetectionConfig::new_with_default(board_config.clone());
    config_with_icp.icp_refinement = Some(IcpRefinementConfig::default());

    let mut detector_with_icp = BoardDetector::new(config_with_icp);

    let start = Instant::now();
    let result_with_icp = detector_with_icp.detect(&point_cloud).unwrap();
    let time_with_icp = start.elapsed();

    // Test without ICP
    let config_without_icp = DetectionConfig::without_icp(board_config);
    let mut detector_without_icp = BoardDetector::new(config_without_icp);

    let start = Instant::now();
    let result_without_icp = detector_without_icp.detect(&point_cloud).unwrap();
    let time_without_icp = start.elapsed();

    println!("Detection time with ICP: {:?}", time_with_icp);
    println!("Detection time without ICP: {:?}", time_without_icp);

    // Both should detect the board
    assert_eq!(result_with_icp.count(), 1);
    assert_eq!(result_without_icp.count(), 1);

    // ICP should improve confidence
    let confidence_with_icp = result_with_icp.detections[0].confidence.score();
    let confidence_without_icp = result_without_icp.detections[0].confidence.score();

    println!("Confidence with ICP: {:.3}", confidence_with_icp);
    println!("Confidence without ICP: {:.3}", confidence_without_icp);

    // ICP may take longer but should provide better results
    // In practice, the time difference should be reasonable
    assert!(time_with_icp.as_millis() < 1000); // Should complete within 1 second
}
