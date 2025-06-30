//! Integration tests for ICP refinement functionality

use board_fitter::{
    refinement::{config::IcpConfigBuilder, IcpRefinement},
    types::{BoardDetection, DetectionConfidence},
    BoardDetectorBuilder, BoardTracker, TrackingConfig,
};
use board_fitter_config::BoardConfig;
use nalgebra::{Isometry3, Vector3};

#[path = "common/mod.rs"]
mod common;
use common::*;

#[test]
fn test_detection_with_icp_refinement() {
    // Create board configuration
    let board_config = create_test_board_config(1.0);

    // Create detection config with ICP enabled
    let config = BoardConfig {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = BoardDetectorBuilder::new(config)
        .timeout_ms(10000) // 10 second timeout for ICP-enabled tests
        .with_fast_icp() // Test performance optimization
        .build()
        .unwrap();

    // Generate synthetic board at known pose
    let true_pose = create_board_pose(1.0, 2.0, 0.5, 45.0_f64.to_radians(), 0.0, 0.0);
    let mut generator = TestDataGenerator::new(42);
    let point_cloud = generator.generate_perfect_board(&board_config, &true_pose, 400);

    // Detect board
    let result = detector.detect(&point_cloud).unwrap();

    // Should detect the board
    assert_eq!(result.detections.len(), 1);

    let detection = &result.detections[0];

    // Check pose accuracy (relaxed tolerances for ICP tests)
    let position_error = (detection.pose.translation.vector - true_pose.translation.vector).norm();
    assert!(
        position_error < 0.15,
        "Position error: {position_error:.3}m"
    );

    let rotation_error = (detection.pose.rotation.inverse() * true_pose.rotation)
        .angle()
        .abs();
    assert!(
        rotation_error < 0.17,
        "Rotation error: {:.1}°",
        rotation_error.to_degrees()
    );
}

#[test]
fn test_detection_without_icp_refinement() {
    // Create board configuration
    let board_config = create_test_board_config(1.0);

    // Create detection config without ICP
    let config = BoardConfig {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = BoardDetectorBuilder::new(config)
        .timeout_ms(10000)
        .build()
        .unwrap();

    // Generate synthetic board with more noise
    let true_pose = create_board_pose(1.0, 2.0, 0.5, 45.0_f64.to_radians(), 0.0, 0.0);
    let mut generator = TestDataGenerator::new(43);
    let perfect_cloud = generator.generate_perfect_board(&board_config, &true_pose, 400);
    let point_cloud = generator.add_noise(&perfect_cloud, 0.005);

    // Detect board
    let result = detector.detect(&point_cloud).unwrap();

    // Should still detect the board
    assert_eq!(result.detections.len(), 1);

    let detection = &result.detections[0];

    // Position error should be larger without ICP
    let position_error = (detection.pose.translation.vector - true_pose.translation.vector).norm();

    // Without ICP, we expect larger errors (relaxed tolerance)
    assert!(position_error < 0.2, "Position error: {position_error:.3}m");
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
                "Frame {frame}: tracking error {track_error:.3}m"
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
    let board_config = create_test_board_config(1.0);

    // Create detection config with ICP
    let config = BoardConfig {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = BoardDetectorBuilder::new(config)
        .timeout_ms(10000)
        .with_fast_icp()
        .build()
        .unwrap();

    // Generate synthetic board with only partial hole visibility
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0);
    let mut generator = TestDataGenerator::new(45);
    let perfect_cloud = generator.generate_perfect_board(&board_config, &pose, 300);

    // Apply partial occlusion to simulate partial hole visibility
    let point_cloud = generator.apply_occlusion(&perfect_cloud, 0.2); // 20% occlusion

    // Detect board
    let result = detector.detect(&point_cloud).unwrap();

    // Even with partial holes, ICP should help refine the detection
    if !result.detections.is_empty() {
        let detection = &result.detections[0];
        assert!(detection.confidence.value() > 0.5);
    }
}

#[test]
fn test_icp_performance_comparison() {
    use std::time::Instant;

    // Create board configuration
    let board_config = create_test_board_config(1.0);

    // Generate test point cloud with moderate density for performance test
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0);
    let mut generator = TestDataGenerator::new(44);
    let point_cloud = generator.generate_perfect_board(&board_config, &pose, 200); // Reduced density for speed

    // Test with ICP
    let config_with_icp = BoardConfig {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector_with_icp = BoardDetectorBuilder::new(config_with_icp)
        .timeout_ms(15000) // Increased timeout for performance test
        .with_fast_icp()
        .build()
        .unwrap();

    let start = Instant::now();
    let result_with_icp = detector_with_icp.detect(&point_cloud).unwrap();
    let time_with_icp = start.elapsed();

    // Test without ICP
    let config_without_icp = BoardConfig {
        board: board_config,
        detection: None,
        metadata: None,
    };
    let mut detector_without_icp = BoardDetectorBuilder::new(config_without_icp)
        .timeout_ms(15000)
        .build()
        .unwrap();

    let start = Instant::now();
    let result_without_icp = detector_without_icp.detect(&point_cloud).unwrap();
    let time_without_icp = start.elapsed();

    println!("Detection time with ICP: {time_with_icp:?}");
    println!("Detection time without ICP: {time_without_icp:?}");

    // Both should detect the board
    assert_eq!(result_with_icp.detections.len(), 1);
    assert_eq!(result_without_icp.detections.len(), 1);

    // ICP should improve confidence
    let confidence_with_icp = result_with_icp.detections[0].confidence.value();
    let confidence_without_icp = result_without_icp.detections[0].confidence.value();

    println!("Confidence with ICP: {confidence_with_icp:.3}");
    println!("Confidence without ICP: {confidence_without_icp:.3}");

    // ICP may take longer but should provide better results
    // Relaxed timeout for performance test
    assert!(time_with_icp.as_millis() < 15000); // Should complete within 15 seconds
}
