//! End-to-end pipeline integration tests

use board_fitter::DiamondBoardDetectorBuilder;
use board_fitter_config::Config;
use std::f64::consts::PI;

#[path = "common/mod.rs"]
mod common;
use common::*;

#[test]
fn test_perfect_board_detection() {
    // Create test board configuration
    let board_config = create_test_board_config(1.0); // 1m board

    // Create detector
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = DiamondBoardDetectorBuilder::new().build(config).unwrap();

    // Generate perfect board at known pose
    let mut generator = TestDataGenerator::new(42);
    let ground_truth_pose = create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    let point_cloud = generator.generate_perfect_board(&board_config, &ground_truth_pose, 400);

    // Detect board
    let timer = PerfTimer::new("Perfect board detection");
    let detections = detector.detect(&point_cloud).unwrap();
    let elapsed = timer.elapsed_ms();

    // Verify results
    assert!(!detections.is_empty(), "Should detect at least one board");
    let detected = &detections[0];

    // Check accuracy
    let position_error =
        (detected.pose.translation.vector - ground_truth_pose.translation.vector).norm();
    assert!(
        position_error < 0.01,
        "Position error {:.3}m exceeds tolerance",
        position_error
    );

    // Check that all holes were detected
    assert_eq!(detected.holes.len(), 3, "Should detect 3 holes");

    // Check confidence
    assert!(
        detected.confidence.value() > 0.8,
        "Confidence should be high for perfect data"
    );

    // Check processing time
    assert!(
        elapsed < 50.0,
        "Processing time {:.1}ms exceeds limit",
        elapsed
    );

    println!("Perfect board detection passed:");
    println!("  Position error: {:.3}mm", position_error * 1000.0);
    println!("  Processing time: {:.1}ms", elapsed);
    println!("  Confidence: {:.2}", detected.confidence.value());
}

#[test]
fn test_noisy_board_detection() {
    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = DiamondBoardDetectorBuilder::new().build(config).unwrap();

    let mut generator = TestDataGenerator::new(43);
    let ground_truth_pose = create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0);

    // Test with different noise levels
    let noise_levels = [0.01, 0.02, 0.05]; // 1cm, 2cm, 5cm
    let mut results = TestResults::new();

    for &noise_level in &noise_levels {
        let perfect_cloud =
            generator.generate_perfect_board(&board_config, &ground_truth_pose, 400);
        let noisy_cloud = generator.add_noise(&perfect_cloud, noise_level);

        let timer = PerfTimer::new(&format!("Noisy board (σ={}cm)", noise_level * 100.0));
        let detections = detector.detect(&noisy_cloud).unwrap();
        let elapsed = timer.elapsed_ms();

        if !detections.is_empty() {
            let detected = &detections[0];
            let position_error =
                (detected.pose.translation.vector - ground_truth_pose.translation.vector).norm();
            let angle_error =
                (detected.pose.rotation * ground_truth_pose.rotation.inverse()).angle();

            results.add_success(position_error, angle_error, elapsed);

            // Verify reasonable accuracy even with noise
            assert!(
                position_error < noise_level * 3.0,
                "Position error {:.3}m exceeds 3σ for noise level {}",
                position_error,
                noise_level
            );
        } else {
            results.add_failure();
        }
    }

    results.print_summary();
    assert!(
        results.success_rate() >= 0.9,
        "Success rate too low for noisy data"
    );
}

#[test]
fn test_partial_occlusion() {
    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = DiamondBoardDetectorBuilder::new().build(config).unwrap();

    let mut generator = TestDataGenerator::new(44);
    let ground_truth_pose = create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    let perfect_cloud = generator.generate_perfect_board(&board_config, &ground_truth_pose, 500);

    // Test with different occlusion levels
    let occlusion_levels = [0.1, 0.2, 0.3, 0.4]; // 10%, 20%, 30%, 40%

    for &occlusion in &occlusion_levels {
        let occluded_cloud = generator.apply_occlusion(&perfect_cloud, occlusion);

        println!(
            "\nTesting with {:.0}% occlusion ({} points remaining)",
            occlusion * 100.0,
            occluded_cloud.points.len()
        );

        let detections = detector.detect(&occluded_cloud).unwrap();

        if occlusion <= 0.3 {
            // Should still detect with up to 30% occlusion
            assert!(
                !detections.is_empty(),
                "Failed to detect board with {:.0}% occlusion",
                occlusion * 100.0
            );

            if !detections.is_empty() {
                let detected = &detections[0];
                println!(
                    "  Detected with confidence: {:.2}",
                    detected.confidence.value()
                );
                println!("  Holes detected: {}/{}", detected.holes.len(), 3);
            }
        }
    }
}

#[test]
fn test_extreme_poses() {
    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = DiamondBoardDetectorBuilder::new().build(config).unwrap();

    let mut generator = TestDataGenerator::new(45);
    let mut results = TestResults::new();

    // Test various extreme orientations
    let test_poses = [
        ("Frontal", create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0)),
        (
            "45° yaw",
            create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, PI / 4.0),
        ),
        (
            "30° pitch",
            create_board_pose(2.0, 0.0, 1.0, 0.0, PI / 6.0, 0.0),
        ),
        (
            "60° pitch",
            create_board_pose(2.0, 0.0, 1.0, 0.0, PI / 3.0, 0.0),
        ),
        (
            "Combined",
            create_board_pose(2.0, 0.0, 1.0, PI / 6.0, PI / 4.0, PI / 6.0),
        ),
    ];

    for (name, pose) in &test_poses {
        println!("\nTesting pose: {}", name);

        let point_cloud = generator.generate_perfect_board(&board_config, pose, 400);
        let timer = PerfTimer::new(name);
        let detections = detector.detect(&point_cloud).unwrap();
        let elapsed = timer.elapsed_ms();

        if !detections.is_empty() {
            let detected = &detections[0];
            let position_error =
                (detected.pose.translation.vector - pose.translation.vector).norm();
            let angle_error = (detected.pose.rotation * pose.rotation.inverse()).angle();

            results.add_success(position_error, angle_error, elapsed);

            println!("  Position error: {:.1}mm", position_error * 1000.0);
            println!("  Angle error: {:.1}°", angle_error.to_degrees());
            println!("  Confidence: {:.2}", detected.confidence.value());
        } else {
            results.add_failure();
            println!("  Detection failed!");
        }
    }

    results.print_summary();
    assert!(
        results.success_rate() >= 0.8,
        "Too many failures with extreme poses"
    );
}

#[test]
fn test_multi_board_scene() {
    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = DiamondBoardDetectorBuilder::new().build(config).unwrap();

    let mut generator = TestDataGenerator::new(46);

    // Create scene with 3 boards at different positions
    let board_poses = vec![
        (
            board_config.clone(),
            create_board_pose(1.0, -1.0, 1.0, 0.0, 0.0, 0.0),
        ),
        (
            board_config.clone(),
            create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, PI / 4.0),
        ),
        (
            board_config.clone(),
            create_board_pose(3.0, 1.0, 1.0, 0.0, PI / 6.0, 0.0),
        ),
    ];

    let scene = generator.generate_multi_board_scene(&board_poses, 200, true);

    println!(
        "\nDetecting multiple boards in scene with {} points",
        scene.points.len()
    );
    let timer = PerfTimer::new("Multi-board detection");
    let detections = detector.detect(&scene).unwrap();
    let elapsed = timer.elapsed_ms();

    println!("  Detected {} boards in {:.1}ms", detections.len(), elapsed);

    // Should detect at least 2 out of 3 boards
    assert!(
        detections.len() >= 2,
        "Should detect at least 2 boards, found {}",
        detections.len()
    );

    // Verify each detection
    for (i, board) in detections.iter().enumerate() {
        println!(
            "  Board {}: confidence={:.2}, holes={}",
            i + 1,
            board.confidence.value(),
            board.holes.len()
        );
    }
}

#[test]
fn test_varying_distances() {
    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };
    let mut detector = DiamondBoardDetectorBuilder::new().build(config).unwrap();

    let mut generator = TestDataGenerator::new(47);
    let mut results = TestResults::new();

    // Test at different distances
    let distances = [1.0, 2.0, 5.0, 10.0, 20.0]; // meters

    for &distance in &distances {
        println!("\nTesting at distance: {}m", distance);

        let pose = create_board_pose(distance, 0.0, 1.0, 0.0, 0.0, 0.0);

        // Adjust point density based on distance (simulate LiDAR behavior)
        let point_density = (1000.0 / (distance * distance)) as usize;
        let point_density = point_density.max(50); // Minimum 50 points

        let point_cloud = generator.generate_perfect_board(&board_config, &pose, point_density);

        println!("  Generated {} points", point_cloud.points.len());

        let timer = PerfTimer::new(&format!("Detection at {}m", distance));
        let detections = detector.detect(&point_cloud).unwrap();
        let elapsed = timer.elapsed_ms();

        if !detections.is_empty() {
            let detected = &detections[0];
            let position_error =
                (detected.pose.translation.vector - pose.translation.vector).norm();
            let angle_error = (detected.pose.rotation * pose.rotation.inverse()).angle();

            results.add_success(position_error, angle_error, elapsed);

            println!(
                "  Success! Position error: {:.1}mm",
                position_error * 1000.0
            );

            // Expect higher error at longer distances
            let tolerance = 0.01 * distance; // 1cm per meter of distance
            assert!(
                position_error < tolerance,
                "Position error {:.3}m exceeds tolerance {:.3}m at {}m",
                position_error,
                tolerance,
                distance
            );
        } else {
            results.add_failure();
            println!("  Failed to detect board");
        }
    }

    results.print_summary();
}
