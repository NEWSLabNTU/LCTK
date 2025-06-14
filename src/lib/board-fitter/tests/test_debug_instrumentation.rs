//! Test debug instrumentation functionality

use board_fitter::{DiamondBoardDetectorBuilder, PointCloud};
use board_fitter_config::Config;

#[path = "common/mod.rs"]
mod common;
use common::*;

/// Test basic debug instrumentation with console output
#[test]
fn test_debug_instrumentation_basic() {
    println!("\n=== Debug Instrumentation Test ===");

    // Create test board configuration
    let board_config = create_test_board_config(1.0);

    // Create detector with debug instrumentation enabled
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };

    let mut detector = DiamondBoardDetectorBuilder::new()
        .with_console_debug(false) // Non-verbose mode
        .min_confidence(0.1) // Lower threshold for testing
        .timeout_ms(5000)
        .build(config)
        .expect("Failed to create detector with debug");

    // Generate test data - use a tilted board pose that will pass plane filtering
    let mut generator = TestDataGenerator::new(42);
    // Tilt the board 45 degrees around X-axis to create a diamond orientation
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0);
    // Use high point density for better hole detection
    let point_cloud = generator.generate_perfect_board(&board_config, &pose, 10000);

    println!(
        "Generated test point cloud with {} points",
        point_cloud.len()
    );

    // Run detection with debug output
    let result = detector.detect(&point_cloud);

    match result {
        Ok(detections) => {
            println!(
                "Detection completed successfully with {} detections",
                detections.len()
            );
            for (i, detection) in detections.iter().enumerate() {
                println!(
                    "  Detection {}: confidence={:.3}, holes={}",
                    i,
                    detection.confidence.value(),
                    detection.holes.len()
                );
            }
        }
        Err(e) => {
            println!("Detection failed: {}", e);
        }
    }

    println!("=== End Debug Test ===\n");
}

/// Test verbose debug instrumentation
#[test]
fn test_debug_instrumentation_verbose() {
    println!("\n=== Verbose Debug Instrumentation Test ===");

    let board_config = create_test_board_config(0.5); // Smaller board

    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };

    let mut detector = DiamondBoardDetectorBuilder::new()
        .with_console_debug(true) // Verbose mode
        .min_confidence(0.1)
        .max_detections(3)
        .timeout_ms(3000)
        .build(config)
        .expect("Failed to create verbose debug detector");

    // Generate minimal test data
    let mut generator = TestDataGenerator::new(123);
    let pose = create_board_pose(1.5, 0.0, 0.5, 0.0, 0.0, 0.0);
    let point_cloud = generator.generate_perfect_board(&board_config, &pose, 200);

    println!("Running verbose debug detection...");

    let result = detector.detect(&point_cloud);

    match result {
        Ok(detections) => {
            println!("RESULT: {} detections found", detections.len());
        }
        Err(e) => {
            println!("RESULT: Detection failed - {}", e);
        }
    }

    println!("=== End Verbose Debug Test ===\n");
}

/// Test debug instrumentation with empty point cloud
#[test]
fn test_debug_empty_cloud() {
    println!("\n=== Debug Empty Cloud Test ===");

    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config,
        detection: None,
        metadata: None,
    };

    let mut detector = DiamondBoardDetectorBuilder::new()
        .with_console_debug(false)
        .build(config)
        .expect("Failed to create detector");

    let empty_cloud = PointCloud::new(Vec::new(), "empty_test".to_string());

    let result = detector.detect(&empty_cloud);
    assert!(result.is_ok());
    assert!(result.unwrap().is_empty());

    println!("=== End Empty Cloud Test ===\n");
}

/// Test debug instrumentation with noisy data
#[test]
fn test_debug_noisy_data() {
    println!("\n=== Debug Noisy Data Test ===");

    let board_config = create_test_board_config(1.0);
    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };

    let mut detector = DiamondBoardDetectorBuilder::new()
        .with_console_debug(false)
        .min_confidence(0.05) // Very low for noisy data
        .build(config)
        .expect("Failed to create detector");

    let mut generator = TestDataGenerator::new(456);
    let pose = create_board_pose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    let clean_cloud = generator.generate_perfect_board(&board_config, &pose, 300);
    let noisy_cloud = generator.add_noise(&clean_cloud, 0.02); // 2cm noise

    println!("Testing with noisy data ({} points)", noisy_cloud.len());

    let result = detector.detect(&noisy_cloud);

    match result {
        Ok(detections) => {
            println!("Noisy detection result: {} detections", detections.len());
        }
        Err(e) => {
            println!("Noisy detection failed: {}", e);
        }
    }

    println!("=== End Noisy Data Test ===\n");
}
