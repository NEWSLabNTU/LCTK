//! Performance benchmarks for board detection pipeline

use board_fitter::{DiamondBoardDetector, PointCloud};
use board_fitter_config::Config;
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use std::time::Instant;

// Import test utilities
#[path = "../tests/common/mod.rs"]
mod common;
use common::*;

fn benchmark_detection_pipeline(c: &mut Criterion) {
    let board_config = create_test_board_config(1.0);

    let mut generator = TestDataGenerator::new(100);
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0); // Tilted 45° for diamond board

    // Create test data sets of different sizes
    let point_counts = [100, 500, 1000, 5000, 10000];
    let mut test_clouds = Vec::new();

    for &count in &point_counts {
        let cloud = generator.generate_perfect_board(&board_config, &pose, count);
        test_clouds.push((count, cloud));
    }

    let mut group = c.benchmark_group("detection_pipeline");

    for (count, cloud) in &test_clouds {
        group.bench_with_input(BenchmarkId::new("points", count), cloud, |b, cloud| {
            b.iter(|| {
                let config = Config {
                    board: board_config.clone(),
                    detection: None,
                    metadata: None,
                };
                let mut detector = DiamondBoardDetector::new(config).unwrap();
                let result = detector.detect(black_box(cloud));
                assert!(result.is_ok());
                assert!(!result.unwrap().is_empty());
            });
        });
    }

    group.finish();
}

fn benchmark_plane_detection(c: &mut Criterion) {
    use board_fitter::plane::{PlaneDetectionConfig, RansacPlaneDetector};

    let mut generator = TestDataGenerator::new(101);
    let board_config = create_test_board_config(1.0);
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0); // Tilted 45° for diamond board

    // Generate test cloud with multiple planes
    let cloud1 = generator.generate_perfect_board(&board_config, &pose, 1000);
    let pose2 = create_board_pose(3.0, 1.0, 1.0, 0.0, 0.3, 0.0);
    let cloud2 = generator.generate_perfect_board(&board_config, &pose2, 1000);

    // Combine clouds
    let mut combined_points = cloud1.points.clone();
    combined_points.extend(cloud2.points);
    let combined_cloud = PointCloud {
        points: combined_points,
        intensities: None,
        colors: None,
        timestamp: Instant::now(),
        frame_id: "test_frame".to_string(),
    };

    let config = PlaneDetectionConfig::default();
    let _detector = RansacPlaneDetector::new(config.clone());

    let mut group = c.benchmark_group("plane_detection");

    // Benchmark with different RANSAC iterations
    let iterations = [100, 500, 1000, 2000];

    for &iter in &iterations {
        let mut custom_config = config.clone();
        custom_config.ransac_iterations = iter;
        let mut detector = RansacPlaneDetector::new(custom_config);

        group.bench_with_input(
            BenchmarkId::new("iterations", iter),
            &combined_cloud,
            |b, cloud| {
                b.iter(|| {
                    let planes = detector.detect_planes(black_box(cloud)).unwrap();
                    assert!(!planes.is_empty());
                });
            },
        );
    }

    group.finish();
}

fn benchmark_hole_detection(c: &mut Criterion) {
    use board_fitter::{
        diamond::DiamondSquareFitter,
        hole::{HoleDetectionConfig, HoleDetector},
        plane::{PlaneDetectionConfig, RansacPlaneDetector},
    };

    let mut generator = TestDataGenerator::new(102);
    let board_config = create_test_board_config(1.0);
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0); // Tilted 45° for diamond board
    let cloud = generator.generate_perfect_board(&board_config, &pose, 1000);

    // First detect plane and fit square
    let mut plane_detector = RansacPlaneDetector::new(PlaneDetectionConfig::default());
    let planes = plane_detector.detect_planes(&cloud).unwrap();
    let plane = &planes[0];

    let square_fitter = DiamondSquareFitter::from_board_config(&board_config);
    let square = square_fitter.fit_square(&cloud, plane).unwrap().unwrap();

    let hole_detector = HoleDetector::new(HoleDetectionConfig::default());

    c.bench_function("hole_detection", |b| {
        b.iter(|| {
            let holes = hole_detector
                .detect_holes_in_square(black_box(&cloud), black_box(&square))
                .unwrap();
            assert!(!holes.is_empty());
        });
    });
}

fn benchmark_with_noise(c: &mut Criterion) {
    let board_config = create_test_board_config(1.0);

    let mut generator = TestDataGenerator::new(103);
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0); // Tilted 45° for diamond board
    let perfect_cloud = generator.generate_perfect_board(&board_config, &pose, 1000);

    let mut group = c.benchmark_group("noise_robustness");

    let noise_levels = [0.0, 0.01, 0.02, 0.05]; // 0cm, 1cm, 2cm, 5cm

    for &noise in &noise_levels {
        let noisy_cloud = if noise > 0.0 {
            generator.add_noise(&perfect_cloud, noise)
        } else {
            perfect_cloud.clone()
        };

        group.bench_with_input(
            BenchmarkId::new("noise_cm", (noise * 100.0) as u32),
            &noisy_cloud,
            |b, cloud| {
                b.iter(|| {
                    let config = Config {
                        board: board_config.clone(),
                        detection: None,
                        metadata: None,
                    };
                    let mut detector = DiamondBoardDetector::new(config).unwrap();
                    let result = detector.detect(black_box(cloud)).unwrap();
                    assert!(!result.is_empty());
                });
            },
        );
    }

    group.finish();
}

fn benchmark_roi_preprocessing(c: &mut Criterion) {
    use board_fitter::{roi::AdaptivePreprocessor, RoiType};

    let mut generator = TestDataGenerator::new(104);
    let board_config = create_test_board_config(1.0);
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0); // Tilted 45° for diamond board

    // Generate dense point cloud
    let cloud = generator.generate_perfect_board(&board_config, &pose, 10000);

    let preprocessor = AdaptivePreprocessor::new();

    let mut group = c.benchmark_group("roi_preprocessing");

    let roi_types = [
        ("global", RoiType::GlobalSearch),
        ("local", RoiType::LocalTracking),
        ("expanding", RoiType::ExpandingSearch),
    ];

    for (name, roi_type) in &roi_types {
        group.bench_with_input(BenchmarkId::new("roi_type", name), &cloud, |b, cloud| {
            b.iter(|| {
                let processed = preprocessor
                    .preprocess(black_box(cloud), black_box(*roi_type))
                    .unwrap();
                assert!(processed.points.len() <= cloud.points.len());
            });
        });
    }

    group.finish();
}

fn benchmark_debug_overhead(c: &mut Criterion) {
    use board_fitter::debug::{DebugConfigBuilder, DebugContext};

    let board_config = create_test_board_config(1.0);
    let mut generator = TestDataGenerator::new(105);
    let pose = create_board_pose(2.0, 0.0, 1.0, 45.0_f64.to_radians(), 0.0, 0.0); // Tilted 45° for diamond board
    let cloud = generator.generate_perfect_board(&board_config, &pose, 1000);

    let config = Config {
        board: board_config.clone(),
        detection: None,
        metadata: None,
    };

    // Config is created inside the benchmark iterations

    let mut group = c.benchmark_group("debug_overhead");

    group.bench_with_input(BenchmarkId::new("debug", "disabled"), &cloud, |b, cloud| {
        b.iter(|| {
            let config = Config {
                board: board_config.clone(),
                detection: None,
                metadata: None,
            };
            let mut detector = DiamondBoardDetector::new(config).unwrap();
            let result = detector.detect(black_box(cloud)).unwrap();
            assert!(!result.is_empty());
        });
    });

    group.bench_with_input(BenchmarkId::new("debug", "enabled"), &cloud, |b, cloud| {
        b.iter(|| {
            let config = Config {
                board: board_config.clone(),
                detection: None,
                metadata: None,
            };
            let mut detector = DiamondBoardDetector::new(config).unwrap();

            let debug_config = DebugConfigBuilder::new()
                .with_timing()
                .verbosity(board_fitter::DebugVerbosity::Detailed)
                .capture_stages(["plane_detection", "diamond_fitting", "hole_detection"])
                .build();

            let debug_context = DebugContext::new(debug_config);
            detector.with_debug_context(debug_context);

            let result = detector.detect(black_box(cloud)).unwrap();
            assert!(!result.is_empty());
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    benchmark_detection_pipeline,
    benchmark_plane_detection,
    benchmark_hole_detection,
    benchmark_with_noise,
    benchmark_roi_preprocessing,
    benchmark_debug_overhead
);
criterion_main!(benches);
