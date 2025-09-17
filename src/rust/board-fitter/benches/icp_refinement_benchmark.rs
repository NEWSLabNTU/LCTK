//! Performance benchmarks for ICP refinement stages

use board_fitter::{
    refinement::{config::IcpConfigBuilder, IcpRefinement, IcpRefinementConfig},
    types::DetectionConfidence,
};
use board_fitter_config::BoardConfig;
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::PI;

// Import test utilities
#[path = "../tests/common/mod.rs"]
mod common;
use common::*;

/// Generate noisy point cloud for ICP testing
fn generate_noisy_board_cloud(
    pose: &Isometry3<f64>,
    board_size: f64,
    num_points: usize,
    noise_level: f64,
) -> Vec<Point3<f64>> {
    use rand::{Rng, SeedableRng};
    use rand_chacha::ChaCha8Rng;

    let mut rng = ChaCha8Rng::seed_from_u64(42);
    let mut points = Vec::new();

    let points_per_side = (num_points as f64).sqrt() as usize;
    let spacing = board_size / points_per_side as f64;

    for i in 0..points_per_side {
        for j in 0..points_per_side {
            let x = -board_size / 2.0 + i as f64 * spacing;
            let y = -board_size / 2.0 + j as f64 * spacing;

            // Add Gaussian noise
            let noise_x = rng.gen_range(-noise_level..noise_level);
            let noise_y = rng.gen_range(-noise_level..noise_level);
            let noise_z = rng.gen_range(-noise_level..noise_level);

            let local_point = Point3::new(x + noise_x, y + noise_y, noise_z);
            let world_point = pose * local_point;

            points.push(world_point);
        }
    }

    points
}

fn benchmark_board_pose_refinement(c: &mut Criterion) {
    let mut group = c.benchmark_group("board_pose_refinement");

    // Create ICP refinement with default config
    let icp_config = IcpRefinementConfig::default();
    let refiner = IcpRefinement::new(icp_config);

    // Test different point cloud sizes
    let point_counts = [100, 500, 1000, 2500, 5000];

    for &count in &point_counts {
        group.throughput(Throughput::Elements(count as u64));

        group.bench_with_input(BenchmarkId::new("points", count), &count, |b, &count| {
            // Generate source and target point clouds
            let true_pose = Isometry3::identity();
            let perturbed_pose = Isometry3::from_parts(
                Translation3::new(0.05, 0.05, 0.02),
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.1),
            );

            let source_points = generate_noisy_board_cloud(&perturbed_pose, 1.0, count, 0.001);
            let target_points = generate_noisy_board_cloud(&true_pose, 1.0, count, 0.001);

            b.iter(|| {
                let result = refiner.refine_board_pose(
                    black_box(&source_points),
                    black_box(&target_points),
                    black_box(&perturbed_pose),
                    None,
                );
                assert!(result.is_ok());
            });
        });
    }

    group.finish();
}

fn benchmark_detection_with_without_icp(c: &mut Criterion) {
    use board_fitter::BoardDetectorBuilder;

    let mut group = c.benchmark_group("detection_icp_comparison");

    let board_config = create_test_board_config(1.0);
    let mut generator = TestDataGenerator::new(100);
    let pose = create_board_pose(2.0, 0.0, 1.0, PI / 4.0, 0.0, 0.0);

    // Generate test point cloud
    let point_cloud = generator.generate_perfect_board(&board_config, &pose, 5000);

    group.bench_function("without_icp", |b| {
        let board_config_instance = BoardConfig {
            board: board_config.clone(),
            detection: None,
            metadata: None,
        };

        b.iter(|| {
            let mut detector = BoardDetectorBuilder::new(board_config_instance.clone())
                .timeout_ms(30000) // 30 seconds timeout
                .build()
                .unwrap();
            let result = detector.detect(black_box(&point_cloud));
            match result {
                Ok(_) => {
                    // Success - continue benchmark
                }
                Err(e) => {
                    panic!("Detection failed: {e}");
                }
            }
        });
    });

    group.bench_function("with_icp", |b| {
        let board_config_instance = BoardConfig {
            board: board_config.clone(),
            detection: None,
            metadata: None,
        };

        b.iter(|| {
            let mut detector = BoardDetectorBuilder::new(board_config_instance.clone())
                .timeout_ms(30000) // 30 seconds timeout
                .with_fast_icp() // Enable ICP
                .build()
                .unwrap();
            let result = detector.detect(black_box(&point_cloud));
            match result {
                Ok(_) => {
                    // Success - continue benchmark
                }
                Err(e) => {
                    panic!("Detection failed: {e}");
                }
            }
        });
    });

    group.finish();
}

fn benchmark_square_pose_refinement(c: &mut Criterion) {
    let mut group = c.benchmark_group("square_pose_refinement");

    // Create ICP refinement with square refinement enabled
    let icp_config = IcpConfigBuilder::new()
        .with_square_refinement(true)
        .build()
        .unwrap();
    let refiner = IcpRefinement::new(icp_config);

    // Generate square region points
    let square_size = 1.0;
    let num_points = 500;
    let initial_pose = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.05), // Small rotation error
    );

    let square_points = generate_noisy_board_cloud(&initial_pose, square_size, num_points, 0.002);
    let plane_normal = Vector3::new(0.0, 0.0, 1.0);

    group.bench_function("refine_square", |b| {
        b.iter(|| {
            let result = refiner.refine_square_pose(
                black_box(&square_points),
                black_box(square_size),
                black_box(&initial_pose),
                black_box(&plane_normal),
                None,
            );
            assert!(result.is_ok());
        });
    });

    group.finish();
}

fn benchmark_hole_pattern_alignment(c: &mut Criterion) {
    use board_fitter::{
        refinement::hole_pattern_alignment::{HolePattern, HoleTemplate},
        types::DetectedHole,
    };

    let mut group = c.benchmark_group("hole_pattern_alignment");

    // Create ICP refinement with hole alignment enabled
    let icp_config = IcpConfigBuilder::new()
        .with_hole_alignment(true)
        .build()
        .unwrap();
    let refiner = IcpRefinement::new(icp_config);

    // Create test hole pattern
    let pattern = HolePattern {
        holes: vec![
            HoleTemplate {
                position: Point3::new(0.0, 0.0, 0.0),
                radius: 0.1,
                variant: 0,
            },
            HoleTemplate {
                position: Point3::new(0.3, 0.0, 0.0),
                radius: 0.05,
                variant: 1,
            },
            HoleTemplate {
                position: Point3::new(0.0, 0.3, 0.0),
                radius: 0.05,
                variant: 1,
            },
        ],
        min_holes: 2,
    };

    // Create detected holes with some error
    let transform_error = Isometry3::from_parts(
        Translation3::new(0.02, 0.01, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.05),
    );

    let detected_holes: Vec<DetectedHole> = pattern
        .holes
        .iter()
        .map(|template| DetectedHole {
            center: transform_error * template.position,
            radius: template.radius,
            confidence: DetectionConfidence::new(0.9),
            id: None,
        })
        .collect();

    group.bench_function("align_holes", |b| {
        b.iter(|| {
            let result = refiner.align_hole_pattern(
                black_box(&detected_holes),
                black_box(&pattern),
                None,
                None,
            );
            assert!(result.is_ok());
        });
    });

    group.finish();
}

fn benchmark_icp_configurations(c: &mut Criterion) {
    let mut group = c.benchmark_group("icp_configurations");

    // Generate test data
    let num_points = 1000;
    let true_pose = Isometry3::identity();
    let perturbed_pose = Isometry3::from_parts(
        Translation3::new(0.05, 0.05, 0.02),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.1),
    );

    let source_points = generate_noisy_board_cloud(&perturbed_pose, 1.0, num_points, 0.001);
    let target_points = generate_noisy_board_cloud(&true_pose, 1.0, num_points, 0.001);

    // Test different configurations
    let configs = vec![
        ("default", IcpRefinementConfig::default()),
        ("high_precision", {
            let mut config = IcpRefinementConfig::default();
            config.board_pose_refinement.max_iterations = 100;
            config
                .board_pose_refinement
                .convergence_criteria
                .rotation_epsilon = 0.0001;
            config
                .board_pose_refinement
                .convergence_criteria
                .translation_epsilon = 0.00001;
            config
        }),
        ("fast", {
            let mut config = IcpRefinementConfig::default();
            config.board_pose_refinement.max_iterations = 10;
            config.board_pose_refinement.downsampling_resolution = Some(0.05);
            config
        }),
        ("all_stages", {
            let mut config = IcpRefinementConfig::default();
            config.square_pose_refinement.enabled = true;
            config.hole_pattern_alignment.enabled = true;
            config.board_pose_refinement.enabled = true;
            config.temporal_alignment.enabled = true;
            config
        }),
    ];

    for (name, config) in configs {
        group.bench_function(name, |b| {
            let refiner = IcpRefinement::new(config.clone());

            b.iter(|| {
                let result = refiner.refine_board_pose(
                    black_box(&source_points),
                    black_box(&target_points),
                    black_box(&perturbed_pose),
                    None,
                );
                assert!(result.is_ok());
            });
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    benchmark_board_pose_refinement,
    benchmark_detection_with_without_icp,
    benchmark_square_pose_refinement,
    benchmark_hole_pattern_alignment,
    benchmark_icp_configurations,
);
criterion_main!(benches);
