use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use hollow_board_detector::algo::voxel_downsample;
use nalgebra::Point3;
use rand::{rngs::StdRng, Rng, SeedableRng};

fn generate_random_cloud(num_points: usize, seed: u64) -> Vec<Point3<f64>> {
    let mut rng = StdRng::seed_from_u64(seed);
    (0..num_points)
        .map(|_| {
            Point3::new(
                rng.gen::<f64>() * 10.0,
                rng.gen::<f64>() * 10.0,
                rng.gen::<f64>() * 10.0,
            )
        })
        .collect()
}

fn bench_voxel_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("voxel_size");
    let points = generate_random_cloud(10_000, 42);

    for voxel_size in [0.01, 0.02, 0.03, 0.05].iter() {
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{:.2}m", voxel_size)),
            voxel_size,
            |b, &size| {
                b.iter(|| {
                    voxel_downsample(
                        black_box(&points),
                        black_box(size),
                        black_box(true),
                        black_box(50_000),
                    )
                });
            },
        );
    }
    group.finish();
}

fn bench_point_counts(c: &mut Criterion) {
    let mut group = c.benchmark_group("point_count");

    for &count in [1_000, 5_000, 10_000, 50_000, 100_000].iter() {
        let points = generate_random_cloud(count, 42);
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{}pts", count)),
            &points,
            |b, pts| {
                b.iter(|| {
                    voxel_downsample(
                        black_box(pts),
                        black_box(0.02),
                        black_box(true),
                        black_box(50_000),
                    )
                });
            },
        );
    }
    group.finish();
}

fn bench_centroid_vs_first(c: &mut Criterion) {
    let mut group = c.benchmark_group("strategy");
    let points = generate_random_cloud(10_000, 42);

    group.bench_function("centroid", |b| {
        b.iter(|| {
            voxel_downsample(
                black_box(&points),
                black_box(0.02),
                black_box(true),
                black_box(50_000),
            )
        });
    });

    group.bench_function("first_point", |b| {
        b.iter(|| {
            voxel_downsample(
                black_box(&points),
                black_box(0.02),
                black_box(false),
                black_box(50_000),
            )
        });
    });

    group.finish();
}

#[cfg(feature = "parallel")]
fn bench_parallel_threshold(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_threshold");

    // Test with point count near threshold
    for &count in [40_000, 50_000, 60_000, 100_000].iter() {
        let points = generate_random_cloud(count, 42);
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{}pts", count)),
            &points,
            |b, pts| {
                b.iter(|| {
                    voxel_downsample(
                        black_box(pts),
                        black_box(0.02),
                        black_box(true),
                        black_box(50_000),
                    )
                });
            },
        );
    }
    group.finish();
}

#[cfg(feature = "parallel")]
criterion_group!(
    benches,
    bench_voxel_sizes,
    bench_point_counts,
    bench_centroid_vs_first,
    bench_parallel_threshold,
);

#[cfg(not(feature = "parallel"))]
criterion_group!(
    benches,
    bench_voxel_sizes,
    bench_point_counts,
    bench_centroid_vs_first,
);

criterion_main!(benches);
