//! Example showing how to use BoardDetector with debug callbacks to get intermediate outputs

use board_fitter::{
    debug::{
        AlgorithmStats, DataCallback, DebugConfigBuilder, DebugContext, DebugData, MetricsCallback,
        StageMetrics, TimingCallback,
    },
    BoardDetectorBuilder, PointCloud,
};
use board_fitter_config::{BoardConfig, Point2D, SquareBoard};
use measurements::Length;
use nalgebra::Point3;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

/// Custom debug callback that collects all intermediate data
#[derive(Default)]
struct CollectingDebugCallback {
    /// Timing information for each stage
    pub timings: Arc<Mutex<Vec<(String, Duration)>>>,
    /// Intermediate data from each stage
    pub stage_data: Arc<Mutex<HashMap<String, Vec<DebugData>>>>,
    /// Metrics from each stage
    pub stage_metrics: Arc<Mutex<HashMap<String, StageMetrics>>>,
    /// Algorithm statistics
    pub algorithm_stats: Arc<Mutex<HashMap<String, AlgorithmStats>>>,
}

impl CollectingDebugCallback {
    fn new() -> Self {
        Self::default()
    }
}

impl TimingCallback for CollectingDebugCallback {
    fn on_stage_start(&self, stage: &str, _timestamp: Instant) {
        println!("🚀 Stage started: {stage}");
    }

    fn on_stage_end(&self, stage: &str, duration: Duration, memory_usage: Option<usize>) {
        println!("✅ Stage completed: {stage} in {duration:?}");
        if let Some(mem) = memory_usage {
            println!("   Memory used: {} MB", mem / 1_048_576);
        }

        self.timings
            .lock()
            .unwrap()
            .push((stage.to_string(), duration));
    }
}

impl DataCallback for CollectingDebugCallback {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        println!("📊 Intermediate data from stage: {stage}");

        match data {
            DebugData::PointCloud { cloud, metadata } => {
                println!("   Point cloud: {} points", cloud.points.len());
                for (key, value) in metadata {
                    println!("   {key}: {value}");
                }
            }
            DebugData::DetectionResult {
                detections,
                confidence_scores,
                metadata,
            } => {
                println!("   Detections: {}", detections.len());
                for (i, score) in confidence_scores.iter().enumerate() {
                    println!("   Detection {i}: confidence = {score:.3}");
                }
                for (key, value) in metadata {
                    println!("   {key}: {value}");
                }
            }
            DebugData::PlaneData {
                planes,
                inlier_counts,
                quality_scores,
                metadata,
            } => {
                println!("   Planes detected: {}", planes.len());
                for (i, (inliers, quality)) in inlier_counts.iter().zip(quality_scores).enumerate()
                {
                    println!("   Plane {i}: {inliers} inliers, quality = {quality:.3}");
                }
                for (key, value) in metadata {
                    println!("   {key}: {value}");
                }
            }
            DebugData::CircleData {
                holes,
                fitting_residuals,
                iteration_counts,
                metadata,
            } => {
                println!("   Holes detected: {}", holes.len());
                for (i, (residual, iterations)) in
                    fitting_residuals.iter().zip(iteration_counts).enumerate()
                {
                    println!("   Hole {i}: residual = {residual:.6}, iterations = {iterations}");
                }
                for (key, value) in metadata {
                    println!("   {key}: {value}");
                }
            }
            DebugData::Generic { data } => {
                for (key, value) in data {
                    println!("   {key}: {value}");
                }
            }
        }

        self.stage_data
            .lock()
            .unwrap()
            .entry(stage.to_string())
            .or_default()
            .push(data.clone());
    }

    fn on_point_cloud(&self, stage: &str, cloud: &PointCloud) {
        println!(
            "☁️  Point cloud from stage '{}': {} points",
            stage,
            cloud.points.len()
        );

        // Show sample points
        if !cloud.points.is_empty() {
            println!("   First point: {:?}", cloud.points[0]);
            if cloud.points.len() > 1 {
                println!("   Last point: {:?}", cloud.points[cloud.points.len() - 1]);
            }
        }

        // Show intensity statistics if available
        if let Some(intensities) = &cloud.intensities {
            let min = intensities
                .iter()
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            let max = intensities
                .iter()
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            let avg = intensities.iter().sum::<f32>() / intensities.len() as f32;
            println!("   Intensities: min={min:.1}, max={max:.1}, avg={avg:.1}");
        }
    }
}

impl MetricsCallback for CollectingDebugCallback {
    fn on_metrics(&self, stage: &str, metrics: &StageMetrics) {
        println!("📈 Metrics for stage '{stage}':");
        println!("   Input points: {}", metrics.input_points);
        println!("   Output points: {}", metrics.output_points);
        println!("   Processing time: {:?}", metrics.processing_time);

        if let Some(mem) = metrics.memory_usage {
            println!("   Memory usage: {} MB", mem / 1_048_576);
        }

        for (key, value) in &metrics.custom_metrics {
            println!("   {key}: {value:.3}");
        }

        self.stage_metrics
            .lock()
            .unwrap()
            .insert(stage.to_string(), metrics.clone());
    }

    fn on_algorithm_stats(&self, stage: &str, stats: &AlgorithmStats) {
        println!("🔬 Algorithm statistics for stage '{stage}':");
        println!("   Algorithm: {}", stats.algorithm);
        println!("   Iterations: {}", stats.iterations);
        println!("   Converged: {}", stats.converged);

        if let Some(error) = stats.final_error {
            println!("   Final error: {error:.6}");
        }

        if let Some(tol) = stats.tolerance {
            println!("   Tolerance: {tol:.6}");
        }

        for (key, value) in &stats.custom_stats {
            println!("   {key}: {value}");
        }

        self.algorithm_stats
            .lock()
            .unwrap()
            .insert(stage.to_string(), stats.clone());
    }
}

fn main() {
    println!("Board Detection with Debug Output Example");
    println!("========================================\n");

    // Create a test board configuration
    let mut board = SquareBoard::new(Length::from_meters(1.0));

    // Add holes in a pattern
    board.add_hole(
        Length::from_meters(0.1),
        Point2D {
            x: Length::from_meters(0.0),
            y: Length::from_meters(0.3),
        },
        Some("top_hole".to_string()),
    );

    board.add_hole(
        Length::from_meters(0.05),
        Point2D {
            x: Length::from_meters(-0.3),
            y: Length::from_meters(0.0),
        },
        Some("left_hole".to_string()),
    );

    board.add_hole(
        Length::from_meters(0.05),
        Point2D {
            x: Length::from_meters(0.3),
            y: Length::from_meters(0.0),
        },
        Some("right_hole".to_string()),
    );

    let board_config = BoardConfig {
        board,
        detection: None,
        metadata: None,
    };

    // Create debug callback
    let debug_callback = Arc::new(CollectingDebugCallback::new());

    // Create debug configuration
    let debug_config = DebugConfigBuilder::new()
        .with_timing()
        .with_memory_tracking()
        .capture_stages([
            "preprocessing",
            "plane_detection",
            "diamond_fitting",
            "hole_detection",
            "validation",
            "board_tracking",
        ])
        .max_point_clouds(20)
        .build();

    // Create debug context
    let mut debug_context = DebugContext::new(debug_config.clone());
    debug_context.timing_callback = Some(debug_callback.clone());
    debug_context.data_callback = Some(debug_callback.clone());
    debug_context.metrics_callback = Some(debug_callback.clone());

    // Note: The BoardDetectorBuilder takes DebugConfig, not DebugContext.
    // The callbacks need to be set up differently - this is just for demonstration.
    // In a real implementation, you would need to integrate the callbacks into the
    // detector's processing pipeline.

    // Create detector with debug configuration
    let mut detector = BoardDetectorBuilder::new(board_config)
        .with_debug(debug_config) // Pass debug_config, not debug_context
        .timeout_ms(10000) // 10 seconds
        .min_confidence(0.7)
        .build()
        .expect("Failed to create detector");

    // Create a simple test point cloud
    let mut points = Vec::new();
    let mut intensities = Vec::new();

    // Generate a tilted square pattern
    for i in 0..50 {
        for j in 0..50 {
            let x = -0.5 + (i as f64 / 49.0);
            let y = -0.5 + (j as f64 / 49.0);
            let z = 2.0 + 0.1 * x + 0.05 * y; // Tilted plane

            // Skip points inside holes
            let dist_to_top = ((x - 0.0).powi(2) + (y - 0.3).powi(2)).sqrt();
            let dist_to_left = ((x + 0.3).powi(2) + (y - 0.0).powi(2)).sqrt();
            let dist_to_right = ((x - 0.3).powi(2) + (y - 0.0).powi(2)).sqrt();

            if dist_to_top < 0.1 || dist_to_left < 0.05 || dist_to_right < 0.05 {
                continue; // Skip points inside holes
            }

            points.push(Point3::new(x, y, z));

            // Lower intensity near holes
            let min_dist = dist_to_top.min(dist_to_left).min(dist_to_right);
            let intensity = (min_dist * 500.0).min(128.0) as f32;
            intensities.push(intensity);
        }
    }

    let point_cloud = PointCloud {
        points,
        intensities: Some(intensities),
        colors: None,
        timestamp: Instant::now(),
        frame_id: "test_frame".to_string(),
    };

    println!("Input point cloud: {} points\n", point_cloud.points.len());

    // Run detection
    println!("Starting detection...\n");
    match detector.detect(&point_cloud) {
        Ok(result) => {
            println!("\n✨ Detection completed successfully!");
            println!("   Boards detected: {}", result.detections.len());

            for (i, detection) in result.detections.iter().enumerate() {
                println!("\n   Board {i}:");
                println!("     ID: {}", detection.id);
                println!("     Confidence: {:.3}", detection.confidence.value());
                println!(
                    "     Position: [{:.3}, {:.3}, {:.3}]",
                    detection.pose.translation.x,
                    detection.pose.translation.y,
                    detection.pose.translation.z
                );
                println!("     Holes detected: {}", detection.holes.len());
                for (j, hole) in detection.holes.iter().enumerate() {
                    println!(
                        "       Hole {}: center=[{:.3}, {:.3}, {:.3}], radius={:.3}m",
                        j, hole.center.x, hole.center.y, hole.center.z, hole.radius
                    );
                }
            }

            // Print processing statistics
            println!("\n📊 Processing Statistics:");
            println!("   Total processing time: {:?}", result.stats.total_time);
            println!("   Points processed: {}", result.stats.points_processed);
            println!("   Planes detected: {}", result.stats.planes_detected);
            println!("   Boards detected: {}", result.stats.boards_detected);

            // Print collected debug information
            println!("\n🔍 Debug Information Summary:");

            let timings = debug_callback.timings.lock().unwrap();
            println!("\n⏱️  Stage Timings:");
            for (stage, duration) in timings.iter() {
                println!("   {stage}: {duration:?}");
            }

            let stage_data = debug_callback.stage_data.lock().unwrap();
            println!("\n📦 Intermediate Data Collected:");
            for (stage, data_vec) in stage_data.iter() {
                println!("   {}: {} data items", stage, data_vec.len());
            }

            let metrics = debug_callback.stage_metrics.lock().unwrap();
            println!("\n📈 Stage Metrics:");
            for (stage, metric) in metrics.iter() {
                let reduction = if metric.input_points > 0 {
                    100.0 * (1.0 - metric.output_points as f64 / metric.input_points as f64)
                } else {
                    0.0
                };
                println!(
                    "   {}: {} → {} points ({:.1}% reduction)",
                    stage, metric.input_points, metric.output_points, reduction
                );
            }
        }
        Err(e) => {
            println!("❌ Detection failed: {e}");
        }
    }
}
