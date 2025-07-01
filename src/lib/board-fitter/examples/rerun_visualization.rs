//! Visualize board detection pipeline using Rerun
//!
//! This example shows how to use BoardDetector with debug callbacks to visualize
//! intermediate processing stages in Rerun.
//!
//! Prerequisites:
//! - Install Rerun viewer: https://www.rerun.io/docs/getting-started/installing-viewer
//! - Add to Cargo.toml: rerun = "0.23"
//!
//! Usage:
//! 1. Start Rerun viewer: `rerun`
//! 2. Run this example: `cargo run --example rerun_visualization`

use board_fitter::{
    debug::{
        AlgorithmStats, DataCallback, DebugConfigBuilder, DebugContext, DebugData, MetricsCallback,
        StageMetrics, TimingCallback,
    },
    BoardDetectorBuilder, PointCloud,
};
use board_fitter_config::{BoardConfig, Point2D, SquareBoard};
use measurements::Length;
use nalgebra::{Point3, Vector3};
use rerun::RecordingStream;
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

/// Rerun visualization callback
struct RerunCallback {
    rec: RecordingStream,
}

impl RerunCallback {
    fn new(app_id: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let rec = rerun::RecordingStreamBuilder::new(app_id).spawn()?;

        Ok(Self { rec })
    }

    /// Convert board-fitter PointCloud to rerun points
    fn log_point_cloud(
        &self,
        path: &str,
        cloud: &PointCloud,
        colors: Option<Vec<[u8; 3]>>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let points: Vec<[f32; 3]> = cloud
            .points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();

        if let Some(colors) = colors {
            self.rec.log(
                path,
                &rerun::Points3D::new(points)
                    .with_colors(colors)
                    .with_radii(vec![0.002f32; cloud.points.len()]),
            )?;
        } else if let Some(intensities) = &cloud.intensities {
            // Map intensities to colors
            let colors: Vec<[u8; 3]> = intensities
                .iter()
                .map(|&i| {
                    let normalized = (i / 255.0).clamp(0.0, 1.0);
                    let val = (normalized * 255.0) as u8;
                    [val, val, val] // Grayscale based on intensity
                })
                .collect();

            self.rec.log(
                path,
                &rerun::Points3D::new(points)
                    .with_colors(colors)
                    .with_radii(vec![0.002f32; cloud.points.len()]),
            )?;
        } else {
            self.rec.log(
                path,
                &rerun::Points3D::new(points).with_radii(vec![0.002f32; cloud.points.len()]),
            )?;
        }
        Ok(())
    }

    /// Log a plane as a mesh
    fn log_plane(
        &self,
        path: &str,
        plane: &board_fitter::types::DetectedPlane,
        size: f32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Create a square mesh for the plane
        let normal = Vector3::new(plane.normal.x, plane.normal.y, plane.normal.z);
        let up = if normal.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };

        let right = normal.cross(&up).normalize() * size as f64;
        let up = normal.cross(&right).normalize() * size as f64;
        let center = plane.point; // Use plane.point instead of plane.center

        let vertices = [
            center - right - up,
            center + right - up,
            center + right + up,
            center - right + up,
        ];

        let positions: Vec<[f32; 3]> = vertices
            .iter()
            .map(|v| [v.x as f32, v.y as f32, v.z as f32])
            .collect();

        let indices = vec![[0u32, 1, 2], [0, 2, 3]];

        self.rec.log(
            path,
            &rerun::Mesh3D::new(positions)
                .with_triangle_indices(indices)
                .with_vertex_colors(vec![[100u8, 200u8, 255u8]; 4]),
        )?; // Semi-transparent blue

        // Also log plane normal
        let arrow_origin = [center.x as f32, center.y as f32, center.z as f32];
        let arrow_vector = [
            normal.x as f32 * 0.5,
            normal.y as f32 * 0.5,
            normal.z as f32 * 0.5,
        ];

        self.rec.log(
            format!("{path}/normal"),
            &rerun::Arrows3D::from_vectors(vec![arrow_vector])
                .with_origins(vec![arrow_origin])
                .with_colors(vec![[255u8, 255u8, 0u8]]),
        )?;

        Ok(())
    }

    /// Log detected holes as circles
    fn log_holes(
        &self,
        path: &str,
        holes: &[board_fitter::types::DetectedHole],
    ) -> Result<(), Box<dyn std::error::Error>> {
        for (i, hole) in holes.iter().enumerate() {
            // Log hole center
            self.rec.log(
                format!("{path}/hole_{i}/center"),
                &rerun::Points3D::new(vec![[
                    hole.center.x as f32,
                    hole.center.y as f32,
                    hole.center.z as f32,
                ]])
                .with_colors(vec![[255u8, 0u8, 0u8]])
                .with_radii(vec![hole.radius as f32]),
            )?;

            // Create circle outline
            let num_points = 32;
            let mut circle_points = Vec::new();

            for j in 0..num_points {
                let angle = 2.0 * std::f64::consts::PI * j as f64 / num_points as f64;
                let x = hole.center.x + hole.radius * angle.cos();
                let y = hole.center.y + hole.radius * angle.sin();
                let z = hole.center.z;
                circle_points.push([x as f32, y as f32, z as f32]);
            }
            circle_points.push(circle_points[0]); // Close the circle

            self.rec.log(
                format!("{path}/hole_{i}/outline"),
                &rerun::LineStrips3D::new(vec![circle_points]).with_colors(vec![[255u8, 0u8, 0u8]]),
            )?;
        }
        Ok(())
    }

    /// Log board detection as a bounding box
    fn log_board(
        &self,
        path: &str,
        detection: &board_fitter::types::BoardDetection,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pose = &detection.pose;
        let dims = &detection.dimensions;

        // Create box corners
        let half_dims = dims * 0.5;
        let corners_local = [
            Point3::new(-half_dims.x, -half_dims.y, -half_dims.z),
            Point3::new(half_dims.x, -half_dims.y, -half_dims.z),
            Point3::new(half_dims.x, half_dims.y, -half_dims.z),
            Point3::new(-half_dims.x, half_dims.y, -half_dims.z),
            Point3::new(-half_dims.x, -half_dims.y, half_dims.z),
            Point3::new(half_dims.x, -half_dims.y, half_dims.z),
            Point3::new(half_dims.x, half_dims.y, half_dims.z),
            Point3::new(-half_dims.x, half_dims.y, half_dims.z),
        ];

        // Transform to world coordinates
        let corners: Vec<[f32; 3]> = corners_local
            .iter()
            .map(|&c| {
                let world = pose * c;
                [world.x as f32, world.y as f32, world.z as f32]
            })
            .collect();

        // Define box edges
        let edges = vec![
            // Bottom face
            vec![corners[0], corners[1], corners[2], corners[3], corners[0]],
            // Top face
            vec![corners[4], corners[5], corners[6], corners[7], corners[4]],
            // Vertical edges
            vec![corners[0], corners[4]],
            vec![corners[1], corners[5]],
            vec![corners[2], corners[6]],
            vec![corners[3], corners[7]],
        ];

        self.rec.log(
            format!("{path}/box"),
            &rerun::LineStrips3D::new(edges).with_colors(vec![[0u8, 255u8, 0u8]]),
        )?;

        // Log confidence as text
        self.rec.log(
            format!("{path}/confidence"),
            &rerun::TextLog::new(format!("Confidence: {:.3}", detection.confidence.value())),
        )?;

        // Log detected holes
        if !detection.holes.is_empty() {
            self.log_holes(&format!("{path}/holes"), &detection.holes)?;
        }
        Ok(())
    }
}

impl TimingCallback for RerunCallback {
    fn on_stage_start(&self, stage: &str, _timestamp: Instant) {
        self.rec
            .log(
                "pipeline/events",
                &rerun::TextLog::new(format!("▶️ Stage started: {stage}")),
            )
            .ok();
    }

    fn on_stage_end(&self, stage: &str, duration: Duration, memory_usage: Option<usize>) {
        let msg = if let Some(mem) = memory_usage {
            format!(
                "✅ Stage {stage}: {duration:?} ({mem}MB)",
                mem = mem / 1_048_576
            )
        } else {
            format!("✅ Stage {stage}: {duration:?}")
        };

        self.rec
            .log("pipeline/events", &rerun::TextLog::new(msg))
            .ok();

        // Log timing as scalar
        self.rec
            .log(
                format!("metrics/timing/{stage}"),
                &rerun::Scalars::new([duration.as_secs_f64() * 1000.0]),
            )
            .ok();
    }
}

impl DataCallback for RerunCallback {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        match data {
            DebugData::PointCloud { cloud, metadata } => {
                let _ = self.log_point_cloud(&format!("pipeline/{stage}/cloud"), cloud, None);

                // Log metadata
                for (key, value) in metadata {
                    self.rec
                        .log(
                            format!("pipeline/{stage}/metadata/{key}"),
                            &rerun::TextLog::new(value.as_str()),
                        )
                        .ok();
                }
            }

            DebugData::PlaneData {
                planes,
                inlier_counts,
                quality_scores,
                ..
            } => {
                for (i, (plane, (&inliers, &quality))) in planes
                    .iter()
                    .zip(inlier_counts.iter().zip(quality_scores))
                    .enumerate()
                {
                    let _ = self.log_plane(&format!("pipeline/{stage}/plane_{i}"), plane, 1.0);

                    self.rec
                        .log(
                            format!("pipeline/{stage}/plane_{i}/info"),
                            &rerun::TextLog::new(format!(
                                "Inliers: {inliers}, Quality: {quality:.3}"
                            )),
                        )
                        .ok();
                }
            }

            DebugData::CircleData {
                holes,
                fitting_residuals,
                ..
            } => {
                let _ = self.log_holes(&format!("pipeline/{stage}"), holes);

                // Log residuals as scalars
                for (i, &residual) in fitting_residuals.iter().enumerate() {
                    self.rec
                        .log(
                            format!("metrics/hole_residuals/hole_{i}"),
                            &rerun::Scalars::new([residual]),
                        )
                        .ok();
                }
            }

            DebugData::DetectionResult {
                detections,
                confidence_scores,
                ..
            } => {
                for (i, detection) in detections.iter().enumerate() {
                    let _ = self.log_board(&format!("pipeline/{stage}/board_{i}"), detection);

                    if i < confidence_scores.len() {
                        self.rec
                            .log(
                                format!("metrics/confidence/board_{i}"),
                                &rerun::Scalars::new([confidence_scores[i]]),
                            )
                            .ok();
                    }
                }
            }

            DebugData::Generic { data } => {
                for (key, value) in data {
                    self.rec
                        .log(
                            format!("pipeline/{stage}/{key}"),
                            &rerun::TextLog::new(value.to_string()),
                        )
                        .ok();
                }
            }
        }
    }

    fn on_point_cloud(&self, stage: &str, cloud: &PointCloud) {
        // Color points by stage
        let color = match stage {
            "preprocessing" => [200u8, 200u8, 200u8],
            "plane_detection" => [100u8, 200u8, 255u8],
            "diamond_fitting" => [255u8, 200u8, 100u8],
            "hole_detection" => [255u8, 100u8, 100u8],
            _ => [128u8, 128u8, 128u8],
        };

        let colors = vec![color; cloud.points.len()];
        let _ = self.log_point_cloud(&format!("pipeline/{stage}/points"), cloud, Some(colors));

        // Log point count as scalar
        self.rec
            .log(
                format!("metrics/point_count/{stage}"),
                &rerun::Scalars::new([cloud.points.len() as f64]),
            )
            .ok();
    }
}

impl MetricsCallback for RerunCallback {
    fn on_metrics(&self, stage: &str, metrics: &StageMetrics) {
        // Log metrics as scalars
        self.rec
            .log(
                format!("metrics/points_in/{stage}"),
                &rerun::Scalars::new([metrics.input_points as f64]),
            )
            .ok();

        self.rec
            .log(
                format!("metrics/points_out/{stage}"),
                &rerun::Scalars::new([metrics.output_points as f64]),
            )
            .ok();

        let reduction = if metrics.input_points > 0 {
            100.0 * (1.0 - metrics.output_points as f64 / metrics.input_points as f64)
        } else {
            0.0
        };

        self.rec
            .log(
                format!("metrics/reduction/{stage}"),
                &rerun::Scalars::new([reduction]),
            )
            .ok();

        // Log custom metrics
        for (key, value) in &metrics.custom_metrics {
            self.rec
                .log(
                    format!("metrics/{stage}/{key}"),
                    &rerun::Scalars::new([*value]),
                )
                .ok();
        }
    }

    fn on_algorithm_stats(&self, stage: &str, stats: &AlgorithmStats) {
        self.rec
            .log(
                format!("algorithms/{stage}/{}", stats.algorithm),
                &rerun::TextLog::new(format!(
                    "Iterations: {iterations}, Converged: {converged}, Error: {error:.6}",
                    iterations = stats.iterations,
                    converged = stats.converged,
                    error = stats.final_error.unwrap_or(0.0)
                )),
            )
            .ok();

        // Log convergence metrics
        self.rec
            .log(
                format!("metrics/iterations/{stage}"),
                &rerun::Scalars::new([stats.iterations as f64]),
            )
            .ok();

        if let Some(error) = stats.final_error {
            self.rec
                .log(
                    format!("metrics/error/{stage}"),
                    &rerun::Scalars::new([error]),
                )
                .ok();
        }
    }
}

/// Generate test point cloud with board pattern
fn generate_test_cloud() -> PointCloud {
    let mut points = Vec::new();
    let mut intensities = Vec::new();

    // Generate a tilted square board at 2m distance
    for i in 0..100 {
        for j in 0..100 {
            let x = -0.5 + (i as f64 / 99.0);
            let y = -0.5 + (j as f64 / 99.0);
            let z = 2.0 + 0.1 * x + 0.05 * y; // Tilted plane

            // Skip points inside holes
            let dist_to_top = ((x - 0.0).powi(2) + (y - 0.3).powi(2)).sqrt();
            let dist_to_left = ((x + 0.3).powi(2) + (y - 0.0).powi(2)).sqrt();
            let dist_to_right = ((x - 0.3).powi(2) + (y - 0.0).powi(2)).sqrt();

            if dist_to_top < 0.1 || dist_to_left < 0.05 || dist_to_right < 0.05 {
                continue; // Skip points inside holes
            }

            // Add some noise
            let noise = 0.002;
            let x = x + (rand::random::<f64>() - 0.5) * noise;
            let y = y + (rand::random::<f64>() - 0.5) * noise;
            let z = z + (rand::random::<f64>() - 0.5) * noise;

            points.push(Point3::new(x, y, z));

            // Lower intensity near holes
            let min_dist = dist_to_top.min(dist_to_left).min(dist_to_right);
            let intensity = (min_dist * 500.0).min(200.0) as f32;
            intensities.push(intensity);
        }
    }

    // Add some background noise points
    for _ in 0..500 {
        let x = (rand::random::<f64>() - 0.5) * 3.0;
        let y = (rand::random::<f64>() - 0.5) * 3.0;
        let z = rand::random::<f64>() * 3.0;
        points.push(Point3::new(x, y, z));
        intensities.push(50.0);
    }

    PointCloud {
        points,
        intensities: Some(intensities),
        colors: None,
        timestamp: Instant::now(),
        frame_id: "sensor_frame".to_string(),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Board Detection Visualization with Rerun");
    println!("=======================================");
    println!();
    println!("Make sure Rerun viewer is running: `rerun`");
    println!();

    // Create board configuration
    let mut board = SquareBoard::new(Length::from_meters(1.0));

    // Add holes
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

    // Create Rerun callback
    let rerun_callback = Arc::new(RerunCallback::new("board_detection_demo")?);

    // Configure debug to capture all stages
    let debug_config = DebugConfigBuilder::new()
        .with_timing()
        .with_memory_tracking()
        .capture_stages([
            "preprocessing",
            "plane_detection",
            "diamond_fitting",
            "hole_detection",
            "validation",
        ])
        .build();

    // Create debug context with Rerun callback
    let mut debug_context = DebugContext::new(debug_config.clone());
    debug_context.timing_callback = Some(rerun_callback.clone());
    debug_context.data_callback = Some(rerun_callback.clone());
    debug_context.metrics_callback = Some(rerun_callback.clone());

    // Create detector
    let mut detector = BoardDetectorBuilder::new(board_config)
        .with_debug(debug_config) // Use debug_config, not debug_context
        .timeout_ms(30000)
        .min_confidence(0.7)
        .build()?;

    // Generate test point cloud
    let point_cloud = generate_test_cloud();

    // Log input cloud
    rerun_callback.log_point_cloud("input/point_cloud", &point_cloud, None)?;
    rerun_callback.rec.log(
        "input/info",
        &rerun::TextLog::new(format!("Input: {} points", point_cloud.points.len())),
    )?;

    println!("Processing {} points...", point_cloud.points.len());

    // Run detection
    match detector.detect(&point_cloud) {
        Ok(result) => {
            println!("\n✅ Detection completed!");
            println!("   Boards detected: {}", result.detections.len());

            // Log final results
            for (i, detection) in result.detections.iter().enumerate() {
                rerun_callback.log_board(&format!("results/board_{i}"), detection)?;

                println!(
                    "   Board {}: confidence = {:.3}",
                    i,
                    detection.confidence.value()
                );
            }

            // Log summary
            rerun_callback.rec.log(
                "results/summary",
                &rerun::TextLog::new(format!(
                    "Detection complete: {} boards found in {:?}",
                    result.detections.len(),
                    result.stats.total_time
                )),
            )?;
        }
        Err(e) => {
            println!("❌ Detection failed: {e}");
            rerun_callback.rec.log(
                "results/error",
                &rerun::TextLog::new(format!("Detection failed: {e}")),
            )?;
        }
    }

    println!("\nVisualization complete! Check the Rerun viewer.");

    Ok(())
}
