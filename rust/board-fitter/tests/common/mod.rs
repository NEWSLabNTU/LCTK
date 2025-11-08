//! Common test utilities for board-fitter tests

use board_fitter::types::{BoardDetection, PointCloud};
use board_fitter_config::{Point2D, SquareBoard};
use measurements::Length;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::f64::consts::PI;

/// Test data generator for creating synthetic point clouds
#[allow(unused)]
pub struct TestDataGenerator {
    rng: ChaCha8Rng,
}

#[allow(unused)]
impl TestDataGenerator {
    /// Create a new test data generator with a fixed seed for reproducibility
    pub fn new(seed: u64) -> Self {
        Self {
            rng: ChaCha8Rng::seed_from_u64(seed),
        }
    }

    /// Generate a perfect diamond board point cloud
    pub fn generate_perfect_board(
        &mut self,
        board_config: &SquareBoard,
        pose: &Isometry3<f64>,
        point_density: usize,
    ) -> PointCloud {
        let size = board_config.size.as_meters();
        let half_size = size / 2.0;

        let mut points = Vec::new();
        let mut intensities = Vec::new();

        // Generate points on board surface
        let points_per_side = (point_density as f64).sqrt() as usize;
        for i in 0..points_per_side {
            for j in 0..points_per_side {
                let x = -half_size + (i as f64 / (points_per_side - 1) as f64) * size;
                let y = -half_size + (j as f64 / (points_per_side - 1) as f64) * size;

                // Check if point is inside a hole
                let mut in_hole = false;
                for hole in &board_config.holes {
                    let hole_pos = Point3::new(
                        hole.position.x.as_meters(),
                        hole.position.y.as_meters(),
                        0.0,
                    );
                    let point_pos = Point3::new(x, y, 0.0);
                    if (point_pos - hole_pos).norm() < hole.radius.as_meters() {
                        in_hole = true;
                        break;
                    }
                }

                if !in_hole {
                    // Transform to world coordinates
                    let local_point = Point3::new(x, y, 0.0);
                    let world_point = pose * local_point;
                    points.push(world_point);

                    // Simulate intensity (lower at holes, higher on surface)
                    intensities.push(128.0);
                }
            }
        }

        // Add points around hole boundaries for better definition
        for hole in &board_config.holes {
            let hole_center = Point3::new(
                hole.position.x.as_meters(),
                hole.position.y.as_meters(),
                0.0,
            );
            let radius = hole.radius.as_meters();

            // Generate dense points around hole circumference at multiple radii
            let num_rings = 3;
            let num_points_per_ring = 40;

            for ring in 0..num_rings {
                let ring_radius = radius * (1.1 + 0.1 * ring as f64); // 1.1x, 1.2x, 1.3x radius

                for i in 0..num_points_per_ring {
                    let angle = 2.0 * PI * i as f64 / num_points_per_ring as f64;
                    let x = hole_center.x + ring_radius * angle.cos();
                    let y = hole_center.y + ring_radius * angle.sin();

                    // Check if point is still within board bounds
                    if x.abs() <= half_size && y.abs() <= half_size {
                        let local_point = Point3::new(x, y, 0.0);
                        let world_point = pose * local_point;
                        points.push(world_point);
                        intensities.push(64.0 - 10.0 * ring as f32); // Decreasing intensity away from hole
                    }
                }
            }

            // Also add dense grid of points in a square region around each hole
            let grid_size = radius * 3.0; // 3x radius on each side
            let grid_points = 20; // 20x20 grid

            for i in 0..grid_points {
                for j in 0..grid_points {
                    let x = hole_center.x
                        + (i as f64 / (grid_points - 1) as f64 - 0.5) * grid_size * 2.0;
                    let y = hole_center.y
                        + (j as f64 / (grid_points - 1) as f64 - 0.5) * grid_size * 2.0;

                    // Check if point is outside hole but within board
                    let dist_to_hole =
                        ((x - hole_center.x).powi(2) + (y - hole_center.y).powi(2)).sqrt();
                    if dist_to_hole > radius && x.abs() <= half_size && y.abs() <= half_size {
                        let local_point = Point3::new(x, y, 0.0);
                        let world_point = pose * local_point;
                        points.push(world_point);

                        // Intensity based on distance from hole
                        let intensity = (64.0 + (dist_to_hole / radius * 64.0).min(64.0)) as f32;
                        intensities.push(intensity);
                    }
                }
            }
        }

        PointCloud {
            points,
            intensities: Some(intensities),
            colors: None,
            timestamp: std::time::Instant::now(),
            frame_id: "test_frame".to_string(),
        }
    }

    /// Add Gaussian noise to a point cloud
    pub fn add_noise(&mut self, cloud: &PointCloud, noise_stddev: f64) -> PointCloud {
        let mut noisy_points = Vec::new();

        for point in &cloud.points {
            let noise_x = self.rng.gen::<f64>() * noise_stddev * 2.0 - noise_stddev;
            let noise_y = self.rng.gen::<f64>() * noise_stddev * 2.0 - noise_stddev;
            let noise_z = self.rng.gen::<f64>() * noise_stddev * 2.0 - noise_stddev;

            let noisy_point = Point3::new(point.x + noise_x, point.y + noise_y, point.z + noise_z);
            noisy_points.push(noisy_point);
        }

        PointCloud {
            points: noisy_points,
            intensities: cloud.intensities.clone(),
            colors: cloud.colors.clone(),
            timestamp: std::time::Instant::now(),
            frame_id: cloud.frame_id.clone(),
        }
    }

    /// Generate a scene with multiple boards
    pub fn generate_multi_board_scene(
        &mut self,
        board_configs: &[(SquareBoard, Isometry3<f64>)],
        point_density: usize,
        add_background: bool,
    ) -> PointCloud {
        let mut all_points = Vec::new();
        let mut all_intensities = Vec::new();

        // Generate each board
        for (config, pose) in board_configs {
            let board_cloud = self.generate_perfect_board(config, pose, point_density);
            all_points.extend(board_cloud.points);
            if let Some(intensities) = board_cloud.intensities {
                all_intensities.extend(intensities);
            }
        }

        // Add background clutter if requested
        if add_background {
            let num_background = all_points.len() / 4; // 25% background points
            for _ in 0..num_background {
                let x = self.rng.gen_range(-3.0..3.0);
                let y = self.rng.gen_range(-3.0..3.0);
                let z = self.rng.gen_range(-0.5..2.5);
                all_points.push(Point3::new(x, y, z));
                all_intensities.push(self.rng.gen_range(50.0..150.0));
            }
        }

        PointCloud {
            points: all_points,
            intensities: if all_intensities.is_empty() {
                None
            } else {
                Some(all_intensities)
            },
            colors: None,
            timestamp: std::time::Instant::now(),
            frame_id: "multi_board_scene".to_string(),
        }
    }

    /// Simulate partial occlusion by removing points
    pub fn apply_occlusion(&mut self, cloud: &PointCloud, occlusion_ratio: f64) -> PointCloud {
        let mut remaining_points = Vec::new();
        let mut remaining_intensities = Vec::new();

        for (i, point) in cloud.points.iter().enumerate() {
            if self.rng.gen::<f64>() > occlusion_ratio {
                remaining_points.push(*point);
                if let Some(ref intensities) = cloud.intensities {
                    remaining_intensities.push(intensities[i]);
                }
            }
        }

        PointCloud {
            points: remaining_points,
            intensities: if remaining_intensities.is_empty() {
                None
            } else {
                Some(remaining_intensities)
            },
            colors: cloud.colors.clone(),
            timestamp: std::time::Instant::now(),
            frame_id: cloud.frame_id.clone(),
        }
    }
}

/// Create a standard test board configuration
pub fn create_test_board_config(size: f64) -> SquareBoard {
    let mut board = SquareBoard::new(Length::from_meters(size));

    // Add asymmetric hole pattern for diamond boards
    board.add_hole(
        Length::from_meters(0.1), // 10cm identification hole
        Point2D {
            x: Length::from_meters(0.0),
            y: Length::from_meters(size * 0.35), // Top hole
        },
        Some("top_hole".to_string()),
    );

    board.add_hole(
        Length::from_meters(0.05), // 5cm corner hole
        Point2D {
            x: Length::from_meters(-size * 0.35), // Left hole
            y: Length::from_meters(0.0),
        },
        Some("left_hole".to_string()),
    );

    board.add_hole(
        Length::from_meters(0.05), // 5cm corner hole
        Point2D {
            x: Length::from_meters(size * 0.35), // Right hole
            y: Length::from_meters(0.0),
        },
        Some("right_hole".to_string()),
    );

    board
}

/// Create a board pose at a specific position and orientation
#[allow(unused)]
pub fn create_board_pose(
    x: f64,
    y: f64,
    z: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
) -> Isometry3<f64> {
    let translation = Translation3::new(x, y, z);
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    Isometry3::from_parts(translation, rotation)
}

/// Verify detection accuracy against ground truth
pub fn _verify_detection_accuracy(
    detected: &BoardDetection,
    ground_truth_pose: &Isometry3<f64>,
    position_tolerance: f64,
    angle_tolerance: f64,
) -> bool {
    // Check position error
    let position_error =
        (detected.pose.translation.vector - ground_truth_pose.translation.vector).norm();

    // Check orientation error
    let detected_rotation = detected.pose.rotation;
    let gt_rotation = ground_truth_pose.rotation;
    let rotation_diff = detected_rotation * gt_rotation.inverse();
    let angle_error = rotation_diff.angle();

    position_error < position_tolerance && angle_error < angle_tolerance
}

/// Performance timer for benchmarking
pub struct PerfTimer {
    start: std::time::Instant,
    name: String,
}

#[allow(unused)]
impl PerfTimer {
    pub fn new(name: &str) -> Self {
        Self {
            start: std::time::Instant::now(),
            name: name.to_string(),
        }
    }

    pub fn elapsed_ms(&self) -> f64 {
        self.start.elapsed().as_secs_f64() * 1000.0
    }
}

impl Drop for PerfTimer {
    fn drop(&mut self) {
        println!("{}: {:.2}ms", self.name, self.elapsed_ms());
    }
}

/// Test result collector for aggregate statistics
#[allow(unused)]
pub struct TestResults {
    pub successes: usize,
    pub failures: usize,
    pub position_errors: Vec<f64>,
    pub angle_errors: Vec<f64>,
    pub processing_times: Vec<f64>,
}

#[allow(unused)]
impl TestResults {
    pub fn new() -> Self {
        Self {
            successes: 0,
            failures: 0,
            position_errors: Vec::new(),
            angle_errors: Vec::new(),
            processing_times: Vec::new(),
        }
    }

    pub fn add_success(&mut self, position_error: f64, angle_error: f64, time_ms: f64) {
        self.successes += 1;
        self.position_errors.push(position_error);
        self.angle_errors.push(angle_error);
        self.processing_times.push(time_ms);
    }

    pub fn add_failure(&mut self) {
        self.failures += 1;
    }

    pub fn success_rate(&self) -> f64 {
        let total = self.successes + self.failures;
        if total == 0 {
            0.0
        } else {
            self.successes as f64 / total as f64
        }
    }

    pub fn mean_position_error(&self) -> f64 {
        if self.position_errors.is_empty() {
            0.0
        } else {
            self.position_errors.iter().sum::<f64>() / self.position_errors.len() as f64
        }
    }

    pub fn mean_angle_error(&self) -> f64 {
        if self.angle_errors.is_empty() {
            0.0
        } else {
            self.angle_errors.iter().sum::<f64>() / self.angle_errors.len() as f64
        }
    }

    pub fn mean_processing_time(&self) -> f64 {
        if self.processing_times.is_empty() {
            0.0
        } else {
            self.processing_times.iter().sum::<f64>() / self.processing_times.len() as f64
        }
    }

    pub fn print_summary(&self) {
        println!("\nTest Results Summary:");
        println!("  Success Rate: {:.1}%", self.success_rate() * 100.0);
        println!("  Mean Position Error: {:.3}m", self.mean_position_error());
        println!(
            "  Mean Angle Error: {:.1}°",
            self.mean_angle_error().to_degrees()
        );
        println!(
            "  Mean Processing Time: {:.1}ms",
            self.mean_processing_time()
        );
        println!("  Total Tests: {}", self.successes + self.failures);
    }
}
