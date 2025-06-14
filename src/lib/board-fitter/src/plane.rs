//! Plane detection using RANSAC and related algorithms

use crate::debug::{stages, AlgorithmStats, DebugData, StageMetrics};
use anyhow::Result;
use nalgebra::{Point3, Vector3};
use rand::prelude::*;
use rand_chacha::ChaCha8Rng;
use std::{collections::HashMap, time::Instant};

use crate::{
    debug::DebugContext,
    types::{BoundingBox, DetectedPlane, PointCloud, ProcessingStage, ProcessingStats},
};

/// Configuration for plane detection algorithms
#[derive(Debug, Clone)]
pub struct PlaneDetectionConfig {
    /// Number of RANSAC iterations
    pub ransac_iterations: usize,
    /// Distance threshold for inlier classification (meters)
    pub distance_threshold: f64,
    /// Minimum number of inliers for a valid plane
    pub min_inliers: usize,
    /// Maximum number of planes to detect
    pub max_planes: usize,
    /// Minimum plane area (square meters)
    pub min_plane_area: f64,
    /// Enable multi-plane detection
    pub multi_plane: bool,
}

impl Default for PlaneDetectionConfig {
    fn default() -> Self {
        Self {
            ransac_iterations: 1000,
            distance_threshold: 0.01, // 1cm
            min_inliers: 100,
            max_planes: 5,
            min_plane_area: 0.1, // 0.1 m²
            multi_plane: true,
        }
    }
}

/// RANSAC-based plane detector
pub struct RansacPlaneDetector {
    config: PlaneDetectionConfig,
    rng: ChaCha8Rng,
    stats: ProcessingStats,
}

impl RansacPlaneDetector {
    /// Create a new RANSAC plane detector
    pub fn new(config: PlaneDetectionConfig) -> Self {
        Self {
            config,
            rng: ChaCha8Rng::seed_from_u64(42), // Deterministic for testing
            stats: ProcessingStats::new(),
        }
    }

    /// Create with default configuration
    pub fn default() -> Self {
        Self::new(PlaneDetectionConfig::default())
    }

    /// Detect planes in a point cloud
    pub fn detect_planes(&mut self, point_cloud: &PointCloud) -> Result<Vec<DetectedPlane>> {
        self.detect_planes_with_debug(point_cloud, None)
    }

    /// Detect planes in a point cloud with optional debug context
    pub fn detect_planes_with_debug(
        &mut self,
        point_cloud: &PointCloud,
        mut debug_ctx: Option<&mut DebugContext>,
    ) -> Result<Vec<DetectedPlane>> {
        let start_time = Instant::now();

        if let Some(ref mut ctx) = debug_ctx {
            ctx.start_stage(stages::PLANE_DETECTION);
            ctx.emit_point_cloud(stages::PLANE_DETECTION, point_cloud);
        }

        if point_cloud.is_empty() {
            if let Some(ref mut ctx) = debug_ctx {
                ctx.end_stage(stages::PLANE_DETECTION);
            }
            return Ok(Vec::new());
        }

        let mut planes = Vec::new();
        let mut remaining_points: Vec<usize> = (0..point_cloud.len()).collect();
        let mut total_iterations = 0;

        for plane_idx in 0..self.config.max_planes {
            if remaining_points.len() < self.config.min_inliers {
                break;
            }

            if let Some((plane, iterations)) =
                self.debug_single_plane(point_cloud, &remaining_points, debug_ctx.as_deref_mut())?
            {
                total_iterations += iterations;

                // Remove inliers from remaining points for multi-plane detection
                if self.config.multi_plane {
                    remaining_points.retain(|&idx| !plane.inliers.contains(&idx));
                }

                // Emit debug data for this plane
                if let Some(ref mut ctx) = debug_ctx {
                    let mut metadata = HashMap::new();
                    metadata.insert("plane_index".to_string(), plane_idx.to_string());
                    metadata.insert("inlier_count".to_string(), plane.inliers.len().to_string());
                    metadata.insert("iterations".to_string(), iterations.to_string());

                    let debug_data = DebugData::PlaneData {
                        planes: vec![plane.clone()],
                        inlier_counts: vec![plane.inliers.len()],
                        quality_scores: vec![plane.score],
                        metadata,
                    };
                    ctx.emit_data(stages::PLANE_DETECTION, &debug_data);
                }

                planes.push(plane);

                if !self.config.multi_plane {
                    break;
                }
            } else {
                break;
            }
        }

        let duration = start_time.elapsed();
        self.stats
            .add_time(ProcessingStage::PlaneDetection, duration);
        self.stats.planes_detected = planes.len();

        // Emit debug metrics and algorithm stats
        if let Some(ref mut ctx) = debug_ctx {
            let metrics = StageMetrics::new(point_cloud.len(), planes.len(), duration);
            ctx.emit_metrics(stages::PLANE_DETECTION, &metrics);

            let mut algo_stats = AlgorithmStats::new(
                "RANSAC_Plane_Detection",
                total_iterations,
                !planes.is_empty(),
            );
            algo_stats.add_stat("planes_detected", planes.len());
            algo_stats.add_stat(
                "avg_iterations_per_plane",
                if !planes.is_empty() {
                    total_iterations as f64 / planes.len() as f64
                } else {
                    0.0
                },
            );
            ctx.emit_algorithm_stats(stages::PLANE_DETECTION, &algo_stats);

            ctx.end_stage(stages::PLANE_DETECTION);
        }

        Ok(planes)
    }

    /// Detect a single plane using RANSAC with debug support
    fn debug_single_plane(
        &mut self,
        point_cloud: &PointCloud,
        point_indices: &[usize],
        _debug_ctx: Option<&mut DebugContext>,
    ) -> Result<Option<(DetectedPlane, usize)>> {
        if point_indices.len() < 3 {
            return Ok(None);
        }

        let mut best_plane: Option<DetectedPlane> = None;
        let mut best_score = 0;

        for _iteration in 0..self.config.ransac_iterations {
            // Sample 3 random points
            let sample = self.sample_three_points(point_indices);
            let p1 = point_cloud.points[sample[0]];
            let p2 = point_cloud.points[sample[1]];
            let p3 = point_cloud.points[sample[2]];

            // Compute plane from 3 points
            if let Some((normal, point)) = self.compute_plane_from_points(p1, p2, p3) {
                // Count inliers
                let inliers = self.find_inliers(point_cloud, point_indices, &normal, &point);

                if inliers.len() >= self.config.min_inliers {
                    let score = inliers.len();
                    if score > best_score {
                        let mut plane = DetectedPlane::new(normal, point, inliers);
                        plane.score = score as f64;
                        plane.bbox = self.compute_plane_bbox(point_cloud, &plane.inliers);

                        // Check minimum area requirement
                        let area = self.estimate_plane_area(&plane.bbox);
                        if area >= self.config.min_plane_area {
                            best_plane = Some(plane);
                            best_score = score;
                        }
                    }
                }
            }
        }

        Ok(best_plane.map(|plane| (plane, self.config.ransac_iterations)))
    }

    /// Sample three random point indices
    fn sample_three_points(&mut self, point_indices: &[usize]) -> [usize; 3] {
        let mut sample = [0; 3];
        for i in 0..3 {
            sample[i] = point_indices[self.rng.gen_range(0..point_indices.len())];
        }
        sample
    }

    /// Compute plane normal and point from three points
    fn compute_plane_from_points(
        &self,
        p1: Point3<f64>,
        p2: Point3<f64>,
        p3: Point3<f64>,
    ) -> Option<(Vector3<f64>, Point3<f64>)> {
        let v1 = p2 - p1;
        let v2 = p3 - p1;

        let normal = v1.cross(&v2);
        let normal_length = normal.norm();

        if normal_length < 1e-10 {
            // Points are collinear
            return None;
        }

        let normal = normal / normal_length;
        Some((normal, p1))
    }

    /// Find all inliers for a given plane
    fn find_inliers(
        &self,
        point_cloud: &PointCloud,
        point_indices: &[usize],
        normal: &Vector3<f64>,
        plane_point: &Point3<f64>,
    ) -> Vec<usize> {
        point_indices
            .iter()
            .copied()
            .filter(|&idx| {
                let point = point_cloud.points[idx];
                let distance = self.point_to_plane_distance(point, normal, plane_point);
                distance <= self.config.distance_threshold
            })
            .collect()
    }

    /// Calculate distance from point to plane
    fn point_to_plane_distance(
        &self,
        point: Point3<f64>,
        normal: &Vector3<f64>,
        plane_point: &Point3<f64>,
    ) -> f64 {
        let to_point = point - plane_point;
        to_point.dot(normal).abs()
    }

    /// Compute bounding box of plane points
    fn compute_plane_bbox(&self, point_cloud: &PointCloud, inliers: &[usize]) -> BoundingBox {
        if inliers.is_empty() {
            return BoundingBox {
                min: Point3::origin(),
                max: Point3::origin(),
            };
        }

        let first_point = point_cloud.points[inliers[0]];
        let mut min = first_point;
        let mut max = first_point;

        for &idx in inliers.iter().skip(1) {
            let point = point_cloud.points[idx];
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }

        BoundingBox { min, max }
    }

    /// Estimate plane area from bounding box
    fn estimate_plane_area(&self, bbox: &BoundingBox) -> f64 {
        let size = bbox.size();
        // Simple approximation: area of largest two dimensions
        let areas = [size.x * size.y, size.y * size.z, size.x * size.z];
        areas.iter().fold(0.0, |a, &b| a.max(b))
    }

    /// Get processing statistics
    pub fn stats(&self) -> &ProcessingStats {
        &self.stats
    }

    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.stats = ProcessingStats::new();
    }
}

/// Plane filtering utilities
pub struct PlaneFilter {
    /// Minimum normal angle with Z-axis (for roughly horizontal planes)
    pub min_z_angle: f64,
    /// Maximum normal angle with Z-axis
    pub max_z_angle: f64,
    /// Minimum plane dimensions
    pub min_dimensions: Vector3<f64>,
    /// Maximum plane dimensions  
    pub max_dimensions: Vector3<f64>,
}

impl PlaneFilter {
    /// Create a new plane filter for diamond board detection
    pub fn for_diamond_boards() -> Self {
        Self {
            min_z_angle: 30.0_f64.to_radians(), // 30° minimum angle with horizontal
            max_z_angle: 150.0_f64.to_radians(), // 150° maximum angle
            min_dimensions: Vector3::new(0.5, 0.5, 0.01), // 0.5m x 0.5m minimum
            max_dimensions: Vector3::new(2.0, 2.0, 0.1), // 2m x 2m maximum
        }
    }

    /// Filter planes based on criteria
    pub fn filter_planes(&self, planes: Vec<DetectedPlane>) -> Vec<DetectedPlane> {
        planes
            .into_iter()
            .filter(|plane| self.is_valid_plane(plane))
            .collect()
    }

    /// Check if a plane meets the filtering criteria
    fn is_valid_plane(&self, plane: &DetectedPlane) -> bool {
        // Check normal angle with Z-axis
        let z_axis = Vector3::new(0.0, 0.0, 1.0);
        let angle = plane.normal.angle(&z_axis);
        if angle < self.min_z_angle || angle > self.max_z_angle {
            return false;
        }

        // Check plane dimensions
        let size = plane.bbox.size();
        if size.x < self.min_dimensions.x
            || size.y < self.min_dimensions.y
            || size.x > self.max_dimensions.x
            || size.y > self.max_dimensions.y
        {
            return false;
        }

        true
    }
}

/// Advanced plane detection with orientation constraints
pub struct OrientedPlaneDetector {
    ransac_detector: RansacPlaneDetector,
    filter: PlaneFilter,
    target_orientation: Option<Vector3<f64>>,
    orientation_tolerance: f64,
}

impl OrientedPlaneDetector {
    /// Create detector targeting specific orientation (e.g., for diamond boards)
    pub fn new(
        config: PlaneDetectionConfig,
        target_orientation: Option<Vector3<f64>>,
        orientation_tolerance: f64,
    ) -> Self {
        Self {
            ransac_detector: RansacPlaneDetector::new(config),
            filter: PlaneFilter::for_diamond_boards(),
            target_orientation,
            orientation_tolerance,
        }
    }

    /// Detect planes with orientation filtering
    pub fn detect_oriented_planes(
        &mut self,
        point_cloud: &PointCloud,
    ) -> Result<Vec<DetectedPlane>> {
        // Run basic plane detection
        let planes = self.ransac_detector.detect_planes(point_cloud)?;

        // Apply orientation filtering
        let filtered_planes = if let Some(target) = self.target_orientation {
            self.filter_by_orientation(planes, target)
        } else {
            self.filter.filter_planes(planes)
        };

        Ok(filtered_planes)
    }

    /// Filter planes by target orientation
    fn filter_by_orientation(
        &self,
        planes: Vec<DetectedPlane>,
        target: Vector3<f64>,
    ) -> Vec<DetectedPlane> {
        planes
            .into_iter()
            .filter(|plane| {
                let angle = plane.normal.angle(&target);
                angle <= self.orientation_tolerance
            })
            .collect()
    }

    /// Get processing statistics
    pub fn stats(&self) -> &ProcessingStats {
        self.ransac_detector.stats()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn create_test_point_cloud() -> PointCloud {
        // Create a simple horizontal plane at z=1
        let mut points = Vec::new();
        for x in 0..10 {
            for y in 0..10 {
                points.push(Point3::new(x as f64 * 0.1, y as f64 * 0.1, 1.0));
            }
        }
        PointCloud::new(points, "test".to_string())
    }

    #[test]
    fn test_ransac_plane_detector() {
        let mut detector = RansacPlaneDetector::default();
        let cloud = create_test_point_cloud();

        let planes = detector.detect_planes(&cloud).unwrap();
        assert!(!planes.is_empty());

        let plane = &planes[0];
        assert_relative_eq!(plane.normal.z.abs(), 1.0, epsilon = 0.1);
        assert!(plane.inliers.len() >= 50); // Should detect most points
    }

    #[test]
    fn test_plane_filter() {
        let filter = PlaneFilter::for_diamond_boards();

        // Create a diamond-oriented plane at 45° angle (should pass)
        // Normal at 45° from Z-axis
        let diamond_plane = DetectedPlane {
            normal: Vector3::new(0.0, 1.0, 1.0).normalize(), // 45° angle with Z-axis
            point: Point3::origin(),
            inliers: vec![],
            score: 100.0,
            bbox: BoundingBox {
                min: Point3::new(0.0, 0.0, 0.0),
                max: Point3::new(1.0, 1.0, 0.1),
            },
        };

        // Create a horizontal plane (should fail - angle too small)
        let horizontal_plane = DetectedPlane {
            normal: Vector3::new(0.0, 0.0, 1.0), // 0° angle with Z-axis
            point: Point3::origin(),
            inliers: vec![],
            score: 100.0,
            bbox: BoundingBox {
                min: Point3::new(0.0, 0.0, 0.0),
                max: Point3::new(1.0, 1.0, 0.1),
            },
        };

        // Create a vertical plane (should pass - 90° angle)
        let vertical_plane = DetectedPlane {
            normal: Vector3::new(1.0, 0.0, 0.0), // 90° angle with Z-axis
            point: Point3::origin(),
            inliers: vec![],
            score: 100.0,
            bbox: BoundingBox {
                min: Point3::new(0.0, 0.0, 0.0),
                max: Point3::new(1.0, 1.0, 0.1), // Fixed dimensions
            },
        };

        assert!(filter.is_valid_plane(&diamond_plane));
        assert!(!filter.is_valid_plane(&horizontal_plane));
        assert!(filter.is_valid_plane(&vertical_plane));
    }

    #[test]
    fn test_point_to_plane_distance() {
        let detector = RansacPlaneDetector::default();
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let plane_point = Point3::new(0.0, 0.0, 5.0);
        let test_point = Point3::new(1.0, 1.0, 8.0);

        let distance = detector.point_to_plane_distance(test_point, &normal, &plane_point);
        assert_relative_eq!(distance, 3.0, epsilon = 1e-10);
    }
}
