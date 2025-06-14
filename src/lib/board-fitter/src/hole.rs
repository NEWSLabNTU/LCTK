//! Circular hole detection and pattern matching for calibration boards

use anyhow::Result;
use nalgebra::{Point2, Point3, Vector2};
use std::{
    collections::{HashMap, HashSet},
    time::Instant,
};

use crate::{
    debug::{stages, AlgorithmStats, DebugContext, DebugData, StageMetrics},
    diamond::DiamondSquare,
    types::{DetectedHole, DetectionConfidence, PointCloud},
};
use board_fitter_config::{CircleHole, SquareBoard};

/// Configuration for hole detection
#[derive(Debug, Clone)]
pub struct HoleDetectionConfig {
    /// Minimum hole radius (meters)
    pub min_radius: f64,
    /// Maximum hole radius (meters)
    pub max_radius: f64,
    /// Radius tolerance for matching expected holes
    pub radius_tolerance: f64,
    /// Position tolerance for matching expected holes (meters)
    pub position_tolerance: f64,
    /// Minimum points required to fit a circle
    pub min_points: usize,
    /// Grid resolution for hole detection (meters)
    pub grid_resolution: f64,
    /// Minimum depth for hole detection (intensity-based)
    pub min_depth_threshold: f32,
}

impl Default for HoleDetectionConfig {
    fn default() -> Self {
        Self {
            min_radius: 0.02,         // 2cm
            max_radius: 0.15,         // 15cm
            radius_tolerance: 0.01,   // 1cm tolerance
            position_tolerance: 0.05, // 5cm position tolerance
            min_points: 10,
            grid_resolution: 0.005, // 5mm grid
            min_depth_threshold: 0.1,
        }
    }
}

/// Hole detector for calibration boards
pub struct HoleDetector {
    config: HoleDetectionConfig,
}

impl HoleDetector {
    /// Create a new hole detector
    pub fn new(config: HoleDetectionConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration
    pub fn default() -> Self {
        Self::new(HoleDetectionConfig::default())
    }

    /// Detect holes in a diamond square region
    pub fn detect_holes_in_square(
        &self,
        point_cloud: &PointCloud,
        square: &DiamondSquare,
    ) -> Result<Vec<DetectedHole>> {
        self.detect_holes_in_square_with_debug(point_cloud, square, None)
    }

    /// Detect holes in a diamond square region with optional debug context
    pub fn detect_holes_in_square_with_debug(
        &self,
        point_cloud: &PointCloud,
        square: &DiamondSquare,
        mut debug_ctx: Option<&mut DebugContext>,
    ) -> Result<Vec<DetectedHole>> {
        let start_time = Instant::now();

        if let Some(ref mut ctx) = debug_ctx {
            ctx.start_stage(stages::HOLE_DETECTION);

            // Emit input square data
            let mut metadata = HashMap::new();
            metadata.insert("square_size".to_string(), square.size.to_string());
            metadata.insert(
                "square_center".to_string(),
                format!(
                    "[{}, {}, {}]",
                    square.center.x, square.center.y, square.center.z
                ),
            );

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert("square_size".to_string(), square.size.into());
                    data.insert(
                        "square_center".to_string(),
                        serde_json::json!([square.center.x, square.center.y, square.center.z]),
                    );
                    data.insert(
                        "square_corners".to_string(),
                        serde_json::json!(square
                            .corners
                            .iter()
                            .map(|c| [c.x, c.y, c.z])
                            .collect::<Vec<_>>()),
                    );
                    data
                },
            };
            ctx.emit_data(stages::HOLE_DETECTION, &debug_data);
        }

        // Filter points within the square region
        let square_points = self.filter_points_in_square(point_cloud, square);

        if square_points.is_empty() {
            if let Some(ref mut ctx) = debug_ctx {
                let mut algo_stats = AlgorithmStats::new("Hole_Detection", 0, false);
                algo_stats.add_stat("no_points_in_square", 1.0);
                ctx.emit_algorithm_stats(stages::HOLE_DETECTION, &algo_stats);
                ctx.end_stage(stages::HOLE_DETECTION);
            }
            return Ok(Vec::new());
        }

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert(
                "points_in_square".to_string(),
                square_points.len().to_string(),
            );
            metadata.insert("stage".to_string(), "point_filtering".to_string());

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert(
                        "filtered_points_count".to_string(),
                        square_points.len().into(),
                    );
                    data.insert("total_points".to_string(), point_cloud.len().into());
                    data
                },
            };
            ctx.emit_data(stages::HOLE_DETECTION, &debug_data);
        }

        // Project points to 2D for hole detection
        let projected_points =
            self.project_points_to_square_plane(point_cloud, &square_points, square)?;

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert(
                "projected_points".to_string(),
                projected_points.len().to_string(),
            );
            metadata.insert("stage".to_string(), "projection".to_string());

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert(
                        "projected_points_count".to_string(),
                        projected_points.len().into(),
                    );
                    data
                },
            };
            ctx.emit_data(stages::HOLE_DETECTION, &debug_data);
        }

        // Detect holes using different methods
        let mut holes = Vec::new();
        let mut intensity_holes_count = 0;
        let mut geometric_holes_count = 0;

        // Method 1: Intensity-based hole detection (if intensity data available)
        if point_cloud.intensities.is_some() {
            let intensity_holes =
                self.detect_holes_by_intensity(&projected_points, point_cloud, &square_points)?;
            intensity_holes_count = intensity_holes.len();
            holes.extend(intensity_holes);

            if let Some(ref mut ctx) = debug_ctx {
                let mut metadata = HashMap::new();
                metadata.insert(
                    "intensity_holes".to_string(),
                    intensity_holes_count.to_string(),
                );
                metadata.insert("stage".to_string(), "intensity_detection".to_string());

                let debug_data = DebugData::Generic {
                    data: {
                        let mut data = HashMap::new();
                        data.insert(
                            "intensity_holes_count".to_string(),
                            intensity_holes_count.into(),
                        );
                        data
                    },
                };
                ctx.emit_data(stages::HOLE_DETECTION, &debug_data);
            }
        }

        // Method 2: Geometric hole detection (negative space)
        let geometric_holes = self.detect_holes_by_geometry(&projected_points)?;
        geometric_holes_count = geometric_holes.len();
        holes.extend(geometric_holes);

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert(
                "geometric_holes".to_string(),
                geometric_holes_count.to_string(),
            );
            metadata.insert("stage".to_string(), "geometric_detection".to_string());

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert(
                        "geometric_holes_count".to_string(),
                        geometric_holes_count.into(),
                    );
                    data
                },
            };
            ctx.emit_data(stages::HOLE_DETECTION, &debug_data);
        }

        // Remove duplicates and validate
        let validated_holes = self.validate_and_deduplicate_holes(holes);

        let duration = start_time.elapsed();

        if let Some(ref mut ctx) = debug_ctx {
            // Emit final hole data
            let debug_data = DebugData::CircleData {
                holes: validated_holes.clone(),
                fitting_residuals: validated_holes
                    .iter()
                    .map(|h| h.confidence.value())
                    .collect(),
                iteration_counts: vec![1; validated_holes.len()], // Placeholder
                metadata: {
                    let mut metadata = HashMap::new();
                    metadata.insert(
                        "final_holes_count".to_string(),
                        validated_holes.len().to_string(),
                    );
                    metadata.insert(
                        "intensity_holes_count".to_string(),
                        intensity_holes_count.to_string(),
                    );
                    metadata.insert(
                        "geometric_holes_count".to_string(),
                        geometric_holes_count.to_string(),
                    );
                    metadata
                },
            };
            ctx.emit_data(stages::HOLE_DETECTION, &debug_data);

            // Emit metrics
            let metrics = StageMetrics::new(square_points.len(), validated_holes.len(), duration);
            ctx.emit_metrics(stages::HOLE_DETECTION, &metrics);

            let mut algo_stats =
                AlgorithmStats::new("Hole_Detection", 1, !validated_holes.is_empty());
            algo_stats.add_stat("intensity_holes", intensity_holes_count as f64);
            algo_stats.add_stat("geometric_holes", geometric_holes_count as f64);
            algo_stats.add_stat("final_holes", validated_holes.len() as f64);
            algo_stats.add_stat(
                "has_intensity_data",
                if point_cloud.intensities.is_some() {
                    1.0
                } else {
                    0.0
                },
            );
            ctx.emit_algorithm_stats(stages::HOLE_DETECTION, &algo_stats);

            ctx.end_stage(stages::HOLE_DETECTION);
        }

        Ok(validated_holes)
    }

    /// Match detected holes with expected pattern from configuration
    pub fn match_hole_pattern(
        &self,
        detected_holes: &[DetectedHole],
        expected_pattern: &[CircleHole],
    ) -> Result<HoleMatchResult> {
        let mut matches = HashMap::new();
        let mut unmatched_detected = detected_holes.to_vec();
        let mut unmatched_expected = expected_pattern.to_vec();

        // Try to match each expected hole with detected holes
        for expected in expected_pattern {
            let expected_pos = Point3::new(
                expected.position.x.as_meters(),
                expected.position.y.as_meters(),
                0.0, // Assume holes are on the board surface
            );
            let expected_radius = expected.radius.as_meters();

            // Find best matching detected hole
            let mut best_match: Option<(usize, f64)> = None;

            for (i, detected) in unmatched_detected.iter().enumerate() {
                let position_error = (detected.center - expected_pos).norm();
                let radius_error = (detected.radius - expected_radius).abs();

                if position_error <= self.config.position_tolerance
                    && radius_error <= self.config.radius_tolerance
                {
                    let combined_error = position_error + radius_error;

                    match best_match {
                        None => best_match = Some((i, combined_error)),
                        Some((_, current_error)) if combined_error < current_error => {
                            best_match = Some((i, combined_error));
                        }
                        _ => {}
                    }
                }
            }

            // Record match if found
            if let Some((idx, error)) = best_match {
                let detected_hole = unmatched_detected.remove(idx);
                matches.insert(expected.id.clone(), (detected_hole, error));

                // Remove from unmatched expected
                unmatched_expected.retain(|h| h.id != expected.id);
            }
        }

        Ok(HoleMatchResult {
            matches,
            unmatched_detected,
            unmatched_expected,
        })
    }

    /// Filter points that are within the diamond square
    fn filter_points_in_square(
        &self,
        point_cloud: &PointCloud,
        square: &DiamondSquare,
    ) -> Vec<usize> {
        (0..point_cloud.len())
            .filter(|&i| square.contains_point(point_cloud.points[i], 0.05))
            .collect()
    }

    /// Project points to the square's local 2D coordinate system
    fn project_points_to_square_plane(
        &self,
        point_cloud: &PointCloud,
        point_indices: &[usize],
        square: &DiamondSquare,
    ) -> Result<Vec<ProjectedPoint>> {
        let mut projected = Vec::new();

        for &idx in point_indices {
            let world_point = point_cloud.points[idx];
            let local_point = square.pose.inverse() * world_point;

            let projected_point = ProjectedPoint {
                point_2d: Point2::new(local_point.x, local_point.y),
                original_index: idx,
                depth: local_point.z, // Distance from board surface
            };

            projected.push(projected_point);
        }

        Ok(projected)
    }

    /// Detect holes using intensity information
    fn detect_holes_by_intensity(
        &self,
        projected_points: &[ProjectedPoint],
        point_cloud: &PointCloud,
        _point_indices: &[usize],
    ) -> Result<Vec<DetectedHole>> {
        let intensities = match &point_cloud.intensities {
            Some(intensities) => intensities,
            None => return Ok(Vec::new()),
        };

        if projected_points.is_empty() {
            return Ok(Vec::new());
        }

        // Create intensity grid for the projected 2D space
        let grid_size = self.config.grid_resolution;
        let intensity_grid =
            self.create_intensity_grid(projected_points, intensities, grid_size)?;

        // Find low-intensity regions (holes typically have lower reflectivity)
        let low_intensity_regions = self.find_low_intensity_regions(&intensity_grid)?;

        // Fit circles to low-intensity regions
        let mut detected_holes = Vec::new();
        let circle_fitter = CircleFitter::new(self.config.min_points);

        for region in low_intensity_regions {
            // Convert grid cells back to 2D points
            let region_points = region
                .iter()
                .map(|&(x, y)| Point2::new(x as f64 * grid_size, y as f64 * grid_size))
                .collect::<Vec<_>>();

            if region_points.len() < 3 {
                continue;
            }

            // Try to fit a circle to the region
            if let Some(circle) = circle_fitter.fit_circle(&region_points) {
                // Validate circle properties
                if self.is_valid_hole_circle(&circle) {
                    let hole = DetectedHole {
                        center: Point3::new(circle.center.x, circle.center.y, 0.0), // Will be transformed later
                        radius: circle.radius,
                        confidence: self.calculate_intensity_confidence(&circle, &intensity_grid),
                        id: None,
                    };
                    detected_holes.push(hole);
                }
            }
        }

        Ok(detected_holes)
    }

    /// Detect holes using geometric analysis (negative space)
    fn detect_holes_by_geometry(
        &self,
        projected_points: &[ProjectedPoint],
    ) -> Result<Vec<DetectedHole>> {
        if projected_points.is_empty() {
            return Ok(Vec::new());
        }

        // Create occupancy grid for the projected 2D space
        let grid_size = self.config.grid_resolution;
        let occupancy_grid = self.create_occupancy_grid(projected_points, grid_size)?;

        // Find empty circular regions using morphological operations
        let empty_regions = self.find_empty_circular_regions(&occupancy_grid)?;

        // Fit circles to empty regions
        let mut detected_holes = Vec::new();
        let circle_fitter = CircleFitter::new(self.config.min_points);

        for region in empty_regions {
            // Convert empty region to boundary points for circle fitting
            let boundary_points = self.extract_region_boundary(&region, grid_size);

            if boundary_points.len() < 3 {
                continue;
            }

            // Try RANSAC circle fitting for robustness
            if let Some(circle) = circle_fitter.fit_circle_ransac(&boundary_points, 100) {
                // Validate circle properties
                if self.is_valid_hole_circle(&circle) {
                    let hole = DetectedHole {
                        center: Point3::new(circle.center.x, circle.center.y, 0.0), // Will be transformed later
                        radius: circle.radius,
                        confidence: self.calculate_geometric_confidence(
                            &circle,
                            &occupancy_grid,
                            grid_size,
                        ),
                        id: None,
                    };
                    detected_holes.push(hole);
                }
            }
        }

        Ok(detected_holes)
    }

    /// Validate holes and remove duplicates
    fn validate_and_deduplicate_holes(&self, holes: Vec<DetectedHole>) -> Vec<DetectedHole> {
        let mut validated = Vec::new();

        for hole in holes {
            // Check radius bounds
            if hole.radius < self.config.min_radius || hole.radius > self.config.max_radius {
                continue;
            }

            // Check for duplicates
            let is_duplicate = validated.iter().any(|existing: &DetectedHole| {
                let distance = (existing.center - hole.center).norm();
                distance < self.config.position_tolerance
            });

            if !is_duplicate {
                validated.push(hole);
            }
        }

        validated
    }

    // Helper methods for hole detection algorithms

    /// Create intensity grid from projected points
    fn create_intensity_grid(
        &self,
        projected_points: &[ProjectedPoint],
        intensities: &[f32],
        grid_size: f64,
    ) -> Result<IntensityGrid> {
        // Find bounding box of projected points
        let (min_x, max_x, min_y, max_y) = self.compute_2d_bounding_box(projected_points);

        let width = ((max_x - min_x) / grid_size).ceil() as usize + 1;
        let height = ((max_y - min_y) / grid_size).ceil() as usize + 1;

        let mut grid = IntensityGrid {
            data: vec![vec![IntensityCell::default(); width]; height],
            min_x,
            min_y,
            grid_size,
            width,
            height,
        };

        // Populate grid with intensity data
        for point in projected_points {
            let grid_x = ((point.point_2d.x - min_x) / grid_size) as usize;
            let grid_y = ((point.point_2d.y - min_y) / grid_size) as usize;

            if grid_x < width && grid_y < height {
                let intensity = intensities[point.original_index];
                grid.data[grid_y][grid_x].intensities.push(intensity);
                grid.data[grid_y][grid_x].count += 1;
            }
        }

        // Calculate average intensities for each cell
        for row in &mut grid.data {
            for cell in row {
                if !cell.intensities.is_empty() {
                    cell.avg_intensity =
                        cell.intensities.iter().sum::<f32>() / cell.intensities.len() as f32;
                }
            }
        }

        Ok(grid)
    }

    /// Create occupancy grid from projected points
    fn create_occupancy_grid(
        &self,
        projected_points: &[ProjectedPoint],
        grid_size: f64,
    ) -> Result<OccupancyGrid> {
        // Find bounding box of projected points
        let (min_x, max_x, min_y, max_y) = self.compute_2d_bounding_box(projected_points);

        let width = ((max_x - min_x) / grid_size).ceil() as usize + 1;
        let height = ((max_y - min_y) / grid_size).ceil() as usize + 1;

        let mut grid = OccupancyGrid {
            data: vec![vec![false; width]; height],
            min_x,
            min_y,
            grid_size,
            width,
            height,
        };

        // Mark occupied cells
        for point in projected_points {
            let grid_x = ((point.point_2d.x - min_x) / grid_size) as usize;
            let grid_y = ((point.point_2d.y - min_y) / grid_size) as usize;

            if grid_x < width && grid_y < height {
                grid.data[grid_y][grid_x] = true;
            }
        }

        Ok(grid)
    }

    /// Compute 2D bounding box of projected points
    fn compute_2d_bounding_box(&self, points: &[ProjectedPoint]) -> (f64, f64, f64, f64) {
        if points.is_empty() {
            return (0.0, 0.0, 0.0, 0.0);
        }

        let first = points[0].point_2d;
        let mut min_x = first.x;
        let mut max_x = first.x;
        let mut min_y = first.y;
        let mut max_y = first.y;

        for point in points.iter().skip(1) {
            min_x = min_x.min(point.point_2d.x);
            max_x = max_x.max(point.point_2d.x);
            min_y = min_y.min(point.point_2d.y);
            max_y = max_y.max(point.point_2d.y);
        }

        (min_x, max_x, min_y, max_y)
    }

    /// Find low-intensity regions in the intensity grid
    fn find_low_intensity_regions(&self, grid: &IntensityGrid) -> Result<Vec<Vec<(usize, usize)>>> {
        let mut regions = Vec::new();
        let mut visited = vec![vec![false; grid.width]; grid.height];

        // Calculate intensity threshold (e.g., mean - std_dev)
        let all_intensities: Vec<f32> = grid
            .data
            .iter()
            .flatten()
            .filter(|cell| cell.count > 0)
            .map(|cell| cell.avg_intensity)
            .collect();

        if all_intensities.is_empty() {
            return Ok(regions);
        }

        let mean_intensity = all_intensities.iter().sum::<f32>() / all_intensities.len() as f32;
        let variance = all_intensities
            .iter()
            .map(|&x| (x - mean_intensity).powi(2))
            .sum::<f32>()
            / all_intensities.len() as f32;
        let std_dev = variance.sqrt();
        let threshold = mean_intensity - std_dev;

        // Find connected components of low-intensity cells
        for y in 0..grid.height {
            for x in 0..grid.width {
                if !visited[y][x]
                    && grid.data[y][x].count > 0
                    && grid.data[y][x].avg_intensity < threshold
                {
                    let region = self.flood_fill_intensity(grid, &mut visited, x, y, threshold);
                    if region.len() >= 4 {
                        // Minimum size for a meaningful region
                        regions.push(region);
                    }
                }
            }
        }

        Ok(regions)
    }

    /// Find empty circular regions in occupancy grid
    fn find_empty_circular_regions(
        &self,
        grid: &OccupancyGrid,
    ) -> Result<Vec<Vec<(usize, usize)>>> {
        let mut regions = Vec::new();
        let mut visited = vec![vec![false; grid.width]; grid.height];

        // Find connected components of empty cells
        for y in 0..grid.height {
            for x in 0..grid.width {
                if !visited[y][x] && !grid.data[y][x] {
                    let region = self.flood_fill_occupancy(grid, &mut visited, x, y);
                    if region.len() >= 10 {
                        // Minimum size for a meaningful hole
                        // Check if region is roughly circular
                        if self.is_roughly_circular_region(&region) {
                            regions.push(region);
                        }
                    }
                }
            }
        }

        Ok(regions)
    }

    /// Flood fill for intensity-based region growing
    fn flood_fill_intensity(
        &self,
        grid: &IntensityGrid,
        visited: &mut [Vec<bool>],
        start_x: usize,
        start_y: usize,
        threshold: f32,
    ) -> Vec<(usize, usize)> {
        let mut region = Vec::new();
        let mut stack = vec![(start_x, start_y)];

        while let Some((x, y)) = stack.pop() {
            if visited[y][x] {
                continue;
            }

            visited[y][x] = true;
            region.push((x, y));

            // Check 8-connected neighbors
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;

                    if nx >= 0
                        && ny >= 0
                        && (nx as usize) < grid.width
                        && (ny as usize) < grid.height
                    {
                        let nx = nx as usize;
                        let ny = ny as usize;

                        if !visited[ny][nx]
                            && grid.data[ny][nx].count > 0
                            && grid.data[ny][nx].avg_intensity < threshold
                        {
                            stack.push((nx, ny));
                        }
                    }
                }
            }
        }

        region
    }

    /// Flood fill for occupancy-based region growing
    fn flood_fill_occupancy(
        &self,
        grid: &OccupancyGrid,
        visited: &mut [Vec<bool>],
        start_x: usize,
        start_y: usize,
    ) -> Vec<(usize, usize)> {
        let mut region = Vec::new();
        let mut stack = vec![(start_x, start_y)];

        while let Some((x, y)) = stack.pop() {
            if visited[y][x] {
                continue;
            }

            visited[y][x] = true;
            region.push((x, y));

            // Check 4-connected neighbors for tighter regions
            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;

                if nx >= 0 && ny >= 0 && (nx as usize) < grid.width && (ny as usize) < grid.height {
                    let nx = nx as usize;
                    let ny = ny as usize;

                    if !visited[ny][nx] && !grid.data[ny][nx] {
                        stack.push((nx, ny));
                    }
                }
            }
        }

        region
    }

    /// Check if a region is roughly circular
    fn is_roughly_circular_region(&self, region: &[(usize, usize)]) -> bool {
        if region.len() < 4 {
            return false;
        }

        // Calculate centroid
        let sum_x: usize = region.iter().map(|(x, _)| *x).sum();
        let sum_y: usize = region.iter().map(|(_, y)| *y).sum();
        let centroid_x = sum_x as f64 / region.len() as f64;
        let centroid_y = sum_y as f64 / region.len() as f64;

        // Calculate distances from centroid
        let distances: Vec<f64> = region
            .iter()
            .map(|(x, y)| {
                let dx = *x as f64 - centroid_x;
                let dy = *y as f64 - centroid_y;
                (dx * dx + dy * dy).sqrt()
            })
            .collect();

        if distances.is_empty() {
            return false;
        }

        let mean_distance = distances.iter().sum::<f64>() / distances.len() as f64;
        let variance = distances
            .iter()
            .map(|&d| (d - mean_distance).powi(2))
            .sum::<f64>()
            / distances.len() as f64;
        let std_dev = variance.sqrt();

        // A circular region should have low variance in distances from centroid
        let coefficient_of_variation = std_dev / mean_distance;
        coefficient_of_variation < 0.3 // Adjust threshold as needed
    }

    /// Extract boundary points from a region
    fn extract_region_boundary(
        &self,
        region: &[(usize, usize)],
        grid_size: f64,
    ) -> Vec<Point2<f64>> {
        // Simple boundary extraction: find points that have at least one neighbor not in the region
        let region_set: HashSet<(usize, usize)> = region.iter().copied().collect();
        let mut boundary_points = Vec::new();

        for &(x, y) in region {
            let mut is_boundary = false;

            // Check 8-connected neighbors
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;

                    if nx < 0 || ny < 0 || !region_set.contains(&(nx as usize, ny as usize)) {
                        is_boundary = true;
                        break;
                    }
                }
                if is_boundary {
                    break;
                }
            }

            if is_boundary {
                boundary_points.push(Point2::new(x as f64 * grid_size, y as f64 * grid_size));
            }
        }

        boundary_points
    }

    /// Validate if a fitted circle represents a valid hole
    fn is_valid_hole_circle(&self, circle: &Circle2D) -> bool {
        circle.radius >= self.config.min_radius && circle.radius <= self.config.max_radius
    }

    /// Calculate confidence for intensity-based detection
    fn calculate_intensity_confidence(
        &self,
        _circle: &Circle2D,
        _grid: &IntensityGrid,
    ) -> crate::types::DetectionConfidence {
        // Simple confidence calculation - could be enhanced
        crate::types::DetectionConfidence::new(0.7)
    }

    /// Calculate confidence for geometric detection
    fn calculate_geometric_confidence(
        &self,
        _circle: &Circle2D,
        _grid: &OccupancyGrid,
        _grid_size: f64,
    ) -> crate::types::DetectionConfidence {
        // Simple confidence calculation - could be enhanced
        crate::types::DetectionConfidence::new(0.8)
    }
}

/// Intensity grid for hole detection
#[derive(Debug)]
struct IntensityGrid {
    data: Vec<Vec<IntensityCell>>,
    min_x: f64,
    min_y: f64,
    grid_size: f64,
    width: usize,
    height: usize,
}

/// Cell in intensity grid
#[derive(Debug, Default, Clone)]
struct IntensityCell {
    intensities: Vec<f32>,
    avg_intensity: f32,
    count: usize,
}

/// Occupancy grid for geometric hole detection
#[derive(Debug)]
struct OccupancyGrid {
    data: Vec<Vec<bool>>,
    min_x: f64,
    min_y: f64,
    grid_size: f64,
    width: usize,
    height: usize,
}

/// 2D projected point for hole detection
#[derive(Debug, Clone)]
struct ProjectedPoint {
    point_2d: Point2<f64>,
    original_index: usize,
    depth: f64, // Distance from board surface
}

/// Result of hole pattern matching
#[derive(Debug)]
pub struct HoleMatchResult {
    /// Successfully matched holes (expected_id -> (detected_hole, error))
    pub matches: HashMap<Option<String>, (DetectedHole, f64)>,
    /// Detected holes that couldn't be matched
    pub unmatched_detected: Vec<DetectedHole>,
    /// Expected holes that weren't found
    pub unmatched_expected: Vec<CircleHole>,
}

impl HoleMatchResult {
    /// Check if all expected holes were found
    pub fn is_complete_match(&self) -> bool {
        self.unmatched_expected.is_empty()
    }

    /// Get match quality score (0.0 = worst, 1.0 = perfect)
    pub fn match_quality(&self) -> f64 {
        if self.matches.is_empty() {
            return 0.0;
        }

        let total_expected = self.matches.len() + self.unmatched_expected.len();
        let match_ratio = self.matches.len() as f64 / total_expected as f64;

        // Factor in matching errors
        let avg_error =
            self.matches.values().map(|(_, error)| *error).sum::<f64>() / self.matches.len() as f64;

        let error_factor = (1.0 - avg_error).max(0.0);

        match_ratio * error_factor
    }

    /// Get detected hole by expected ID
    pub fn get_hole_by_id(&self, id: &str) -> Option<&DetectedHole> {
        for (expected_id, (detected_hole, _)) in &self.matches {
            if let Some(ref expected_id_str) = expected_id {
                if expected_id_str == id {
                    return Some(detected_hole);
                }
            }
        }
        None
    }
}

/// Circle fitting utilities
pub struct CircleFitter {
    /// Minimum points required for fitting
    min_points: usize,
}

impl CircleFitter {
    /// Create a new circle fitter
    pub fn new(min_points: usize) -> Self {
        Self { min_points }
    }

    /// Fit circle to 2D points using least squares
    pub fn fit_circle(&self, points: &[Point2<f64>]) -> Option<Circle2D> {
        if points.len() < self.min_points {
            return None;
        }

        // Use algebraic circle fitting method
        // Circle equation: (x-a)² + (y-b)² = r²
        // Expanded: x² + y² - 2ax - 2by + (a² + b² - r²) = 0
        // Linear form: x² + y² = 2ax + 2by + c, where c = r² - a² - b²

        let n = points.len() as f64;

        // Calculate sums for least squares fitting
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_x2 = 0.0;
        let mut sum_y2 = 0.0;
        let mut sum_x3 = 0.0;
        let mut sum_y3 = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_x2y = 0.0;
        let mut sum_xy2 = 0.0;

        for point in points {
            let x = point.x;
            let y = point.y;
            let x2 = x * x;
            let y2 = y * y;

            sum_x += x;
            sum_y += y;
            sum_x2 += x2;
            sum_y2 += y2;
            sum_x3 += x2 * x;
            sum_y3 += y2 * y;
            sum_xy += x * y;
            sum_x2y += x2 * y;
            sum_xy2 += x * y2;
        }

        // Solve the linear system using Cramer's rule
        // [sum_x2  sum_xy  sum_x ] [a]   [sum_x3 + sum_xy2]
        // [sum_xy  sum_y2  sum_y ] [b] = [sum_y3 + sum_x2y]
        // [sum_x   sum_y   n     ] [c]   [sum_x2 + sum_y2 ]

        let det = sum_x2 * (sum_y2 * n - sum_y * sum_y) - sum_xy * (sum_xy * n - sum_x * sum_y)
            + sum_x * (sum_xy * sum_y - sum_y2 * sum_x);

        if det.abs() < 1e-10 {
            return None; // Singular matrix
        }

        let rhs1 = sum_x3 + sum_xy2;
        let rhs2 = sum_y3 + sum_x2y;
        let rhs3 = sum_x2 + sum_y2;

        let a = (rhs1 * (sum_y2 * n - sum_y * sum_y) - sum_xy * (rhs2 * n - sum_y * rhs3)
            + sum_x * (rhs2 * sum_y - sum_y2 * rhs3))
            / det;

        let b = (sum_x2 * (rhs2 * n - sum_y * rhs3) - rhs1 * (sum_xy * n - sum_x * sum_y)
            + sum_x * (sum_xy * rhs3 - rhs2 * sum_x))
            / det;

        let c = (sum_x2 * (sum_y2 * rhs3 - sum_y * rhs2) - sum_xy * (sum_xy * rhs3 - sum_x * rhs2)
            + rhs1 * (sum_xy * sum_y - sum_y2 * sum_x))
            / det;

        // Convert back to center and radius
        let center_x = a / 2.0;
        let center_y = b / 2.0;
        let radius_squared = center_x * center_x + center_y * center_y + c;

        if radius_squared <= 0.0 {
            return None; // Invalid circle
        }

        let radius = radius_squared.sqrt();

        // Validate radius bounds
        if radius < 0.001 || radius > 10.0 {
            return None;
        }

        Some(Circle2D {
            center: Point2::new(center_x, center_y),
            radius,
        })
    }

    /// Fit circle using RANSAC for robustness
    pub fn fit_circle_ransac(&self, points: &[Point2<f64>], iterations: usize) -> Option<Circle2D> {
        if points.len() < self.min_points {
            return None;
        }

        // RANSAC circle fitting implementation:
        // 1. Sample minimum points to define a circle
        // 2. Count inliers for the circle
        // 3. Keep best circle with most inliers

        use rand::prelude::*;
        use rand_chacha::ChaCha8Rng;

        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let mut best_circle: Option<Circle2D> = None;
        let mut best_inlier_count = 0;
        let distance_threshold = 0.01; // 1cm tolerance

        for _ in 0..iterations {
            // Sample 3 random points to define a circle
            if points.len() < 3 {
                continue;
            }

            let mut sample_indices = Vec::new();
            while sample_indices.len() < 3 {
                let idx = rng.gen_range(0..points.len());
                if !sample_indices.contains(&idx) {
                    sample_indices.push(idx);
                }
            }

            let sample_points: Vec<Point2<f64>> =
                sample_indices.iter().map(|&i| points[i]).collect();

            // Fit circle to the 3 sample points
            if let Some(circle) = self.fit_circle_three_points(&sample_points) {
                // Count inliers
                let inlier_count = points
                    .iter()
                    .filter(|&point| {
                        let distance = (point - circle.center).norm();
                        (distance - circle.radius).abs() <= distance_threshold
                    })
                    .count();

                if inlier_count > best_inlier_count {
                    best_circle = Some(circle);
                    best_inlier_count = inlier_count;
                }
            }
        }

        // If we found a good circle, refine it using all inliers
        if let Some(ref circle) = best_circle {
            if best_inlier_count >= self.min_points {
                let inliers: Vec<Point2<f64>> = points
                    .iter()
                    .filter(|&point| {
                        let distance = (point - circle.center).norm();
                        (distance - circle.radius).abs() <= distance_threshold
                    })
                    .copied()
                    .collect();

                // Refine using least squares on all inliers
                if let Some(refined_circle) = self.fit_circle(&inliers) {
                    return Some(refined_circle);
                }
            }
        }

        best_circle
    }

    /// Fit circle to exactly 3 points (geometric method)
    fn fit_circle_three_points(&self, points: &[Point2<f64>]) -> Option<Circle2D> {
        if points.len() != 3 {
            return None;
        }

        let p1 = points[0];
        let p2 = points[1];
        let p3 = points[2];

        // Check if points are collinear
        let area = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
        if area.abs() < 1e-10 {
            return None; // Collinear points
        }

        // Calculate perpendicular bisectors
        let mid12 = Point2::new((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
        let mid23 = Point2::new((p2.x + p3.x) / 2.0, (p2.y + p3.y) / 2.0);

        let dir12 = Vector2::new(p2.x - p1.x, p2.y - p1.y);
        let dir23 = Vector2::new(p3.x - p2.x, p3.y - p2.y);

        let perp12 = Vector2::new(-dir12.y, dir12.x); // Perpendicular to p1-p2
        let perp23 = Vector2::new(-dir23.y, dir23.x); // Perpendicular to p2-p3

        // Find intersection of perpendicular bisectors (circle center)
        let det = perp12.x * perp23.y - perp12.y * perp23.x;
        if det.abs() < 1e-10 {
            return None; // Parallel bisectors
        }

        let diff = mid23 - mid12;
        let t = (diff.x * perp23.y - diff.y * perp23.x) / det;

        let center = mid12 + t * perp12;
        let radius = (p1 - center).norm();

        // Validate radius
        if radius < 0.001 || radius > 10.0 {
            return None;
        }

        Some(Circle2D { center, radius })
    }
}

/// 2D circle representation
#[derive(Debug, Clone)]
pub struct Circle2D {
    pub center: Point2<f64>,
    pub radius: f64,
}

impl Circle2D {
    /// Convert to 3D detected hole in world coordinates
    pub fn to_detected_hole(
        &self,
        square: &DiamondSquare,
        confidence: DetectionConfidence,
        id: Option<String>,
    ) -> DetectedHole {
        // Transform 2D center to 3D world coordinates
        let local_3d = Point3::new(self.center.x, self.center.y, 0.0);
        let world_center = square.pose * local_3d;

        DetectedHole {
            center: world_center,
            radius: self.radius,
            confidence,
            id,
        }
    }

    /// Check if a point is inside the circle
    pub fn contains_point(&self, point: Point2<f64>, tolerance: f64) -> bool {
        let distance = (point - self.center).norm();
        distance <= self.radius + tolerance
    }

    /// Get circle area
    pub fn area(&self) -> f64 {
        std::f64::consts::PI * self.radius * self.radius
    }
}

/// Hole pattern analyzer for asymmetric patterns
pub struct AsymmetricPatternAnalyzer {
    /// Expected pattern from configuration
    expected_pattern: Vec<CircleHole>,
}

impl AsymmetricPatternAnalyzer {
    /// Create analyzer for diamond board pattern
    pub fn for_diamond_board(board: &SquareBoard) -> Self {
        Self {
            expected_pattern: board.holes.clone(),
        }
    }

    /// Analyze detected pattern and determine board orientation
    pub fn analyze_pattern(&self, match_result: &HoleMatchResult) -> PatternAnalysis {
        let mut analysis = PatternAnalysis {
            orientation_determined: false,
            confidence: 0.0,
            missing_holes: Vec::new(),
            extra_holes: match_result.unmatched_detected.len(),
        };

        // Check if we have the asymmetric pattern (large top, small left/right)
        let has_top = match_result.get_hole_by_id("top_hole").is_some();
        let has_left = match_result.get_hole_by_id("left_hole").is_some();
        let has_right = match_result.get_hole_by_id("right_hole").is_some();

        if has_top && has_left && has_right {
            analysis.orientation_determined = true;
            analysis.confidence = match_result.match_quality();
        } else {
            // Record missing holes
            if !has_top {
                analysis.missing_holes.push("top_hole".to_string());
            }
            if !has_left {
                analysis.missing_holes.push("left_hole".to_string());
            }
            if !has_right {
                analysis.missing_holes.push("right_hole".to_string());
            }
        }

        analysis
    }
}

/// Result of pattern analysis
#[derive(Debug)]
pub struct PatternAnalysis {
    /// Whether board orientation was successfully determined
    pub orientation_determined: bool,
    /// Confidence in the pattern match (0.0 - 1.0)
    pub confidence: f64,
    /// List of missing expected holes
    pub missing_holes: Vec<String>,
    /// Number of extra detected holes
    pub extra_holes: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DetectionConfidence;
    use board_fitter_config::{Point2D, SquareBoard};
    use measurements::Length;
    use nalgebra::{Isometry3, Point3, Vector3};

    fn create_test_diamond_square() -> DiamondSquare {
        DiamondSquare {
            center: Point3::origin(),
            size: 1.0,
            pose: Isometry3::identity(),
            corners: [Point3::origin(); 4],
            normal: Vector3::z(),
        }
    }

    fn create_test_board_config() -> SquareBoard {
        let mut board = SquareBoard::new(Length::from_meters(1.0));

        // Add asymmetric hole pattern
        board.add_hole(
            Length::from_meters(0.1),
            Point2D {
                x: Length::from_meters(0.0),
                y: Length::from_meters(0.5),
            },
            Some("top_hole".to_string()),
        );
        board.add_hole(
            Length::from_meters(0.05),
            Point2D {
                x: Length::from_meters(-0.5),
                y: Length::from_meters(0.0),
            },
            Some("left_hole".to_string()),
        );
        board.add_hole(
            Length::from_meters(0.05),
            Point2D {
                x: Length::from_meters(0.5),
                y: Length::from_meters(0.0),
            },
            Some("right_hole".to_string()),
        );

        board
    }

    #[test]
    fn test_hole_detection_config() {
        let config = HoleDetectionConfig::default();
        assert_eq!(config.min_radius, 0.02);
        assert_eq!(config.max_radius, 0.15);
    }

    #[test]
    fn test_circle_2d_contains_point() {
        let circle = Circle2D {
            center: Point2::new(0.0, 0.0),
            radius: 1.0,
        };

        assert!(circle.contains_point(Point2::new(0.5, 0.5), 0.0));
        assert!(!circle.contains_point(Point2::new(1.5, 1.5), 0.0));
        assert!(circle.contains_point(Point2::new(1.5, 1.5), 1.2)); // With sufficient tolerance
    }

    #[test]
    fn test_hole_match_result_quality() {
        let mut matches = HashMap::new();
        matches.insert(
            Some("top_hole".to_string()),
            (
                DetectedHole {
                    center: Point3::origin(),
                    radius: 0.1,
                    confidence: DetectionConfidence::new(0.8),
                    id: Some("top_hole".to_string()),
                },
                0.01,
            ), // Low error
        );

        let result = HoleMatchResult {
            matches,
            unmatched_detected: Vec::new(),
            unmatched_expected: Vec::new(),
        };

        assert!(result.is_complete_match());
        assert!(result.match_quality() > 0.9); // High quality due to low error
    }

    #[test]
    fn test_asymmetric_pattern_analyzer() {
        let board = create_test_board_config();
        let analyzer = AsymmetricPatternAnalyzer::for_diamond_board(&board);

        // Create a complete match result
        let mut matches = HashMap::new();
        matches.insert(
            Some("top_hole".to_string()),
            (
                DetectedHole {
                    center: Point3::new(0.0, 0.5, 0.0),
                    radius: 0.1,
                    confidence: DetectionConfidence::new(0.9),
                    id: Some("top_hole".to_string()),
                },
                0.01,
            ),
        );
        matches.insert(
            Some("left_hole".to_string()),
            (
                DetectedHole {
                    center: Point3::new(-0.5, 0.0, 0.0),
                    radius: 0.05,
                    confidence: DetectionConfidence::new(0.9),
                    id: Some("left_hole".to_string()),
                },
                0.01,
            ),
        );
        matches.insert(
            Some("right_hole".to_string()),
            (
                DetectedHole {
                    center: Point3::new(0.5, 0.0, 0.0),
                    radius: 0.05,
                    confidence: DetectionConfidence::new(0.9),
                    id: Some("right_hole".to_string()),
                },
                0.01,
            ),
        );

        let match_result = HoleMatchResult {
            matches,
            unmatched_detected: Vec::new(),
            unmatched_expected: Vec::new(),
        };

        let analysis = analyzer.analyze_pattern(&match_result);
        assert!(analysis.orientation_determined);
        assert!(analysis.confidence > 0.8);
        assert!(analysis.missing_holes.is_empty());
    }

    #[test]
    fn test_circle_fitting_least_squares() {
        let fitter = CircleFitter::new(3);

        // Create points on a circle with center (1, 2) and radius 3
        let center = Point2::new(1.0, 2.0);
        let radius = 3.0;
        let mut points = Vec::new();

        for i in 0..8 {
            let angle = i as f64 * std::f64::consts::PI / 4.0;
            let x = center.x + radius * angle.cos();
            let y = center.y + radius * angle.sin();
            points.push(Point2::new(x, y));
        }

        let result = fitter.fit_circle(&points);
        assert!(result.is_some());

        let circle = result.unwrap();
        assert!((circle.center.x - center.x).abs() < 0.1);
        assert!((circle.center.y - center.y).abs() < 0.1);
        assert!((circle.radius - radius).abs() < 0.1);
    }

    #[test]
    fn test_circle_fitting_three_points() {
        let fitter = CircleFitter::new(3);

        // Three points on a circle: (0,0), (2,0), (1,1)
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(1.0, 1.0),
        ];

        let result = fitter.fit_circle_three_points(&points);
        assert!(result.is_some());

        let circle = result.unwrap();
        // Expected center around (1, 0) with radius 1
        assert!((circle.center.x - 1.0).abs() < 0.1);
        assert!((circle.radius - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_circle_fitting_ransac() {
        let fitter = CircleFitter::new(3);

        // Create points on a circle with some noise and outliers
        let center = Point2::new(0.0, 0.0);
        let radius = 1.0;
        let mut points = Vec::new();

        // Add inlier points on the circle
        for i in 0..20 {
            let angle = i as f64 * std::f64::consts::PI / 10.0;
            let x = center.x + radius * angle.cos();
            let y = center.y + radius * angle.sin();
            points.push(Point2::new(x, y));
        }

        // Add some outliers
        points.push(Point2::new(5.0, 5.0));
        points.push(Point2::new(-5.0, -5.0));

        let result = fitter.fit_circle_ransac(&points, 100);
        assert!(result.is_some());

        let circle = result.unwrap();
        assert!((circle.center.x - center.x).abs() < 0.2);
        assert!((circle.center.y - center.y).abs() < 0.2);
        assert!((circle.radius - radius).abs() < 0.2);
    }

    #[test]
    fn test_circle_fitting_collinear_points() {
        let fitter = CircleFitter::new(3);

        // Three collinear points - should fail
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
        ];

        let result = fitter.fit_circle_three_points(&points);
        assert!(result.is_none());
    }

    #[test]
    fn test_hole_detection_geometric() {
        let detector = HoleDetector::default();

        // Create mock projected points with a circular hole pattern
        let mut projected_points = Vec::new();

        // Create points around a circle (simulating the boundary of a hole)
        for i in 0..20 {
            let angle = i as f64 * std::f64::consts::PI / 10.0;
            let radius = 0.1; // 10cm hole
            let x = radius * angle.cos();
            let y = radius * angle.sin();

            // Don't add points inside the circle to simulate a hole
            if (x * x + y * y).sqrt() > 0.08 {
                projected_points.push(ProjectedPoint {
                    point_2d: Point2::new(x, y),
                    original_index: i,
                    depth: 0.0,
                });
            }
        }

        let result = detector.detect_holes_by_geometry(&projected_points);
        assert!(result.is_ok());
        // Result may be empty due to simple test data, but should not error
    }

    #[test]
    fn test_occupancy_grid_creation() {
        let detector = HoleDetector::default();

        let projected_points = vec![
            ProjectedPoint {
                point_2d: Point2::new(0.0, 0.0),
                original_index: 0,
                depth: 0.0,
            },
            ProjectedPoint {
                point_2d: Point2::new(0.1, 0.1),
                original_index: 1,
                depth: 0.0,
            },
        ];

        let result = detector.create_occupancy_grid(&projected_points, 0.05);
        assert!(result.is_ok());

        let grid = result.unwrap();
        assert!(grid.width > 0);
        assert!(grid.height > 0);
    }

    #[test]
    fn test_circular_region_validation() {
        let detector = HoleDetector::default();

        // Create a roughly circular region
        let mut circular_region = Vec::new();
        for i in 0..16 {
            let angle = i as f64 * std::f64::consts::PI / 8.0;
            let x = (5.0 + 2.0 * angle.cos()) as usize;
            let y = (5.0 + 2.0 * angle.sin()) as usize;
            circular_region.push((x, y));
        }

        assert!(detector.is_roughly_circular_region(&circular_region));

        // Create a linear region (should not be circular)
        let linear_region = vec![(0, 0), (1, 0), (2, 0), (3, 0)];
        assert!(!detector.is_roughly_circular_region(&linear_region));
    }
}
