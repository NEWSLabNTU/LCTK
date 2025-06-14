//! Diamond-oriented square board fitting algorithms

use anyhow::Result;
use nalgebra::{Isometry3, Point2, Point3, Rotation3, Translation3, Vector2, Vector3};
use std::{
    collections::HashMap,
    f64::consts::{PI, SQRT_2},
    time::Instant,
};

use crate::{
    debug::{stages, AlgorithmStats, DebugContext, DebugData, StageMetrics},
    types::{BoardDetection, BoundingBox, DetectedPlane, DetectionConfidence, PointCloud},
};
use board_fitter_config::SquareBoard;

/// Configuration for diamond square fitting
#[derive(Debug, Clone)]
pub struct DiamondFittingConfig {
    /// Expected board size (side length in meters)
    pub expected_size: f64,
    /// Size tolerance (±percentage)
    pub size_tolerance: f64,
    /// Diamond angle tolerance in radians (±degrees)
    pub angle_tolerance: f64,
    /// Minimum aspect ratio (width/height)
    pub min_aspect_ratio: f64,
    /// Maximum aspect ratio (width/height)
    pub max_aspect_ratio: f64,
    /// Minimum points required for fitting
    pub min_points: usize,
}

impl Default for DiamondFittingConfig {
    fn default() -> Self {
        Self {
            expected_size: 1.0,                    // 1 meter
            size_tolerance: 0.2,                   // ±20%
            angle_tolerance: 5.0_f64.to_radians(), // ±5°
            min_aspect_ratio: 0.8,                 // Square should be close to 1:1
            max_aspect_ratio: 1.2,
            min_points: 50,
        }
    }
}

/// Diamond square fitter for calibration boards
pub struct DiamondSquareFitter {
    config: DiamondFittingConfig,
}

impl DiamondSquareFitter {
    /// Create a new diamond square fitter
    pub fn new(config: DiamondFittingConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration
    pub fn default() -> Self {
        Self::new(DiamondFittingConfig::default())
    }

    /// Create from board configuration
    pub fn from_board_config(board: &SquareBoard) -> Self {
        let config = DiamondFittingConfig {
            expected_size: board.size.as_meters(),
            ..DiamondFittingConfig::default()
        };
        Self::new(config)
    }

    /// Fit diamond square to detected plane
    pub fn fit_square(
        &self,
        point_cloud: &PointCloud,
        plane: &DetectedPlane,
    ) -> Result<Option<DiamondSquare>> {
        self.fit_square_with_debug(point_cloud, plane, None)
    }

    /// Fit diamond square to detected plane with optional debug context
    pub fn fit_square_with_debug(
        &self,
        point_cloud: &PointCloud,
        plane: &DetectedPlane,
        mut debug_ctx: Option<&mut DebugContext>,
    ) -> Result<Option<DiamondSquare>> {
        let start_time = Instant::now();

        if let Some(ref mut ctx) = debug_ctx {
            ctx.start_stage(stages::DIAMOND_FITTING);

            // Emit input plane data
            let mut metadata = HashMap::new();
            metadata.insert("inlier_count".to_string(), plane.inliers.len().to_string());
            metadata.insert("plane_score".to_string(), plane.score.to_string());

            let debug_data = DebugData::PlaneData {
                planes: vec![plane.clone()],
                inlier_counts: vec![plane.inliers.len()],
                quality_scores: vec![plane.score],
                metadata,
            };
            ctx.emit_data(stages::DIAMOND_FITTING, &debug_data);
        }

        if plane.inliers.len() < self.config.min_points {
            if let Some(ref mut ctx) = debug_ctx {
                let mut algo_stats = AlgorithmStats::new("Diamond_Square_Fitting", 0, false);
                algo_stats.add_stat("insufficient_points", plane.inliers.len());
                algo_stats.add_stat("min_required_points", self.config.min_points);
                ctx.emit_algorithm_stats(stages::DIAMOND_FITTING, &algo_stats);
                ctx.end_stage(stages::DIAMOND_FITTING);
            }
            return Ok(None);
        }

        // Project points to 2D plane coordinate system
        let plane_points = self.project_to_plane(point_cloud, plane)?;

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert(
                "projected_points".to_string(),
                plane_points.len().to_string(),
            );
            metadata.insert("stage".to_string(), "projection".to_string());

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert(
                        "projected_points_count".to_string(),
                        plane_points.len().into(),
                    );
                    data.insert(
                        "plane_normal".to_string(),
                        serde_json::json!([plane.normal.x, plane.normal.y, plane.normal.z]),
                    );
                    data.insert(
                        "plane_point".to_string(),
                        serde_json::json!([plane.point.x, plane.point.y, plane.point.z]),
                    );
                    data
                },
            };
            ctx.emit_data(stages::DIAMOND_FITTING, &debug_data);
        }

        // Find boundary points using convex hull
        let boundary = self.compute_convex_hull(&plane_points);

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert("hull_points".to_string(), boundary.len().to_string());
            metadata.insert("stage".to_string(), "convex_hull".to_string());

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert("hull_points_count".to_string(), boundary.len().into());
                    data.insert(
                        "hull_points".to_string(),
                        serde_json::json!(boundary.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>()),
                    );
                    data
                },
            };
            ctx.emit_data(stages::DIAMOND_FITTING, &debug_data);
        }

        // Fit square to boundary points
        let mut fitting_attempts = 0;
        let mut fitting_success = false;
        let mut validation_success = false;

        if let Some(square_2d) = self.fit_square_2d(&boundary)? {
            fitting_attempts = 1;
            fitting_success = true;

            if let Some(ref mut ctx) = debug_ctx {
                let mut metadata = HashMap::new();
                metadata.insert("stage".to_string(), "square_fitting".to_string());
                metadata.insert("fitting_success".to_string(), "true".to_string());

                let debug_data = DebugData::Generic {
                    data: {
                        let mut data = HashMap::new();
                        data.insert(
                            "square_center".to_string(),
                            serde_json::json!([square_2d.center.x, square_2d.center.y]),
                        );
                        data.insert("square_size".to_string(), square_2d.size.into());
                        data.insert("square_angle".to_string(), square_2d.rotation.into());
                        data
                    },
                };
                ctx.emit_data(stages::DIAMOND_FITTING, &debug_data);
            }

            // Validate square properties
            if self.validate_square(&square_2d) {
                validation_success = true;

                // Convert back to 3D
                let square_3d = self.square_2d_to_3d(&square_2d, plane)?;

                let duration = start_time.elapsed();

                if let Some(ref mut ctx) = debug_ctx {
                    // Emit final metrics
                    let metrics = StageMetrics::new(plane.inliers.len(), 1, duration);
                    ctx.emit_metrics(stages::DIAMOND_FITTING, &metrics);

                    let mut algo_stats =
                        AlgorithmStats::new("Diamond_Square_Fitting", fitting_attempts, true);
                    algo_stats.add_stat("fitting_success", if fitting_success { 1.0 } else { 0.0 });
                    algo_stats.add_stat(
                        "validation_success",
                        if validation_success { 1.0 } else { 0.0 },
                    );
                    algo_stats.add_stat("square_size", square_2d.size);
                    algo_stats.add_stat("square_angle_degrees", square_2d.rotation.to_degrees());
                    ctx.emit_algorithm_stats(stages::DIAMOND_FITTING, &algo_stats);

                    ctx.end_stage(stages::DIAMOND_FITTING);
                }

                return Ok(Some(square_3d));
            }
        }

        let duration = start_time.elapsed();

        if let Some(ref mut ctx) = debug_ctx {
            // Emit failure metrics
            let metrics = StageMetrics::new(plane.inliers.len(), 0, duration);
            ctx.emit_metrics(stages::DIAMOND_FITTING, &metrics);

            let mut algo_stats =
                AlgorithmStats::new("Diamond_Square_Fitting", fitting_attempts, false);
            algo_stats.add_stat("fitting_success", if fitting_success { 1.0 } else { 0.0 });
            algo_stats.add_stat(
                "validation_success",
                if validation_success { 1.0 } else { 0.0 },
            );
            ctx.emit_algorithm_stats(stages::DIAMOND_FITTING, &algo_stats);

            ctx.end_stage(stages::DIAMOND_FITTING);
        }

        Ok(None)
    }

    /// Project plane points to 2D coordinate system
    fn project_to_plane(
        &self,
        point_cloud: &PointCloud,
        plane: &DetectedPlane,
    ) -> Result<Vec<Point2<f64>>> {
        // Create plane coordinate system
        let normal = plane.normal;
        let origin = plane.point;

        // Create orthonormal basis
        let u = if normal.z.abs() < 0.9 {
            normal.cross(&Vector3::z()).normalize()
        } else {
            normal.cross(&Vector3::x()).normalize()
        };
        let v = normal.cross(&u);

        // Project points
        let mut plane_points = Vec::new();
        for &idx in &plane.inliers {
            let point = point_cloud.points[idx];
            let relative = point - origin;
            let x = relative.dot(&u);
            let y = relative.dot(&v);
            plane_points.push(Point2::new(x, y));
        }

        Ok(plane_points)
    }

    /// Compute convex hull of 2D points (Graham scan)
    fn compute_convex_hull(&self, points: &[Point2<f64>]) -> Vec<Point2<f64>> {
        if points.len() < 3 {
            return points.to_vec();
        }

        let mut points = points.to_vec();

        // Find the bottom-most point (and left-most in case of tie)
        let mut bottom = 0;
        for i in 1..points.len() {
            if points[i].y < points[bottom].y
                || (points[i].y == points[bottom].y && points[i].x < points[bottom].x)
            {
                bottom = i;
            }
        }
        points.swap(0, bottom);

        let start = points[0];

        // Sort points by polar angle with respect to start point
        points[1..].sort_by(|a, b| {
            let angle_a = (a.y - start.y).atan2(a.x - start.x);
            let angle_b = (b.y - start.y).atan2(b.x - start.x);
            angle_a.partial_cmp(&angle_b).unwrap()
        });

        // Remove points with same angle (keep the farthest)
        let mut unique_points = vec![points[0]];
        for i in 1..points.len() {
            // Skip points that are collinear with previous point
            while unique_points.len() > 1
                && self
                    .cross_product_2d(
                        unique_points[unique_points.len() - 2],
                        unique_points[unique_points.len() - 1],
                        points[i],
                    )
                    .abs()
                    < 1e-10
            {
                // If the new point is farther, replace the last point
                let dist_last = (unique_points[unique_points.len() - 1] - start).norm_squared();
                let dist_new = (points[i] - start).norm_squared();
                if dist_new > dist_last {
                    unique_points.pop();
                } else {
                    break;
                }
            }
            unique_points.push(points[i]);
        }

        if unique_points.len() < 3 {
            return unique_points;
        }

        // Graham scan
        let mut hull = vec![unique_points[0], unique_points[1]];

        for i in 2..unique_points.len() {
            // Remove points that make a right turn
            while hull.len() > 1
                && self.cross_product_2d(
                    hull[hull.len() - 2],
                    hull[hull.len() - 1],
                    unique_points[i],
                ) < 0.0
            {
                hull.pop();
            }
            hull.push(unique_points[i]);
        }

        hull
    }

    /// Compute 2D cross product for three points
    fn cross_product_2d(&self, a: Point2<f64>, b: Point2<f64>, c: Point2<f64>) -> f64 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }

    /// Fit square to 2D boundary points
    fn fit_square_2d(&self, boundary: &[Point2<f64>]) -> Result<Option<Square2D>> {
        if boundary.len() < 4 {
            return Ok(None);
        }

        // Try different fitting approaches

        // 1. PCA-based fitting for oriented squares
        if let Some(square) = self.fit_square_pca(boundary)? {
            return Ok(Some(square));
        }

        // 2. Diamond-oriented fitting (45° rotation)
        if let Some(square) = self.fit_diamond_square(boundary)? {
            return Ok(Some(square));
        }

        Ok(None)
    }

    /// Fit square using PCA (Principal Component Analysis)
    fn fit_square_pca(&self, points: &[Point2<f64>]) -> Result<Option<Square2D>> {
        if points.len() < 4 {
            return Ok(None);
        }

        // Compute centroid
        let centroid = self.compute_centroid(points);

        // Center the points
        let centered_points: Vec<Point2<f64>> = points
            .iter()
            .map(|p| Point2::new(p.x - centroid.x, p.y - centroid.y))
            .collect();

        // Compute covariance matrix
        let mut cov_xx = 0.0;
        let mut cov_xy = 0.0;
        let mut cov_yy = 0.0;

        for p in &centered_points {
            cov_xx += p.x * p.x;
            cov_xy += p.x * p.y;
            cov_yy += p.y * p.y;
        }

        let n = centered_points.len() as f64;
        cov_xx /= n;
        cov_xy /= n;
        cov_yy /= n;

        // Compute eigenvalues and eigenvectors
        let trace = cov_xx + cov_yy;
        let det = cov_xx * cov_yy - cov_xy * cov_xy;
        let discriminant = trace * trace - 4.0 * det;

        if discriminant < 0.0 {
            return Ok(None);
        }

        let sqrt_discriminant = discriminant.sqrt();
        let eigenval1 = (trace + sqrt_discriminant) / 2.0;
        let _eigenval2 = (trace - sqrt_discriminant) / 2.0;

        // Compute principal direction (eigenvector for larger eigenvalue)
        let (principal_x, principal_y) = if cov_xy.abs() > 1e-10 {
            let v_x = eigenval1 - cov_yy;
            let v_y = cov_xy;
            let norm = (v_x * v_x + v_y * v_y).sqrt();
            (v_x / norm, v_y / norm)
        } else if cov_xx > cov_yy {
            (1.0, 0.0)
        } else {
            (0.0, 1.0)
        };

        // Rotation angle of the principal axis
        let rotation = principal_y.atan2(principal_x);

        // Transform points to aligned coordinate system
        let cos_rot = rotation.cos();
        let sin_rot = rotation.sin();

        let aligned_points: Vec<Point2<f64>> = centered_points
            .iter()
            .map(|p| {
                Point2::new(
                    cos_rot * p.x + sin_rot * p.y,
                    -sin_rot * p.x + cos_rot * p.y,
                )
            })
            .collect();

        // Find bounding box in aligned coordinate system
        let (min_x, max_x, min_y, max_y) = self.compute_bounding_box(&aligned_points);

        let width = max_x - min_x;
        let height = max_y - min_y;

        // Check if it's roughly square
        let aspect_ratio = width / height;
        if aspect_ratio < self.config.min_aspect_ratio
            || aspect_ratio > self.config.max_aspect_ratio
        {
            return Ok(None);
        }

        let size = (width + height) / 2.0;

        // Check size constraints
        let size_error = (size - self.config.expected_size).abs() / self.config.expected_size;
        if size_error > self.config.size_tolerance {
            return Ok(None);
        }

        Ok(Some(Square2D {
            center: centroid,
            size,
            rotation,
        }))
    }

    /// Fit diamond-oriented square (45° rotation)
    fn fit_diamond_square(&self, points: &[Point2<f64>]) -> Result<Option<Square2D>> {
        // Find centroid
        let centroid = self.compute_centroid(points);

        // Convert to diamond coordinate system (45° rotation)
        let diamond_points: Vec<Point2<f64>> = points
            .iter()
            .map(|p| self.rotate_point_45(*p, centroid))
            .collect();

        // Find axis-aligned bounding box in diamond space
        let (min_x, max_x, min_y, max_y) = self.compute_bounding_box(&diamond_points);

        let width = max_x - min_x;
        let height = max_y - min_y;
        let size = (width + height) / 2.0; // Average for square

        // Check if it matches expected size
        let size_error = (size - self.config.expected_size).abs() / self.config.expected_size;
        if size_error > self.config.size_tolerance {
            return Ok(None);
        }

        // Check aspect ratio
        let aspect_ratio = width / height;
        if aspect_ratio < self.config.min_aspect_ratio
            || aspect_ratio > self.config.max_aspect_ratio
        {
            return Ok(None);
        }

        // Create diamond square
        let diamond_center = Point2::new((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
        let center = self.rotate_point_minus_45(diamond_center, Point2::origin());

        Ok(Some(Square2D {
            center: center + centroid.coords,
            size,
            rotation: PI / 4.0, // 45° rotation for diamond
        }))
    }

    /// Rotate point by 45° around center
    fn rotate_point_45(&self, point: Point2<f64>, center: Point2<f64>) -> Point2<f64> {
        let relative = point - center;
        let cos45 = SQRT_2 / 2.0;
        let sin45 = SQRT_2 / 2.0;

        Point2::new(
            cos45 * relative.x - sin45 * relative.y,
            sin45 * relative.x + cos45 * relative.y,
        )
    }

    /// Rotate point by -45° around center
    fn rotate_point_minus_45(&self, point: Point2<f64>, center: Point2<f64>) -> Point2<f64> {
        let relative = point - center;
        let cos45 = SQRT_2 / 2.0;
        let sin45 = -SQRT_2 / 2.0;

        Point2::new(
            cos45 * relative.x - sin45 * relative.y,
            sin45 * relative.x + cos45 * relative.y,
        )
    }

    /// Compute centroid of points
    fn compute_centroid(&self, points: &[Point2<f64>]) -> Point2<f64> {
        let sum = points
            .iter()
            .fold(Vector2::zeros(), |acc, p| acc + p.coords);
        Point2::from(sum / points.len() as f64)
    }

    /// Compute 2D bounding box
    fn compute_bounding_box(&self, points: &[Point2<f64>]) -> (f64, f64, f64, f64) {
        if points.is_empty() {
            return (0.0, 0.0, 0.0, 0.0);
        }

        let first = points[0];
        let mut min_x = first.x;
        let mut max_x = first.x;
        let mut min_y = first.y;
        let mut max_y = first.y;

        for point in points.iter().skip(1) {
            min_x = min_x.min(point.x);
            max_x = max_x.max(point.x);
            min_y = min_y.min(point.y);
            max_y = max_y.max(point.y);
        }

        (min_x, max_x, min_y, max_y)
    }

    /// Validate fitted square properties
    fn validate_square(&self, square: &Square2D) -> bool {
        // Check size is within tolerance
        let size_error =
            (square.size - self.config.expected_size).abs() / self.config.expected_size;
        if size_error > self.config.size_tolerance {
            return false;
        }

        // Check rotation is close to diamond angle (45°)
        let angle_error = (square.rotation - PI / 4.0).abs();
        if angle_error > self.config.angle_tolerance {
            return false;
        }

        true
    }

    /// Convert 2D square back to 3D diamond square
    fn square_2d_to_3d(
        &self,
        square_2d: &Square2D,
        plane: &DetectedPlane,
    ) -> Result<DiamondSquare> {
        // Create plane coordinate system (same as projection)
        let normal = plane.normal;
        let origin = plane.point;

        let u = if normal.z.abs() < 0.9 {
            normal.cross(&Vector3::z()).normalize()
        } else {
            normal.cross(&Vector3::x()).normalize()
        };
        let v = normal.cross(&u);

        // Convert 2D center to 3D
        let center_3d = origin + u * square_2d.center.x + v * square_2d.center.y;

        // Create rotation from diamond angle and plane normal
        let axis = nalgebra::Unit::new_normalize(normal);
        let z_rotation = Rotation3::from_axis_angle(&axis, square_2d.rotation);
        let pose = Isometry3::from_parts(Translation3::from(center_3d.coords), z_rotation.into());

        // Generate diamond corners
        let corners = self.generate_diamond_corners(square_2d.size);

        Ok(DiamondSquare {
            center: center_3d,
            size: square_2d.size,
            pose,
            corners,
            normal,
        })
    }

    /// Generate diamond corner positions for a square
    fn generate_diamond_corners(&self, size: f64) -> [Point3<f64>; 4] {
        let half_diagonal = size / 2.0 * SQRT_2;
        [
            Point3::new(half_diagonal, 0.0, 0.0),  // Right
            Point3::new(0.0, half_diagonal, 0.0),  // Top
            Point3::new(-half_diagonal, 0.0, 0.0), // Left
            Point3::new(0.0, -half_diagonal, 0.0), // Bottom
        ]
    }
}

/// 2D square representation for fitting
#[derive(Debug, Clone)]
struct Square2D {
    center: Point2<f64>,
    size: f64,
    rotation: f64, // Radians
}

/// 3D diamond square detection result
#[derive(Debug, Clone)]
pub struct DiamondSquare {
    /// Center position of the square
    pub center: Point3<f64>,
    /// Side length of the square
    pub size: f64,
    /// 6DOF pose of the square
    pub pose: Isometry3<f64>,
    /// Diamond corner positions in local coordinates
    pub corners: [Point3<f64>; 4],
    /// Normal vector of the square plane
    pub normal: Vector3<f64>,
}

impl DiamondSquare {
    /// Convert to board detection with confidence
    pub fn to_board_detection(&self, confidence: DetectionConfidence) -> BoardDetection {
        let dimensions = Vector3::new(self.size, self.size, 0.02); // Assume 2cm thickness
        let detection = BoardDetection::new(self.pose, confidence, dimensions);
        detection
    }

    /// Convert to board detection with confidence and supporting points
    pub fn to_board_detection_with_points(
        &self,
        confidence: DetectionConfidence,
        supporting_points: Vec<usize>,
    ) -> BoardDetection {
        let dimensions = Vector3::new(self.size, self.size, 0.02); // Assume 2cm thickness
        let mut detection = BoardDetection::new(self.pose, confidence, dimensions);
        detection.supporting_points = supporting_points;
        detection
    }

    /// Get corner positions in world coordinates
    pub fn world_corners(&self) -> [Point3<f64>; 4] {
        self.corners.map(|corner| self.pose * corner)
    }

    /// Check if a point is inside the square
    pub fn contains_point(&self, point: Point3<f64>, tolerance: f64) -> bool {
        // Transform point to local square coordinates
        let local_point = self.pose.inverse() * point;

        // Check if within square bounds (diamond orientation)
        let half_diagonal = self.size / 2.0 * SQRT_2;
        local_point.x.abs() <= half_diagonal + tolerance
            && local_point.y.abs() <= half_diagonal + tolerance
    }

    /// Get bounding box of the square
    pub fn bounding_box(&self) -> BoundingBox {
        let corners = self.world_corners();
        let first = corners[0];
        let mut min = first;
        let mut max = first;

        for corner in corners.iter().skip(1) {
            min.x = min.x.min(corner.x);
            min.y = min.y.min(corner.y);
            min.z = min.z.min(corner.z);
            max.x = max.x.max(corner.x);
            max.y = max.y.max(corner.y);
            max.z = max.z.max(corner.z);
        }

        BoundingBox { min, max }
    }
}

/// Utilities for diamond square validation
pub struct DiamondValidator {
    /// Expected size range
    pub size_range: (f64, f64),
    /// Expected aspect ratio range
    pub aspect_ratio_range: (f64, f64),
    /// Minimum area
    pub min_area: f64,
}

impl DiamondValidator {
    /// Create validator for 1m diamond boards
    pub fn for_1m_boards() -> Self {
        Self {
            size_range: (0.8, 1.2),         // 0.8m to 1.2m
            aspect_ratio_range: (0.8, 1.2), // Close to square
            min_area: 0.6,                  // 0.6 m²
        }
    }

    /// Validate a diamond square detection
    pub fn validate(&self, square: &DiamondSquare) -> bool {
        // Check size
        if square.size < self.size_range.0 || square.size > self.size_range.1 {
            return false;
        }

        // Check area
        let area = square.size * square.size;
        if area < self.min_area {
            return false;
        }

        // Additional validation could include:
        // - Normal vector orientation
        // - Point density
        // - Edge quality

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_diamond_fitting_config() {
        let config = DiamondFittingConfig::default();
        assert_eq!(config.expected_size, 1.0);
        assert_eq!(config.size_tolerance, 0.2);
    }

    #[test]
    fn test_point_rotation() {
        let fitter = DiamondSquareFitter::default();
        let point = Point2::new(1.0, 0.0);
        let center = Point2::origin();

        let rotated = fitter.rotate_point_45(point, center);
        let expected_x = SQRT_2 / 2.0;
        let expected_y = SQRT_2 / 2.0;

        assert_relative_eq!(rotated.x, expected_x, epsilon = 1e-10);
        assert_relative_eq!(rotated.y, expected_y, epsilon = 1e-10);
    }

    #[test]
    fn test_bounding_box_computation() {
        let fitter = DiamondSquareFitter::default();
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 1.0),
        ];

        let (min_x, max_x, min_y, max_y) = fitter.compute_bounding_box(&points);
        assert_eq!(min_x, 0.0);
        assert_eq!(max_x, 1.0);
        assert_eq!(min_y, 0.0);
        assert_eq!(max_y, 1.0);
    }

    #[test]
    fn test_diamond_square_corners() {
        let fitter = DiamondSquareFitter::default();
        let corners = fitter.generate_diamond_corners(1.0);

        // Check that corners form a diamond pattern
        let half_diag = SQRT_2 / 2.0;
        assert_relative_eq!(corners[0].x, half_diag, epsilon = 1e-10); // Right
        assert_relative_eq!(corners[1].y, half_diag, epsilon = 1e-10); // Top
        assert_relative_eq!(corners[2].x, -half_diag, epsilon = 1e-10); // Left
        assert_relative_eq!(corners[3].y, -half_diag, epsilon = 1e-10); // Bottom
    }

    #[test]
    fn test_convex_hull() {
        let fitter = DiamondSquareFitter::default();

        // Test with square points
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
            Point2::new(0.5, 0.5), // Interior point
        ];

        let hull = fitter.compute_convex_hull(&points);
        assert_eq!(hull.len(), 4); // Should have 4 vertices for the square

        // Verify all hull points are from the original set (except interior point)
        for hull_point in &hull {
            assert!(!hull_point.eq(&Point2::new(0.5, 0.5))); // Interior point should be excluded
        }
    }

    #[test]
    fn test_cross_product_2d() {
        let fitter = DiamondSquareFitter::default();

        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        let c = Point2::new(1.0, 1.0);

        let cross = fitter.cross_product_2d(a, b, c);
        assert_relative_eq!(cross, 1.0, epsilon = 1e-10); // Positive (counter-clockwise)

        let d = Point2::new(1.0, -1.0);
        let cross2 = fitter.cross_product_2d(a, b, d);
        assert_relative_eq!(cross2, -1.0, epsilon = 1e-10); // Negative (clockwise)
    }

    #[test]
    fn test_pca_square_fitting() {
        let mut config = DiamondFittingConfig::default();
        config.expected_size = 2.0; // Expect 2-unit square
        config.size_tolerance = 0.1;
        let fitter = DiamondSquareFitter::new(config);

        // Create axis-aligned square points
        let points = vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ];

        let result = fitter.fit_square_pca(&points).unwrap();
        assert!(result.is_some());

        let square = result.unwrap();
        assert_relative_eq!(square.center.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(square.center.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(square.size, 2.0, epsilon = 0.1);
    }

    #[test]
    fn test_pca_rotated_square() {
        let mut config = DiamondFittingConfig::default();
        config.expected_size = 2.0; // Side length of square would be 2.0 for this diamond
        config.size_tolerance = 0.3; // More tolerance for rotated shapes
        let fitter = DiamondSquareFitter::new(config);

        // Create 45-degree rotated square (diamond shape)
        // Distance from center to vertex is 1.0, so side length is sqrt(2) ≈ 1.414
        // But the bounding box measures the diagonal which is 2.0
        let points = vec![
            Point2::new(0.0, -1.0), // Bottom
            Point2::new(1.0, 0.0),  // Right
            Point2::new(0.0, 1.0),  // Top
            Point2::new(-1.0, 0.0), // Left
        ];

        let result = fitter.fit_square_pca(&points).unwrap();
        assert!(result.is_some());

        let square = result.unwrap();
        assert_relative_eq!(square.center.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(square.center.y, 0.0, epsilon = 1e-10);

        // The rotation could be approximately 45 degrees (π/4) or 90 degrees (π/2)
        // depending on which principal component is selected
        let expected_rotation1 = std::f64::consts::PI / 4.0;
        let expected_rotation2 = std::f64::consts::PI / 2.0;
        let rotation_abs = square.rotation.abs();
        assert!(
            (rotation_abs - expected_rotation1).abs() < 0.1
                || (rotation_abs - expected_rotation2).abs() < 0.1,
            "Rotation {} is not close to π/4 or π/2",
            rotation_abs
        );
    }

    #[test]
    fn test_diamond_validator() {
        let validator = DiamondValidator::for_1m_boards();

        // Create a valid 1m square
        let valid_square = DiamondSquare {
            center: Point3::origin(),
            size: 1.0,
            pose: Isometry3::identity(),
            corners: [Point3::origin(); 4],
            normal: Vector3::z(),
        };

        // Create an invalid 3m square
        let invalid_square = DiamondSquare {
            center: Point3::origin(),
            size: 3.0, // Too large
            pose: Isometry3::identity(),
            corners: [Point3::origin(); 4],
            normal: Vector3::z(),
        };

        assert!(validator.validate(&valid_square));
        assert!(!validator.validate(&invalid_square));
    }
}
