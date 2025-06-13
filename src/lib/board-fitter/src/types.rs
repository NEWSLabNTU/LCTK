//! Core types for board detection

use nalgebra::{Isometry3, Point3, Vector3};
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};
use uuid::Uuid;

/// 3D point cloud representation
#[derive(Debug, Clone)]
pub struct PointCloud {
    /// 3D points in the cloud
    pub points: Vec<Point3<f64>>,
    /// Optional intensity values for each point
    pub intensities: Option<Vec<f32>>,
    /// Optional color information (RGB)
    pub colors: Option<Vec<[u8; 3]>>,
    /// Timestamp when the point cloud was captured
    pub timestamp: Instant,
    /// Frame ID for coordinate system reference
    pub frame_id: String,
}

impl PointCloud {
    /// Create a new point cloud
    pub fn new(points: Vec<Point3<f64>>, frame_id: String) -> Self {
        Self {
            points,
            intensities: None,
            colors: None,
            timestamp: Instant::now(),
            frame_id,
        }
    }

    /// Get the number of points in the cloud
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the point cloud is empty
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Get points within a bounding box
    pub fn points_in_bbox(&self, bbox: &BoundingBox) -> Vec<usize> {
        self.points
            .iter()
            .enumerate()
            .filter_map(|(idx, point)| {
                if bbox.contains(point) {
                    Some(idx)
                } else {
                    None
                }
            })
            .collect()
    }
}

/// 3D bounding box for region of interest
#[derive(Debug, Clone, PartialEq)]
pub struct BoundingBox {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl BoundingBox {
    /// Create a bounding box from center and size
    pub fn from_center(center: Point3<f64>, size: Vector3<f64>) -> Self {
        let half_size = size / 2.0;
        Self {
            min: center - half_size,
            max: center + half_size,
        }
    }

    /// Check if a point is inside the bounding box
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Get the center of the bounding box
    pub fn center(&self) -> Point3<f64> {
        Point3::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }

    /// Get the size of the bounding box
    pub fn size(&self) -> Vector3<f64> {
        Vector3::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Get the volume of the bounding box
    pub fn volume(&self) -> f64 {
        let size = self.size();
        size.x * size.y * size.z
    }

    /// Expand the bounding box by a given amount in all directions
    pub fn expand(&self, amount: f64) -> Self {
        let expansion = Vector3::new(amount, amount, amount);
        Self {
            min: self.min - expansion,
            max: self.max + expansion,
        }
    }

    /// Check if this bounding box intersects with another
    pub fn intersects(&self, other: &BoundingBox) -> bool {
        self.max.x >= other.min.x
            && self.min.x <= other.max.x
            && self.max.y >= other.min.y
            && self.min.y <= other.max.y
            && self.max.z >= other.min.z
            && self.min.z <= other.max.z
    }
}

/// Unique identifier for tracked boards
pub type BoardId = Uuid;

/// Confidence score for detection results
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct DetectionConfidence(f64);

impl DetectionConfidence {
    /// Create a new confidence score (clamped to [0.0, 1.0])
    pub fn new(confidence: f64) -> Self {
        Self(confidence.clamp(0.0, 1.0))
    }

    /// Get the confidence value
    pub fn value(&self) -> f64 {
        self.0
    }

    /// Check if confidence is above a threshold
    pub fn above_threshold(&self, threshold: f64) -> bool {
        self.0 > threshold
    }
}

/// Detected board with pose and confidence
#[derive(Debug, Clone)]
pub struct BoardDetection {
    /// Unique identifier for this detection
    pub id: BoardId,
    /// 6DOF pose of the board (position + orientation)
    pub pose: Isometry3<f64>,
    /// Detection confidence score
    pub confidence: DetectionConfidence,
    /// Timestamp of detection
    pub timestamp: Instant,
    /// Detected board dimensions
    pub dimensions: Vector3<f64>,
    /// Detected holes (if any)
    pub holes: Vec<DetectedHole>,
    /// Supporting point indices from the original cloud
    pub supporting_points: Vec<usize>,
}

impl BoardDetection {
    /// Create a new board detection
    pub fn new(
        pose: Isometry3<f64>,
        confidence: DetectionConfidence,
        dimensions: Vector3<f64>,
    ) -> Self {
        Self {
            id: Uuid::new_v4(),
            pose,
            confidence,
            timestamp: Instant::now(),
            dimensions,
            holes: Vec::new(),
            supporting_points: Vec::new(),
        }
    }

    /// Get the board center position
    pub fn center(&self) -> Point3<f64> {
        self.pose.translation.vector.into()
    }

    /// Get the board normal vector
    pub fn normal(&self) -> Vector3<f64> {
        self.pose.rotation * Vector3::z()
    }
}

/// Detected circular hole in a board
#[derive(Debug, Clone)]
pub struct DetectedHole {
    /// Center position of the hole
    pub center: Point3<f64>,
    /// Radius of the hole
    pub radius: f64,
    /// Confidence of hole detection
    pub confidence: DetectionConfidence,
    /// Optional ID matching the configuration
    pub id: Option<String>,
}

/// Planar surface detected in point cloud
#[derive(Debug, Clone)]
pub struct DetectedPlane {
    /// Plane normal vector
    pub normal: Vector3<f64>,
    /// A point on the plane
    pub point: Point3<f64>,
    /// Indices of points that belong to this plane
    pub inliers: Vec<usize>,
    /// Quality score of the plane fitting
    pub score: f64,
    /// Bounding box of the plane
    pub bbox: BoundingBox,
}

impl DetectedPlane {
    /// Create a new detected plane
    pub fn new(normal: Vector3<f64>, point: Point3<f64>, inliers: Vec<usize>) -> Self {
        Self {
            normal,
            point,
            inliers,
            score: 0.0,
            bbox: BoundingBox {
                min: Point3::origin(),
                max: Point3::origin(),
            },
        }
    }

    /// Get the plane equation coefficients (ax + by + cz + d = 0)
    pub fn equation(&self) -> [f64; 4] {
        let d = -self.normal.dot(&self.point.coords);
        [self.normal.x, self.normal.y, self.normal.z, d]
    }

    /// Calculate distance from a point to this plane
    pub fn distance_to_point(&self, point: &Point3<f64>) -> f64 {
        let eq = self.equation();
        (eq[0] * point.x + eq[1] * point.y + eq[2] * point.z + eq[3]).abs()
            / (eq[0] * eq[0] + eq[1] * eq[1] + eq[2] * eq[2]).sqrt()
    }
}

/// ROI (Region of Interest) for focused processing
#[derive(Debug, Clone)]
pub struct RegionOfInterest {
    /// 3D bounding box defining the region
    pub bbox: BoundingBox,
    /// Priority of this ROI for processing
    pub priority: f64,
    /// Associated board ID (if tracking)
    pub board_id: Option<BoardId>,
    /// ROI type/mode
    pub roi_type: RoiType,
}

/// Types of ROI for different processing modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoiType {
    /// Global search across entire workspace
    GlobalSearch,
    /// Local tracking around known board positions
    LocalTracking,
    /// Expanding search when boards are lost
    ExpandingSearch,
}

/// Processing statistics for performance monitoring
#[derive(Debug, Clone, Default)]
pub struct ProcessingStats {
    /// Total processing time
    pub total_time: Duration,
    /// Time spent on plane detection
    pub plane_detection_time: Duration,
    /// Time spent on board fitting
    pub board_fitting_time: Duration,
    /// Time spent on hole detection
    pub hole_detection_time: Duration,
    /// Number of points processed
    pub points_processed: usize,
    /// Number of planes detected
    pub planes_detected: usize,
    /// Number of boards detected
    pub boards_detected: usize,
}

impl ProcessingStats {
    /// Create new empty statistics
    pub fn new() -> Self {
        Self::default()
    }

    /// Add processing time for a specific stage
    pub fn add_time(&mut self, stage: ProcessingStage, duration: Duration) {
        match stage {
            ProcessingStage::PlaneDetection => self.plane_detection_time += duration,
            ProcessingStage::BoardFitting => self.board_fitting_time += duration,
            ProcessingStage::HoleDetection => self.hole_detection_time += duration,
            ProcessingStage::Detection => {
                // Detection encompasses all sub-stages, so just update total time
            }
        }
        self.total_time += duration;
    }

    /// Get processing rate in points per second
    pub fn points_per_second(&self) -> f64 {
        if self.total_time.as_secs_f64() > 0.0 {
            self.points_processed as f64 / self.total_time.as_secs_f64()
        } else {
            0.0
        }
    }

    /// Get detection efficiency (boards detected per plane)
    pub fn detection_efficiency(&self) -> f64 {
        if self.planes_detected > 0 {
            self.boards_detected as f64 / self.planes_detected as f64
        } else {
            0.0
        }
    }

    /// Get total processing time as milliseconds
    pub fn total_time_ms(&self) -> f64 {
        self.total_time.as_secs_f64() * 1000.0
    }
}

/// Processing stages for time tracking
#[derive(Debug, Clone, Copy)]
pub enum ProcessingStage {
    PlaneDetection,
    BoardFitting,
    HoleDetection,
    Detection,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_bounding_box_creation() {
        let center = Point3::new(1.0, 2.0, 3.0);
        let size = Vector3::new(2.0, 4.0, 6.0);
        let bbox = BoundingBox::from_center(center, size);

        assert_eq!(bbox.center(), center);
        assert_eq!(bbox.size(), size);
    }

    #[test]
    fn test_confidence_clamping() {
        let conf1 = DetectionConfidence::new(1.5);
        assert_eq!(conf1.value(), 1.0);

        let conf2 = DetectionConfidence::new(-0.5);
        assert_eq!(conf2.value(), 0.0);

        let conf3 = DetectionConfidence::new(0.7);
        assert_eq!(conf3.value(), 0.7);
    }

    #[test]
    fn test_plane_distance_calculation() {
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let point = Point3::new(0.0, 0.0, 5.0);
        let plane = DetectedPlane::new(normal, point, vec![]);

        let test_point = Point3::new(1.0, 1.0, 8.0);
        let distance = plane.distance_to_point(&test_point);
        assert_relative_eq!(distance, 3.0, epsilon = 1e-10);
    }
}
