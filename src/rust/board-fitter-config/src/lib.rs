use measurements::Length;
use serde::{Deserialize, Serialize};

/// A 2D point with physical units
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point2D {
    pub x: Length,
    pub y: Length,
}

/// Circular hole feature in the calibration board
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CircleHole {
    /// Radius of the circular hole
    pub radius: Length,
    /// Position of the hole center relative to board center
    pub position: Point2D,
    /// Optional identifier for the hole (useful for orientation detection)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

/// Square calibration board configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SquareBoard {
    /// Side length of the square board
    pub size: Length,
    /// Circular holes in the board for orientation detection
    pub holes: Vec<CircleHole>,
    /// Board thickness (optional, for 3D modeling)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thickness: Option<Length>,
}

/// Detection algorithm configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionConfig {
    /// Detection method name (e.g., "ransac_square_fitting")
    pub method: String,
    /// Algorithm-specific parameters
    #[serde(default)]
    pub parameters: serde_json::Map<String, serde_json::Value>,
}

/// Complete board fitter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardConfig {
    /// Square board model definition
    pub board: SquareBoard,
    /// Detection algorithm configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detection: Option<DetectionConfig>,
    /// Optional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

impl SquareBoard {
    /// Create a new square board with specified size
    pub fn new(size: Length) -> Self {
        Self {
            size,
            holes: Vec::new(),
            thickness: None,
        }
    }

    /// Add a circular hole to the board
    pub fn add_hole(&mut self, radius: Length, position: Point2D, id: Option<String>) {
        self.holes.push(CircleHole {
            radius,
            position,
            id,
        });
    }

    /// Get board center position
    pub fn center(&self) -> Point2D {
        Point2D {
            x: Length::from_meters(0.0),
            y: Length::from_meters(0.0),
        }
    }

    /// Get board corners relative to center (assuming diamond orientation)
    pub fn corners_diamond(&self) -> [Point2D; 4] {
        let half_diagonal = self.size.as_meters() / 2.0 * std::f64::consts::SQRT_2;
        [
            Point2D {
                x: Length::from_meters(half_diagonal),
                y: Length::from_meters(0.0),
            }, // Right
            Point2D {
                x: Length::from_meters(0.0),
                y: Length::from_meters(half_diagonal),
            }, // Top
            Point2D {
                x: Length::from_meters(-half_diagonal),
                y: Length::from_meters(0.0),
            }, // Left
            Point2D {
                x: Length::from_meters(0.0),
                y: Length::from_meters(-half_diagonal),
            }, // Bottom
        ]
    }

    /// Get board corners relative to center (standard square orientation)
    pub fn corners_aligned(&self) -> [Point2D; 4] {
        let half_size = self.size.as_meters() / 2.0;
        [
            Point2D {
                x: Length::from_meters(half_size),
                y: Length::from_meters(half_size),
            }, // Top-right
            Point2D {
                x: Length::from_meters(-half_size),
                y: Length::from_meters(half_size),
            }, // Top-left
            Point2D {
                x: Length::from_meters(-half_size),
                y: Length::from_meters(-half_size),
            }, // Bottom-left
            Point2D {
                x: Length::from_meters(half_size),
                y: Length::from_meters(-half_size),
            }, // Bottom-right
        ]
    }

    /// Check if a hole exists at the given position (within tolerance)
    pub fn has_hole_at(&self, position: &Point2D, tolerance: Length) -> Option<&CircleHole> {
        self.holes.iter().find(|hole| {
            let dx = hole.position.x.as_meters() - position.x.as_meters();
            let dy = hole.position.y.as_meters() - position.y.as_meters();
            let distance = (dx * dx + dy * dy).sqrt();
            distance <= tolerance.as_meters()
        })
    }

    /// Get bounding box dimensions
    pub fn bounding_box_size(&self) -> Length {
        self.size
    }
}

impl Default for DetectionConfig {
    fn default() -> Self {
        Self {
            method: "ransac_square_fitting".to_string(),
            parameters: serde_json::Map::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_square_board_creation() {
        let mut board = SquareBoard::new(Length::from_meters(0.6));
        board.add_hole(
            Length::from_meters(0.02),
            Point2D {
                x: Length::from_meters(0.1),
                y: Length::from_meters(0.1),
            },
            Some("hole_1".to_string()),
        );

        assert_eq!(board.size.as_meters(), 0.6);
        assert_eq!(board.holes.len(), 1);
        assert_eq!(board.holes[0].radius.as_meters(), 0.02);
    }

    #[test]
    fn test_diamond_corners() {
        let board = SquareBoard::new(Length::from_meters(0.6));
        let corners = board.corners_diamond();

        // Check that corners form a diamond (rotated square)
        let half_diag = 0.6 / 2.0 * std::f64::consts::SQRT_2;
        assert!((corners[0].x.as_meters() - half_diag).abs() < 1e-10);
        assert!((corners[0].y.as_meters()).abs() < 1e-10);
    }

    #[test]
    fn test_hole_detection() {
        let mut board = SquareBoard::new(Length::from_meters(0.6));
        board.add_hole(
            Length::from_meters(0.02),
            Point2D {
                x: Length::from_meters(0.1),
                y: Length::from_meters(0.1),
            },
            None,
        );

        let test_pos = Point2D {
            x: Length::from_meters(0.105),
            y: Length::from_meters(0.095),
        };

        assert!(board
            .has_hole_at(&test_pos, Length::from_meters(0.01))
            .is_some());
        assert!(board
            .has_hole_at(&test_pos, Length::from_meters(0.001))
            .is_none());
    }
}
