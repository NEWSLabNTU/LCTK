use crate::types::BoardDetection;
use eyre::Result;
use hollow_board_detector::Detector;
use nalgebra::Point3;

/// Trait for board detection
pub trait DetectionProcessor: Send + Sync {
    fn process(&self, points: &[Point3<f64>]) -> Result<Option<BoardDetection>>;
}

/// Wrapper around hollow-board-detector
pub struct HollowBoardDetectionProcessor {
    detector: Detector,
}

impl HollowBoardDetectionProcessor {
    pub fn new(detector: Detector) -> Self {
        Self { detector }
    }

    pub fn from_config_file(config_path: &str) -> Result<Self> {
        use aruco_config::{ArucoDictionary, MultiArucoPattern};
        use hollow_board_detector::Config;
        use measurements::Length;
        use noisy_float::prelude::*;
        use std::fs;

        // Load config from file
        let config_content = fs::read_to_string(config_path)
            .map_err(|e| eyre::eyre!("Failed to read config file {}: {}", config_path, e))?;
        let config: Config = json5::from_str(&config_content)
            .map_err(|e| eyre::eyre!("Failed to parse config file {}: {}", config_path, e))?;

        let aruco_pattern = MultiArucoPattern {
            marker_ids: vec![1, 2, 3, 4],
            dictionary: ArucoDictionary::DICT_4X4_50,
            board_size: Length::from_inches(12.0),
            board_border_size: Length::from_inches(1.0),
            marker_square_size_ratio: R64::new(0.8),
            num_squares_per_side: 2,
            border_bits: 1,
        };

        let detector = Detector::new(config, aruco_pattern);
        Ok(Self { detector })
    }
}

impl DetectionProcessor for HollowBoardDetectionProcessor {
    fn process(&self, points: &[Point3<f64>]) -> Result<Option<BoardDetection>> {
        match self.detector.detect(points) {
            Ok(Some(detection)) => {
                // Convert hollow_board_detector::Detection to our BoardDetection
                let board_detection = BoardDetection {
                    pose: detection.board_model.pose,
                    confidence: 0.8, // Default confidence since detector doesn't provide this
                    inlier_count: detection.plane_ransac_data.inlier_points.len(),
                    timestamp: std::time::SystemTime::now(),
                };
                Ok(Some(board_detection))
            }
            Ok(None) => Ok(None),
            Err(e) => Err(eyre::eyre!("Detection failed: {}", e)),
        }
    }
}

/// Mock detector for testing
#[cfg(test)]
pub struct MockDetectionProcessor {
    should_detect: bool,
}

#[cfg(test)]
impl MockDetectionProcessor {
    pub fn new(should_detect: bool) -> Self {
        Self { should_detect }
    }
}

#[cfg(test)]
impl DetectionProcessor for MockDetectionProcessor {
    fn process(&self, _points: &[Point3<f64>]) -> Result<Option<BoardDetection>> {
        if self.should_detect {
            Ok(Some(BoardDetection {
                pose: nalgebra::Isometry3::identity(),
                confidence: 0.8,
                inlier_count: 100,
                timestamp: std::time::SystemTime::now(),
            }))
        } else {
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_detector_positive() {
        let detector = MockDetectionProcessor::new(true);
        let points = vec![Point3::new(0.0, 0.0, 0.0)];

        let result = detector.process(&points).unwrap();
        assert!(result.is_some());

        let detection = result.unwrap();
        assert_eq!(detection.confidence, 0.8);
        assert_eq!(detection.inlier_count, 100);
    }

    #[test]
    fn test_mock_detector_negative() {
        let detector = MockDetectionProcessor::new(false);
        let points = vec![Point3::new(0.0, 0.0, 0.0)];

        let result = detector.process(&points).unwrap();
        assert!(result.is_none());
    }
}
