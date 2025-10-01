use crate::types::BoardDetection;
use eyre::Result;
use hollow_board_detector::Detector;
use nalgebra::Point3;
use rclrs::log_info;

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

    pub fn from_config_file(_config_path: &str) -> Result<Self> {
        // For now, create a minimal detector for compilation
        // TODO: Implement proper config loading from file
        use aruco_config::{ArucoDictionary, MultiArucoPattern};
        use hollow_board_config::BoardShape;
        use hollow_board_detector::Config;
        use measurements::Length;
        use noisy_float::prelude::*;

        let config = Config {
            max_icp_iterations: 100,
            icp_pose_weight_threshold: 0.95,
            icp_rejection_threshold: 0.01,
            plane_ransac_max_iterations: 1000,
            plane_ransac_inlier_threshold: 0.5,
            skip_ransac: false,

            // ICP algorithm tuning parameters
            icp_good_fit_threshold: 0.015,
            icp_outlier_threshold: 0.1,
            icp_damping_factor: 0.3,
            icp_min_inlier_points: 200,

            board_shape: BoardShape {
                board_width: Length::from_inches(12.0),
                hole_radius: Length::from_inches(0.5),
                hole_center_shift: Length::from_inches(2.0),
            },
        };

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
                // Log ICP result to service logs
                let final_loss = detection
                    .icp_losses
                    .iter()
                    .copied()
                    .min_by(|a, b| a.partial_cmp(b).unwrap())
                    .unwrap_or(0.0);

                log_info!(
                    "hollow_board_detector",
                    "FINAL ICP RESULT: pose=({:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}), loss={:.6}",
                    detection.board_model.pose.translation.x,
                    detection.board_model.pose.translation.y,
                    detection.board_model.pose.translation.z,
                    detection.board_model.pose.rotation.i,
                    detection.board_model.pose.rotation.j,
                    detection.board_model.pose.rotation.k,
                    final_loss
                );

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
