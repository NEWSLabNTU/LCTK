use crate::config::Config;
use aruco_config::MultiArucoPattern;

/// Holds the board-detector configuration and the ArUco pattern it was built
/// with.
///
/// This used to own a full `detect()` pipeline (RANSAC → initial pose → ICP).
/// `lidar_board_detector` drives that pipeline itself — see
/// `process_pointcloud` — and never called these methods, so they were removed
/// along with `algo::fit_board_icp*`. What remains is the config carrier the
/// node still constructs. Reach for `algo::{fit_plane_ransac, BoardIcpIterator}`
/// to assemble a pipeline.
#[derive(Debug, Clone)]
pub struct Detector {
    config: Config,
    aruco_pattern: MultiArucoPattern,
}

impl Detector {
    pub fn new(config: Config, aruco_pattern: MultiArucoPattern) -> Self {
        Self {
            config,
            aruco_pattern,
        }
    }

    /// Get a reference to the detector configuration
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Get a reference to the ArUco pattern
    pub fn aruco_pattern(&self) -> &MultiArucoPattern {
        &self.aruco_pattern
    }
}
