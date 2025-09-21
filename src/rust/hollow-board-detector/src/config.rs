use hollow_board_config::BoardShape;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Config {
    pub max_icp_iterations: usize,
    pub icp_pose_weight_threshold: f64,
    pub icp_rejection_threshold: f64,
    pub plane_ransac_max_iterations: usize,
    pub plane_ransac_inlier_threshold: f64,

    // ICP algorithm tuning parameters
    pub icp_good_fit_threshold: f64,
    pub icp_outlier_threshold: f64,
    pub icp_adaptive_threshold_multiplier: f64,
    pub icp_adaptive_threshold_min: f64,
    pub icp_adaptive_threshold_max: f64,
    pub icp_damping_factor: f64,
    pub icp_min_inlier_points: usize,

    #[serde(flatten)]
    pub board_shape: BoardShape,
}
