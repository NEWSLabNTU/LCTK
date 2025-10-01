use hollow_board_config::BoardShape;
use serde::{Deserialize, Serialize};

fn default_voxel_size() -> f64 {
    0.02
}

fn default_true() -> bool {
    true
}

fn default_parallel_threshold() -> usize {
    50_000
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Config {
    pub max_icp_iterations: usize,
    pub icp_pose_weight_threshold: f64,
    pub icp_rejection_threshold: f64,
    pub plane_ransac_max_iterations: usize,
    pub plane_ransac_inlier_threshold: f64,

    // Skip RANSAC plane fitting and use all bbox-filtered points directly for ICP
    #[serde(default)]
    pub skip_ransac: bool,

    // ICP algorithm tuning parameters
    pub icp_good_fit_threshold: f64,
    pub icp_outlier_threshold: f64,
    pub icp_damping_factor: f64,
    pub icp_min_inlier_points: usize,

    // Voxel downsampling configuration
    /// Enable voxel grid downsampling before ICP
    #[serde(default)]
    pub voxel_downsample_enabled: bool,

    /// Voxel grid size in meters (e.g., 0.02 = 2cm voxels)
    #[serde(default = "default_voxel_size")]
    pub voxel_downsample_size: f64,

    /// Use centroid averaging (vs first-point strategy)
    /// true: Better edge preservation, slightly slower
    /// false: Faster, may lose some edge detail
    #[serde(default = "default_true")]
    pub voxel_downsample_use_centroid: bool,

    /// Threshold to enable parallel downsampling (point count)
    /// Only used if 'parallel' feature is enabled
    #[serde(default = "default_parallel_threshold")]
    pub voxel_parallel_threshold: usize,

    #[serde(flatten)]
    pub board_shape: BoardShape,
}
