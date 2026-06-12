use hollow_board_config::BoardShape;
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Axis that points "up" in the sensor's coordinate frame.
/// Used to resolve the board's in-plane rotation during initial pose estimation.
/// Default is Z (standard ROS convention). Set to X for sensors like Seyond
/// where X is up and Z is forward.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum SensorUpAxis {
    #[default]
    Z,
    X,
    Y,
}

impl SensorUpAxis {
    pub fn as_vector(&self) -> na::Vector3<f64> {
        match self {
            SensorUpAxis::X => na::Vector3::x(),
            SensorUpAxis::Y => na::Vector3::y(),
            SensorUpAxis::Z => na::Vector3::z(),
        }
    }
}

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

    /// Axis that points "up" in the sensor frame (default: z).
    /// Controls how the board's in-plane rotation is estimated from the plane normal.
    /// Set to "x" for Seyond LiDAR (X=up, Z=forward).
    #[serde(default)]
    pub sensor_up_axis: SensorUpAxis,

    /// Extra rotation around the board normal applied to the initial pose (degrees, default: 0).
    /// Use this to correct a fixed in-plane rotational offset visible in RViz.
    /// Positive = counter-clockwise when viewed from sensor. Try ±45 or ±90.
    #[serde(default)]
    pub initial_inplane_rotation_deg: f64,

    #[serde(flatten)]
    pub board_shape: BoardShape,
}
