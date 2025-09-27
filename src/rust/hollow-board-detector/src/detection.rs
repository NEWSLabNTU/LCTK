use hollow_board_config::{BoardModel, BoardShape};
use measurements::Length;
use nalgebra::{self as na, Isometry3, Point3};
use plane_estimator::PlaneModel;
use std::f64;

#[derive(Debug, Clone)]
pub struct Detection {
    pub board_model: BoardModel,
    pub plane_ransac_data: PlaneRansacData,
    pub icp_data: IcpData,
    pub icp_losses: Vec<f64>,
    pub initial_pose: na::Isometry3<f64>,
    pub icp_stats: IcpStatistics,
}

#[derive(Debug, Clone)]
pub struct IcpData {
    pub correspondences: Vec<(na::Point3<f64>, na::Point3<f64>)>, // (data_point, model_point)
    pub board_model: BoardModel,
}

#[derive(Debug, Clone)]
pub struct PlaneRansacData {
    pub plane_model: PlaneModel,
    pub inlier_points: Vec<na::Point3<f64>>,
}

#[derive(Debug, Clone)]
pub struct FitBoardIcp {
    pub board_pose: na::Isometry3<f64>,
    pub icp_losses: Vec<f64>,
    pub icp_data: IcpData,
    pub successful: bool,
    pub initial_pose: na::Isometry3<f64>,
    pub icp_stats: IcpStatistics,
}

#[derive(Debug, Clone)]
pub struct FitPlaneRansac<'a> {
    pub plane_model: PlaneModel,
    pub inlier_points: Vec<&'a na::Point3<f64>>,
    pub ransac_data: PlaneRansacData,
}

#[derive(Debug, Clone)]
pub struct IcpStatistics {
    pub iterations: usize,
    pub final_loss: f64,
    pub min_loss: f64,
    pub successful: bool,
    pub initial_loss: f64,
    pub convergence_reason: String,
}

/// Represents the complete state of board ICP at a given iteration
#[derive(Clone, Debug)]
pub struct BoardIcpState {
    /// Current iteration number (starts at 0)
    pub iteration: usize,

    /// Current board pose estimate
    pub board_pose: Isometry3<f64>,

    /// Current inlier points from point cloud
    pub inlier_points: Vec<Point3<f64>>,

    /// Correspondences: (point_cloud_point, board_model_point)
    pub correspondences: Vec<(Point3<f64>, Point3<f64>)>,

    /// Average loss for this iteration
    pub avg_loss: f64,

    /// Previous iteration's loss (None for iteration 0)
    pub previous_loss: Option<f64>,

    /// Adaptive threshold used for outlier filtering
    pub adaptive_threshold: f64,

    /// Number of correspondences before outlier filtering
    pub total_correspondences: usize,

    /// Number of good correspondences after filtering
    pub good_correspondences: usize,

    /// Convergence metadata
    pub termination_count: usize,
}

/// Parameters for board model construction
#[derive(Clone, Debug)]
pub struct BoardModelParams {
    pub board_shape: BoardShape,
    pub marker_paper_size: Length,
}
