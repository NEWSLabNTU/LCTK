use hollow_board_config::BoardModel;
use nalgebra as na;
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
