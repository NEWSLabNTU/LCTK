use crate::config::Config;
use crate::detection::{IcpData, PlaneRansacData};
use hollow_board_config::BoardModel;
use nalgebra as na;
use std::sync::Arc;

/// Debug visualization data for ICP iterations
#[derive(Debug, Clone)]
pub struct DebugVisualizationData {
    /// Current ICP iteration step
    pub iteration: usize,
    /// Current board model pose
    pub board_model: BoardModel,
    /// Input point cloud (inlier points)
    pub inlier_points: Vec<na::Point3<f64>>,
    /// Corresponding model points
    pub corresponding_points: Vec<na::Point3<f64>>,
    /// Current ICP loss
    pub current_loss: f64,
    /// Pose weight (translation + rotation)
    pub pose_weight: f64,
    /// Plane RANSAC data
    pub plane_ransac_data: PlaneRansacData,
}

/// Trait for publishing debug visualization data
pub trait DebugVisualizationPublisher: Send + Sync {
    /// Publish debug visualization data for an ICP iteration
    fn publish_icp_debug_data(&self, data: &DebugVisualizationData) -> anyhow::Result<()>;
    
    /// Publish board model visualization markers
    fn publish_board_model_markers(&self, board_model: &BoardModel, iteration: usize) -> anyhow::Result<()>;
    
    /// Publish point cloud visualization
    fn publish_point_cloud_debug(&self, points: &[na::Point3<f64>], topic_suffix: &str) -> anyhow::Result<()>;
}

/// No-op implementation for when debug visualization is disabled
pub struct NoOpDebugPublisher;

impl DebugVisualizationPublisher for NoOpDebugPublisher {
    fn publish_icp_debug_data(&self, _data: &DebugVisualizationData) -> anyhow::Result<()> {
        Ok(())
    }
    
    fn publish_board_model_markers(&self, _board_model: &BoardModel, _iteration: usize) -> anyhow::Result<()> {
        Ok(())
    }
    
    fn publish_point_cloud_debug(&self, _points: &[na::Point3<f64>], _topic_suffix: &str) -> anyhow::Result<()> {
        Ok(())
    }
}

/// Create a debug visualization publisher based on configuration
pub fn create_debug_publisher(config: &Config) -> Arc<dyn DebugVisualizationPublisher> {
    if config.enable_debug_visualization {
        // Create ROS2 debug publisher with appropriate topic names
        use crate::ros2_debug_publisher::ROS2DebugPublisher;
        Arc::new(ROS2DebugPublisher::new(
            "/debug/board_model_markers",
            "/debug/input_point_cloud",
            "/debug/corresponding_points",
            "/debug/icp_iteration_data",
        ))
    } else {
        Arc::new(NoOpDebugPublisher)
    }
}

/// Convert board model to visualization markers
pub fn board_model_to_markers(board_model: &BoardModel, iteration: usize) -> Vec<DebugMarker> {
    let mut markers = Vec::new();
    
    // Board outline (rectangle)
    let board_corners = vec![
        board_model.bottom_corner(),
        board_model.left_corner(),
        board_model.top_corner(),
        board_model.right_corner(),
        board_model.bottom_corner(), // Close the rectangle
    ];
    
    markers.push(DebugMarker::LineStrip {
        points: board_corners,
        color: [1.0, 1.0, 0.0, 0.8], // Yellow
        line_width: 0.02,
        id: format!("board_outline_{}", iteration),
    });
    
    // Hole circles
    let hole_centers = vec![
        board_model.left_circle_center(),
        board_model.right_circle_center(),
        board_model.top_circle_center(),
    ];
    
    for (i, center) in hole_centers.iter().enumerate() {
        let circle_points = generate_circle_points(*center, board_model.board_shape.hole_radius.as_meters(), 32);
        markers.push(DebugMarker::LineStrip {
            points: circle_points,
            color: [1.0, 0.0, 1.0, 0.8], // Magenta
            line_width: 0.02,
            id: format!("hole_{}_{}", i, iteration),
        });
    }
    
    // Coordinate frame
    let center = board_model.board_center();
    let x_axis = board_model.board_x_axis();
    let y_axis = board_model.board_y_axis();
    let z_axis = board_model.board_z_axis();
    
    let frame_length = 0.2; // 20cm frame arrows
    
    markers.push(DebugMarker::Arrow {
        start: center,
        end: center + x_axis.scale(frame_length),
        color: [1.0, 0.0, 0.0, 1.0], // Red for X
        id: format!("frame_x_{}", iteration),
    });
    
    markers.push(DebugMarker::Arrow {
        start: center,
        end: center + y_axis.scale(frame_length),
        color: [0.0, 1.0, 0.0, 1.0], // Green for Y
        id: format!("frame_y_{}", iteration),
    });
    
    markers.push(DebugMarker::Arrow {
        start: center,
        end: center + z_axis.scale(frame_length),
        color: [0.0, 0.0, 1.0, 1.0], // Blue for Z
        id: format!("frame_z_{}", iteration),
    });
    
    markers
}

/// Generate points for a circle
fn generate_circle_points(center: na::Point3<f64>, radius: f64, num_points: usize) -> Vec<na::Point3<f64>> {
    let mut points = Vec::with_capacity(num_points + 1);
    
    for i in 0..=num_points {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / num_points as f64;
        let x = center.x + radius * angle.cos();
        let y = center.y + radius * angle.sin();
        points.push(na::Point3::new(x, y, center.z));
    }
    
    points
}

/// Debug marker types for visualization
#[derive(Debug, Clone)]
pub enum DebugMarker {
    LineStrip {
        points: Vec<na::Point3<f64>>,
        color: [f32; 4], // RGBA
        line_width: f64,
        id: String,
    },
    Arrow {
        start: na::Point3<f64>,
        end: na::Point3<f64>,
        color: [f32; 4], // RGBA
        id: String,
    },
    Points {
        points: Vec<na::Point3<f64>>,
        color: [f32; 4], // RGBA
        point_size: f64,
        id: String,
    },
}

/// Convert debug markers to ROS2 visualization_msgs::MarkerArray
/// This would be implemented when integrating with ROS2
pub fn markers_to_ros2_marker_array(_markers: &[DebugMarker]) -> Vec<u8> {
    // TODO: Implement conversion to ROS2 MarkerArray message
    // This would serialize the markers to the appropriate ROS2 message format
    vec![]
}
