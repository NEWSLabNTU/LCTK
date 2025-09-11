use crate::debug_visualization::{DebugVisualizationData, DebugVisualizationPublisher, DebugMarker, board_model_to_markers};
use hollow_board_config::BoardModel;
use nalgebra as na;
use std::sync::Arc;

// ROS2 dependencies
use rclrs::*;
use visualization_msgs::msg::{MarkerArray, Marker};
use sensor_msgs::msg::{PointCloud2, PointField};
use std_msgs::msg::Header;
use builtin_interfaces::msg::Time;

/// ROS2 implementation of debug visualization publisher
/// This is a comprehensive logging implementation that will be used until we can integrate
/// with the main calibration_board_locator node's publishers
pub struct ROS2DebugPublisher {
    // Topic names for reference
    board_markers_topic: String,
    input_points_topic: String,
    corresponding_points_topic: String,
}

impl ROS2DebugPublisher {
    pub fn new(
        board_markers_topic: &str,
        input_points_topic: &str,
        corresponding_points_topic: &str,
        _icp_data_topic: &str,
    ) -> Self {
        println!("Creating ROS2DebugPublisher with topics:");
        println!("  - Board markers: {}", board_markers_topic);
        println!("  - Input points: {}", input_points_topic);
        println!("  - Corresponding points: {}", corresponding_points_topic);
        
        Self {
            board_markers_topic: board_markers_topic.to_string(),
            input_points_topic: input_points_topic.to_string(),
            corresponding_points_topic: corresponding_points_topic.to_string(),
        }
    }
}

impl DebugVisualizationPublisher for ROS2DebugPublisher {
    fn publish_icp_debug_data(&self, data: &DebugVisualizationData) -> anyhow::Result<()> {
        // For now, just log the debug data
        // In a full implementation, this would publish to a custom message topic
        println!("Publishing ICP debug data for iteration {}: loss={:.6}, pose_weight={:.6}", 
                 data.iteration, data.current_loss, data.pose_weight);
        
        Ok(())
    }
    
    fn publish_board_model_markers(&self, board_model: &BoardModel, iteration: usize) -> anyhow::Result<()> {
        // Convert board model to visualization markers
        let markers = board_model_to_markers(board_model, iteration);
        
        println!("=== DEBUG VISUALIZATION: Board Model Markers ===");
        println!("Iteration: {}", iteration);
        println!("Topic: {}", self.board_markers_topic);
        println!("Board Model Pose:");
        println!("  Position: ({:.3}, {:.3}, {:.3})", 
                 board_model.pose.translation.x, 
                 board_model.pose.translation.y, 
                 board_model.pose.translation.z);
        println!("  Rotation: ({:.3}, {:.3}, {:.3}, {:.3})", 
                 board_model.pose.rotation.coords.x, 
                 board_model.pose.rotation.coords.y, 
                 board_model.pose.rotation.coords.z, 
                 board_model.pose.rotation.coords.w);
        println!("Board Shape: width={:?}, hole_radius={:?}, hole_center_shift={:?}", 
                 board_model.board_shape.board_width, 
                 board_model.board_shape.hole_radius, 
                 board_model.board_shape.hole_center_shift);
        println!("Number of markers: {}", markers.len());
        
        for (i, marker) in markers.iter().enumerate() {
            match marker {
                DebugMarker::LineStrip { points, color, line_width, id } => {
                    println!("  Marker {}: LineStrip '{}' - {} points, width={:.3}, color=({:.2}, {:.2}, {:.2}, {:.2})", 
                             i, id, points.len(), line_width, color[0], color[1], color[2], color[3]);
                },
                DebugMarker::Arrow { start, end, color, id } => {
                    println!("  Marker {}: Arrow '{}' - from ({:.3}, {:.3}, {:.3}) to ({:.3}, {:.3}, {:.3}), color=({:.2}, {:.2}, {:.2}, {:.2})", 
                             i, id, start.x, start.y, start.z, end.x, end.y, end.z, color[0], color[1], color[2], color[3]);
                },
                DebugMarker::Points { points, color, point_size, id } => {
                    println!("  Marker {}: Points '{}' - {} points, size={:.3}, color=({:.2}, {:.2}, {:.2}, {:.2})", 
                             i, id, points.len(), point_size, color[0], color[1], color[2], color[3]);
                }
            }
        }
        println!("=== END DEBUG VISUALIZATION ===");
        
        Ok(())
    }
    
    fn publish_point_cloud_debug(&self, points: &[na::Point3<f64>], topic_suffix: &str) -> anyhow::Result<()> {
        let topic_name = match topic_suffix {
            "input_points" => &self.input_points_topic,
            "corresponding_points" => &self.corresponding_points_topic,
            _ => return Err(anyhow::anyhow!("Unknown topic suffix: {}", topic_suffix)),
        };
        
        println!("=== DEBUG VISUALIZATION: Point Cloud ===");
        println!("Topic: {}", topic_name);
        println!("Suffix: {}", topic_suffix);
        println!("Number of points: {}", points.len());
        
        if !points.is_empty() {
            // Calculate bounding box
            let mut min_x = points[0].x;
            let mut max_x = points[0].x;
            let mut min_y = points[0].y;
            let mut max_y = points[0].y;
            let mut min_z = points[0].z;
            let mut max_z = points[0].z;
            
            for point in points {
                min_x = min_x.min(point.x);
                max_x = max_x.max(point.x);
                min_y = min_y.min(point.y);
                max_y = max_y.max(point.y);
                min_z = min_z.min(point.z);
                max_z = max_z.max(point.z);
            }
            
            println!("Bounding box:");
            println!("  X: [{:.3}, {:.3}] (range: {:.3})", min_x, max_x, max_x - min_x);
            println!("  Y: [{:.3}, {:.3}] (range: {:.3})", min_y, max_y, max_y - min_y);
            println!("  Z: [{:.3}, {:.3}] (range: {:.3})", min_z, max_z, max_z - min_z);
            
            // Show first few points
            let num_to_show = points.len().min(5);
            println!("First {} points:", num_to_show);
            for (i, point) in points.iter().take(num_to_show).enumerate() {
                println!("  Point {}: ({:.3}, {:.3}, {:.3})", i, point.x, point.y, point.z);
            }
            if points.len() > num_to_show {
                println!("  ... and {} more points", points.len() - num_to_show);
            }
        }
        println!("=== END DEBUG VISUALIZATION ===");
        
        Ok(())
    }
}

/// Convert debug markers to ROS2 MarkerArray
pub fn convert_debug_markers_to_ros2(markers: Vec<DebugMarker>, iteration: usize) -> MarkerArray {
    let mut ros2_markers = Vec::new();
    
    for (i, marker) in markers.iter().enumerate() {
        let mut ros2_marker = Marker::default();
        ros2_marker.header = Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: "velodyne".to_string(),
        };
        ros2_marker.ns = "board_model".to_string();
        ros2_marker.id = (iteration * 1000 + i) as i32;
        ros2_marker.action = 0; // ADD
        
        match marker {
            DebugMarker::LineStrip { points, color, line_width: _, id: _ } => {
                ros2_marker.type_ = 4; // LINE_STRIP
                ros2_marker.scale.x = 0.01; // Line width
                ros2_marker.color.r = color[0];
                ros2_marker.color.g = color[1];
                ros2_marker.color.b = color[2];
                ros2_marker.color.a = color[3];
                // TODO: Add points to marker
            },
            DebugMarker::Arrow { start, end, color, id: _ } => {
                ros2_marker.type_ = 0; // ARROW
                ros2_marker.pose.position.x = start.x;
                ros2_marker.pose.position.y = start.y;
                ros2_marker.pose.position.z = start.z;
                ros2_marker.scale.x = (end.x - start.x).abs();
                ros2_marker.scale.y = (end.y - start.y).abs();
                ros2_marker.scale.z = (end.z - start.z).abs();
                ros2_marker.color.r = color[0];
                ros2_marker.color.g = color[1];
                ros2_marker.color.b = color[2];
                ros2_marker.color.a = color[3];
            },
            DebugMarker::Points { points, color, point_size, id: _ } => {
                ros2_marker.type_ = 8; // POINTS
                ros2_marker.scale.x = *point_size;
                ros2_marker.color.r = color[0];
                ros2_marker.color.g = color[1];
                ros2_marker.color.b = color[2];
                ros2_marker.color.a = color[3];
                // TODO: Add points to marker
            }
        }
        
        ros2_markers.push(ros2_marker);
    }
    
    MarkerArray { markers: ros2_markers }
}

/// Convert debug markers to ROS2 MarkerArray message
/// This is a placeholder implementation - in reality this would use the ROS2 Rust bindings
pub fn convert_markers_to_ros2_marker_array(markers: &[DebugMarker]) -> Vec<u8> {
    // TODO: Implement actual conversion to visualization_msgs::MarkerArray
    // This would involve:
    // 1. Creating Marker messages for each DebugMarker
    // 2. Setting appropriate headers, namespaces, IDs, types, actions
    // 3. Converting colors, scales, poses, etc.
    // 4. Serializing to ROS2 message format
    
    println!("Converting {} markers to ROS2 MarkerArray", markers.len());
    
    // Placeholder serialization
    vec![]
}

/// Convert points to ROS2 PointCloud2
pub fn convert_points_to_ros2_pointcloud2(points: &[na::Point3<f64>]) -> PointCloud2 {
    // Create point fields for x, y, z coordinates
    let fields = vec![
        PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: 7, // FLOAT32
            count: 1,
        },
        PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: 7, // FLOAT32
            count: 1,
        },
        PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: 7, // FLOAT32
            count: 1,
        },
    ];
    
    // Convert points to bytes
    let mut data = Vec::new();
    for point in points {
        data.extend_from_slice(&(point.x as f32).to_le_bytes());
        data.extend_from_slice(&(point.y as f32).to_le_bytes());
        data.extend_from_slice(&(point.z as f32).to_le_bytes());
    }
    
    PointCloud2 {
        header: Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: "velodyne".to_string(),
        },
        height: 1,
        width: points.len() as u32,
        fields,
        is_bigendian: false,
        point_step: 12, // 3 floats * 4 bytes
        row_step: (points.len() * 12) as u32,
        data,
        is_dense: true,
    }
}

/// Convert points to ROS2 PointCloud2 message
/// This is a placeholder implementation - in reality this would use the ROS2 Rust bindings
pub fn convert_points_to_ros2_pointcloud2_bytes(points: &[na::Point3<f64>]) -> Vec<u8> {
    // TODO: Implement actual conversion to sensor_msgs::PointCloud2
    // This would involve:
    // 1. Creating PointField definitions for x, y, z coordinates
    // 2. Setting appropriate header with frame_id and timestamp
    // 3. Converting points to byte array with proper padding
    // 4. Setting width, height, point_step, row_step, is_dense
    // 5. Serializing to ROS2 message format
    
    println!("Converting {} points to ROS2 PointCloud2", points.len());
    
    // Placeholder serialization
    vec![]
}

/// Convert debug visualization data to custom ROS2 message
/// This is a placeholder implementation - in reality this would use a custom message type
pub fn convert_to_icp_debug_message(data: &DebugVisualizationData) -> Vec<u8> {
    // TODO: Implement actual conversion to custom ICP debug message
    // This would involve creating a custom ROS2 message type that includes:
    // - iteration number
    // - current loss
    // - pose weight
    // - board model pose
    // - number of inlier points
    // - number of corresponding points
    // - plane RANSAC data
    
    println!("Converting ICP debug data to custom message for iteration {}", data.iteration);
    
    // Placeholder serialization
    vec![]
}
