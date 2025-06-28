#![allow(clippy::field_reassign_with_default)]

use crate::types::BoardDetection;
use geometry_msgs::msg::{Point, Quaternion, Vector3};
use std_msgs::msg::{ColorRGBA, Header};
use visualization_msgs::msg::{Marker, MarkerArray};

/// Trait for generating board visualization markers
pub trait BoardMarkerGenerator: Send + Sync {
    fn generate_board_markers(
        &self,
        detection: &BoardDetection,
        lidar_id: u8,
        header: Header,
    ) -> MarkerArray;
}

/// Default implementation of BoardMarkerGenerator
pub struct DefaultBoardMarkerGenerator;

impl BoardMarkerGenerator for DefaultBoardMarkerGenerator {
    fn generate_board_markers(
        &self,
        detection: &BoardDetection,
        lidar_id: u8,
        header: Header,
    ) -> MarkerArray {
        let mut markers = MarkerArray::default();
        let base_id = (lidar_id as i32) * 1000;

        // Extract position and orientation from isometry
        let translation = detection.pose.translation;
        let rotation = detection.pose.rotation;

        let position = Point {
            x: translation.x,
            y: translation.y,
            z: translation.z,
        };

        let quaternion = Quaternion {
            x: rotation.i,
            y: rotation.j,
            z: rotation.k,
            w: rotation.w,
        };

        // Color based on lidar_id
        let color = if lidar_id == 1 {
            ColorRGBA {
                r: 1.0,
                g: 0.0,
                b: 0.0,
                a: 0.8,
            } // Red for LiDAR 1
        } else {
            ColorRGBA {
                r: 0.0,
                g: 0.0,
                b: 1.0,
                a: 0.8,
            } // Blue for LiDAR 2
        };

        // 1. Board outline marker (cube)
        let mut outline_marker = Marker::default();
        outline_marker.header = header.clone();
        outline_marker.ns = format!("board_detection_lidar_{}", lidar_id);
        outline_marker.id = base_id + 1;
        outline_marker.type_ = 1; // CUBE
        outline_marker.action = 0; // ADD
        outline_marker.pose.position = position.clone();
        outline_marker.pose.orientation = quaternion.clone();
        outline_marker.scale = Vector3 {
            x: 0.5, // Board size - should come from config
            y: 0.5,
            z: 0.02,
        };
        outline_marker.color = color.clone();
        outline_marker.lifetime = builtin_interfaces::msg::Duration {
            sec: 0,
            nanosec: 500_000_000,
        };
        markers.markers.push(outline_marker);

        // 2. Coordinate frame marker (arrows)
        // X-axis (red arrow)
        let mut x_arrow = Marker::default();
        x_arrow.header = header.clone();
        x_arrow.ns = format!("board_frame_lidar_{}", lidar_id);
        x_arrow.id = base_id + 2;
        x_arrow.type_ = 0; // ARROW
        x_arrow.action = 0; // ADD
        x_arrow.pose.position = position.clone();
        x_arrow.pose.orientation = quaternion.clone();
        x_arrow.scale = Vector3 {
            x: 0.3,
            y: 0.02,
            z: 0.02,
        };
        x_arrow.color = ColorRGBA {
            r: 1.0,
            g: 0.0,
            b: 0.0,
            a: 1.0,
        };
        x_arrow.lifetime = builtin_interfaces::msg::Duration {
            sec: 0,
            nanosec: 500_000_000,
        };
        markers.markers.push(x_arrow);

        // 3. Detection info text
        let mut text_marker = Marker::default();
        text_marker.header = header.clone();
        text_marker.ns = format!("board_info_lidar_{}", lidar_id);
        text_marker.id = base_id + 3;
        text_marker.type_ = 9; // TEXT_VIEW_FACING
        text_marker.action = 0; // ADD
        text_marker.pose.position = Point {
            x: position.x,
            y: position.y,
            z: position.z + 0.3, // Offset above board
        };
        text_marker.pose.orientation = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        text_marker.scale = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.1, // Text size
        };
        text_marker.color = ColorRGBA {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        };
        text_marker.text = format!(
            "LiDAR {} Board\nConf: {:.2}\nInliers: {}",
            lidar_id, detection.confidence, detection.inlier_count
        );
        text_marker.lifetime = builtin_interfaces::msg::Duration {
            sec: 0,
            nanosec: 500_000_000,
        };
        markers.markers.push(text_marker);

        markers
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;
    use std::time::SystemTime;

    #[test]
    fn test_generate_board_markers() {
        let generator = DefaultBoardMarkerGenerator;

        let detection = BoardDetection {
            pose: Isometry3::identity(),
            confidence: 0.85,
            inlier_count: 150,
            timestamp: SystemTime::now(),
        };

        let header = Header::default();
        let markers = generator.generate_board_markers(&detection, 1, header);

        assert_eq!(markers.markers.len(), 3);

        // Check marker types
        assert_eq!(markers.markers[0].type_, 1); // CUBE
        assert_eq!(markers.markers[1].type_, 0); // ARROW
        assert_eq!(markers.markers[2].type_, 9); // TEXT_VIEW_FACING

        // Check namespaces
        assert!(markers.markers[0].ns.contains("board_detection_lidar_1"));
        assert!(markers.markers[1].ns.contains("board_frame_lidar_1"));
        assert!(markers.markers[2].ns.contains("board_info_lidar_1"));

        // Check text content
        assert!(markers.markers[2].text.contains("LiDAR 1 Board"));
        assert!(markers.markers[2].text.contains("Conf: 0.85"));
        assert!(markers.markers[2].text.contains("Inliers: 150"));
    }

    #[test]
    fn test_lidar_id_color_mapping() {
        let generator = DefaultBoardMarkerGenerator;

        let detection = BoardDetection {
            pose: Isometry3::identity(),
            confidence: 0.5,
            inlier_count: 100,
            timestamp: SystemTime::now(),
        };

        let header = Header::default();

        // Test LiDAR 1 (should be red)
        let markers1 = generator.generate_board_markers(&detection, 1, header.clone());
        let color1 = &markers1.markers[0].color;
        assert_eq!(color1.r, 1.0);
        assert_eq!(color1.g, 0.0);
        assert_eq!(color1.b, 0.0);

        // Test LiDAR 2 (should be blue)
        let markers2 = generator.generate_board_markers(&detection, 2, header);
        let color2 = &markers2.markers[0].color;
        assert_eq!(color2.r, 0.0);
        assert_eq!(color2.g, 0.0);
        assert_eq!(color2.b, 1.0);
    }
}
