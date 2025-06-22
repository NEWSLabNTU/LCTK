use crate::types::RoiBounds;
use geometry_msgs::msg::{Point, Vector3};
use std_msgs::msg::{ColorRGBA, Header};
use visualization_msgs::msg::{Marker, MarkerArray};

/// Trait for generating ROI visualization markers
pub trait RoiMarkerGenerator: Send + Sync {
    fn generate_roi_marker(&self, bounds: &RoiBounds, lidar_id: u8, header: Header) -> MarkerArray;
}

/// Default implementation of RoiMarkerGenerator
pub struct DefaultRoiMarkerGenerator;

impl RoiMarkerGenerator for DefaultRoiMarkerGenerator {
    fn generate_roi_marker(&self, bounds: &RoiBounds, lidar_id: u8, header: Header) -> MarkerArray {
        let mut markers = MarkerArray::default();

        // Calculate center and size
        let center_x = (bounds.min_x + bounds.max_x) / 2.0;
        let center_y = (bounds.min_y + bounds.max_y) / 2.0;
        let center_z = (bounds.min_z + bounds.max_z) / 2.0;

        let size_x = bounds.max_x - bounds.min_x;
        let size_y = bounds.max_y - bounds.min_y;
        let size_z = bounds.max_z - bounds.min_z;

        // Color based on lidar_id
        let color = if lidar_id == 1 {
            ColorRGBA {
                r: 1.0,
                g: 0.0,
                b: 0.0,
                a: 0.3,
            } // Semi-transparent red for LiDAR 1
        } else {
            ColorRGBA {
                r: 0.0,
                g: 0.0,
                b: 1.0,
                a: 0.3,
            } // Semi-transparent blue for LiDAR 2
        };

        // ROI box marker
        let mut roi_marker = Marker::default();
        roi_marker.header = header.clone();
        roi_marker.ns = format!("roi_bounds_lidar_{}", lidar_id);
        roi_marker.id = lidar_id as i32;
        roi_marker.type_ = 1; // CUBE
        roi_marker.action = 0; // ADD

        roi_marker.pose.position = Point {
            x: center_x,
            y: center_y,
            z: center_z,
        };

        roi_marker.pose.orientation.w = 1.0; // No rotation

        roi_marker.scale = Vector3 {
            x: size_x,
            y: size_y,
            z: size_z,
        };

        roi_marker.color = color.clone();
        roi_marker.lifetime = builtin_interfaces::msg::Duration { sec: 0, nanosec: 0 }; // Persistent

        markers.markers.push(roi_marker);

        // Add text label
        let mut text_marker = Marker::default();
        text_marker.header = header;
        text_marker.ns = format!("roi_label_lidar_{}", lidar_id);
        text_marker.id = (lidar_id as i32) + 100;
        text_marker.type_ = 9; // TEXT_VIEW_FACING
        text_marker.action = 0; // ADD

        text_marker.pose.position = Point {
            x: center_x,
            y: center_y,
            z: center_z + size_z / 2.0 + 0.2, // Above the box
        };

        text_marker.pose.orientation.w = 1.0;

        text_marker.scale = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.15, // Text size
        };

        text_marker.color = ColorRGBA {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        };

        text_marker.text = format!(
            "LiDAR {} ROI\n{:.1}×{:.1}×{:.1}m",
            lidar_id, size_x, size_y, size_z
        );

        text_marker.lifetime = builtin_interfaces::msg::Duration { sec: 0, nanosec: 0 }; // Persistent

        markers.markers.push(text_marker);

        markers
    }
}

/// Generates ROI markers for multiple LiDARs
pub fn generate_all_roi_markers(
    generator: &dyn RoiMarkerGenerator,
    all_bounds: &std::collections::HashMap<u8, RoiBounds>,
    header: Header,
) -> MarkerArray {
    let mut all_markers = MarkerArray::default();

    for (&lidar_id, bounds) in all_bounds {
        let roi_markers = generator.generate_roi_marker(bounds, lidar_id, header.clone());
        all_markers.markers.extend(roi_markers.markers);
    }

    all_markers
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn test_generate_roi_marker() {
        let generator = DefaultRoiMarkerGenerator;

        let bounds = RoiBounds {
            min_x: -2.0,
            max_x: 2.0,
            min_y: -2.0,
            max_y: 2.0,
            min_z: -1.0,
            max_z: 1.0,
        };

        let header = Header::default();
        let markers = generator.generate_roi_marker(&bounds, 1, header);

        assert_eq!(markers.markers.len(), 2);

        // Check box marker
        let box_marker = &markers.markers[0];
        assert_eq!(box_marker.type_, 1); // CUBE
        assert_eq!(box_marker.pose.position.x, 0.0); // Center
        assert_eq!(box_marker.pose.position.y, 0.0);
        assert_eq!(box_marker.pose.position.z, 0.0);
        assert_eq!(box_marker.scale.x, 4.0); // Size
        assert_eq!(box_marker.scale.y, 4.0);
        assert_eq!(box_marker.scale.z, 2.0);

        // Check text marker
        let text_marker = &markers.markers[1];
        assert_eq!(text_marker.type_, 9); // TEXT_VIEW_FACING
        assert!(text_marker.text.contains("LiDAR 1 ROI"));
        assert!(text_marker.text.contains("4.0×4.0×2.0m"));
    }

    #[test]
    fn test_lidar_color_mapping() {
        let generator = DefaultRoiMarkerGenerator;

        let bounds = RoiBounds {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: -1.0,
            max_z: 1.0,
        };

        let header = Header::default();

        // Test LiDAR 1 (should be red)
        let markers1 = generator.generate_roi_marker(&bounds, 1, header.clone());
        let color1 = &markers1.markers[0].color;
        assert_eq!(color1.r, 1.0);
        assert_eq!(color1.g, 0.0);
        assert_eq!(color1.b, 0.0);

        // Test LiDAR 2 (should be blue)
        let markers2 = generator.generate_roi_marker(&bounds, 2, header);
        let color2 = &markers2.markers[0].color;
        assert_eq!(color2.r, 0.0);
        assert_eq!(color2.g, 0.0);
        assert_eq!(color2.b, 1.0);
    }

    #[test]
    fn test_generate_all_roi_markers() {
        let generator = DefaultRoiMarkerGenerator;

        let mut all_bounds = HashMap::new();
        all_bounds.insert(
            1,
            RoiBounds {
                min_x: -1.0,
                max_x: 1.0,
                min_y: -1.0,
                max_y: 1.0,
                min_z: -1.0,
                max_z: 1.0,
            },
        );
        all_bounds.insert(
            2,
            RoiBounds {
                min_x: -2.0,
                max_x: 2.0,
                min_y: -2.0,
                max_y: 2.0,
                min_z: -2.0,
                max_z: 2.0,
            },
        );

        let header = Header::default();
        let all_markers = generate_all_roi_markers(&generator, &all_bounds, header);

        assert_eq!(all_markers.markers.len(), 4); // 2 markers × 2 LiDARs
    }
}
