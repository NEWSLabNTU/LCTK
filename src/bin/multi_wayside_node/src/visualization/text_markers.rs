use geometry_msgs::msg::{Point, Vector3};
use std_msgs::msg::{ColorRGBA, Header};
use visualization_msgs::msg::{Marker, MarkerArray};

/// Trait for generating text visualization markers
pub trait TextMarkerGenerator: Send + Sync {
    fn generate_status_text(&self, text: &str, position: Point, header: Header) -> Marker;
    fn generate_detection_status(
        &self,
        lidar1_detected: bool,
        lidar2_detected: bool,
        sync_status: &str,
        header: Header,
    ) -> MarkerArray;
}

/// Default implementation of TextMarkerGenerator
pub struct DefaultTextMarkerGenerator;

impl TextMarkerGenerator for DefaultTextMarkerGenerator {
    fn generate_status_text(&self, text: &str, position: Point, header: Header) -> Marker {
        let mut marker = Marker::default();
        marker.header = header;
        marker.ns = "status_text".to_string();
        marker.id = 0;
        marker.type_ = 9; // TEXT_VIEW_FACING
        marker.action = 0; // ADD
        marker.pose.position = position;
        marker.pose.orientation.w = 1.0;
        marker.scale = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.2, // Text size
        };
        marker.color = ColorRGBA {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        };
        marker.text = text.to_string();
        marker.lifetime = builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 };
        marker
    }

    fn generate_detection_status(
        &self,
        lidar1_detected: bool,
        lidar2_detected: bool,
        sync_status: &str,
        header: Header,
    ) -> MarkerArray {
        let mut markers = MarkerArray::default();

        // Status summary text
        let status_text = format!(
            "Multi-Wayside Detection Status\n\nLiDAR 1: {}\nLiDAR 2: {}\nSync: {}",
            if lidar1_detected {
                "✓ DETECTED"
            } else {
                "✗ NO DETECTION"
            },
            if lidar2_detected {
                "✓ DETECTED"
            } else {
                "✗ NO DETECTION"
            },
            sync_status
        );

        let mut status_marker = Marker::default();
        status_marker.header = header.clone();
        status_marker.ns = "detection_status".to_string();
        status_marker.id = 1;
        status_marker.type_ = 9; // TEXT_VIEW_FACING
        status_marker.action = 0; // ADD
        status_marker.pose.position = Point {
            x: 0.0,
            y: 0.0,
            z: 3.0, // High above origin
        };
        status_marker.pose.orientation.w = 1.0;
        status_marker.scale = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.15,
        };

        // Color based on overall status
        status_marker.color = if lidar1_detected && lidar2_detected {
            ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            } // Green for good
        } else if lidar1_detected || lidar2_detected {
            ColorRGBA {
                r: 1.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            } // Yellow for partial
        } else {
            ColorRGBA {
                r: 1.0,
                g: 0.0,
                b: 0.0,
                a: 1.0,
            } // Red for none
        };

        status_marker.text = status_text;
        status_marker.lifetime = builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 };

        markers.markers.push(status_marker);

        // Individual LiDAR status indicators
        for lidar_id in 1..=2 {
            let detected = if lidar_id == 1 {
                lidar1_detected
            } else {
                lidar2_detected
            };

            let mut indicator = Marker::default();
            indicator.header = header.clone();
            indicator.ns = format!("lidar_{}_status", lidar_id);
            indicator.id = lidar_id as i32;
            indicator.type_ = 2; // SPHERE
            indicator.action = 0; // ADD
            indicator.pose.position = Point {
                x: if lidar_id == 1 { -1.0 } else { 1.0 },
                y: 0.0,
                z: 2.5,
            };
            indicator.pose.orientation.w = 1.0;
            indicator.scale = Vector3 {
                x: 0.3,
                y: 0.3,
                z: 0.3,
            };

            indicator.color = if detected {
                ColorRGBA {
                    r: 0.0,
                    g: 1.0,
                    b: 0.0,
                    a: 0.8,
                }
            } else {
                ColorRGBA {
                    r: 1.0,
                    g: 0.0,
                    b: 0.0,
                    a: 0.8,
                }
            };

            indicator.lifetime = builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 };
            markers.markers.push(indicator);
        }

        markers
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_status_text() {
        let generator = DefaultTextMarkerGenerator;
        let position = Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let header = Header::default();

        let marker = generator.generate_status_text("Test Status", position, header);

        assert_eq!(marker.type_, 9); // TEXT_VIEW_FACING
        assert_eq!(marker.text, "Test Status");
        assert_eq!(marker.pose.position.x, 1.0);
        assert_eq!(marker.pose.position.y, 2.0);
        assert_eq!(marker.pose.position.z, 3.0);
    }

    #[test]
    fn test_generate_detection_status_all_detected() {
        let generator = DefaultTextMarkerGenerator;
        let header = Header::default();

        let markers = generator.generate_detection_status(true, true, "SYNCHRONIZED", header);

        assert_eq!(markers.markers.len(), 3); // Status text + 2 indicators

        // Check status text
        let status_marker = &markers.markers[0];
        assert_eq!(status_marker.type_, 9); // TEXT_VIEW_FACING
        assert!(status_marker.text.contains("✓ DETECTED"));
        assert!(status_marker.text.contains("SYNCHRONIZED"));
        assert_eq!(status_marker.color.g, 1.0); // Green

        // Check indicators
        for i in 1..=2 {
            let indicator = &markers.markers[i];
            assert_eq!(indicator.type_, 2); // SPHERE
            assert_eq!(indicator.color.g, 1.0); // Green
            assert_eq!(indicator.color.r, 0.0);
        }
    }

    #[test]
    fn test_generate_detection_status_none_detected() {
        let generator = DefaultTextMarkerGenerator;
        let header = Header::default();

        let markers = generator.generate_detection_status(false, false, "NO SYNC", header);

        // Check status text color (should be red)
        let status_marker = &markers.markers[0];
        assert_eq!(status_marker.color.r, 1.0); // Red
        assert_eq!(status_marker.color.g, 0.0);
        assert!(status_marker.text.contains("✗ NO DETECTION"));

        // Check indicators (should be red)
        for i in 1..=2 {
            let indicator = &markers.markers[i];
            assert_eq!(indicator.color.r, 1.0); // Red
            assert_eq!(indicator.color.g, 0.0);
        }
    }

    #[test]
    fn test_generate_detection_status_partial() {
        let generator = DefaultTextMarkerGenerator;
        let header = Header::default();

        let markers = generator.generate_detection_status(true, false, "WAITING", header);

        // Check status text color (should be yellow)
        let status_marker = &markers.markers[0];
        assert_eq!(status_marker.color.r, 1.0); // Yellow
        assert_eq!(status_marker.color.g, 1.0);
        assert_eq!(status_marker.color.b, 0.0);

        // Check individual indicators
        let indicator1 = &markers.markers[1]; // LiDAR 1
        assert_eq!(indicator1.color.g, 1.0); // Green

        let indicator2 = &markers.markers[2]; // LiDAR 2
        assert_eq!(indicator2.color.r, 1.0); // Red
        assert_eq!(indicator2.color.g, 0.0);
    }
}
