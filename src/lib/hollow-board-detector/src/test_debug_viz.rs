// Simple test to verify the debug visualization code compiles correctly
// This test doesn't require ROS2 dependencies

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;
    use crate::debug_visualization::{create_debug_publisher, NoOpDebugPublisher, DebugVisualizationData};
    use crate::detection::PlaneRansacData;
    use hollow_board_config::{BoardModel, BoardShape};
    use measurements::Length;
    use nalgebra as na;
    use plane_estimator::PlaneModel;

    fn create_test_debug_data() -> DebugVisualizationData {
        DebugVisualizationData {
            iteration: 0,
            board_model: BoardModel {
                pose: na::Isometry3::identity(),
                board_shape: BoardShape {
                    board_width: Length::from_meters(1.0),
                    hole_radius: Length::from_meters(0.15),
                    hole_center_shift: Length::from_meters(0.2),
                },
                marker_paper_size: Length::from_meters(0.5),
            },
            inlier_points: vec![],
            corresponding_points: vec![],
            current_loss: 0.0,
            pose_weight: 0.0,
            plane_ransac_data: PlaneRansacData {
                plane_model: PlaneModel::new(na::UnitVector3::new_normalize(na::Vector3::z_axis()), 0.0),
                inlier_points: vec![],
            },
        }
    }

    #[test]
    fn test_debug_publisher_creation() {
        // Test with debug disabled
        let config_disabled = Config {
            max_icp_iterations: 100,
            icp_pose_weight_threshold: 1e-6,
            icp_rejection_threshold: 0.1,
            plane_ransac_max_iterations: 500,
            plane_ransac_inlier_threshold: 0.05,
            enable_debug_visualization: false,
            board_shape: BoardShape {
                board_width: Length::from_meters(1.0),
                hole_radius: Length::from_meters(0.15),
                hole_center_shift: Length::from_meters(0.2),
            },
        };

        let publisher_disabled = create_debug_publisher(&config_disabled);
        // Test that we can call methods on the publisher
        assert!(publisher_disabled.publish_icp_debug_data(&create_test_debug_data()).is_ok());

        // Test with debug enabled
        let config_enabled = Config {
            max_icp_iterations: 100,
            icp_pose_weight_threshold: 1e-6,
            icp_rejection_threshold: 0.1,
            plane_ransac_max_iterations: 500,
            plane_ransac_inlier_threshold: 0.05,
            enable_debug_visualization: true,
            board_shape: BoardShape {
                board_width: Length::from_meters(1.0),
                hole_radius: Length::from_meters(0.15),
                hole_center_shift: Length::from_meters(0.2),
            },
        };

        let publisher_enabled = create_debug_publisher(&config_enabled);
        // Test that we can call methods on the publisher
        assert!(publisher_enabled.publish_icp_debug_data(&create_test_debug_data()).is_ok());
    }

    #[test]
    fn test_board_model_markers() {
        use crate::debug_visualization::board_model_to_markers;

        let board_model = BoardModel {
            pose: na::Isometry3::identity(),
            board_shape: BoardShape {
                board_width: Length::from_meters(1.0),
                hole_radius: Length::from_meters(0.15),
                hole_center_shift: Length::from_meters(0.2),
            },
            marker_paper_size: Length::from_meters(0.5),
        };

        let markers = board_model_to_markers(&board_model, 0);
        
        // Should have at least the board outline and coordinate frame markers
        assert!(markers.len() >= 4); // board outline + 3 coordinate frame arrows
        
        // Check that we have the expected marker types
        let has_line_strip = markers.iter().any(|m| matches!(m, crate::debug_visualization::DebugMarker::LineStrip { .. }));
        let has_arrow = markers.iter().any(|m| matches!(m, crate::debug_visualization::DebugMarker::Arrow { .. }));
        
        assert!(has_line_strip, "Should have LineStrip markers for board outline and holes");
        assert!(has_arrow, "Should have Arrow markers for coordinate frame");
    }

    #[test]
    fn test_no_op_publisher() {
        let publisher = NoOpDebugPublisher;
        let debug_data = create_test_debug_data();

        // These should all succeed without error
        assert!(publisher.publish_icp_debug_data(&debug_data).is_ok());
        assert!(publisher.publish_board_model_markers(&debug_data.board_model, 0).is_ok());
        assert!(publisher.publish_point_cloud_debug(&[], "test").is_ok());
    }
}
