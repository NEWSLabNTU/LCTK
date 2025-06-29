use crate::{
    roi::{bounds_from_center_size, center_size_from_bounds, RoiManager},
    visualization::RoiMarkerGenerator,
};
use rclrs::{log_error, log_info, rmw_request_id_t as ServiceRequestHeader, Node, ToLogParams};
// Temporarily disabled - rosbag_deck_interface not available as Rust crate
// use rosbag_deck_interface::srv::{SetRoiBoundsRequest, SetRoiBoundsResponse};

// Mock types for now
#[derive(Default)]
pub struct SetRoiBoundsRequest {
    pub lidar_id: u8,
    pub center_x: f64,
    pub center_y: f64,
    pub center_z: f64,
    pub size_x: f64,
    pub size_y: f64,
    pub size_z: f64,
}

#[derive(Default)]
pub struct SetRoiBoundsResponse {
    pub success: bool,
    pub message: String,
}
use std::sync::Arc;

/// Handler for ROI-related services
pub struct RoiServiceHandler<R: RoiManager, M: RoiMarkerGenerator> {
    roi_manager: Arc<R>,
    marker_generator: Arc<M>,
    node: Arc<Node>,
}

impl<R: RoiManager, M: RoiMarkerGenerator> RoiServiceHandler<R, M> {
    pub fn new(roi_manager: Arc<R>, marker_generator: Arc<M>, node: Arc<Node>) -> Self {
        Self {
            roi_manager,
            marker_generator,
            node,
        }
    }

    pub fn handle_set_roi_bounds(
        &self,
        _request_header: &ServiceRequestHeader,
        request: SetRoiBoundsRequest,
    ) -> SetRoiBoundsResponse {
        let mut response = SetRoiBoundsResponse::default();

        // Validate lidar_id
        if request.lidar_id != 1 && request.lidar_id != 2 {
            response.success = false;
            response.message = format!("Invalid lidar_id: {}. Must be 1 or 2.", request.lidar_id);
            return response;
        }

        // Create bounds from center and size
        let bounds = bounds_from_center_size(
            request.center_x,
            request.center_y,
            request.center_z,
            request.size_x,
            request.size_y,
            request.size_z,
        );

        // Update ROI bounds
        match self
            .roi_manager
            .set_bounds(request.lidar_id, bounds.clone())
        {
            Ok(_) => {
                log_info!("multi_wayside_node", 
                    "Updated ROI bounds for LiDAR {}: center=({:.2}, {:.2}, {:.2}), size=({:.2}, {:.2}, {:.2})",
                    request.lidar_id,
                    request.center_x, request.center_y, request.center_z,
                    request.size_x, request.size_y, request.size_z
                );

                response.success = true;
                response.message = format!("ROI bounds updated for LiDAR {}", request.lidar_id);
            }
            Err(e) => {
                log_error!(
                    "multi_wayside_node",
                    "Failed to update ROI bounds for LiDAR {}: {}",
                    request.lidar_id,
                    e
                );

                response.success = false;
                response.message = format!("Failed to update ROI bounds: {e}");
            }
        }

        response
    }

    #[allow(clippy::type_complexity)]
    pub fn get_roi_bounds(&self, lidar_id: u8) -> Option<((f64, f64, f64), (f64, f64, f64))> {
        self.roi_manager
            .get_bounds(lidar_id)
            .map(|bounds| center_size_from_bounds(&bounds))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{roi::DefaultRoiManager, visualization::DefaultRoiMarkerGenerator};
    use rclrs::{Context, CreateBasicExecutor, InitOptions};
    use std::sync::Arc;

    #[test]
    fn test_set_roi_bounds_service() {
        let context = Context::new(std::env::args(), InitOptions::default()).unwrap();
        let executor = context.create_basic_executor();
        let node = executor.create_node("test_node").unwrap();
        let roi_manager = Arc::new(DefaultRoiManager::new());
        let marker_generator = Arc::new(DefaultRoiMarkerGenerator);

        let handler = RoiServiceHandler::new(roi_manager.clone(), marker_generator, Arc::new(node));

        // Test valid request
        let request = SetRoiBoundsRequest {
            lidar_id: 1,
            center_x: 2.0,
            center_y: 0.0,
            center_z: 0.0,
            size_x: 4.0,
            size_y: 4.0,
            size_z: 2.0,
        };

        let header = ServiceRequestHeader {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        let response = handler.handle_set_roi_bounds(&header, request);

        assert!(response.success);
        assert!(response.message.contains("updated"));

        // Verify bounds were set
        let bounds = roi_manager.get_bounds(1).unwrap();
        assert_eq!(bounds.min_x, 0.0);
        assert_eq!(bounds.max_x, 4.0);
    }

    #[test]
    fn test_invalid_lidar_id() {
        let context = Context::new(std::env::args(), InitOptions::default()).unwrap();
        let executor = context.create_basic_executor();
        let node = Arc::new(executor.create_node("test_node").unwrap());
        let roi_manager = Arc::new(DefaultRoiManager::new());
        let marker_generator = Arc::new(DefaultRoiMarkerGenerator);

        let handler = RoiServiceHandler::new(roi_manager, marker_generator, node);

        let request = SetRoiBoundsRequest {
            lidar_id: 3, // Invalid
            ..Default::default()
        };

        let header = ServiceRequestHeader {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        let response = handler.handle_set_roi_bounds(&header, request);

        assert!(!response.success);
        assert!(response.message.contains("Invalid lidar_id"));
    }
}
