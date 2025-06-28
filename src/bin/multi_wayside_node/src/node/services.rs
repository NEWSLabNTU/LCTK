// Temporarily disabled - rosbag_deck_interface not available as Rust crate
// use rosbag_deck_interface::srv::SetROIBounds;

// Mock type for now
pub struct SetROIBounds;

/// Trait for managing ROS 2 services
pub trait ServiceManager: Send + Sync {
    // Interface for service management
}

/// Factory for creating ROS 2 services
pub struct ServiceFactory;

impl ServiceFactory {
    // Create ROI bounds service
    // Temporarily disabled due to rosbag_deck_interface dependency
    /*
    pub fn create_roi_service<R, M>(
        node: &Node,
        roi_manager: Arc<R>,
        marker_generator: Arc<M>,
    ) -> Result<Arc<Service<SetROIBounds>>>
    where
        R: RoiManager + 'static,
        M: RoiMarkerGenerator + 'static,
    {
        let node_arc = Arc::new(node.clone());
        let handler = RoiServiceHandler::new(roi_manager, marker_generator, node_arc.clone());

        let service = Arc::new(node.create_service::<SetROIBounds, _>(
            "/set_roi_bounds",
            move |request_header, request| {
                handler.handle_set_roi_bounds(&request_header, request)
            },
        )?);

        Ok(service)
    }
    */
}

// Container for all services to keep them alive
// Temporarily disabled due to rosbag_deck_interface dependency
/*
pub struct ServiceContainer {
    _set_roi_service: Arc<Service<SetROIBounds>>,
}

impl ServiceContainer {
    pub fn new(set_roi_service: Arc<Service<SetROIBounds>>) -> Self {
        Self {
            _set_roi_service: set_roi_service,
        }
    }
}
*/

#[cfg(test)]
mod tests {

    // Temporarily disabled due to rosbag_deck_interface dependency
    /*
    #[test]
    fn test_service_factory() {
        let context = Context::new(vec![]).unwrap();
        let node = context.create_node("test_node").unwrap();

        let roi_manager = Arc::new(DefaultRoiManager::new());
        let marker_generator = Arc::new(DefaultRoiMarkerGenerator);

        let result = ServiceFactory::create_roi_service(
            &node,
            roi_manager,
            marker_generator,
        );
        assert!(result.is_ok());
    }
    */
}
