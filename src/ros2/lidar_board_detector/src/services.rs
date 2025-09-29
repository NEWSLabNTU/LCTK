use crate::bbox::BBox;
use anyhow::Result;
use arc_swap::ArcSwap;
use lctk_interfaces::srv::{GetBBoxParams, SaveBBoxParams, SetBBoxParams};
use rclrs::{log_debug, log_error, log_info, log_warn, Node, Service};
use std::sync::Arc;

// Type aliases to avoid ambiguity
type GetBBoxParamsResponse = lctk_interfaces::srv::GetBBoxParams_Response;
type SetBBoxParamsRequest = lctk_interfaces::srv::SetBBoxParams_Request;
type SetBBoxParamsResponse = lctk_interfaces::srv::SetBBoxParams_Response;
type SaveBBoxParamsRequest = lctk_interfaces::srv::SaveBBoxParams_Request;
type SaveBBoxParamsResponse = lctk_interfaces::srv::SaveBBoxParams_Response;

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

/// Container for all BBox-related services
pub struct BBoxServices {
    pub _get_bbox_service: Service<GetBBoxParams>,
    pub _set_bbox_service: Service<SetBBoxParams>,
    pub _save_bbox_service: Service<SaveBBoxParams>,
}

impl BBoxServices {
    /// Create all BBox services
    pub fn new(node: &Node, bbox: Arc<ArcSwap<BBox>>, bbox_file_path: String) -> Result<Self> {
        // Create GetBBoxParams service
        let get_bbox = Arc::clone(&bbox);
        let get_service = node
            .create_service::<GetBBoxParams, _>("get_bbox_params", move |request, info| {
                handle_get_bbox_params(request, info, &get_bbox)
            })?;

        // Create SetBBoxParams service
        let set_bbox = Arc::clone(&bbox);
        let set_service = node
            .create_service::<SetBBoxParams, _>("set_bbox_params", move |request, info| {
                handle_set_bbox_params(request, info, &set_bbox)
            })?;

        // Create SaveBBoxParams service
        let save_bbox = Arc::clone(&bbox);
        let save_bbox_file_path = bbox_file_path.clone();
        let save_service =
            node.create_service::<SaveBBoxParams, _>("save_bbox_params", move |request, info| {
                handle_save_bbox_params(request, info, &save_bbox, &save_bbox_file_path)
            })?;

        log_info!(
            LOGGER_NAME,
            "BBox services created: get_bbox_params, set_bbox_params, save_bbox_params"
        );

        Ok(BBoxServices {
            _get_bbox_service: get_service,
            _set_bbox_service: set_service,
            _save_bbox_service: save_service,
        })
    }
}

/// Handle GetBBoxParams service request
fn handle_get_bbox_params(
    _request: lctk_interfaces::srv::GetBBoxParams_Request,
    _info: rclrs::ServiceInfo,
    bbox: &Arc<ArcSwap<BBox>>,
) -> GetBBoxParamsResponse {
    log_debug!(LOGGER_NAME, "GetBBoxParams service called");

    // Load bbox using lock-free arc-swap
    let bbox_arc = bbox.load();

    let response = GetBBoxParamsResponse {
        pose: bbox_arc.to_ros_pose(),
        size_xyz: bbox_arc.size_xyz,
    };

    log_debug!(
        LOGGER_NAME,
        "GetBBoxParams success: pose=({:.3}, {:.3}, {:.3}), size=[{:.1}, {:.1}, {:.1}]",
        response.pose.position.x,
        response.pose.position.y,
        response.pose.position.z,
        response.size_xyz[0],
        response.size_xyz[1],
        response.size_xyz[2]
    );

    response
}

/// Handle SetBBoxParams service request
fn handle_set_bbox_params(
    request: SetBBoxParamsRequest,
    _info: rclrs::ServiceInfo,
    bbox: &Arc<ArcSwap<BBox>>,
) -> SetBBoxParamsResponse {
    log_debug!(
        LOGGER_NAME,
        "SetBBoxParams service called with pose=({:.3}, {:.3}, {:.3}), size=[{:.1}, {:.1}, {:.1}]",
        request.pose.position.x,
        request.pose.position.y,
        request.pose.position.z,
        request.size_xyz[0],
        request.size_xyz[1],
        request.size_xyz[2]
    );

    // Use the size array directly (it's already [f64; 3])
    let size_array = request.size_xyz;

    // Create new BBox from request
    let new_bbox = match BBox::from_ros_pose(&request.pose, size_array) {
        Ok(bbox) => bbox,
        Err(e) => {
            let error_msg = format!("Invalid bbox parameters: {}", e);
            log_warn!(
                LOGGER_NAME,
                "SetBBoxParams validation failed: {}",
                error_msg
            );
            return SetBBoxParamsResponse {
                success: false,
                message: error_msg,
            };
        }
    };

    // Update the shared bbox using lock-free arc-swap
    bbox.store(Arc::new(new_bbox));

    log_info!(
        LOGGER_NAME,
        "BBox parameters updated successfully: pose=({:.3}, {:.3}, {:.3}), size=[{:.1}, {:.1}, {:.1}]",
        request.pose.position.x,
        request.pose.position.y,
        request.pose.position.z,
        size_array[0],
        size_array[1],
        size_array[2]
    );

    SetBBoxParamsResponse {
        success: true,
        message: "BBox parameters updated successfully".to_string(),
    }
}

/// Handle SaveBBoxParams service request
fn handle_save_bbox_params(
    request: SaveBBoxParamsRequest,
    _info: rclrs::ServiceInfo,
    bbox: &Arc<ArcSwap<BBox>>,
    default_file_path: &str,
) -> SaveBBoxParamsResponse {
    log_debug!(
        LOGGER_NAME,
        "SaveBBoxParams service called with file_path='{}'",
        request.file_path
    );

    // Determine the file path to use
    let file_path = if request.file_path.is_empty() {
        default_file_path.to_string()
    } else {
        request.file_path.clone()
    };

    log_debug!(LOGGER_NAME, "Using file path: {}", file_path);

    // Load current bbox and save it using lock-free arc-swap
    let bbox_arc = bbox.load();
    match bbox_arc.save_to_file(&file_path) {
        Ok(()) => {
            log_info!(
                LOGGER_NAME,
                "BBox parameters saved successfully to: {}",
                file_path
            );

            SaveBBoxParamsResponse {
                success: true,
                message: "BBox parameters saved successfully".to_string(),
                saved_file_path: file_path,
            }
        }
        Err(e) => {
            let error_msg = format!("Failed to save bbox to file '{}': {}", file_path, e);
            log_error!(LOGGER_NAME, "SaveBBoxParams failed: {}", error_msg);
            SaveBBoxParamsResponse {
                success: false,
                message: error_msg,
                saved_file_path: String::new(),
            }
        }
    }
}
