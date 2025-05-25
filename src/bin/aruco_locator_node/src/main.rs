use anyhow::{anyhow, bail, Result};
use aruco_locator::{ArucoDetector, ArucoDetectorConfig};
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion};
use opencv::{core::CV_8UC3, prelude::*};
use rclrs::{log_error, log_info, log_warn, *};
use sensor_msgs::msg::{CameraInfo, Image as ImageMsg};
use serde_loader::Json5Path;
use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
};
use std_msgs::msg::Header;
use vision_msgs::msg::{
    BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose,
    Point2D, Pose2D,
};

// Binary name for logging
const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

const ARUCO_PATTERN_CONFIG: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/config/aruco_pattern.json5");

/// Convert aruco_locator::DetectionResult to Detection2DArray message
fn convert_detection_result(
    result: &aruco_locator::DetectionResult,
    header: Header,
) -> Detection2DArray {
    let mut detections = Vec::new();

    // Convert each detected marker
    if result.markers_found {
        for (i, &marker_id) in result.marker_ids.iter().enumerate() {
            if let Some(marker_data) = result.markers.get(i) {
                match convert_marker_to_detection2d(marker_id, marker_data, &header) {
                    Ok(detection) => detections.push(detection),
                    Err(e) => {
                        log_warn!(LOGGER_NAME, "Failed to convert marker {marker_id}: {e}");
                        continue;
                    }
                }
            }
        }
    }

    Detection2DArray { header, detections }
}

/// Convert a single marker from JSON to Detection2D message
fn convert_marker_to_detection2d(
    marker_id: i32,
    marker_data: &serde_json::Value,
    header: &Header,
) -> Result<Detection2D> {
    // Extract corners from the JSON data
    let corners = extract_corners_from_json(marker_data)?;

    // Calculate bounding box from corners
    let bbox = calculate_bounding_box(&corners);

    // Create object hypothesis with marker ID
    let hypothesis = ObjectHypothesis {
        class_id: marker_id.to_string(),
        score: 1.0, // ArUco detections are binary (detected or not)
    };

    // Create pose (placeholder for now - would need actual pose estimation)
    let pose_with_covariance = PoseWithCovariance {
        pose: Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        },
        covariance: [0.0; 36], // 6x6 covariance matrix
    };

    let object_hypothesis_with_pose = ObjectHypothesisWithPose {
        hypothesis,
        pose: pose_with_covariance,
    };

    Ok(Detection2D {
        header: header.clone(),
        results: vec![object_hypothesis_with_pose],
        bbox,
        id: format!("aruco_{}", marker_id),
    })
}

/// Extract corner points from JSON marker data
fn extract_corners_from_json(marker_data: &serde_json::Value) -> Result<Vec<Point2D>> {
    let corners_array = marker_data
        .get("corners")
        .and_then(|v| v.as_array())
        .ok_or_else(|| anyhow!("Missing or invalid corners array"))?;

    let mut corners = Vec::new();
    for corner in corners_array {
        let x = corner
            .get("x")
            .and_then(|v| v.as_f64())
            .ok_or_else(|| anyhow!("Missing or invalid corner x coordinate"))?;
        let y = corner
            .get("y")
            .and_then(|v| v.as_f64())
            .ok_or_else(|| anyhow!("Missing or invalid corner y coordinate"))?;

        corners.push(Point2D { x, y });
    }

    if corners.len() != 4 {
        bail!("Expected 4 corners, got {}", corners.len());
    }

    Ok(corners)
}

/// Calculate bounding box from corner points
fn calculate_bounding_box(corners: &[Point2D]) -> BoundingBox2D {
    if corners.is_empty() {
        return BoundingBox2D {
            center: Pose2D {
                position: Point2D { x: 0.0, y: 0.0 },
                theta: 0.0,
            },
            size_x: 0.0,
            size_y: 0.0,
        };
    }

    // Find min/max x and y coordinates
    let min_x = corners.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let max_x = corners
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = corners.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let max_y = corners
        .iter()
        .map(|p| p.y)
        .fold(f64::NEG_INFINITY, f64::max);

    // Calculate center and size
    let center_x = (min_x + max_x) / 2.0;
    let center_y = (min_y + max_y) / 2.0;
    let size_x = max_x - min_x;
    let size_y = max_y - min_y;

    BoundingBox2D {
        center: Pose2D {
            position: Point2D {
                x: center_x,
                y: center_y,
            },
            theta: 0.0, // TODO: Calculate actual rotation if needed
        },
        size_x,
        size_y,
    }
}

/// ArUco detection ROS 2 node
pub struct ArucoLocatorNode {
    _camera_info_subscription: Subscription<CameraInfo>,
    image_subscription: Option<Subscription<ImageMsg>>,
    detection_publisher: Publisher<Detection2DArray>,
    _camera_namespace: String,
    detector_state: Arc<Mutex<Option<Arc<ArucoDetector>>>>,
}

impl ArucoLocatorNode {
    /// Create a new ArUco locator node
    pub fn new(node: &Node) -> Result<Self> {
        // Create the detector state
        let detector_state = Arc::new(Mutex::new(None));

        // Try to declare the parameter, but have a default fallback for any errors
        let camera_namespace = match node
            .declare_parameter::<Arc<str>>("camera_namespace")
            .mandatory()
        {
            Ok(param) => param.get().to_string(),
            Err(e) => {
                bail!("Failed to declare parameter 'camera_namespace': {e}.");
            }
        };

        log_info!(LOGGER_NAME, "Using camera namespace: {camera_namespace}");

        // Form the camera_info topic name with namespace
        let camera_info_topic = format!("{camera_namespace}/camera_info");

        // Define potential image topics in priority order
        let potential_image_topics = vec![
            format!("{camera_namespace}/image_rect_color"),
            format!("{camera_namespace}/image_rect"),
            format!("{camera_namespace}/image_color"),
            format!("{camera_namespace}/image"),
            format!("{camera_namespace}/image_raw"),
        ];

        // Create detection publisher
        let detection_publisher = node.create_publisher::<Detection2DArray>("aruco_detections")?;

        // Subscribe to camera_info
        let detector_state_camera_info = Arc::clone(&detector_state);
        let camera_info_subscription = node.create_subscription::<CameraInfo, _>(
            &camera_info_topic,
            move |msg: CameraInfo| {
                Self::camera_info_callback(msg, Arc::clone(&detector_state_camera_info));
            },
        )?;

        log_info!(LOGGER_NAME, "Camera namespace: {camera_namespace}");
        log_info!(
            LOGGER_NAME,
            "Waiting for camera_info on topic: {camera_info_topic}"
        );

        // Create the node instance
        let mut node_instance = Self {
            _camera_info_subscription: camera_info_subscription,
            image_subscription: None,
            _camera_namespace: camera_namespace,
            detection_publisher,
            detector_state,
        };

        // Try to find an available image topic and subscribe to it with image processing callback
        let image_subscription =
            node_instance.subscribe_to_image_topic(node, &potential_image_topics);
        node_instance.image_subscription = image_subscription;

        if node_instance.image_subscription.is_some() {
            log_info!(
                LOGGER_NAME,
                "Subscribed to image topic and will publish detections to /aruco_detections"
            );
        } else {
            log_warn!(LOGGER_NAME, "No available image topics found. The node will wait for cameras to become available.");
        }

        Ok(node_instance)
    }

    /// Handle camera info updates
    fn camera_info_callback(
        camera_info: CameraInfo,
        detector_state: Arc<Mutex<Option<Arc<ArucoDetector>>>>,
    ) {
        let aruco_pattern = match Self::load_aruco_pattern() {
            Ok(pattern) => pattern,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to load ArUco pattern: {e}");
                return;
            }
        };

        let config = ArucoDetectorConfig {
            camera_info,
            aruco_pattern,
        };

        let detector = match ArucoDetector::new(config) {
            Ok(detector) => detector,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to create ArUco detector: {e}");
                return;
            }
        };

        let mut state = match detector_state.lock() {
            Ok(state) => state,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to lock detector state: {e}");
                return;
            }
        };

        *state = Some(Arc::new(detector));
        log_info!(LOGGER_NAME, "Camera info updated from camera_info topic");
    }

    /// Load ArUco pattern from config file
    fn load_aruco_pattern() -> Result<aruco_config::MultiArucoPattern> {
        Ok(Json5Path::open_and_take(&PathBuf::from(
            ARUCO_PATTERN_CONFIG,
        ))?)
    }

    /// Process the incoming image
    fn process_image(
        msg: &ImageMsg,
        detector: &ArucoDetector,
    ) -> Result<aruco_locator::DetectionResult> {
        // Create OpenCV Mat from raw image data
        // Assuming the image is in BGR8 format (common for ROS)
        let mat = unsafe {
            Mat::new_rows_cols_with_data(
                msg.height as i32,
                msg.width as i32,
                CV_8UC3,
                msg.data.as_ptr() as *mut std::ffi::c_void,
                opencv::core::Mat_AUTO_STEP,
            )?
        };

        // Detect ArUco markers
        detector.detect_markers(&mat)
    }

    /// Helper method to try subscribing to an image topic from a list of candidates
    /// with image processing callback
    fn subscribe_to_image_topic(
        &self,
        node: &Node,
        potential_topics: &[String],
    ) -> Option<Subscription<ImageMsg>> {
        // Find first topic with publishers
        for topic in potential_topics {
            match node.count_publishers(topic) {
                Ok(count) if count > 0 => {
                    // Topic has publishers, try to subscribe with image processing callback
                    let detector_state = Arc::clone(&self.detector_state);
                    let publisher = self.detection_publisher.clone();

                    match node.create_subscription::<ImageMsg, _>(topic, move |msg: ImageMsg| {
                        Self::image_callback(msg, Arc::clone(&detector_state), &publisher);
                    }) {
                        Ok(sub) => {
                            log_info!(LOGGER_NAME, "Subscribed to image topic: {topic}");
                            return Some(sub);
                        }
                        Err(e) => {
                            log_warn!(LOGGER_NAME, "Failed to subscribe to {topic}: {e}");
                            continue;
                        }
                    }
                }
                Ok(_) => {
                    // Topic exists but has no publishers
                    log_info!(LOGGER_NAME, "Topic {topic} has no publishers");
                    continue;
                }
                Err(e) => {
                    log_error!(LOGGER_NAME, "Error checking publishers for {topic}: {e}");
                    continue;
                }
            }
        }
        None
    }

    /// Process incoming image messages and publish detection results
    fn image_callback(
        msg: ImageMsg,
        detector_state: Arc<Mutex<Option<Arc<ArucoDetector>>>>,
        publisher: &Publisher<Detection2DArray>,
    ) {
        // Get detector
        let detector = {
            let state_lock = match detector_state.lock() {
                Ok(guard) => guard,
                Err(e) => {
                    log_error!(
                        LOGGER_NAME,
                        "Failed to lock detector state in image_callback: {e}"
                    );
                    return;
                }
            };

            match state_lock.as_ref() {
                Some(detector) => Arc::clone(detector),
                None => {
                    // Detector not initialized yet, skip this frame
                    return;
                }
            }
        };

        // Process the image
        match Self::process_image(&msg, &detector) {
            Ok(detection_result) => {
                // Create message header
                let header = Header {
                    stamp: msg.header.stamp.clone(),
                    frame_id: msg.header.frame_id.clone(),
                };

                // Convert detection result to vision_msgs Detection2DArray
                let detection_msg = convert_detection_result(&detection_result, header);

                // Publish the detection result
                if let Err(e) = publisher.publish(detection_msg) {
                    log_error!(LOGGER_NAME, "Failed to publish detection result: {e}");
                }
            }
            Err(e) => {
                log_error!(LOGGER_NAME, "Detection failed: {e}");
            }
        }
    }
}

/// Main function for ROS node
pub fn run_node() -> Result<()> {
    // Initialize ROS 2
    let context = Context::new(std::env::args(), InitOptions::new())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("aruco_locator_node")?;

    // Create the node (automatically creates all its components)
    let _aruco_node = ArucoLocatorNode::new(&node)?;
    log_info!(LOGGER_NAME, "ArUco Locator node started");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}

fn main() -> Result<()> {
    run_node()
}
