use anyhow::{anyhow, bail, Result};
use aruco_locator::{ArucoDetector, ArucoDetectorConfig};
use aruco_locator_msgs::msg::{ArucoDetection, ArucoMarker, Point2D};
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion};
use noisy_float::prelude::*;
use opencv::{core::CV_8UC3, prelude::*};
use rclrs::{log_error, log_info, log_warn, *};
use sensor_msgs::msg::{CameraInfo, Image as ImageMsg};
use serde_loader::Json5Path;
use serde_types::{CameraIntrinsics, CameraMatrix, DistortionCoefs};
use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
};
use std_msgs::msg::Header;

// Binary name for logging
const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

const ARUCO_PATTERN_CONFIG: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/config/aruco_pattern.json5");

/// Convert ROS CameraInfo message to CameraIntrinsics
fn camera_info_to_intrinsics(camera_info: &CameraInfo) -> Result<CameraIntrinsics> {
    let k = &camera_info.k;
    let d = &camera_info.d;

    // CameraInfo K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    let camera_matrix = CameraMatrix([
        [r64(k[0]), r64(k[1]), r64(k[2])],
        [r64(k[3]), r64(k[4]), r64(k[5])],
        [r64(k[6]), r64(k[7]), r64(k[8])],
    ]);

    // Ensure we have at least 5 distortion coefficients, pad with zeros if needed
    let mut distortion = [r64(0.0); 5];
    for (i, &coef) in d.iter().take(5).enumerate() {
        distortion[i] = r64(coef);
    }
    let distortion_coefs = DistortionCoefs(distortion);

    Ok(CameraIntrinsics {
        camera_matrix,
        distortion_coefs,
    })
}

/// Convert aruco_locator::DetectionResult to ArucoDetection message
fn convert_detection_result(
    result: &aruco_locator::DetectionResult,
    header: Header,
) -> ArucoDetection {
    let mut aruco_detection = ArucoDetection {
        header,
        markers_found: result.markers_found,
        marker_count: result.marker_ids.len() as u32,
        markers: Vec::new(),
        processing_time_ms: 0.0, // TODO: Add timing measurement
    };

    // Convert each detected marker
    if result.markers_found {
        for (i, &marker_id) in result.marker_ids.iter().enumerate() {
            if let Some(marker_data) = result.markers.get(i) {
                let aruco_marker = match convert_marker_data(marker_id, marker_data) {
                    Ok(marker) => marker,
                    Err(e) => {
                        log_warn!(LOGGER_NAME, "Failed to convert marker {marker_id}: {e}");
                        continue;
                    }
                };
                aruco_detection.markers.push(aruco_marker);
            }
        }
    }

    aruco_detection
}

/// Convert a single marker from JSON to ArucoMarker message
fn convert_marker_data(marker_id: i32, marker_data: &serde_json::Value) -> Result<ArucoMarker> {
    // Extract corners from the JSON data
    let corners = if let Some(corners_array) = marker_data.get("corners") {
        if let Some(corners_vec) = corners_array.as_array() {
            let mut corners: Vec<Point2D> = Vec::new();
            for corner in corners_vec {
                if let (Some(x), Some(y)) = (corner.get("x"), corner.get("y")) {
                    if let (Some(x_val), Some(y_val)) = (x.as_f64(), y.as_f64()) {
                        corners.push(Point2D { x: x_val, y: y_val });
                    }
                }
            }

            if corners.len() == 4 {
                [
                    corners[0].clone(),
                    corners[1].clone(),
                    corners[2].clone(),
                    corners[3].clone(),
                ]
            } else {
                // Default corners if parsing fails
                [
                    Point2D { x: 0.0, y: 0.0 },
                    Point2D { x: 0.0, y: 0.0 },
                    Point2D { x: 0.0, y: 0.0 },
                    Point2D { x: 0.0, y: 0.0 },
                ]
            }
        } else {
            // Default corners
            [
                Point2D { x: 0.0, y: 0.0 },
                Point2D { x: 0.0, y: 0.0 },
                Point2D { x: 0.0, y: 0.0 },
                Point2D { x: 0.0, y: 0.0 },
            ]
        }
    } else {
        // Default corners
        [
            Point2D { x: 0.0, y: 0.0 },
            Point2D { x: 0.0, y: 0.0 },
            Point2D { x: 0.0, y: 0.0 },
            Point2D { x: 0.0, y: 0.0 },
        ]
    };

    // Extract pose if available (for now, we'll set pose_valid to false as the current
    // DetectionResult doesn't include pose information in a structured way)
    let pose_valid = false;
    let pose = PoseWithCovariance {
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

    Ok(ArucoMarker {
        id: marker_id,
        corners,
        pose_valid,
        pose,
    })
}

/// ArUco detection ROS 2 node
pub struct ArucoLocatorNode {
    _camera_info_subscription: Subscription<CameraInfo>,
    image_subscription: Option<Subscription<ImageMsg>>,
    detection_publisher: Publisher<ArucoDetection>,
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
        let detection_publisher = node.create_publisher::<ArucoDetection>("aruco_detections")?;

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
        let camera_intrinsics = match camera_info_to_intrinsics(&camera_info) {
            Ok(intrinsics) => intrinsics,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to convert camera info: {e}");
                return;
            }
        };

        let aruco_pattern = match Self::load_aruco_pattern() {
            Ok(pattern) => pattern,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to load ArUco pattern: {e}");
                return;
            }
        };

        let config = ArucoDetectorConfig {
            camera_intrinsics,
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
        log_info!(
            LOGGER_NAME,
            "Camera intrinsics updated from camera_info topic"
        );
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
        publisher: &Publisher<ArucoDetection>,
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

                // Convert detection result to ArUco detection message
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
