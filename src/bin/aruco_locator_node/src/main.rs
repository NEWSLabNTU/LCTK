use anyhow::{anyhow, bail, Result};
use aruco_detector::multi_aruco::ImageMarker;
use aruco_locator::{ArucoDetector, ArucoDetectorConfig};
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion};
use opencv::{
    core::{CV_8UC1, CV_8UC3, CV_8UC4},
    imgproc,
    prelude::*,
};
use rclrs::*;
use sensor_msgs::msg::{CameraInfo, Image as ImageMsg};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};
use std_msgs::msg::Header;
use vision_msgs::msg::{
    BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose,
    Point2D, Pose2D,
};

// Binary name for logging
const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

/// Convert aruco_locator::DetectionResult to Detection2DArray message
fn convert_detection_result(
    result: &aruco_locator::DetectionResult,
    header: Header,
) -> Detection2DArray {
    let mut detections = Vec::new();

    // Convert each detected marker
    if result.markers_found {
        for (i, &marker_id) in result.marker_ids.iter().enumerate() {
            if let Some(marker) = result.markers.get(i) {
                match convert_marker_to_detection2d(marker_id, marker, &header) {
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

/// Convert a single marker to Detection2D message
fn convert_marker_to_detection2d(
    marker_id: i32,
    marker: &ImageMarker,
    header: &Header,
) -> Result<Detection2D> {
    // Convert corners from nalgebra Point2 to vision_msgs Point2D
    let corners: Vec<Point2D> = marker
        .corners
        .iter()
        .map(|corner| Point2D {
            x: corner.x as f64,
            y: corner.y as f64,
        })
        .collect();

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
        id: format!("aruco_{marker_id}"),
    })
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
    _aruco_config_file: String,
}

impl ArucoLocatorNode {
    /// Create a new ArUco locator node
    pub fn new(node: &Node) -> Result<Self> {
        // Create the detector state
        let detector_state = Arc::new(Mutex::new(None));

        // Get the aruco_config_file parameter (mandatory - must be set by user)
        let aruco_config_file = node
            .declare_parameter::<Arc<str>>("aruco_config_file")
            .mandatory()?
            .get()
            .to_string();

        log_info!(LOGGER_NAME, "Using ArUco config file: {aruco_config_file}");

        // Try to declare the parameter with default value
        let camera_namespace = node
            .declare_parameter::<Arc<str>>("camera_namespace")
            .default("/camera/camera".into())
            .mandatory()?
            .get()
            .to_string();

        log_info!(LOGGER_NAME, "Using camera namespace: {camera_namespace}");

        // Form the camera_info topic name with namespace
        let camera_info_topic = format!("{camera_namespace}/camera_info");

        // Define potential image topics in priority order
        let potential_image_topics = vec![
            format!("{camera_namespace}/image_raw"),
            format!("{camera_namespace}/image"),
        ];

        // Create detection publisher
        let detection_publisher = node.create_publisher("aruco_detections")?;

        // Subscribe to camera_info
        let detector_state_camera_info = Arc::clone(&detector_state);
        let config_file_for_callback = aruco_config_file.clone();
        let camera_info_subscription = node.create_subscription::<CameraInfo, _>(
            &camera_info_topic,
            move |msg: CameraInfo| {
                Self::camera_info_callback(
                    msg,
                    Arc::clone(&detector_state_camera_info),
                    &config_file_for_callback,
                );
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
            _aruco_config_file: aruco_config_file,
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
        aruco_config_file: &str,
    ) {
        // Check if detector is already initialized
        let already_initialized = {
            match detector_state.lock() {
                Ok(state) => state.is_some(),
                Err(_) => false,
            }
        };

        let aruco_pattern = match Self::load_aruco_pattern(aruco_config_file) {
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

        // Only log initialization message once
        if !already_initialized {
            log_info!(LOGGER_NAME, "ArUco detector initialized");
        }
    }

    /// Load ArUco pattern from config file
    fn load_aruco_pattern(config_file: &str) -> Result<aruco_config::MultiArucoPattern> {
        let json5_text = std::fs::read_to_string(config_file)?;
        let pattern: aruco_config::MultiArucoPattern = json5::from_str(&json5_text)?;

        // Check if expected markers match our test pattern
        let expected_markers = [696, 64, 306, 195];
        if pattern.marker_ids != expected_markers {
            log_warn!(
                LOGGER_NAME,
                "ArUco pattern markers {:?} don't match expected {:?}",
                pattern.marker_ids,
                expected_markers
            );
        }

        Ok(pattern)
    }

    /// Process the incoming image
    fn process_image(
        msg: &ImageMsg,
        detector: &ArucoDetector,
    ) -> Result<aruco_locator::DetectionResult> {
        // Validate image encoding and convert to OpenCV Mat
        let mat = Self::ros_image_to_opencv_mat(msg)?;

        // Detect ArUco markers
        detector.detect_markers(&mat)
    }

    /// Convert ROS Image message to OpenCV Mat with proper encoding handling
    fn ros_image_to_opencv_mat(msg: &ImageMsg) -> Result<Mat> {
        // Validate data size
        let expected_size = (msg.step * msg.height) as usize;
        if msg.data.len() < expected_size {
            bail!(
                "Image data size ({}) is smaller than expected ({})",
                msg.data.len(),
                expected_size
            );
        }

        // Determine OpenCV type based on encoding
        let cv_type = match msg.encoding.as_str() {
            "mono8" => CV_8UC1,
            "bgr8" | "rgb8" => CV_8UC3,
            "bgra8" | "rgba8" => CV_8UC4,
            // Add more encodings as needed
            _ => bail!("Unsupported image encoding: {}", msg.encoding),
        };

        // Create OpenCV Mat with proper validation
        let mat = unsafe {
            Mat::new_rows_cols_with_data(
                msg.height as i32,
                msg.width as i32,
                cv_type,
                msg.data.as_ptr() as *mut std::ffi::c_void,
                msg.step as usize, // Use actual step from message
            )?
        };

        // Convert RGB to BGR if necessary (OpenCV expects BGR)
        let processed_mat = match msg.encoding.as_str() {
            "rgb8" => {
                let mut bgr_mat = Mat::default();
                imgproc::cvt_color(&mat, &mut bgr_mat, imgproc::COLOR_RGB2BGR, 0)?;
                bgr_mat
            }
            "rgba8" => {
                let mut bgr_mat = Mat::default();
                imgproc::cvt_color(&mat, &mut bgr_mat, imgproc::COLOR_RGBA2BGR, 0)?;
                bgr_mat
            }
            "mono8" => {
                // Convert grayscale to BGR for ArUco detection
                let mut bgr_mat = Mat::default();
                imgproc::cvt_color(&mat, &mut bgr_mat, imgproc::COLOR_GRAY2BGR, 0)?;
                bgr_mat
            }
            "bgr8" | "bgra8" => mat, // Already in correct format
            _ => mat,                // Should not reach here due to earlier validation
        };

        Ok(processed_mat)
    }

    /// Save debug images to visualize the processing pipeline
    fn save_debug_images(mat: &Mat, msg: &ImageMsg) -> Result<()> {
        use opencv::{
            core::Vector,
            imgcodecs::{imwrite, IMWRITE_JPEG_QUALITY},
        };

        // Create debug directory if it doesn't exist
        let debug_dir = "debug_images";
        std::fs::create_dir_all(debug_dir)?;

        // Generate timestamp for unique filenames
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_millis();

        // Save the original BGR/RGB image
        let original_filename = format!(
            "{}/original_{}x{}_{}.jpg",
            debug_dir, msg.width, msg.height, timestamp
        );
        let mut jpeg_params = Vector::new();
        jpeg_params.push(IMWRITE_JPEG_QUALITY);
        jpeg_params.push(90); // Quality 90%

        match imwrite(&original_filename, mat, &jpeg_params) {
            Ok(_) => log_info!(LOGGER_NAME, "Saved original image: {}", original_filename),
            Err(e) => log_warn!(LOGGER_NAME, "Failed to save original image: {}", e),
        }

        // Convert to grayscale to see what ArUco detection actually sees
        let mut gray_mat = Mat::default();
        let channels = mat.channels();
        if channels > 1 {
            opencv::imgproc::cvt_color(mat, &mut gray_mat, opencv::imgproc::COLOR_BGR2GRAY, 0)?;

            let gray_filename = format!(
                "{}/grayscale_{}x{}_{}.jpg",
                debug_dir, msg.width, msg.height, timestamp
            );

            match imwrite(&gray_filename, &gray_mat, &jpeg_params) {
                Ok(_) => log_info!(LOGGER_NAME, "Saved grayscale image: {}", gray_filename),
                Err(e) => log_warn!(LOGGER_NAME, "Failed to save grayscale image: {}", e),
            }
        } else {
            // Already grayscale
            let gray_filename = format!(
                "{}/already_grayscale_{}x{}_{}.jpg",
                debug_dir, msg.width, msg.height, timestamp
            );

            match imwrite(&gray_filename, mat, &jpeg_params) {
                Ok(_) => log_info!(LOGGER_NAME, "Saved grayscale image: {}", gray_filename),
                Err(e) => log_warn!(LOGGER_NAME, "Failed to save grayscale image: {}", e),
            }
        }

        Ok(())
    }

    fn subscribe_to_image_topic(
        &self,
        node: &Node,
        potential_topics: &[String],
    ) -> Option<Subscription<ImageMsg>> {
        log_info!(LOGGER_NAME, "Attempting to subscribe to image topics...");

        // Subscribe to image topic directly
        for topic in potential_topics {
            log_info!(LOGGER_NAME, "Trying to subscribe to topic: {topic}");
            let detector_state = Arc::clone(&self.detector_state);
            let publisher = self.detection_publisher.clone();

            match node.create_subscription::<ImageMsg, _>(topic, move |msg: ImageMsg| {
                Self::image_callback(msg, Arc::clone(&detector_state), &publisher);
            }) {
                Ok(sub) => {
                    log_info!(
                        LOGGER_NAME,
                        "Successfully subscribed to image topic: {topic}"
                    );
                    return Some(sub);
                }
                Err(e) => {
                    log_warn!(LOGGER_NAME, "Failed to subscribe to {topic}: {e}");
                    continue;
                }
            }
        }
        log_error!(LOGGER_NAME, "Failed to subscribe to any image topic");
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

        match Self::process_image(&msg, &detector) {
            Ok(detection_result) => {
                // Create message header
                let header = Header {
                    stamp: msg.header.stamp.clone(),
                    frame_id: msg.header.frame_id.clone(),
                };

                // Convert detection result to vision_msgs Detection2DArray
                let detection_msg = convert_detection_result(&detection_result, header);

                // Only log summary info, not details
                if detection_msg.detections.is_empty() {
                    // Only log occasionally for no detections to avoid spam
                    static mut NO_DETECTION_COUNT: u32 = 0;
                    unsafe {
                        NO_DETECTION_COUNT += 1;
                        if NO_DETECTION_COUNT % 30 == 1 {
                            // Log every 30th frame (approximately once per second at 30fps)
                            log_warn!(
                                LOGGER_NAME,
                                "No ArUco markers detected (suppressing repeated messages)"
                            );
                        }
                    }
                } else {
                    // Log only a brief summary when markers are detected
                    static mut LAST_DETECTION_COUNT: usize = 0;
                    let current_count = detection_msg.detections.len();
                    unsafe {
                        if LAST_DETECTION_COUNT != current_count {
                            // Only log when the number of detected markers changes
                            log_info!(
                                LOGGER_NAME,
                                "Detected {} ArUco markers: {:?}",
                                current_count,
                                detection_result.marker_ids
                            );
                            LAST_DETECTION_COUNT = current_count;
                        }
                    }
                }

                // Publish the detection result
                if let Err(e) = publisher.publish(&detection_msg) {
                    log_error!(LOGGER_NAME, "Failed to publish detection result: {e}");
                }
            }
            Err(e) => {
                log_error!(LOGGER_NAME, "ArUco detection failed: {e}");
            }
        }
    }
}

/// Main function for ROS node
pub fn run_node() -> Result<()> {
    // Initialize ROS 2
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("aruco_locator_node")?;

    // Create the node (automatically creates all its components)
    let _aruco_node = ArucoLocatorNode::new(&node)?;
    log_info!(LOGGER_NAME, "ArUco Locator node started");

    // Set up signal handling
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        log_info!(
            LOGGER_NAME,
            "Received interrupt signal, shutting down gracefully..."
        );
        r.store(false, Ordering::SeqCst);
    })?;

    // Spin the executor with a timeout, checking for signals
    while running.load(Ordering::SeqCst) {
        // Spin once with a short timeout to allow checking the signal flag
        let spin_options = SpinOptions::spin_once().timeout(Duration::from_millis(100));

        let spin_results = executor.spin(spin_options);

        // Check for errors (but ignore timeout errors as they're expected)
        for err in spin_results {
            // Check if it's not a timeout error
            if !format!("{:?}", err).contains("Timeout") {
                log_error!(LOGGER_NAME, "Executor error: {err}");
                return Err(anyhow!("Failed to spin executor: {err}"));
            }
        }
    }

    log_info!(LOGGER_NAME, "ArUco Locator node shutting down");
    Ok(())
}

fn main() -> Result<()> {
    run_node()
}
