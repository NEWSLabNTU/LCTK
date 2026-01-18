use anyhow::{anyhow, bail, Result};
use arc_swap::ArcSwap;
use aruco_detector::multi_aruco::ImageMarker;
use aruco_locator::{ArucoDetector, ArucoDetectorConfig};
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion};
use opencv::{
    calib3d,
    core::{Mat, Scalar, CV_8UC1, CV_8UC3, CV_8UC4},
    imgproc::{self, FONT_HERSHEY_SIMPLEX, LINE_8},
    prelude::*,
};
use rclrs::{PublisherOptions, SubscriptionOptions, *};
use sensor_msgs::msg::{CameraInfo, Image as ImageMsg};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
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

/// Camera calibration data for undistortion (using vectors for thread-safety)
#[derive(Clone, Debug)]
struct CameraCalibration {
    camera_matrix: [f64; 9], // 3x3 matrix in row-major order
    distortion_coeffs: Vec<f64>,
    #[allow(dead_code)]
    width: i32,
    #[allow(dead_code)]
    height: i32,
}

/// ArUco detection ROS 2 node
#[allow(dead_code)]
pub struct ArucoLocatorNode {
    _camera_info_subscription: Subscription<CameraInfo>,
    _image_subscription: Subscription<ImageMsg>,
    detection_publisher: Publisher<Detection2DArray>,
    overlay_publisher: Option<Publisher<ImageMsg>>,
    detector_state: Arc<ArcSwap<Option<Arc<ArucoDetector>>>>,
    camera_calibration: Arc<ArcSwap<Option<CameraCalibration>>>,
    aruco_pattern: Arc<aruco_config::MultiArucoPattern>,
    debug_overlay_enabled: bool,
}

impl ArucoLocatorNode {
    /// Create a new ArUco locator node
    pub fn new(node: &Node) -> Result<Self> {
        // Create the detector state using ArcSwap for lock-free access
        let detector_state = Arc::new(ArcSwap::from_pointee(None));

        // Create camera calibration storage using ArcSwap for lock-free access
        let camera_calibration = Arc::new(ArcSwap::from_pointee(None));

        // Get the aruco_config_file parameter (mandatory - must be set by user)
        let aruco_config_file = node
            .declare_parameter::<Arc<str>>("aruco_config_file")
            .mandatory()?
            .get()
            .to_string();

        log_info!(LOGGER_NAME, "Using ArUco config file: {aruco_config_file}");

        // Load ArUco pattern for use in overlay generation
        let aruco_pattern = Arc::new(Self::load_aruco_pattern(&aruco_config_file)?);

        // Get debug overlay parameter (default: false)
        let debug_overlay_enabled = node
            .declare_parameter("debug_overlay_enabled")
            .default(false)
            .mandatory()?
            .get();

        log_info!(
            LOGGER_NAME,
            "Debug overlay enabled: {debug_overlay_enabled}"
        );

        // Get QoS parameter for sensor input topics
        let use_best_effort_qos = node
            .declare_parameter("use_best_effort_qos")
            .default(true)
            .mandatory()?
            .get();

        log_info!(
            LOGGER_NAME,
            "Using {} QoS for sensor input topics",
            if use_best_effort_qos {
                "best effort"
            } else {
                "reliable"
            }
        );

        // ROS 2 Best Practice: Subscribe to base topics and let launch files handle remapping
        // The node subscribes to generic "image" topic (remapped by launch file)
        // and derives the camera_info topic from the image topic namespace
        let image_topic = "image";

        // Configure QoS for sensor input topics
        let qos_profile = if use_best_effort_qos {
            let mut qos = QoSProfile::sensor_data_default();
            qos.history = rclrs::QoSHistoryPolicy::KeepLast { depth: 1 }; // Prevent buffering delays
            qos
        } else {
            QoSProfile::default() // Reliable for rosbag playback
        };

        // First create image subscription to get the resolved topic name
        let mut image_options = SubscriptionOptions::new(image_topic);
        image_options.qos = qos_profile;

        // Create a temporary subscription to get resolved topic name
        let temp_image_subscription = node.create_subscription(image_options, |_: ImageMsg| {})?;
        let resolved_image_topic = temp_image_subscription.topic_name().clone();

        // Derive camera_info topic from resolved image topic
        let camera_info_topic = if let Some(last_slash) = resolved_image_topic.rfind('/') {
            let base_path = &resolved_image_topic[..last_slash];
            format!("{}/camera_info", base_path)
        } else {
            "camera_info".to_string()
        };

        log_info!(
            LOGGER_NAME,
            "Resolved image topic: {}",
            resolved_image_topic
        );
        log_info!(
            LOGGER_NAME,
            "Derived camera_info topic: {}",
            camera_info_topic
        );

        // Create detection publisher with BEST_EFFORT QoS for timestamp-based matching
        let mut detection_pub_opts = PublisherOptions::new("aruco_detections");
        detection_pub_opts.qos = QoSProfile {
            history: QoSHistoryPolicy::KeepLast { depth: 1 },
            ..QoSProfile::sensor_data_default() // BEST_EFFORT
        };
        let detection_publisher = node.create_publisher(detection_pub_opts)?;

        // Create overlay publisher if debug is enabled (BEST_EFFORT QoS for image streaming)
        let overlay_publisher = if debug_overlay_enabled {
            let mut overlay_pub_opts = PublisherOptions::new("image_with_detections");
            overlay_pub_opts.qos = QoSProfile {
                history: QoSHistoryPolicy::KeepLast { depth: 1 },
                ..QoSProfile::sensor_data_default() // BEST_EFFORT
            };
            Some(node.create_publisher(overlay_pub_opts)?)
        } else {
            None
        };

        // Subscribe to camera_info using derived topic name
        let detector_state_camera_info = Arc::clone(&detector_state);
        let camera_calibration_camera_info = Arc::clone(&camera_calibration);
        let config_file_for_callback = aruco_config_file.clone();
        let mut camera_info_options = SubscriptionOptions::new(&camera_info_topic);
        camera_info_options.qos = qos_profile;
        let camera_info_subscription =
            node.create_subscription(camera_info_options, move |msg: CameraInfo| {
                Self::camera_info_callback(
                    msg,
                    Arc::clone(&detector_state_camera_info),
                    Arc::clone(&camera_calibration_camera_info),
                    &config_file_for_callback,
                );
            })?;
        // Replace temporary subscription with actual image subscription
        std::mem::drop(temp_image_subscription); // Drop the temporary subscription

        let detector_state_image = Arc::clone(&detector_state);
        let camera_calibration_image = Arc::clone(&camera_calibration);
        let detection_publisher_image = detection_publisher.clone();
        let overlay_publisher_image = overlay_publisher.clone();
        let aruco_pattern_image = Arc::clone(&aruco_pattern);

        let mut image_options = SubscriptionOptions::new(image_topic);
        image_options.qos = qos_profile;
        let image_subscription =
            node.create_subscription(image_options, move |msg: ImageMsg| {
                Self::image_callback(
                    msg,
                    Arc::clone(&detector_state_image),
                    Arc::clone(&camera_calibration_image),
                    &detection_publisher_image,
                    &overlay_publisher_image,
                    Arc::clone(&aruco_pattern_image),
                    debug_overlay_enabled,
                );
            })?;

        log_info!(
            LOGGER_NAME,
            "Successfully subscribed to image and camera_info topics"
        );
        log_info!(
            LOGGER_NAME,
            "Will publish ArUco detections to /aruco_detections"
        );

        // Create the simplified node instance
        let node_instance = Self {
            _camera_info_subscription: camera_info_subscription,
            _image_subscription: image_subscription,
            detection_publisher,
            overlay_publisher,
            detector_state,
            camera_calibration,
            aruco_pattern,
            debug_overlay_enabled,
        };

        Ok(node_instance)
    }

    /// Handle camera info updates
    fn camera_info_callback(
        camera_info: CameraInfo,
        detector_state: Arc<ArcSwap<Option<Arc<ArucoDetector>>>>,
        camera_calibration: Arc<ArcSwap<Option<CameraCalibration>>>,
        aruco_config_file: &str,
    ) {
        // Check if detector is already initialized (lock-free read)
        let already_initialized = detector_state.load().is_some();

        let aruco_pattern = match Self::load_aruco_pattern(aruco_config_file) {
            Ok(pattern) => pattern,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to load ArUco pattern: {e}");
                return;
            }
        };

        // Store camera calibration data for undistortion (only log once)
        match Self::store_camera_calibration(&camera_info, Arc::clone(&camera_calibration)) {
            Ok(_) => {
                if !already_initialized {
                    log_info!(LOGGER_NAME, "Camera calibration stored for undistortion");
                }
            }
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to store camera calibration: {e}");
            }
        }

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

        // Store detector using ArcSwap
        detector_state.store(Arc::new(Some(Arc::new(detector))));

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

    /// Store camera calibration data for undistortion
    fn store_camera_calibration(
        camera_info: &CameraInfo,
        camera_calibration: Arc<ArcSwap<Option<CameraCalibration>>>,
    ) -> Result<()> {
        // Extract camera matrix (3x3) as array
        let camera_matrix: [f64; 9] = camera_info.k[0..9]
            .try_into()
            .map_err(|_| anyhow!("Camera matrix must have 9 elements"))?;

        // Extract distortion coefficients as vector
        let distortion_coeffs: Vec<f64> = camera_info.d.to_vec();

        let calibration = CameraCalibration {
            camera_matrix,
            distortion_coeffs,
            width: camera_info.width as i32,
            height: camera_info.height as i32,
        };

        // Store the calibration using ArcSwap
        camera_calibration.store(Arc::new(Some(calibration)));
        Ok(())
    }

    /// Undistort image using camera calibration
    fn undistort_image(image: &Mat, calibration: &CameraCalibration) -> Result<Mat> {
        // Convert camera matrix from array to Mat (CV_64FC1 - double precision)
        let camera_matrix = Mat::from_slice_2d(&[
            &calibration.camera_matrix[0..3],
            &calibration.camera_matrix[3..6],
            &calibration.camera_matrix[6..9],
        ])?;

        // Convert distortion coefficients from Vec to Mat (CV_64FC1 - double precision)
        // OpenCV expects distortion coefficients as a 1xN or Nx1 matrix
        let distortion_coeffs = if calibration.distortion_coeffs.is_empty() {
            // No distortion - use zeros
            Mat::zeros(1, 5, opencv::core::CV_64FC1)?.to_mat()?
        } else {
            // Create 1xN matrix from vector (row vector)
            Mat::from_slice_rows_cols(
                &calibration.distortion_coeffs,
                1,
                calibration.distortion_coeffs.len(),
            )?
        };

        let mut undistorted = Mat::default();

        calib3d::undistort(
            image,
            &mut undistorted,
            &camera_matrix,
            &distortion_coeffs,
            &opencv::core::no_array(),
        )?;

        Ok(undistorted)
    }

    /// Process the incoming image with undistortion
    fn process_image(
        msg: &ImageMsg,
        detector: &ArucoDetector,
        camera_calibration: Option<&CameraCalibration>,
    ) -> Result<(aruco_locator::DetectionResult, Mat)> {
        // Validate image encoding and convert to OpenCV Mat
        let mat = Self::ros_image_to_opencv_mat(msg)?;

        // Camera calibration must be available to proceed
        let calibration = camera_calibration.ok_or_else(|| {
            anyhow!("process_image called without camera calibration - this is a bug")
        })?;

        // Undistort image - fail if undistortion fails
        let processed_mat = Self::undistort_image(&mat, calibration)?;
        log_debug!(LOGGER_NAME, "Image undistorted for ArUco detection");

        // Detect ArUco markers on processed (undistorted) image
        let detection_result = detector.detect_markers(&processed_mat)?;

        Ok((detection_result, processed_mat))
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

        // Convert all formats to BGR for ArUco detection compatibility
        // ArUco detector expects CV_8UC1 or CV_8UC3 (grayscale or BGR)
        let processed_mat = match msg.encoding.as_str() {
            "bgr8" => mat, // Already in correct format
            "rgb8" => {
                let mut bgr_mat = Mat::default();
                imgproc::cvt_color(&mat, &mut bgr_mat, imgproc::COLOR_RGB2BGR, 0)?;
                bgr_mat
            }
            "bgra8" => {
                let mut bgr_mat = Mat::default();
                imgproc::cvt_color(&mat, &mut bgr_mat, imgproc::COLOR_BGRA2BGR, 0)?;
                bgr_mat
            }
            "rgba8" => {
                let mut bgr_mat = Mat::default();
                imgproc::cvt_color(&mat, &mut bgr_mat, imgproc::COLOR_RGBA2BGR, 0)?;
                bgr_mat
            }
            "mono8" => mat, // Grayscale is already compatible with ArUco detector
            _ => mat,       // Should not reach here due to earlier validation
        };

        Ok(processed_mat)
    }

    /// Process incoming image messages and publish detection results
    fn image_callback(
        msg: ImageMsg,
        detector_state: Arc<ArcSwap<Option<Arc<ArucoDetector>>>>,
        camera_calibration: Arc<ArcSwap<Option<CameraCalibration>>>,
        detection_publisher: &Publisher<Detection2DArray>,
        overlay_publisher: &Option<Publisher<ImageMsg>>,
        aruco_pattern: Arc<aruco_config::MultiArucoPattern>,
        debug_overlay_enabled: bool,
    ) {
        // Get detector (lock-free read with ArcSwap)
        let detector = {
            let state = detector_state.load();
            match state.as_ref() {
                Some(detector) => Arc::clone(detector),
                None => {
                    // Detector not initialized yet, skip this frame
                    return;
                }
            }
        };

        // Get camera calibration for undistortion (lock-free read with ArcSwap)
        let calibration = {
            let calib = camera_calibration.load();
            (**calib).clone()
        };

        // Wait for camera_info before processing images
        let calibration = match calibration {
            Some(calib) => calib,
            None => {
                // Camera info not received yet, skip this frame
                // Log occasionally to avoid spam
                static mut NO_CAMERA_INFO_COUNT: u32 = 0;
                unsafe {
                    NO_CAMERA_INFO_COUNT += 1;
                    if NO_CAMERA_INFO_COUNT % 60 == 1 {
                        // Log every 60th frame to indicate waiting for camera_info
                        log_warn!(
                            LOGGER_NAME,
                            "Waiting for camera_info before processing images (suppressing repeated messages)"
                        );
                    }
                }
                return;
            }
        };

        match Self::process_image(&msg, &detector, Some(&calibration)) {
            Ok((detection_result, processed_image)) => {
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
                if let Err(e) = detection_publisher.publish(&detection_msg) {
                    log_error!(LOGGER_NAME, "Failed to publish detection result: {e}");
                }

                // Create and publish overlay image if debug is enabled
                if debug_overlay_enabled {
                    Self::publish_debug_overlay(
                        &msg,
                        &processed_image,
                        &detection_result,
                        overlay_publisher,
                        &aruco_pattern,
                    );
                }
            }
            Err(e) => {
                log_error!(LOGGER_NAME, "ArUco detection failed: {e}");
            }
        }
    }

    /// Publish debug overlay image with ArUco detection visualizations
    fn publish_debug_overlay(
        original_image: &ImageMsg,
        processed_image: &Mat,
        detection_result: &aruco_locator::DetectionResult,
        overlay_publisher: &Option<Publisher<ImageMsg>>,
        aruco_pattern: &aruco_config::MultiArucoPattern,
    ) {
        if let Some(overlay_pub) = overlay_publisher {
            match Self::create_overlay_image(
                original_image,
                processed_image,
                detection_result,
                aruco_pattern,
            ) {
                Ok(overlay_image) => {
                    if let Err(e) = overlay_pub.publish(&overlay_image) {
                        log_error!(LOGGER_NAME, "Failed to publish overlay image: {e}");
                    }
                }
                Err(e) => {
                    log_error!(LOGGER_NAME, "Failed to create overlay image: {e}");
                }
            }
        }
    }

    /// Create an overlay image with ArUco detection visualizations
    fn create_overlay_image(
        original_image: &ImageMsg,
        processed_image: &Mat,
        detection_result: &aruco_locator::DetectionResult,
        aruco_pattern: &aruco_config::MultiArucoPattern,
    ) -> Result<ImageMsg> {
        // Use the processed (potentially undistorted) image for overlay
        let mut cv_image = processed_image.clone();

        // Generate color map from ArUco pattern config
        let predefined_colors = [
            Scalar::new(0.0, 0.0, 255.0, 0.0),   // Red
            Scalar::new(0.0, 255.0, 0.0, 0.0),   // Green
            Scalar::new(255.0, 0.0, 0.0, 0.0),   // Blue
            Scalar::new(0.0, 255.0, 255.0, 0.0), // Cyan
            Scalar::new(255.0, 0.0, 255.0, 0.0), // Magenta
            Scalar::new(255.0, 255.0, 0.0, 0.0), // Yellow
        ];

        let color_map: std::collections::HashMap<String, Scalar> = aruco_pattern
            .marker_ids
            .iter()
            .enumerate()
            .map(|(i, &id)| {
                let color = predefined_colors[i % predefined_colors.len()];
                (id.to_string(), color)
            })
            .collect();

        let default_color = Scalar::new(255.0, 0.0, 255.0, 0.0); // Magenta for unknown IDs

        if detection_result.markers_found {
            // Draw detected markers using actual corner points
            for (i, &marker_id) in detection_result.marker_ids.iter().enumerate() {
                if let Some(marker) = detection_result.markers.get(i) {
                    // Get marker ID string
                    let marker_id_str = marker_id.to_string();

                    // Choose color based on marker ID
                    let color = color_map
                        .get(&marker_id_str)
                        .copied()
                        .unwrap_or(default_color);

                    // Draw marker border using actual corner points (TL, TR, BR, BL)
                    let corners = &marker.corners;
                    let mut pts = opencv::core::Vector::<opencv::core::Point>::new();
                    for corner in corners {
                        pts.push(opencv::core::Point::new(corner.x as i32, corner.y as i32));
                    }

                    // Close the polygon by adding the first point again
                    if !corners.is_empty() {
                        pts.push(opencv::core::Point::new(
                            corners[0].x as i32,
                            corners[0].y as i32,
                        ));
                    }

                    // Draw the marker border as a polyline
                    opencv::imgproc::polylines(
                        &mut cv_image,
                        &opencv::core::Vector::<opencv::core::Vector<opencv::core::Point>>::from_iter(vec![pts]),
                        false, // isClosed=false (we manually closed it)
                        color,
                        3, // thickness
                        LINE_8,
                        0,
                    )?;

                    // Calculate centroid for label positioning
                    let centroid_x = (corners.iter().map(|p| p.x).sum::<f32>() / 4.0) as i32;
                    let centroid_y = (corners.iter().map(|p| p.y).sum::<f32>() / 4.0) as i32;

                    // Find topmost corner for label position
                    let min_y = corners.iter().map(|p| p.y as i32).min().unwrap_or(0);
                    let label_anchor_x = centroid_x;
                    let label_anchor_y = min_y;

                    // Draw marker ID label with background
                    let label = format!("ArUco {}", marker_id);
                    let font_face = FONT_HERSHEY_SIMPLEX;
                    let font_scale = 0.8;
                    let thickness = 2;

                    let mut baseline = 0;
                    let text_size = opencv::imgproc::get_text_size(
                        &label,
                        font_face,
                        font_scale,
                        thickness,
                        &mut baseline,
                    )?;

                    // Position label above the marker if possible
                    let label_y = if label_anchor_y - 10 > text_size.height {
                        label_anchor_y - 10
                    } else {
                        centroid_y + text_size.height + 10
                    };

                    let label_x = label_anchor_x - text_size.width / 2;

                    // Draw label background
                    opencv::imgproc::rectangle(
                        &mut cv_image,
                        opencv::core::Rect::new(
                            label_x,
                            label_y - text_size.height - 5,
                            text_size.width + 10,
                            text_size.height + 10,
                        ),
                        color,
                        -1,
                        LINE_8,
                        0,
                    )?;

                    // Draw label text
                    opencv::imgproc::put_text(
                        &mut cv_image,
                        &label,
                        opencv::core::Point::new(label_x + 5, label_y),
                        font_face,
                        font_scale,
                        Scalar::new(255.0, 255.0, 255.0, 0.0), // White text
                        thickness,
                        LINE_8,
                        false,
                    )?;

                    // Draw corner points
                    let corner_radius = 6;
                    for corner in corners {
                        opencv::imgproc::circle(
                            &mut cv_image,
                            opencv::core::Point::new(corner.x as i32, corner.y as i32),
                            corner_radius,
                            color,
                            -1,
                            LINE_8,
                            0,
                        )?;
                    }

                    // Draw center point
                    opencv::imgproc::circle(
                        &mut cv_image,
                        opencv::core::Point::new(centroid_x, centroid_y),
                        corner_radius,
                        Scalar::new(255.0, 255.0, 0.0, 0.0), // Yellow center
                        -1,
                        LINE_8,
                        0,
                    )?;
                }
            }

            // Add detection count overlay
            let detection_text = format!("ArUco Markers: {}", detection_result.markers.len());
            opencv::imgproc::put_text(
                &mut cv_image,
                &detection_text,
                opencv::core::Point::new(10, 40),
                FONT_HERSHEY_SIMPLEX,
                1.2,
                Scalar::new(0.0, 255.0, 0.0, 0.0), // Green text
                3,
                LINE_8,
                false,
            )?;
        } else {
            // No detections overlay
            opencv::imgproc::put_text(
                &mut cv_image,
                "No ArUco markers detected",
                opencv::core::Point::new(10, 40),
                FONT_HERSHEY_SIMPLEX,
                1.2,
                Scalar::new(0.0, 0.0, 255.0, 0.0), // Red text
                3,
                LINE_8,
                false,
            )?;
        }

        // Convert OpenCV Mat back to ROS image
        Self::opencv_mat_to_ros_image(&cv_image, original_image)
    }

    /// Convert OpenCV Mat to ROS Image message
    fn opencv_mat_to_ros_image(mat: &Mat, original_image: &ImageMsg) -> Result<ImageMsg> {
        // Ensure the image is in BGR format
        let bgr_mat = if mat.channels() == 3 {
            mat.clone()
        } else if mat.channels() == 1 {
            let mut bgr_mat = Mat::default();
            imgproc::cvt_color(mat, &mut bgr_mat, imgproc::COLOR_GRAY2BGR, 0)?;
            bgr_mat
        } else {
            bail!("Unsupported number of channels: {}", mat.channels());
        };

        // Get image data
        let height = bgr_mat.rows() as u32;
        let width = bgr_mat.cols() as u32;
        let step = (width * 3) as u32; // 3 bytes per pixel for BGR

        // Copy data from OpenCV Mat
        let data_size = (height * step) as usize;
        let mut data = vec![0u8; data_size];

        unsafe {
            let mat_data = bgr_mat.ptr(0)?;
            std::ptr::copy_nonoverlapping(mat_data, data.as_mut_ptr(), data_size);
        }

        Ok(ImageMsg {
            header: original_image.header.clone(),
            height,
            width,
            encoding: "bgr8".to_string(),
            is_bigendian: 0,
            step,
            data,
        })
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
