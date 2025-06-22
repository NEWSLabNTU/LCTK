use aruco_config::MultiArucoPattern;
use eyre::{eyre, Context as EyreContext, Result};
use geometry_msgs::msg::{PoseStamped, TransformStamped};
use hollow_board_config::BoardModel;
use hollow_board_detector::{Config as DetectorConfig, Detector};
use nalgebra;
use rclrs::{
    log_error, log_info, log_warn, Context, CreateBasicExecutor, InitOptions, Node, Publisher,
    RclrsErrorFilter, SpinOptions, Subscription, ToLogParams,
};
use sensor_msgs::msg::PointCloud2;
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    time::Duration,
};
use vision_msgs::msg::Detection3DArray;
use visualization_msgs::msg::{Marker, MarkerArray};

mod types;
use types::*;

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

/// State shared between callbacks
struct MultiWaysideState {
    /// Detection buffer for LiDAR 1
    lidar1_detections: Mutex<VecDeque<TimestampedDetection>>,
    /// Detection buffer for LiDAR 2
    lidar2_detections: Mutex<VecDeque<TimestampedDetection>>,
    /// Current calibration result
    calibration_result: Mutex<Option<CalibrationResult>>,
    /// Manual pose adjustments for LiDAR 1
    lidar1_pose_adjustment: Mutex<Option<nalgebra::Isometry3<f64>>>,
    /// Manual pose adjustments for LiDAR 2
    lidar2_pose_adjustment: Mutex<Option<nalgebra::Isometry3<f64>>>,
    /// Board detector
    detector: Detector,
    /// Board model
    board_model: BoardModel,
    /// Maximum queue size for detection buffers
    max_queue_size: usize,
    /// Synchronization tolerance
    sync_tolerance: Duration,
    /// Whether boards face the same direction
    same_face_mode: bool,
    /// Whether to apply VLP16 bug fix
    apply_bug_fix: bool,
}

pub struct MultiWaysideNode {
    _node: Node,
    _lidar1_sub: Arc<Subscription<PointCloud2>>,
    _lidar2_sub: Arc<Subscription<PointCloud2>>,
    _lidar1_pose_sub: Arc<Subscription<PoseStamped>>,
    _lidar2_pose_sub: Arc<Subscription<PoseStamped>>,
    _lidar1_detection_pub: Arc<Publisher<Detection3DArray>>,
    _lidar2_detection_pub: Arc<Publisher<Detection3DArray>>,
    _lidar1_filtered_pub: Arc<Publisher<PointCloud2>>,
    _lidar2_filtered_pub: Arc<Publisher<PointCloud2>>,
    _adjustment_marker_pub: Arc<Publisher<MarkerArray>>,
    _transform_pub: Arc<Publisher<TransformStamped>>,
    _marker_pub: Arc<Publisher<MarkerArray>>,
    state: Arc<MultiWaysideState>,
}

impl MultiWaysideNode {
    pub fn new(node: &Node) -> Result<Self> {
        // Declare and load parameters
        let board_config_file =
            Self::get_parameter_string(node, "board_config_file", "config/hollow_board.yaml")?;
        let detector_config_file =
            Self::get_parameter_string(node, "detector_config_file", "config/detector.yaml")?;
        let aruco_pattern_file =
            Self::get_parameter_string(node, "aruco_pattern_file", "config/aruco_pattern.json5")?;
        let same_face_mode = Self::get_parameter_bool(node, "same_face_mode", true)?;
        let apply_bug_fix = Self::get_parameter_bool(node, "apply_bug_fix", false)?;
        let max_queue_size = Self::get_parameter_i64(node, "max_queue_size", 100)?;
        let sync_tolerance_ms = Self::get_parameter_i64(node, "sync_tolerance_ms", 100)?;

        // Validate all parameters
        Self::validate_parameters(
            max_queue_size,
            sync_tolerance_ms,
            &board_config_file,
            &detector_config_file,
            &aruco_pattern_file,
        )?;

        // Load configurations
        let board_model = Self::load_board_model(&board_config_file)?;
        let detector_config = Self::load_detector_config(&detector_config_file)?;
        let aruco_pattern = Self::load_aruco_pattern(&aruco_pattern_file)?;

        // Create board detector
        let detector = Detector::new(detector_config, aruco_pattern);

        // Create shared state
        let lidar1_detections = Mutex::new(VecDeque::new());
        let lidar2_detections = Mutex::new(VecDeque::new());
        let calibration_result = Mutex::new(None);
        let lidar1_pose_adjustment = Mutex::new(None);
        let lidar2_pose_adjustment = Mutex::new(None);
        let max_queue_size = max_queue_size as usize;
        let sync_tolerance = Duration::from_millis(sync_tolerance_ms as u64);

        let state = Arc::new(MultiWaysideState {
            lidar1_detections,
            lidar2_detections,
            calibration_result,
            lidar1_pose_adjustment,
            lidar2_pose_adjustment,
            detector,
            board_model,
            max_queue_size,
            sync_tolerance,
            same_face_mode,
            apply_bug_fix,
        });

        // Create publishers
        let lidar1_detection_pub = Arc::new(node.create_publisher("/lidar1/board_detection")?);
        let lidar2_detection_pub = Arc::new(node.create_publisher("/lidar2/board_detection")?);
        let lidar1_filtered_pub = Arc::new(node.create_publisher("/lidar1/points_filtered")?);
        let lidar2_filtered_pub = Arc::new(node.create_publisher("/lidar2/points_filtered")?);
        let adjustment_marker_pub = Arc::new(node.create_publisher("/adjustment_markers")?);
        let transform_pub = Arc::new(node.create_publisher("/calibration_transform")?);
        let marker_pub = Arc::new(node.create_publisher("/calibration_markers")?);

        // Create subscribers with callbacks
        let lidar1_sub = {
            let state = Arc::clone(&state);
            let detection_pub = Arc::clone(&lidar1_detection_pub);
            let filtered_pub = Arc::clone(&lidar1_filtered_pub);
            let marker_pub = Arc::clone(&marker_pub);

            Arc::new(node.create_subscription::<PointCloud2, _>(
                "/lidar1/points",
                move |msg: PointCloud2| {
                    Self::process_pointcloud(
                        msg,
                        1,
                        &state,
                        &detection_pub,
                        &filtered_pub,
                        &marker_pub,
                    );
                },
            )?)
        };

        let lidar2_sub = {
            let state = Arc::clone(&state);
            let detection_pub = Arc::clone(&lidar2_detection_pub);
            let filtered_pub = Arc::clone(&lidar2_filtered_pub);
            let marker_pub = Arc::clone(&marker_pub);

            Arc::new(node.create_subscription::<PointCloud2, _>(
                "/lidar2/points",
                move |msg: PointCloud2| {
                    Self::process_pointcloud(
                        msg,
                        2,
                        &state,
                        &detection_pub,
                        &filtered_pub,
                        &marker_pub,
                    );
                },
            )?)
        };

        // Create pose adjustment subscribers
        let lidar1_pose_sub = {
            let state = Arc::clone(&state);
            let adjustment_marker_pub = Arc::clone(&adjustment_marker_pub);

            Arc::new(node.create_subscription::<PoseStamped, _>(
                "/lidar1/board_pose_adjustment",
                move |msg: PoseStamped| {
                    Self::process_pose_adjustment(msg, 1, &state, &adjustment_marker_pub);
                },
            )?)
        };

        let lidar2_pose_sub = {
            let state = Arc::clone(&state);
            let adjustment_marker_pub = Arc::clone(&adjustment_marker_pub);

            Arc::new(node.create_subscription::<PoseStamped, _>(
                "/lidar2/board_pose_adjustment",
                move |msg: PoseStamped| {
                    Self::process_pose_adjustment(msg, 2, &state, &adjustment_marker_pub);
                },
            )?)
        };

        log_info!(LOGGER_NAME, "MultiWaysideNode initialized successfully");

        let _node = node.clone();

        Ok(Self {
            _node,
            _lidar1_sub: lidar1_sub,
            _lidar2_sub: lidar2_sub,
            _lidar1_pose_sub: lidar1_pose_sub,
            _lidar2_pose_sub: lidar2_pose_sub,
            _lidar1_detection_pub: lidar1_detection_pub,
            _lidar2_detection_pub: lidar2_detection_pub,
            _lidar1_filtered_pub: lidar1_filtered_pub,
            _lidar2_filtered_pub: lidar2_filtered_pub,
            _adjustment_marker_pub: adjustment_marker_pub,
            _transform_pub: transform_pub,
            _marker_pub: marker_pub,
            state,
        })
    }

    /// Process incoming point cloud and detect board
    fn process_pointcloud(
        msg: PointCloud2,
        lidar_id: u8,
        state: &Arc<MultiWaysideState>,
        detection_pub: &Arc<Publisher<Detection3DArray>>,
        filtered_pub: &Arc<Publisher<PointCloud2>>,
        marker_pub: &Arc<Publisher<MarkerArray>>,
    ) {
        // Parse point cloud
        let points = match parse_pointcloud2(&msg) {
            Ok(points) => points,
            Err(e) => {
                log_error!(LOGGER_NAME, "Failed to parse point cloud: {}", e);
                return;
            }
        };

        // Convert to nalgebra points for the detector
        let na_points: Vec<nalgebra::Point3<f64>> = points
            .iter()
            .map(|p| nalgebra::Point3::new(p.x as f64, p.y as f64, p.z as f64))
            .collect();

        // Create filtered point cloud message for visualization
        // For now, just republish all points with different color
        let mut filtered_msg = msg.clone();
        // Modify header to indicate filtered data
        filtered_msg.header.frame_id = format!("{}_filtered", filtered_msg.header.frame_id);

        // Publish filtered point cloud
        if let Err(e) = filtered_pub.publish(&filtered_msg) {
            log_error!(LOGGER_NAME, "Failed to publish filtered point cloud: {}", e);
        }

        // Detect board
        match state.detector.detect(&na_points) {
            Ok(Some(detection)) => {
                log_info!(
                    LOGGER_NAME,
                    "Board detected in LiDAR {}: pose translation: {:?}",
                    lidar_id,
                    detection.board_model.pose.translation.vector
                );

                // Convert to ROS message
                let det_msg = create_detection_message(&detection, &msg.header);

                // Publish detection
                if let Err(e) = detection_pub.publish(&det_msg) {
                    log_error!(LOGGER_NAME, "Failed to publish detection: {}", e);
                }

                // Store detection
                let timestamp =
                    msg.header.stamp.sec as u64 * 1_000_000_000 + msg.header.stamp.nanosec as u64;
                let detection_clone = detection.clone();
                let header = msg.header.clone();
                let timestamped = TimestampedDetection {
                    timestamp,
                    detection: detection_clone,
                    header,
                };

                match lidar_id {
                    1 => {
                        if let Ok(mut detections) = state.lidar1_detections.lock() {
                            detections.push_back(timestamped);
                            if detections.len() > state.max_queue_size {
                                detections.pop_front();
                            }
                        }
                    }
                    2 => {
                        if let Ok(mut detections) = state.lidar2_detections.lock() {
                            detections.push_back(timestamped);
                            if detections.len() > state.max_queue_size {
                                detections.pop_front();
                            }
                        }
                    }
                    _ => {}
                }

                // Publish visualization markers
                let markers = create_board_markers(&detection, lidar_id, &msg.header);
                if let Err(e) = marker_pub.publish(&markers) {
                    log_error!(LOGGER_NAME, "Failed to publish markers: {}", e);
                }
            }
            Ok(None) => {
                log_warn!(LOGGER_NAME, "No board detected in LiDAR {}", lidar_id);
            }
            Err(e) => {
                log_error!(LOGGER_NAME, "Board detection failed: {}", e);
            }
        }
    }

    /// Process manual pose adjustment
    fn process_pose_adjustment(
        msg: PoseStamped,
        lidar_id: u8,
        state: &Arc<MultiWaysideState>,
        adjustment_marker_pub: &Arc<Publisher<MarkerArray>>,
    ) {
        log_info!(
            LOGGER_NAME,
            "Received pose adjustment for LiDAR {}",
            lidar_id
        );

        // Convert ROS pose to nalgebra isometry
        let position = nalgebra::Vector3::new(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        );
        let quaternion = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ));

        // Apply constraints
        let constrained_position = Self::apply_position_constraints(position);
        let constrained_quaternion = Self::apply_rotation_constraints(quaternion);

        let adjustment =
            nalgebra::Isometry3::from_parts(constrained_position.into(), constrained_quaternion);

        if constrained_position != position || constrained_quaternion != quaternion {
            log_warn!(
                LOGGER_NAME,
                "Pose adjustment was constrained for LiDAR {}",
                lidar_id
            );
        }

        // Store the adjustment
        match lidar_id {
            1 => {
                if let Ok(mut adj) = state.lidar1_pose_adjustment.lock() {
                    *adj = Some(adjustment);
                    log_info!(LOGGER_NAME, "Applied pose adjustment for LiDAR 1");
                }
            }
            2 => {
                if let Ok(mut adj) = state.lidar2_pose_adjustment.lock() {
                    *adj = Some(adjustment);
                    log_info!(LOGGER_NAME, "Applied pose adjustment for LiDAR 2");
                }
            }
            _ => {
                log_warn!(
                    LOGGER_NAME,
                    "Invalid LiDAR ID for pose adjustment: {}",
                    lidar_id
                );
                return;
            }
        }

        // Create visual feedback markers
        let markers = Self::create_adjustment_markers(&adjustment, lidar_id, &msg.header);
        if let Err(e) = adjustment_marker_pub.publish(&markers) {
            log_error!(LOGGER_NAME, "Failed to publish adjustment markers: {}", e);
        }
    }

    /// Create markers to visualize manual adjustments
    fn create_adjustment_markers(
        adjustment: &nalgebra::Isometry3<f64>,
        lidar_id: u8,
        header: &std_msgs::msg::Header,
    ) -> MarkerArray {
        // Position from adjustment
        let pos = adjustment.translation.vector;
        let position = geometry_msgs::msg::Point {
            x: pos.x,
            y: pos.y,
            z: pos.z,
        };

        // Orientation from adjustment
        let q = adjustment.rotation;
        let orientation = geometry_msgs::msg::Quaternion {
            x: q.i,
            y: q.j,
            z: q.k,
            w: q.w,
        };

        // Create pose for frame marker
        let pose = geometry_msgs::msg::Pose {
            position,
            orientation,
        };

        // Scale
        let scale = geometry_msgs::msg::Vector3 {
            x: 0.4, // Slightly larger than detection markers
            y: 0.06,
            z: 0.06,
        };

        // Color (bright green for adjustments)
        let color = std_msgs::msg::ColorRGBA {
            r: 0.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        };

        // Lifetime
        let lifetime = builtin_interfaces::msg::Duration {
            sec: 5, // 5 second lifetime
            nanosec: 0,
        };

        // Create frame marker
        let header_clone = header.clone();
        let ns = format!("adjustment_lidar{}", lidar_id);
        let id = 0;
        let type_ = 0; // ARROW
        let action = 0; // ADD

        let frame_marker = Marker {
            header: header_clone.clone(),
            ns,
            id,
            type_,
            action,
            pose,
            scale,
            color: color.clone(),
            lifetime: lifetime.clone(),
            ..Default::default()
        };

        // Text marker position (above adjustment)
        let text_position = geometry_msgs::msg::Point {
            x: pos.x,
            y: pos.y,
            z: pos.z + 0.7,
        };

        // Text orientation (identity)
        let text_orientation = geometry_msgs::msg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };

        // Text pose
        let text_pose = geometry_msgs::msg::Pose {
            position: text_position,
            orientation: text_orientation,
        };

        // Text scale
        let text_scale = geometry_msgs::msg::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.15,
        };

        // Create text marker
        let ns = format!("adjustment_text_lidar{}", lidar_id);
        let id = 1;
        let type_ = 9; // TEXT_VIEW_FACING
        let text = format!("LiDAR {} Adjustment", lidar_id);

        let text_marker = Marker {
            header: header_clone,
            ns,
            id,
            type_,
            action,
            pose: text_pose,
            scale: text_scale,
            color,
            lifetime,
            text,
            ..Default::default()
        };

        // Create marker array
        let markers = vec![frame_marker, text_marker];
        MarkerArray { markers }
    }

    /// Apply position constraints to limit translation adjustments
    fn apply_position_constraints(position: nalgebra::Vector3<f64>) -> nalgebra::Vector3<f64> {
        const MAX_TRANSLATION: f64 = 2.0; // Maximum 2 meters adjustment in any direction

        nalgebra::Vector3::new(
            position.x.clamp(-MAX_TRANSLATION, MAX_TRANSLATION),
            position.y.clamp(-MAX_TRANSLATION, MAX_TRANSLATION),
            position.z.clamp(-MAX_TRANSLATION, MAX_TRANSLATION),
        )
    }

    /// Apply rotation constraints to limit rotation adjustments  
    fn apply_rotation_constraints(
        quaternion: nalgebra::UnitQuaternion<f64>,
    ) -> nalgebra::UnitQuaternion<f64> {
        // Extract Euler angles
        let (roll, pitch, yaw) = quaternion.euler_angles();

        const MAX_ROTATION: f64 = std::f64::consts::PI / 6.0; // Maximum 30 degrees in any axis

        // Constrain each rotation axis
        let constrained_roll = roll.clamp(-MAX_ROTATION, MAX_ROTATION);
        let constrained_pitch = pitch.clamp(-MAX_ROTATION, MAX_ROTATION);
        let constrained_yaw = yaw.clamp(-MAX_ROTATION, MAX_ROTATION);

        // Convert back to quaternion
        nalgebra::UnitQuaternion::from_euler_angles(
            constrained_roll,
            constrained_pitch,
            constrained_yaw,
        )
    }

    /// Load board model from file
    fn load_board_model(path: &str) -> Result<BoardModel> {
        let content = std::fs::read_to_string(path).wrap_err("Failed to read board config file")?;
        json5::from_str(&content).wrap_err("Failed to parse board config")
    }

    /// Load detector configuration from file
    fn load_detector_config(path: &str) -> Result<DetectorConfig> {
        let content =
            std::fs::read_to_string(path).wrap_err("Failed to read detector config file")?;
        json5::from_str(&content).wrap_err("Failed to parse detector config")
    }

    /// Load ArUco pattern from file
    fn load_aruco_pattern(path: &str) -> Result<MultiArucoPattern> {
        let content =
            std::fs::read_to_string(path).wrap_err("Failed to read ArUco pattern file")?;
        json5::from_str(&content).wrap_err("Failed to parse ArUco pattern")
    }

    /// Helper function to get string parameter with default value and validation
    fn get_parameter_string(_node: &Node, name: &str, default: &str) -> Result<String> {
        // For now, return default values since parameter API needs investigation
        // TODO: Implement proper parameter loading once rclrs parameter API is clarified
        let value = default.to_string();

        // Validate file paths exist for config files
        if name.ends_with("_file") {
            if !std::path::Path::new(&value).exists() {
                log_warn!(
                    LOGGER_NAME,
                    "Config file '{}' for parameter '{}' does not exist",
                    value,
                    name
                );
            }
        }

        log_info!(
            LOGGER_NAME,
            "Using default value for parameter '{}': {}",
            name,
            value
        );
        Ok(value)
    }

    /// Helper function to get bool parameter with default value
    fn get_parameter_bool(_node: &Node, name: &str, default: bool) -> Result<bool> {
        // For now, return default values since parameter API needs investigation
        // TODO: Implement proper parameter loading once rclrs parameter API is clarified
        log_info!(
            LOGGER_NAME,
            "Using default value for parameter '{}': {}",
            name,
            default
        );
        Ok(default)
    }

    /// Helper function to get i64 parameter with default value and validation
    fn get_parameter_i64(_node: &Node, name: &str, default: i64) -> Result<i64> {
        // For now, return default values since parameter API needs investigation
        // TODO: Implement proper parameter loading once rclrs parameter API is clarified
        let value = default;

        // Validate parameter ranges
        match name {
            "max_queue_size" => {
                if value < 1 || value > 10000 {
                    return Err(eyre!(
                        "Parameter '{}' must be between 1 and 10000, got {}",
                        name,
                        value
                    ));
                }
            }
            "sync_tolerance_ms" => {
                if value < 1 || value > 10000 {
                    return Err(eyre!(
                        "Parameter '{}' must be between 1ms and 10s, got {}ms",
                        name,
                        value
                    ));
                }
            }
            _ => {}
        }

        log_info!(
            LOGGER_NAME,
            "Using default value for parameter '{}': {}",
            name,
            value
        );
        Ok(value)
    }

    /// Validate all loaded parameters
    fn validate_parameters(
        max_queue_size: i64,
        sync_tolerance_ms: i64,
        board_config_file: &str,
        detector_config_file: &str,
        aruco_pattern_file: &str,
    ) -> Result<()> {
        // Check file accessibility
        let config_files = [
            (board_config_file, "board_config_file"),
            (detector_config_file, "detector_config_file"),
            (aruco_pattern_file, "aruco_pattern_file"),
        ];

        for (file_path, param_name) in &config_files {
            if !std::path::Path::new(file_path).exists() {
                return Err(eyre!(
                    "Configuration file '{}' specified by parameter '{}' does not exist",
                    file_path,
                    param_name
                ));
            }
        }

        // Validate parameter relationships
        if max_queue_size < 2 {
            log_warn!(
                LOGGER_NAME,
                "max_queue_size < 2 may cause synchronization issues"
            );
        }

        if sync_tolerance_ms > 1000 {
            log_warn!(
                LOGGER_NAME,
                "sync_tolerance_ms > 1000ms may result in poor synchronization"
            );
        }

        log_info!(LOGGER_NAME, "All parameters validated successfully");
        Ok(())
    }
}

fn main() -> Result<()> {
    // Initialize ROS 2
    let context = Context::new(std::env::args(), InitOptions::new())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(LOGGER_NAME)?;

    // Create the node (automatically creates all its components)
    let _multi_wayside_node = MultiWaysideNode::new(&node)?;
    log_info!(LOGGER_NAME, "MultiWaysideNode started successfully");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| eyre!("Failed to spin executor: {err}"))
}
