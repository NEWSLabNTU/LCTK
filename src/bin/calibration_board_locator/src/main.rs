mod bbox;

use crate::bbox::BBox;
use anyhow::{anyhow, Result};
use aruco_config::MultiArucoPattern;
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion, Vector3};
use hollow_board_detector::{
    Config as BoardDetectorConfig, Detection as BoardDetection, Detector as BoardDetector,
};
use nalgebra as na;
use once_cell::sync::Lazy;
use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, InitOptions, Node, Publisher,
    RclrsErrorFilter, SpinOptions, Subscription, ToLogParams,
};
use sensor_msgs::msg::PointCloud2;
use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use vision_msgs::msg::{BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

static DEFAULT_BOARD_DETECTOR_CONFIG: Lazy<BoardDetectorConfig> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/board_detector.json5"
    ));
    json5::from_str(&text).unwrap()
});

static DEFAULT_ARUCO_PATTERN_CONFIG: Lazy<MultiArucoPattern> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/aruco_pattern.json5"
    ));
    json5::from_str(&text).unwrap()
});

pub struct CalibrationBoardLocatorNode {
    _node: Node,
    _detection_publisher: Publisher<Detection3DArray>,
    _pointcloud_subscription: Subscription<PointCloud2>,
}

impl CalibrationBoardLocatorNode {
    pub fn new(node: Node) -> Result<Self> {
        // Declare parameters with defaults
        let board_detector_file_param: Arc<str> = node
            .declare_parameter("board_detector_file")
            .mandatory()?
            .get();
        let aruco_pattern_file_param: Arc<str> = node
            .declare_parameter("aruco_pattern_file")
            .mandatory()?
            .get();
        let bbox_file_param: Arc<str> = node.declare_parameter("bbox_file").mandatory()?.get();

        // Load configurations
        let board_detector_config = Self::load_board_detector_config(&board_detector_file_param)?;
        let aruco_pattern_config = Self::load_aruco_pattern_config(&aruco_pattern_file_param)?;

        let bbox = Self::load_bbox_config(&bbox_file_param)?;
        let bbox = Arc::new(Mutex::new(bbox));

        // Create detector
        let detector = Arc::new(BoardDetector::new(
            board_detector_config,
            aruco_pattern_config,
        ));

        // Create publisher for detections
        let detection_publisher =
            node.create_publisher::<Detection3DArray>("calibration_board_detections")?;
        let detection_publisher_shared = Arc::clone(&detection_publisher);

        // Create subscription to PointCloud2
        let pointcloud_subscription = node.create_subscription::<PointCloud2, _>(
            "input_pointcloud",
            move |msg: PointCloud2| {
                Self::pointcloud_callback(msg, &detector, &detection_publisher_shared, &bbox);
            },
        )?;

        log_info!(
            LOGGER_NAME,
            "Calibration board locator node initialized. Subscribing to: input_pointcloud, Publishing to: calibration_board_detections"
        );

        Ok(Self {
            _node: node,
            _detection_publisher: detection_publisher,
            _pointcloud_subscription: pointcloud_subscription,
            // bbox: bbox_shared,
        })
    }

    fn load_board_detector_config(file_path: &str) -> Result<BoardDetectorConfig> {
        if file_path.is_empty() {
            log_info!(LOGGER_NAME, "Using default board detector configuration");
            return Ok(DEFAULT_BOARD_DETECTOR_CONFIG.clone());
        }

        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_aruco_pattern_config(file_path: &str) -> Result<MultiArucoPattern> {
        if file_path.is_empty() {
            log_info!(LOGGER_NAME, "Using default ArUco pattern configuration");
            return Ok(DEFAULT_ARUCO_PATTERN_CONFIG.clone());
        }

        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_bbox_config(file_path: &str) -> Result<BBox> {
        if file_path.is_empty() {
            log_info!(LOGGER_NAME, "Using default bounding box configuration");
            return Ok(BBox::default());
        }

        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_json5_file<T>(path: &PathBuf) -> Result<T>
    where
        T: for<'de> serde::Deserialize<'de>,
    {
        let text = fs::read_to_string(path)?;
        let value: T = json5::from_str(&text)?;
        Ok(value)
    }

    fn pointcloud_callback(
        msg: PointCloud2,
        detector: &Arc<BoardDetector>,
        publisher: &Publisher<Detection3DArray>,
        bbox: &Arc<Mutex<BBox>>,
    ) {
        let result = Self::process_pointcloud(&msg, detector, bbox);

        let detection_array = match result {
            Ok(detection_array) => detection_array,
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to process point cloud: {e}");
                return;
            }
        };

        if let Err(e) = publisher.publish(detection_array) {
            log_warn!(LOGGER_NAME, "Failed to publish detection: {e}");
        }
    }

    fn process_pointcloud(
        msg: &PointCloud2,
        detector: &Arc<BoardDetector>,
        bbox: &Arc<Mutex<BBox>>,
    ) -> Result<Detection3DArray> {
        // Convert PointCloud2 to nalgebra points
        let points = Self::convert_pointcloud2_to_points(msg)?;

        // Filter points using bbox
        let bbox_guard = bbox
            .lock()
            .map_err(|e| anyhow!("Failed to lock bbox: {e}"))?;
        let active_points: Vec<_> = points
            .iter()
            .filter(|pt| bbox_guard.contains_point(pt))
            .cloned()
            .collect();
        drop(bbox_guard);

        if active_points.is_empty() {
            log_warn!(LOGGER_NAME, "No points within bounding box");
            return Ok(Detection3DArray {
                header: msg.header.clone(),
                detections: Vec::new(),
            });
        }

        // Detect calibration board
        let detection: Option<BoardDetection> = detector.detect(&active_points)?;

        let mut detections = Vec::new();
        if let Some(board_detection) = detection {
            let detection_3d =
                Self::convert_board_detection_to_detection3d(&board_detection, &msg.header)?;
            detections.push(detection_3d);
        }

        Ok(Detection3DArray {
            header: msg.header.clone(),
            detections,
        })
    }

    fn convert_pointcloud2_to_points(_msg: &PointCloud2) -> Result<Vec<na::Point3<f64>>> {
        // TODO: Implement PointCloud2 parsing
        // This is a simplified placeholder - you'll need to implement proper PointCloud2 parsing
        // based on the message fields and data layout

        // For now, return empty vector to allow compilation
        // In a real implementation, you would:
        // 1. Parse the fields to understand the data layout
        // 2. Extract x, y, z coordinates from the binary data
        // 3. Convert to nalgebra Point3 objects

        log_warn!(LOGGER_NAME, "PointCloud2 parsing not yet implemented");
        Ok(Vec::new())
    }

    fn convert_board_detection_to_detection3d(
        board_detection: &BoardDetection,
        header: &std_msgs::msg::Header,
    ) -> Result<Detection3D> {
        // Extract pose information from board detection
        let board_model = &board_detection.board_model;

        // Create pose from board model pose
        let pose = Pose {
            position: Point {
                x: board_model.pose.translation.x,
                y: board_model.pose.translation.y,
                z: board_model.pose.translation.z,
            },
            orientation: Quaternion {
                x: board_model.pose.rotation.i,
                y: board_model.pose.rotation.j,
                z: board_model.pose.rotation.k,
                w: board_model.pose.rotation.w,
            },
        };

        // Create bounding box
        // Note: You may need to adjust these dimensions based on your board specifications
        let bbox = BoundingBox3D {
            center: pose.clone(),
            size: Vector3 {
                x: 1.0, // Width in meters - adjust based on your board
                y: 1.0, // Height in meters - adjust based on your board
                z: 0.1, // Depth in meters - adjust based on your board
            },
        };

        // Create object hypothesis
        let hypothesis = ObjectHypothesisWithPose {
            hypothesis: vision_msgs::msg::ObjectHypothesis {
                class_id: "calibration_board".to_string(),
                score: 1.0, // Confidence score
            },
            pose: PoseWithCovariance {
                pose,
                covariance: [0.0; 36], // Zero covariance for now
            },
        };

        Ok(Detection3D {
            header: header.clone(),
            results: vec![hypothesis],
            bbox,
            id: "calibration_board".to_string(),
        })
    }
}

fn main() -> Result<()> {
    let context = Context::new(std::env::args(), InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("calibration_board_locator")?;
    let _calibration_board_locator_node = CalibrationBoardLocatorNode::new(node)?;

    log_info!(LOGGER_NAME, "Calibration board locator node started");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
