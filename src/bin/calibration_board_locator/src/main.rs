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
use rclrs::*;
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
    json5::from_str(text).unwrap()
});

static DEFAULT_ARUCO_PATTERN_CONFIG: Lazy<MultiArucoPattern> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/aruco_pattern.json5"
    ));
    json5::from_str(text).unwrap()
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
        let detection_publisher = node.create_publisher("calibration_board_detections")?;
        let detection_publisher_shared = Arc::clone(&detection_publisher);

        // Create subscription to PointCloud2
        let pointcloud_subscription =
            node.create_subscription("input_pointcloud", move |msg: PointCloud2| {
                Self::pointcloud_callback(msg, &detector, &detection_publisher_shared, &bbox);
            })?;

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

    fn convert_pointcloud2_to_points(msg: &PointCloud2) -> Result<Vec<na::Point3<f64>>> {
        // Find the x, y, z fields in the PointCloud2 message
        let x_field = msg
            .fields
            .iter()
            .find(|f| f.name == "x")
            .ok_or_else(|| anyhow!("Missing 'x' field in PointCloud2"))?;
        let y_field = msg
            .fields
            .iter()
            .find(|f| f.name == "y")
            .ok_or_else(|| anyhow!("Missing 'y' field in PointCloud2"))?;
        let z_field = msg
            .fields
            .iter()
            .find(|f| f.name == "z")
            .ok_or_else(|| anyhow!("Missing 'z' field in PointCloud2"))?;

        // Get field offsets
        let x_offset = x_field.offset as usize;
        let y_offset = y_field.offset as usize;
        let z_offset = z_field.offset as usize;

        // Parse points
        let point_step = msg.point_step as usize;
        let num_points = (msg.width * msg.height) as usize;

        let mut points = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let base_offset = i * point_step;

            // Ensure we don't read beyond the data buffer
            if base_offset + point_step > msg.data.len() {
                log_warn!(LOGGER_NAME, "Point data truncated at point {}", i);
                break;
            }

            // Read x, y, z as f32 (assuming FLOAT32 datatype = 7)
            let x = Self::read_f32_le(&msg.data, base_offset + x_offset)?;
            let y = Self::read_f32_le(&msg.data, base_offset + y_offset)?;
            let z = Self::read_f32_le(&msg.data, base_offset + z_offset)?;

            // Skip points with invalid coordinates (NaN or infinity)
            if x.is_finite() && y.is_finite() && z.is_finite() {
                points.push(na::Point3::new(x as f64, y as f64, z as f64));
            }
        }

        log_info!(
            LOGGER_NAME,
            "Parsed {} valid points from PointCloud2 message",
            points.len()
        );

        Ok(points)
    }

    fn read_f32_le(data: &[u8], offset: usize) -> Result<f32> {
        if offset + 4 > data.len() {
            return Err(anyhow!(
                "Buffer overflow when reading f32 at offset {}",
                offset
            ));
        }
        let bytes: [u8; 4] = [
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ];
        Ok(f32::from_le_bytes(bytes))
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
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("calibration_board_locator")?;
    let _calibration_board_locator_node = CalibrationBoardLocatorNode::new(node)?;

    log_info!(LOGGER_NAME, "Calibration board locator node started");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
