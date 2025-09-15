mod bbox;

use crate::bbox::BBox;
use anyhow::{anyhow, Result};
use aruco_config::MultiArucoPattern;
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion, Vector3};
use hollow_board_detector::{
    Config as BoardDetectorConfig, Detection as BoardDetection, Detector as BoardDetector,
    init_logging,
};
use nalgebra as na;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;
use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use vision_msgs::msg::{BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

// Config files are now mandatory parameters - no defaults

pub struct CalibrationBoardLocatorNode {
    _node: Node,
    _detection_publisher: Publisher<Detection3DArray>,
    _pointcloud_subscription: Subscription<PointCloud2>,
    // Debug publishers - only created when debug mode is enabled
    _debug_all_points_publisher: Option<Arc<Publisher<PointCloud2>>>,
    _debug_filtered_points_publisher: Option<Arc<Publisher<PointCloud2>>>,
    _debug_plane_inliers_publisher: Option<Arc<Publisher<PointCloud2>>>,
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

        // Debug mode parameter (optional, defaults to false)
        let debug_param = node
            .declare_parameter("enable_debug")
            .default(false)
            .optional()?;
        let enable_debug = debug_param.get().unwrap_or(false);

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

        // Create debug publishers if debug mode is enabled
        let debug_all_points_publisher = if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Debug mode enabled - creating debug publishers"
            );
            Some(Arc::new(node.create_publisher("debug/all_points")?))
        } else {
            None
        };
        let debug_all_points_shared = debug_all_points_publisher.clone();

        let debug_filtered_points_publisher = if enable_debug {
            Some(Arc::new(node.create_publisher("debug/filtered_points")?))
        } else {
            None
        };
        let debug_filtered_points_shared = debug_filtered_points_publisher.clone();

        let debug_plane_inliers_publisher = if enable_debug {
            Some(Arc::new(node.create_publisher("debug/plane_inliers")?))
        } else {
            None
        };
        let debug_plane_inliers_shared = debug_plane_inliers_publisher.clone();

        // Create subscription to PointCloud2
        let pointcloud_subscription =
            node.create_subscription("input_pointcloud", move |msg: PointCloud2| {
                Self::pointcloud_callback(
                    msg,
                    &detector,
                    &detection_publisher_shared,
                    &bbox,
                    &debug_all_points_shared,
                    &debug_filtered_points_shared,
                    &debug_plane_inliers_shared,
                );
            })?;

        if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Calibration board locator node initialized with debug mode"
            );
            log_info!(
                LOGGER_NAME,
                "Debug topics: debug/all_points, debug/filtered_points, debug/plane_inliers"
            );
        }

        Ok(Self {
            _node: node,
            _detection_publisher: detection_publisher,
            _pointcloud_subscription: pointcloud_subscription,
            _debug_all_points_publisher: debug_all_points_publisher,
            _debug_filtered_points_publisher: debug_filtered_points_publisher,
            _debug_plane_inliers_publisher: debug_plane_inliers_publisher,
        })
    }

    fn load_board_detector_config(file_path: &str) -> Result<BoardDetectorConfig> {
        if file_path.is_empty() {
            return Err(anyhow!(
                "board_detector_file parameter is required but was empty"
            ));
        }

        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_aruco_pattern_config(file_path: &str) -> Result<MultiArucoPattern> {
        if file_path.is_empty() {
            return Err(anyhow!(
                "aruco_pattern_file parameter is required but was empty"
            ));
        }

        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_bbox_config(file_path: &str) -> Result<BBox> {
        if file_path.is_empty() {
            return Err(anyhow!("bbox_file parameter is required but was empty"));
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
        debug_all_points_pub: &Option<Arc<Publisher<PointCloud2>>>,
        debug_filtered_points_pub: &Option<Arc<Publisher<PointCloud2>>>,
        debug_plane_inliers_pub: &Option<Arc<Publisher<PointCloud2>>>,
    ) {
        let result = Self::process_pointcloud(
            &msg,
            detector,
            bbox,
            debug_all_points_pub,
            debug_filtered_points_pub,
            debug_plane_inliers_pub,
        );

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
        debug_all_points_pub: &Option<Arc<Publisher<PointCloud2>>>,
        debug_filtered_points_pub: &Option<Arc<Publisher<PointCloud2>>>,
        debug_plane_inliers_pub: &Option<Arc<Publisher<PointCloud2>>>,
    ) -> Result<Detection3DArray> {
        // Convert PointCloud2 to nalgebra points
        let points = Self::convert_pointcloud2_to_points(msg)?;

        // Publish debug all points if enabled
        if let Some(pub_all) = debug_all_points_pub {
            log_debug!(
                LOGGER_NAME,
                "Publishing {} points to debug/all_points",
                points.len()
            );
            let debug_cloud = Self::create_debug_pointcloud(&points, &msg.header)?;
            if let Err(e) = pub_all.publish(debug_cloud) {
                log_warn!(LOGGER_NAME, "Failed to publish debug all points: {e}");
            }
        }

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
            log_debug!(LOGGER_NAME, "No points within bounding box");
            return Ok(Detection3DArray {
                header: msg.header.clone(),
                detections: Vec::new(),
            });
        }

        log_debug!(
            LOGGER_NAME,
            "Filtered {} points within bounding box",
            active_points.len()
        );

        // Publish debug filtered points if enabled
        if let Some(pub_filtered) = debug_filtered_points_pub {
            log_debug!(
                LOGGER_NAME,
                "Publishing {} filtered points to debug/filtered_points",
                active_points.len()
            );
            let debug_cloud = Self::create_debug_pointcloud(&active_points, &msg.header)?;
            if let Err(e) = pub_filtered.publish(debug_cloud) {
                log_warn!(LOGGER_NAME, "Failed to publish debug filtered points: {e}");
            }
        }

        // Detect calibration board
        log_debug!(
            LOGGER_NAME,
            "Starting board detection with {} points",
            active_points.len()
        );
        let detection: Option<BoardDetection> = match detector.detect(&active_points) {
            Ok(Some(det)) => {
                log_debug!(LOGGER_NAME, "Board detection successful");

                // Publish debug plane inliers if enabled
                if let Some(_pub_inliers) = debug_plane_inliers_pub {
                    // Access the ransac_data if available from the detection
                    // Note: This requires the detection to expose ransac data
                    log_debug!(LOGGER_NAME, "Debug plane inliers publisher available");
                }

                Some(det)
            }
            Ok(None) => {
                log_warn!(LOGGER_NAME, "Detection returned None - board not found");
                None
            }
            Err(e) => {
                log_warn!(LOGGER_NAME, "Detection failed with error: {}", e);
                return Err(e.into());
            }
        };

        let mut detections = Vec::new();
        if let Some(board_detection) = detection {
            let detection_3d =
                Self::convert_board_detection_to_detection3d(&board_detection, &msg.header)?;
            detections.push(detection_3d);
        }

        let num_detections = detections.len();
        let detection_array = Detection3DArray {
            header: msg.header.clone(),
            detections,
        };

        log_debug!(
            LOGGER_NAME,
            "Completed processing with {} detections",
            num_detections
        );

        Ok(detection_array)
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

    fn create_debug_pointcloud(
        points: &[na::Point3<f64>],
        header: &std_msgs::msg::Header,
    ) -> Result<PointCloud2> {
        use sensor_msgs::msg::PointField;

        let point_step = 12; // 3 floats * 4 bytes
        let row_step = point_step * points.len() as u32;
        let data_len = row_step as usize;
        let mut data = vec![0u8; data_len];

        // Write points to data buffer
        for (i, point) in points.iter().enumerate() {
            let offset = i * point_step as usize;
            let x_bytes = (point.x as f32).to_le_bytes();
            let y_bytes = (point.y as f32).to_le_bytes();
            let z_bytes = (point.z as f32).to_le_bytes();

            data[offset..offset + 4].copy_from_slice(&x_bytes);
            data[offset + 4..offset + 8].copy_from_slice(&y_bytes);
            data[offset + 8..offset + 12].copy_from_slice(&z_bytes);
        }

        Ok(PointCloud2 {
            header: header.clone(),
            height: 1,
            width: points.len() as u32,
            fields: vec![
                PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: 7, // FLOAT32
                    count: 1,
                },
                PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: 7, // FLOAT32
                    count: 1,
                },
                PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: 7, // FLOAT32
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step,
            row_step,
            data,
            is_dense: true,
        })
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
    // Initialize logging for the hollow-board-detector library
    init_logging();
    
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("calibration_board_locator")?;
    let _calibration_board_locator_node = CalibrationBoardLocatorNode::new(node)?;

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
