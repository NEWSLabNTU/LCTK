mod bbox;

use crate::bbox::BBox;
use anyhow::{anyhow, Result};
use aruco_config::MultiArucoPattern;
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion, Vector3};
use hollow_board_detector::{
    Config as BoardDetectorConfig, Detection as BoardDetection, Detector as BoardDetector,
};
use nalgebra as na;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;
use std::{
    fs,
    io::Write,
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
            return Err(anyhow!("board_detector_file parameter is required but was empty"));
        }

        log_info!(LOGGER_NAME, "Loading board detector config from: {file_path}");
        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_aruco_pattern_config(file_path: &str) -> Result<MultiArucoPattern> {
        if file_path.is_empty() {
            return Err(anyhow!("aruco_pattern_file parameter is required but was empty"));
        }

        log_info!(LOGGER_NAME, "Loading ArUco pattern config from: {file_path}");
        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_bbox_config(file_path: &str) -> Result<BBox> {
        if file_path.is_empty() {
            return Err(anyhow!("bbox_file parameter is required but was empty"));
        }

        log_info!(LOGGER_NAME, "Loading bounding box config from: {file_path}");
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
        
        // Save all points to CSV for debugging
        if let Err(e) = Self::save_points_to_csv(&points, "points_all.csv") {
            log_warn!(LOGGER_NAME, "Failed to save all points to CSV: {e}");
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

        // Save filtered points to CSV for debugging
        if let Err(e) = Self::save_points_to_csv(&active_points, "points_filtered.csv") {
            log_warn!(LOGGER_NAME, "Failed to save filtered points to CSV: {e}");
        }

        // Add point cloud statistics before detection
        Self::log_point_cloud_statistics(&active_points);

        // Detect calibration board with detailed debugging
        log_info!(LOGGER_NAME, "Starting board detection with {} filtered points", active_points.len());
        let detection: Option<BoardDetection> = match detector.detect(&active_points) {
            Ok(Some(det)) => {
                log_info!(LOGGER_NAME, "✓ Detection successful!");
                Some(det)
            },
            Ok(None) => {
                log_warn!(LOGGER_NAME, "✗ Detection returned None - board not found");
                None
            },
            Err(e) => {
                log_warn!(LOGGER_NAME, "✗ Detection failed with error: {}", e);
                return Err(e.into());
            }
        };

        let mut detections = Vec::new();
        if let Some(board_detection) = detection {
            let detection_3d =
                Self::convert_board_detection_to_detection3d(&board_detection, &msg.header)?;
            detections.push(detection_3d);
        }
        
        let detection_array = Detection3DArray {
            header: msg.header.clone(),
            detections,
        };
        
        // Print Detection3DArray for debugging
        log_info!(LOGGER_NAME, "Detection3DArray: {:?}", detection_array);
        
        Ok(detection_array)
    }

    fn save_points_to_csv(points: &[na::Point3<f64>], filename: &str) -> Result<()> {
        let mut file = fs::File::create(filename)?;
        writeln!(file, "x,y,z")?;
        
        for point in points {
            writeln!(file, "{},{},{}", point.x, point.y, point.z)?;
        }
        
        log_info!(LOGGER_NAME, "Saved {} points to {}", points.len(), filename);
        Ok(())
    }

    fn log_point_cloud_statistics(points: &[na::Point3<f64>]) {
        if points.is_empty() {
            log_warn!(LOGGER_NAME, "Point cloud is empty!");
            return;
        }

        // Calculate basic statistics
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut min_z = f64::INFINITY;
        let mut max_z = f64::NEG_INFINITY;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;

        for point in points {
            min_x = min_x.min(point.x);
            max_x = max_x.max(point.x);
            min_y = min_y.min(point.y);
            max_y = max_y.max(point.y);
            min_z = min_z.min(point.z);
            max_z = max_z.max(point.z);
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
        }

        let count = points.len() as f64;
        let mean_x = sum_x / count;
        let mean_y = sum_y / count;
        let mean_z = sum_z / count;

        // Calculate z-variance to assess planarity
        let mut z_variance = 0.0;
        for point in points {
            let diff = point.z - mean_z;
            z_variance += diff * diff;
        }
        z_variance /= count;
        let z_std_dev = z_variance.sqrt();

        log_info!(LOGGER_NAME, "📊 Point Cloud Statistics:");
        log_info!(LOGGER_NAME, "  Count: {}", points.len());
        log_info!(LOGGER_NAME, "  X range: [{:.3}, {:.3}] (span: {:.3})", min_x, max_x, max_x - min_x);
        log_info!(LOGGER_NAME, "  Y range: [{:.3}, {:.3}] (span: {:.3})", min_y, max_y, max_y - min_y);
        log_info!(LOGGER_NAME, "  Z range: [{:.3}, {:.3}] (span: {:.3})", min_z, max_z, max_z - min_z);
        log_info!(LOGGER_NAME, "  Centroid: ({:.3}, {:.3}, {:.3})", mean_x, mean_y, mean_z);
        log_info!(LOGGER_NAME, "  Z std dev: {:.4} (planarity indicator)", z_std_dev);
        
        // Planarity assessment
        if z_std_dev < 0.05 {
            log_info!(LOGGER_NAME, "  ✓ Points appear to be roughly planar (z_std < 0.05)");
        } else if z_std_dev < 0.1 {
            log_info!(LOGGER_NAME, "  ⚠ Points are somewhat planar (0.05 < z_std < 0.1)");
        } else {
            log_warn!(LOGGER_NAME, "  ✗ Points are highly non-planar (z_std > 0.1) - RANSAC may struggle");
        }
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

        log_debug!(
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
