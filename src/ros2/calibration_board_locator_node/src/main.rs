mod bbox;

use crate::bbox::BBox;
use anyhow::{anyhow, Result};
use aruco_config::MultiArucoPattern;
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion, Vector3};
use hollow_board_detector::{
    init_logging, Config as BoardDetectorConfig, Detection as BoardDetection,
    Detector as BoardDetector,
};
use nalgebra as na;
use rclrs::{SubscriptionOptions, *};
use sensor_msgs::msg::PointCloud2;
use std::{
    f64::consts::FRAC_PI_2,
    fs,
    path::PathBuf,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc, Mutex,
    },
};
use std_msgs::msg::{ColorRGBA, Header, String as StringMsg};
use vision_msgs::msg::{BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose};
use visualization_msgs::msg::{Marker, MarkerArray};

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
    _bbox_marker_publisher: Option<Arc<Publisher<MarkerArray>>>,
    _board_marker_publisher: Option<Arc<Publisher<MarkerArray>>>,
    _board_marker_icp_publisher: Option<Arc<Publisher<MarkerArray>>>,
    _initial_board_marker_publisher: Option<Arc<Publisher<MarkerArray>>>,
    _icp_stats_publisher: Option<Arc<Publisher<StringMsg>>>,
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

        // QoS parameter for sensor input topics
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

        // Create bbox marker publisher for visualization
        let bbox_marker_publisher = if enable_debug {
            Some(Arc::new(node.create_publisher("debug/bbox_marker")?))
        } else {
            None
        };
        let bbox_marker_shared = bbox_marker_publisher.clone();

        // Create final board pose marker publisher for visualization
        let board_marker_publisher = if enable_debug {
            Some(Arc::new(node.create_publisher("debug/final_board_pose")?))
        } else {
            None
        };
        let board_marker_shared = board_marker_publisher.clone();

        // ICP iteration progress marker publisher (always on)
        let board_marker_icp_publisher =
            Some(Arc::new(node.create_publisher("debug/icp_iterations")?));
        let board_marker_icp_shared = board_marker_icp_publisher.clone();

        // Initial board pose marker publisher for debug
        let initial_board_marker_publisher = if enable_debug {
            Some(Arc::new(
                node.create_publisher("debug/initial_board_marker")?,
            ))
        } else {
            None
        };
        let initial_board_marker_shared = initial_board_marker_publisher.clone();

        // ICP statistics publisher for debug
        let icp_stats_publisher = if enable_debug {
            Some(Arc::new(node.create_publisher("debug/icp_stats")?))
        } else {
            None
        };
        let icp_stats_shared = icp_stats_publisher.clone();

        // Configure QoS for sensor input topics
        let qos_profile = if use_best_effort_qos {
            QoSProfile::sensor_data_default() // Best effort for live sensors
        } else {
            QoSProfile::default() // Reliable for rosbag playback
        };

        // Counter for debugging message processing
        let message_counter = Arc::new(AtomicU64::new(0));
        let counter_clone = Arc::clone(&message_counter);

        // Create subscription to PointCloud2 with configurable QoS
        let mut pointcloud_options = SubscriptionOptions::new("input_pointcloud");
        pointcloud_options.qos = qos_profile;
        let pointcloud_subscription =
            node.create_subscription(pointcloud_options, move |msg: PointCloud2| {
                let count = counter_clone.fetch_add(1, Ordering::Relaxed);
                log_debug!(LOGGER_NAME, "Processing message #{}", count + 1);

                Self::pointcloud_callback(
                    msg,
                    &detector,
                    &detection_publisher_shared,
                    &bbox,
                    &debug_all_points_shared,
                    &debug_filtered_points_shared,
                    &debug_plane_inliers_shared,
                    &bbox_marker_shared,
                    &board_marker_shared,
                    &board_marker_icp_shared,
                    &initial_board_marker_shared,
                    &icp_stats_shared,
                );
            })?;

        if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Calibration board locator node initialized with debug mode"
            );
            log_info!(
                LOGGER_NAME,
                "Debug topics: debug/all_points, debug/filtered_points, debug/plane_inliers, debug/bbox_marker, debug/final_board_pose, debug/icp_iterations, debug/initial_board_marker, debug/icp_stats"
            );
        }

        Ok(Self {
            _node: node,
            _detection_publisher: detection_publisher,
            _pointcloud_subscription: pointcloud_subscription,
            _debug_all_points_publisher: debug_all_points_publisher,
            _debug_filtered_points_publisher: debug_filtered_points_publisher,
            _debug_plane_inliers_publisher: debug_plane_inliers_publisher,
            _bbox_marker_publisher: bbox_marker_publisher,
            _board_marker_publisher: board_marker_publisher,
            _board_marker_icp_publisher: board_marker_icp_publisher,
            _initial_board_marker_publisher: initial_board_marker_publisher,
            _icp_stats_publisher: icp_stats_publisher,
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
        bbox_marker_pub: &Option<Arc<Publisher<MarkerArray>>>,
        board_marker_pub: &Option<Arc<Publisher<MarkerArray>>>,
        board_marker_icp_pub: &Option<Arc<Publisher<MarkerArray>>>,
        initial_board_marker_pub: &Option<Arc<Publisher<MarkerArray>>>,
        icp_stats_pub: &Option<Arc<Publisher<StringMsg>>>,
    ) {
        use std::time::Instant;

        let start_time = Instant::now();

        // Log callback invocation with timestamp and data size
        log_debug!(
            LOGGER_NAME,
            "PointCloud callback triggered at timestamp: {}.{:09}, data size: {} bytes, width: {}, height: {}",
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.data.len(),
            msg.width,
            msg.height
        );

        // Check if we have valid data
        if msg.data.is_empty() || msg.width == 0 || msg.height == 0 {
            log_warn!(
                LOGGER_NAME,
                "Received empty or invalid point cloud (data: {} bytes, {}x{})",
                msg.data.len(),
                msg.width,
                msg.height
            );
            // Still try to publish empty debug topics to maintain consistency
        }

        let result = Self::process_pointcloud(
            &msg,
            detector,
            bbox,
            debug_all_points_pub,
            debug_filtered_points_pub,
            debug_plane_inliers_pub,
            bbox_marker_pub,
            board_marker_pub,
            board_marker_icp_pub,
            initial_board_marker_pub,
            icp_stats_pub,
        );

        let processing_duration = start_time.elapsed();
        log_debug!(
            LOGGER_NAME,
            "Processing completed in {:.2}ms",
            processing_duration.as_millis()
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
        bbox_marker_pub: &Option<Arc<Publisher<MarkerArray>>>,
        board_marker_pub: &Option<Arc<Publisher<MarkerArray>>>,
        board_marker_icp_pub: &Option<Arc<Publisher<MarkerArray>>>,
        initial_board_marker_pub: &Option<Arc<Publisher<MarkerArray>>>,
        icp_stats_pub: &Option<Arc<Publisher<StringMsg>>>,
    ) -> Result<Detection3DArray> {
        // Convert PointCloud2 to nalgebra points
        let points = match Self::convert_pointcloud2_to_points(msg) {
            Ok(pts) => pts,
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to convert point cloud: {e}");
                return Err(e);
            }
        };

        log_debug!(
            LOGGER_NAME,
            "Converted {} points from PointCloud2",
            points.len()
        );

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
        log_debug!(LOGGER_NAME, "Attempting to lock bbox mutex...");
        let bbox_guard = match bbox.lock() {
            Ok(guard) => guard,
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to lock bbox mutex: {e}");
                return Err(anyhow!("Failed to lock bbox: {e}"));
            }
        };
        log_debug!(LOGGER_NAME, "Successfully locked bbox mutex");

        log_debug!(
            LOGGER_NAME,
            "Bounding box filter: center=[{:.2}, {:.2}, {:.2}], size=[{:.2}, {:.2}, {:.2}]",
            bbox_guard.pose.translation.x,
            bbox_guard.pose.translation.y,
            bbox_guard.pose.translation.z,
            bbox_guard.size_xyz[0],
            bbox_guard.size_xyz[1],
            bbox_guard.size_xyz[2]
        );

        // Publish bbox marker for visualization in RViz
        if let Some(pub_bbox) = bbox_marker_pub {
            let bbox_marker = Self::create_bbox_marker(&bbox_guard, &msg.header)?;
            let mut marker_array = MarkerArray::default();
            marker_array.markers.push(bbox_marker);
            if let Err(e) = pub_bbox.publish(marker_array) {
                log_warn!(LOGGER_NAME, "Failed to publish bbox marker: {e}");
            }
        }

        let active_points: Vec<_> = points
            .iter()
            .filter(|pt| bbox_guard.contains_point(pt))
            .cloned()
            .collect();
        drop(bbox_guard);

        log_debug!(
            LOGGER_NAME,
            "Filtered {} points within bounding box",
            active_points.len()
        );

        // Publish debug filtered points if enabled (always publish, even if empty)
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

        if active_points.is_empty() {
            log_debug!(
                LOGGER_NAME,
                "No points within bounding box - continuing with empty detection"
            );
            return Ok(Detection3DArray {
                header: msg.header.clone(),
                detections: Vec::new(),
            });
        }

        // Detect calibration board
        log_debug!(
            LOGGER_NAME,
            "Starting board detection with {} points",
            active_points.len()
        );
        let detection: Option<BoardDetection> = match detector.detect_with_progress(
            &active_points,
            |bm| {
                if let Some(pub_icp) = board_marker_icp_pub {
                    if let Ok(arr) = Self::create_board_markers_from_model(bm, &msg.header) {
                        let _ = pub_icp.publish(arr);
                    }
                }
            },
        ) {
            Ok(Some(det)) => {
                log_warn!(LOGGER_NAME, "Board detection successful");

                // Publish debug plane inliers if enabled
                if let Some(_pub_inliers) = debug_plane_inliers_pub {
                    // Access the ransac_data if available from the detection
                    // Note: This requires the detection to expose ransac data
                    log_warn!(LOGGER_NAME, "Debug plane inliers publisher available");
                }

                // Publish initial board pose markers if enabled
                if let Some(pub_initial) = initial_board_marker_pub {
                    // Create board markers using the initial pose before ICP refinement
                    let initial_board_model = hollow_board_config::BoardModel {
                        pose: det.initial_pose,
                        board_shape: det.board_model.board_shape.clone(),
                        marker_paper_size: det.board_model.marker_paper_size,
                    };
                    if let Ok(initial_markers) =
                        Self::create_board_markers_from_model(&initial_board_model, &msg.header)
                    {
                        let _ = pub_initial.publish(initial_markers);
                        log_debug!(LOGGER_NAME, "Published initial board pose markers");
                    }
                }

                // Publish ICP statistics if enabled
                if let Some(pub_stats) = icp_stats_pub {
                    let stats_msg = StringMsg {
                            data: format!(
                                "ICP Stats - Iterations: {}, Initial Loss: {:.6}, Final Loss: {:.6}, Min Loss: {:.6}, Successful: {}, Convergence: {}",
                                det.icp_stats.iterations,
                                det.icp_stats.initial_loss,
                                det.icp_stats.final_loss,
                                det.icp_stats.min_loss,
                                det.icp_stats.successful,
                                det.icp_stats.convergence_reason
                            ),
                        };
                    let _ = pub_stats.publish(stats_msg);
                    log_debug!(
                        LOGGER_NAME,
                        "Published ICP statistics: {} iterations, final loss: {:.6}",
                        det.icp_stats.iterations,
                        det.icp_stats.final_loss
                    );
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
            // Create board markers (cube + axes) using the pose returned by algo.rs and publish them if enabled
            if let Some(pub_board) = board_marker_pub {
                let marker_array = Self::create_board_markers(&board_detection, &msg.header)?;
                if let Err(e) = pub_board.publish(marker_array) {
                    log_warn!(LOGGER_NAME, "Failed to publish board marker array: {e}");
                }
            }

            let detection_3d =
                Self::convert_board_detection_to_detection3d(&board_detection, &msg.header)?;
            detections.push(detection_3d);
        } else {
            // Publish empty marker array to ensure topic is active for debugging
            if let Some(pub_board) = board_marker_pub {
                let marker_array = MarkerArray::default();
                if let Err(e) = pub_board.publish(marker_array) {
                    log_warn!(
                        LOGGER_NAME,
                        "Failed to publish empty board marker array: {e}"
                    );
                }
            }
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

    fn create_bbox_marker(bbox: &BBox, header: &Header) -> Result<Marker> {
        // Create a cube marker to visualize the bounding box
        let mut marker = Marker::default();
        marker.header = header.clone();
        marker.ns = "bbox".to_string();
        marker.id = 0;
        marker.type_ = 1; // CUBE
        marker.action = 0; // ADD

        // Set position from bbox pose
        marker.pose.position.x = bbox.pose.translation.x;
        marker.pose.position.y = bbox.pose.translation.y;
        marker.pose.position.z = bbox.pose.translation.z;

        // Set orientation from bbox pose
        let q = bbox.pose.rotation.quaternion();
        marker.pose.orientation.x = q.i;
        marker.pose.orientation.y = q.j;
        marker.pose.orientation.z = q.k;
        marker.pose.orientation.w = q.w;

        // Set scale from bbox size
        marker.scale.x = bbox.size_xyz[0];
        marker.scale.y = bbox.size_xyz[1];
        marker.scale.z = bbox.size_xyz[2];

        // Set color (semi-transparent green)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        // Set lifetime (0 = forever)
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 0;

        Ok(marker)
    }

    fn create_board_marker(board_detection: &BoardDetection, header: &Header) -> Result<Marker> {
        // Use the pose returned by algo.rs (embedded in board_detection.board_model.pose)
        let board_model = &board_detection.board_model;

        let mut marker = Marker::default();
        marker.header = header.clone();
        marker.ns = "board".to_string();
        marker.id = 0;
        marker.type_ = 1; // CUBE to approximate board plane
        marker.action = 0; // ADD

        // Position from pose
        marker.pose.position.x = board_model.pose.translation.x;
        marker.pose.position.y = board_model.pose.translation.y;
        marker.pose.position.z = board_model.pose.translation.z;

        // Orientation from pose
        let q = board_model.pose.rotation.quaternion();
        marker.pose.orientation.x = q.i;
        marker.pose.orientation.y = q.j;
        marker.pose.orientation.z = q.k;
        marker.pose.orientation.w = q.w;

        // Scale from board shape (width x height, with small thickness)
        // Assuming board is square with width; set small thickness along z
        marker.scale.x = board_model.board_shape.board_width.as_meters();
        marker.scale.y = board_model.board_shape.board_width.as_meters();
        marker.scale.z = 0.02; // 2cm thickness for visualization

        // Color (blue, semi-transparent)
        marker.color.r = 0.0;
        marker.color.g = 0.2;
        marker.color.b = 1.0;
        marker.color.a = 0.4;

        // Lifetime
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 0;

        Ok(marker)
    }

    fn create_board_markers(
        board_detection: &BoardDetection,
        header: &Header,
    ) -> Result<MarkerArray> {
        use nalgebra as na;
        let board_model = &board_detection.board_model;

        // Base pose
        let base_translation = &board_model.pose.translation;
        let base_rotation = &board_model.pose.rotation;

        // Board cube marker (id 0)
        let board_cube = {
            let mut m = Marker::default();
            m.header = header.clone();
            m.ns = "board".to_string();
            m.id = 0;
            m.type_ = 1; // CUBE
            m.action = 0; // ADD
            m.pose.position.x = base_translation.x;
            m.pose.position.y = base_translation.y;
            m.pose.position.z = base_translation.z;
            let q = base_rotation.quaternion();
            m.pose.orientation.x = q.i;
            m.pose.orientation.y = q.j;
            m.pose.orientation.z = q.k;
            m.pose.orientation.w = q.w;
            m.scale.x = board_model.board_shape.board_width.as_meters();
            m.scale.y = board_model.board_shape.board_width.as_meters();
            m.scale.z = 0.02; // 2 cm thickness
            m.color.r = 0.0;
            m.color.g = 0.2;
            m.color.b = 1.0;
            m.color.a = 0.4;
            m
        };

        // Helper to build an arrow marker oriented along the board frame's X axis, then rotated
        let mut make_axis_arrow =
            |id: i32, rot_after_x: na::UnitQuaternion<f64>, r: f32, g: f32, b: f32| -> Marker {
                let mut m = Marker::default();
                m.header = header.clone();
                m.ns = "board_axes".to_string();
                m.id = id;
                m.type_ = 0; // ARROW
                m.action = 0; // ADD
                m.pose.position.x = base_translation.x;
                m.pose.position.y = base_translation.y;
                m.pose.position.z = base_translation.z;

                let rot = base_rotation * rot_after_x; // orientation in world
                let q = rot.quaternion();
                m.pose.orientation.x = q.i;
                m.pose.orientation.y = q.j;
                m.pose.orientation.z = q.k;
                m.pose.orientation.w = q.w;

                // Shaft length = 0.5 * board width, diameters small
                let len = (board_model.board_shape.board_width.as_meters() * 0.5) as f64;
                m.scale.x = len; // shaft length
                m.scale.y = 0.02; // shaft diameter
                m.scale.z = 0.04; // head diameter

                m.color.r = r;
                m.color.g = g;
                m.color.b = b;
                m.color.a = 1.0;
                m
            };

        // Rotations to map X axis to Y/Z in the board frame
        let rot_x = na::UnitQuaternion::identity();
        let rot_y = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), FRAC_PI_2);
        let rot_z = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -FRAC_PI_2);

        let x_arrow = make_axis_arrow(1, rot_x, 1.0, 0.0, 0.0); // Red X
        let y_arrow = make_axis_arrow(2, rot_y, 0.0, 1.0, 0.0); // Green Y
        let z_arrow = make_axis_arrow(3, rot_z, 0.0, 0.0, 1.0); // Blue Z

        let mut arr = MarkerArray::default();
        arr.markers.push(board_cube);
        arr.markers.push(x_arrow);
        arr.markers.push(y_arrow);
        arr.markers.push(z_arrow);
        Ok(arr)
    }

    fn create_board_markers_from_model(
        board_model: &hollow_board_config::BoardModel,
        header: &Header,
    ) -> Result<MarkerArray> {
        use nalgebra as na;

        let base_translation = &board_model.pose.translation;
        let base_rotation = &board_model.pose.rotation;

        let board_cube = {
            let mut m = Marker::default();
            m.header = header.clone();
            m.ns = "board_icp".to_string();
            m.id = 1000;
            m.type_ = 1; // CUBE
            m.action = 0; // ADD
            m.pose.position.x = base_translation.x;
            m.pose.position.y = base_translation.y;
            m.pose.position.z = base_translation.z;
            let q = base_rotation.quaternion();
            m.pose.orientation.x = q.i;
            m.pose.orientation.y = q.j;
            m.pose.orientation.z = q.k;
            m.pose.orientation.w = q.w;
            m.scale.x = board_model.board_shape.board_width.as_meters();
            m.scale.y = board_model.board_shape.board_width.as_meters();
            m.scale.z = 0.02;
            m.color.r = 1.0;
            m.color.g = 0.5;
            m.color.b = 0.0;
            m.color.a = 0.3;
            m
        };

        let mut make_axis_arrow =
            |id: i32, rot_after_x: na::UnitQuaternion<f64>, r: f32, g: f32, b: f32| -> Marker {
                let mut m = Marker::default();
                m.header = header.clone();
                m.ns = "board_axes_icp".to_string();
                m.id = id;
                m.type_ = 0; // ARROW
                m.action = 0; // ADD
                m.pose.position.x = base_translation.x;
                m.pose.position.y = base_translation.y;
                m.pose.position.z = base_translation.z;

                let rot = base_rotation * rot_after_x;
                let q = rot.quaternion();
                m.pose.orientation.x = q.i;
                m.pose.orientation.y = q.j;
                m.pose.orientation.z = q.k;
                m.pose.orientation.w = q.w;

                let len = (board_model.board_shape.board_width.as_meters() * 0.5) as f64;
                m.scale.x = len;
                m.scale.y = 0.02;
                m.scale.z = 0.04;

                m.color.r = r;
                m.color.g = g;
                m.color.b = b;
                m.color.a = 0.9;
                m
            };

        let rot_x = na::UnitQuaternion::identity();
        let rot_y = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), FRAC_PI_2);
        let rot_z = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -FRAC_PI_2);

        let x_arrow = make_axis_arrow(1001, rot_x, 1.0, 0.2, 0.2);
        let y_arrow = make_axis_arrow(1002, rot_y, 0.2, 1.0, 0.2);
        let z_arrow = make_axis_arrow(1003, rot_z, 0.2, 0.2, 1.0);

        let mut arr = MarkerArray::default();
        arr.markers.push(board_cube);
        arr.markers.push(x_arrow);
        arr.markers.push(y_arrow);
        arr.markers.push(z_arrow);
        Ok(arr)
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
