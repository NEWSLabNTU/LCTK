mod bbox;
mod services;

use crate::{bbox::BBox, services::BBoxServices};
use anyhow::{anyhow, bail, Result};
use arc_swap::ArcSwap;
use aruco_config::MultiArucoPattern;
use geometry_msgs::msg::{
    Point, Pose, PoseStamped, PoseWithCovariance, Quaternion, Vector3 as GeomVector3,
};
use hollow_board_config::{BoardModel, BoardShape};
use hollow_board_detector::{
    algo::{fit_plane_ransac, BoardIcpIterator},
    detection::{BoardIcpState, BoardModelParams, IcpStatistics, PlaneRansacData},
    init_logging, Config as BoardDetectorConfig, Detection as BoardDetection,
    Detector as BoardDetector,
};
use nalgebra::{self as na, Translation3, UnitQuaternion};
use ndarray::Array2;
use petal_decomposition::PcaBuilder;
use plane_estimator::PlaneModel;
use rclrs::{SubscriptionOptions, *};
use sensor_msgs::msg::{PointCloud2, PointField};
use std::{
    f64::consts::FRAC_PI_2,
    fs,
    path::PathBuf,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};
use std_msgs::msg::{ColorRGBA, Float64, Header, String as StringMsg};
use vision_msgs::msg::{BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose};
use visualization_msgs::msg::{Marker, MarkerArray};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

/// Debug publishers for ICP iteration debugging
#[derive(Clone)]
struct IcpDebugPublishers {
    iteration_pose: Arc<Publisher<PoseStamped>>,
    board_points: Arc<Publisher<PointCloud2>>,
    correspondences: Arc<Publisher<MarkerArray>>,
    loss: Arc<Publisher<Float64>>,
    stats: Arc<Publisher<StringMsg>>,
}

/// Debug publishers for board detection debugging
#[derive(Clone)]
struct BoardDebugPublishers {
    all_points: Arc<Publisher<PointCloud2>>,
    filtered_points: Arc<Publisher<PointCloud2>>,
    plane_inliers: Arc<Publisher<PointCloud2>>,
    plane_marker: Arc<Publisher<MarkerArray>>,
    bbox_marker: Arc<Publisher<MarkerArray>>,
    board_marker: Arc<Publisher<MarkerArray>>,
    board_marker_icp: Arc<Publisher<MarkerArray>>,
    initial_board_marker: Arc<Publisher<MarkerArray>>,
    icp_stats: Arc<Publisher<StringMsg>>,
    pca_eigenvectors: Arc<Publisher<MarkerArray>>,
}

// Config files are now mandatory parameters - no defaults

pub struct CalibrationBoardLocatorNode {
    _node: Node,
    _detection_publisher: Publisher<Detection3DArray>,
    _pointcloud_subscription: Subscription<PointCloud2>,
    // Board debug publishers - grouped into a single struct
    _board_debug_publishers: Option<BoardDebugPublishers>,
    // ICP iteration debug publishers - grouped into a single struct
    _icp_debug_publishers: Option<IcpDebugPublishers>,
    // BBox configuration services
    _bbox_services: BBoxServices,
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

        // ICP iteration debug mode parameter (optional, defaults to false)
        let icp_debug_param = node
            .declare_parameter("enable_icp_iteration_debug")
            .default(false)
            .optional()?;
        let enable_icp_iteration_debug = icp_debug_param.get().unwrap_or(false);

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
        let bbox = Arc::new(ArcSwap::new(Arc::new(bbox)));

        // Store bbox file path for save service
        let bbox_file_path = bbox_file_param.to_string();

        // Create detector
        let detector = Arc::new(BoardDetector::new(
            board_detector_config,
            aruco_pattern_config,
        ));

        // Create publisher for detections
        let detection_publisher = node.create_publisher("calibration_board_detections")?;
        let detection_publisher_shared = Arc::clone(&detection_publisher);

        // Create board debug publishers if debug mode is enabled
        let board_debug_publishers = if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Debug mode enabled - creating debug publishers"
            );
            Some(BoardDebugPublishers {
                all_points: Arc::new(node.create_publisher("debug/all_points")?),
                filtered_points: Arc::new(node.create_publisher("debug/filtered_points")?),
                plane_inliers: Arc::new(node.create_publisher("debug/plane_inliers")?),
                plane_marker: Arc::new(node.create_publisher("debug/plane_marker")?),
                bbox_marker: Arc::new(node.create_publisher("debug/bbox_marker")?),
                board_marker: Arc::new(node.create_publisher("debug/final_board_pose")?),
                board_marker_icp: Arc::new(node.create_publisher("debug/icp_iterations")?),
                initial_board_marker: Arc::new(
                    node.create_publisher("debug/initial_board_marker")?,
                ),
                icp_stats: Arc::new(node.create_publisher("debug/icp_stats")?),
                pca_eigenvectors: Arc::new(node.create_publisher("debug/pca_eigenvectors")?),
            })
        } else {
            None
        };
        let board_debug_shared = board_debug_publishers.clone();

        // ICP iteration debug publishers - grouped into single struct
        let icp_debug_publishers = if enable_icp_iteration_debug {
            log_info!(
                LOGGER_NAME,
                "ICP iteration debug mode enabled - creating iteration debug publishers"
            );
            Some(IcpDebugPublishers {
                iteration_pose: Arc::new(
                    node.create_publisher("/calibration/icp_debug/iteration_pose")?,
                ),
                board_points: Arc::new(
                    node.create_publisher("/calibration/icp_debug/board_points")?,
                ),
                correspondences: Arc::new(
                    node.create_publisher("/calibration/icp_debug/correspondences")?,
                ),
                loss: Arc::new(node.create_publisher("/calibration/icp_debug/loss")?),
                stats: Arc::new(node.create_publisher("/calibration/icp_debug/stats")?),
            })
        } else {
            None
        };
        let icp_debug_shared = icp_debug_publishers.clone();

        // Configure QoS for sensor input topics
        let qos_profile = if use_best_effort_qos {
            QoSProfile::sensor_data_default() // Best effort for live sensors
        } else {
            QoSProfile::default() // Reliable for rosbag playback
        };

        // Counter for debugging message processing
        let message_counter = Arc::new(AtomicU64::new(0));
        let counter_clone = Arc::clone(&message_counter);

        // Clone bbox for subscription callback
        let bbox_for_callback = Arc::clone(&bbox);

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
                    &bbox_for_callback,
                    &board_debug_shared,
                    &icp_debug_shared,
                );
            })?;

        if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Calibration board locator node initialized with debug mode"
            );
            log_info!(
                LOGGER_NAME,
                "Debug topics: debug/all_points, debug/filtered_points, debug/plane_inliers, debug/plane_marker, debug/bbox_marker, debug/final_board_pose, debug/icp_iterations, debug/initial_board_marker, debug/icp_stats, debug/pca_eigenvectors"
            );
        }

        if enable_icp_iteration_debug {
            log_info!(
                LOGGER_NAME,
                "ICP iteration debug topics: /calibration/icp_debug/iteration_pose, /calibration/icp_debug/board_points, /calibration/icp_debug/correspondences, /calibration/icp_debug/loss, /calibration/icp_debug/stats"
            );
        }

        // Create BBox services
        let bbox_services = BBoxServices::new(&node, Arc::clone(&bbox), bbox_file_path)?;

        Ok(Self {
            _node: node,
            _detection_publisher: detection_publisher,
            _pointcloud_subscription: pointcloud_subscription,
            _board_debug_publishers: board_debug_publishers,
            _icp_debug_publishers: icp_debug_publishers,
            _bbox_services: bbox_services,
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
        bbox: &Arc<ArcSwap<BBox>>,
        board_debug_publishers: &Option<BoardDebugPublishers>,
        icp_debug_publishers: &Option<IcpDebugPublishers>,
    ) {
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
            board_debug_publishers,
            icp_debug_publishers,
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
        bbox: &Arc<ArcSwap<BBox>>,
        board_debug_publishers: &Option<BoardDebugPublishers>,
        icp_debug_publishers: &Option<IcpDebugPublishers>,
    ) -> Result<Detection3DArray> {
        // Convert PointCloud2 to nalgebra points
        let points = {
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
            if let Some(debug_pubs) = board_debug_publishers {
                log_debug!(
                    LOGGER_NAME,
                    "Publishing {} points to debug/all_points",
                    points.len()
                );
                let debug_cloud = Self::create_debug_pointcloud(&points, &msg.header)?;
                if let Err(e) = debug_pubs.all_points.publish(debug_cloud) {
                    log_warn!(LOGGER_NAME, "Failed to publish debug all points: {e}");
                }
            }

            points
        };

        // Stage 1: Filter points by bounding box
        let active_points =
            Self::filter_points_by_bbox(&points, bbox, &msg.header, board_debug_publishers)?;

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

        // Stage 2: RANSAC plane detection
        let (plane_model, plane_inlier_points) = match Self::detect_plane_ransac(
            detector,
            &active_points,
            &msg.header,
            board_debug_publishers,
        )? {
            Some(result) => result,
            None => {
                log_debug!(
                    LOGGER_NAME,
                    "RANSAC plane detection failed - no valid plane found"
                );
                return Ok(Detection3DArray {
                    header: msg.header.clone(),
                    detections: Vec::new(),
                });
            }
        };

        // Stage 3: ICP board pose refinement
        log_debug!(
            LOGGER_NAME,
            "Starting ICP board detection with {} plane inlier points",
            plane_inlier_points.len()
        );

        let detection: Option<BoardDetection> = Self::detect_icp(
            detector,
            &plane_model,
            &plane_inlier_points,
            PlaneRansacData {
                plane_model: plane_model.clone(),
                inlier_points: plane_inlier_points.clone(),
            },
            &msg.header,
            icp_debug_publishers,
            board_debug_publishers,
        );

        let mut detections = Vec::new();
        if let Some(det) = detection {
            log_warn!(LOGGER_NAME, "Board detection successful");

            // Log ICP result to service logs
            let final_loss = det
                .icp_losses
                .iter()
                .copied()
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(0.0);

            log_info!(
                LOGGER_NAME,
                "FINAL ICP RESULT: pose=({:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}), loss={:.6}",
                det.board_model.pose.translation.x,
                det.board_model.pose.translation.y,
                det.board_model.pose.translation.z,
                det.board_model.pose.rotation.i,
                det.board_model.pose.rotation.j,
                det.board_model.pose.rotation.k,
                final_loss
            );

            // Publish initial board pose markers if enabled
            if let Some(debug_pubs) = board_debug_publishers {
                // Create board markers using the initial pose before ICP refinement
                let initial_board_model = hollow_board_config::BoardModel {
                    pose: det.initial_pose,
                    board_shape: det.board_model.board_shape.clone(),
                    marker_paper_size: det.board_model.marker_paper_size,
                };
                if let Ok(initial_markers) =
                    Self::create_board_markers_from_model(&initial_board_model, &msg.header)
                {
                    let _ = debug_pubs.initial_board_marker.publish(initial_markers);
                    log_debug!(LOGGER_NAME, "Published initial board pose markers");
                }
            }

            // Publish ICP statistics if enabled
            if let Some(debug_pubs) = board_debug_publishers {
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
                let _ = debug_pubs.icp_stats.publish(stats_msg);
                log_debug!(
                    LOGGER_NAME,
                    "Published ICP statistics: {} iterations, final loss: {:.6}",
                    det.icp_stats.iterations,
                    det.icp_stats.final_loss
                );
            }

            // Create board markers (cube + axes) using the pose returned by algo.rs and publish them if enabled
            if let Some(debug_pubs) = board_debug_publishers {
                let marker_array = Self::create_board_markers(&det, &msg.header)?;
                let marker_count = marker_array.markers.len();
                if let Err(e) = debug_pubs.board_marker.publish(marker_array) {
                    log_warn!(LOGGER_NAME, "Failed to publish board marker array: {e}");
                } else {
                    log_debug!(
                        LOGGER_NAME,
                        "Published final board pose markers with {} markers",
                        marker_count
                    );
                }
            }

            let detection_3d = Self::convert_board_detection_to_detection3d(&det, &msg.header)?;
            detections.push(detection_3d);
        } else {
            log_warn!(LOGGER_NAME, "Detection returned None - board not found");

            // Publish empty marker array to ensure topic is active for debugging
            if let Some(debug_pubs) = board_debug_publishers {
                let marker_array = MarkerArray::default();
                if let Err(e) = debug_pubs.board_marker.publish(marker_array) {
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

    // Stage 1: Bounding box filter
    fn filter_points_by_bbox(
        points: &[na::Point3<f64>],
        bbox: &Arc<ArcSwap<BBox>>,
        header: &Header,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Result<Vec<na::Point3<f64>>> {
        // Load bbox using lock-free arc-swap
        let bbox_copy = bbox.load();

        log_debug!(
            LOGGER_NAME,
            "Bounding box filter: center=[{:.2}, {:.2}, {:.2}], size=[{:.2}, {:.2}, {:.2}]",
            bbox_copy.pose.translation.x,
            bbox_copy.pose.translation.y,
            bbox_copy.pose.translation.z,
            bbox_copy.size_xyz[0],
            bbox_copy.size_xyz[1],
            bbox_copy.size_xyz[2]
        );

        // Publish bbox marker for visualization in RViz
        if let Some(debug_pubs) = board_debug_publishers {
            let bbox_marker = Self::create_bbox_marker(&bbox_copy, header)?;
            let marker_array = MarkerArray {
                markers: vec![bbox_marker],
            };
            if let Err(e) = debug_pubs.bbox_marker.publish(marker_array) {
                log_warn!(LOGGER_NAME, "Failed to publish bbox marker: {e}");
            }
        }

        let active_points: Vec<_> = points
            .iter()
            .filter(|pt| bbox_copy.contains_point(pt))
            .cloned()
            .collect();

        log_debug!(
            LOGGER_NAME,
            "Filtered {} points within bounding box",
            active_points.len()
        );

        // Publish debug filtered points if enabled (always publish, even if empty)
        if let Some(debug_pubs) = board_debug_publishers {
            log_debug!(
                LOGGER_NAME,
                "Publishing {} filtered points to debug/filtered_points",
                active_points.len()
            );
            let debug_cloud = Self::create_debug_pointcloud(&active_points, header)?;
            if let Err(e) = debug_pubs.filtered_points.publish(debug_cloud) {
                log_warn!(LOGGER_NAME, "Failed to publish debug filtered points: {e}");
            }
        }

        Ok(active_points)
    }

    // Stage 2: RANSAC plane detection
    fn detect_plane_ransac(
        detector: &Arc<BoardDetector>,
        active_points: &[na::Point3<f64>],
        header: &Header,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Result<Option<(PlaneModel, Vec<na::Point3<f64>>)>> {
        let config = detector.config();

        // Fit plane using RANSAC
        let plane_fit = match fit_plane_ransac(config, active_points) {
            Ok(Some(fit)) => fit,
            Ok(None) => {
                log_warn!(LOGGER_NAME, "Plane fitting failed - no valid plane found");
                return Ok(None);
            }
            Err(e) => {
                log_warn!(LOGGER_NAME, "Plane fitting error: {}", e);
                return Err(e.into());
            }
        };

        let plane_model = plane_fit.plane_model;
        let plane_inlier_points: Vec<na::Point3<f64>> =
            plane_fit.inlier_points.iter().map(|p| **p).collect();

        log_debug!(
            LOGGER_NAME,
            "RANSAC plane detection successful: {} inlier points found",
            plane_inlier_points.len()
        );

        // Publish debug plane inliers immediately after RANSAC success
        if let Some(debug_pubs) = board_debug_publishers {
            match Self::create_debug_pointcloud(&plane_inlier_points, header) {
                Ok(plane_inliers_msg) => {
                    let _ = debug_pubs.plane_inliers.publish(plane_inliers_msg);
                    log_debug!(
                        LOGGER_NAME,
                        "Published {} plane inlier points to debug/plane_inliers after RANSAC",
                        plane_inlier_points.len()
                    );
                }
                Err(e) => {
                    log_warn!(LOGGER_NAME, "Failed to create plane inliers message: {e}");
                }
            }
        }

        Ok(Some((plane_model, plane_inlier_points)))
    }

    // Stage 3: ICP board pose refinement
    fn detect_icp(
        detector: &Arc<BoardDetector>,
        plane_model: &PlaneModel,
        plane_inlier_points: &[na::Point3<f64>],
        ransac_data: PlaneRansacData,
        header: &Header,
        icp_debug_publishers: &Option<IcpDebugPublishers>,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Option<BoardDetection> {
        log_info!(LOGGER_NAME, "Starting ICP board pose refinement");

        // Check if we have enough inlier points
        if plane_inlier_points.is_empty() {
            log_warn!(LOGGER_NAME, "No plane inlier points available for ICP");
            return None;
        }

        // Extract detector configuration and aruco pattern
        let config = detector.config();
        let aruco_pattern = detector.aruco_pattern();

        // Extract board shape and marker paper size
        let BoardDetectorConfig {
            board_shape:
                BoardShape {
                    board_width,
                    hole_radius,
                    hole_center_shift,
                },
            ..
        } = *config;
        let marker_paper_size = aruco_pattern.paper_size();

        // Create BoardModelParams
        let board_model_params = BoardModelParams {
            board_shape: BoardShape {
                board_width,
                hole_radius,
                hole_center_shift,
            },
            marker_paper_size,
        };

        // Step 3: Create initial pose using PCA-based alignment
        let initial_pose =
            Self::compute_initial_pose_pca(plane_inlier_points, header, board_debug_publishers)?;

        // Publish initial board pose for debugging (if debug publishers available)
        if let Some(debug_pubs) = board_debug_publishers {
            let initial_board_model = BoardModel {
                pose: initial_pose,
                board_shape: board_model_params.board_shape.clone(),
                marker_paper_size: board_model_params.marker_paper_size,
            };
            if let Ok(initial_markers) =
                Self::create_board_markers_from_model(&initial_board_model, header)
            {
                let _ = debug_pubs.initial_board_marker.publish(initial_markers);
                log_debug!(LOGGER_NAME, "Published initial board pose markers from PCA");
            }

            // Publish RANSAC plane visualization
            if let Ok(plane_markers) =
                Self::create_plane_marker(plane_model, plane_inlier_points, header)
            {
                let _ = debug_pubs.plane_marker.publish(plane_markers);
                log_debug!(LOGGER_NAME, "Published RANSAC plane marker");

                // Debug: Compare RANSAC plane normal with PCA board pose z-axis
                let ransac_normal = plane_model.normal;
                let board_z_axis = initial_pose.rotation * na::Vector3::z();
                let alignment = ransac_normal.dot(&board_z_axis);
                log_debug!(
                    LOGGER_NAME,
                    "RANSAC normal: ({:.3}, {:.3}, {:.3}), Board Z-axis: ({:.3}, {:.3}, {:.3}), Alignment: {:.3}",
                    ransac_normal.x, ransac_normal.y, ransac_normal.z,
                    board_z_axis.x, board_z_axis.y, board_z_axis.z,
                    alignment
                );
            }
        }

        let initial_inlier_points: Vec<na::Point3<f64>> =
            plane_inlier_points.iter().cloned().collect();

        // Step 4: Create BoardIcpIterator
        let mut iterator = BoardIcpIterator::new(
            config,
            board_model_params.clone(),
            None, // No progress callback as we handle debug publishing ourselves
        );

        // Step 5: Create initial state
        let mut state = iterator.initial_state(initial_pose, initial_inlier_points);

        log_debug!(
            LOGGER_NAME,
            "Starting ICP iterations with initial pose: {:?}",
            state.board_pose
        );

        // Step 6: Iterate with optional debug publishing
        loop {
            // Perform one ICP iteration step FIRST
            state = iterator.step(&state);

            log_debug!(
                LOGGER_NAME,
                "ICP iteration {}: avg_loss={:.6}, good_correspondences={}/{}",
                state.iteration,
                state.avg_loss,
                state.good_correspondences,
                state.total_correspondences
            );

            // Publish debug information if debug publishers are available
            if let Some(debug_pubs) = icp_debug_publishers {
                Self::publish_icp_iteration(&state, &board_model_params, header, debug_pubs);
            }

            // Publish board model visualization
            if let Some(debug_pubs) = board_debug_publishers {
                let board_model = BoardModel {
                    pose: state.board_pose,
                    board_shape: board_model_params.board_shape.clone(),
                    marker_paper_size: board_model_params.marker_paper_size,
                };
                if let Ok(arr) = Self::create_board_markers_from_model(&board_model, header) {
                    let _ = debug_pubs.board_marker_icp.publish(arr);
                }
            }

            // Add small delay between iterations only in debug mode for better visualization
            if icp_debug_publishers.is_some() {
                thread::sleep(Duration::from_millis(50));
            }

            // Check termination condition AFTER the step
            if iterator.should_terminate(&state) {
                let reason = iterator.termination_reason(&state);
                log_info!(LOGGER_NAME, "ICP iteration terminated: {}", reason);
                break;
            }
        }

        // Step 7: Check if result is successful
        if state.avg_loss < config.icp_good_fit_threshold
            && state.inlier_points.len() >= config.icp_min_inlier_points
        {
            log_info!(
                LOGGER_NAME,
                "Board detection successful: final_loss={:.6}, inliers={}",
                state.avg_loss,
                state.inlier_points.len()
            );

            // Create final board model and detection
            let board_model = BoardModel {
                pose: state.board_pose,
                board_shape: board_model_params.board_shape,
                marker_paper_size: board_model_params.marker_paper_size,
            };

            // Create Detection with all required fields
            Some(BoardDetection {
                board_model: board_model.clone(),
                plane_ransac_data: ransac_data,
                icp_data: hollow_board_detector::detection::IcpData {
                    correspondences: state.correspondences.clone(),
                    board_model,
                },
                icp_losses: vec![state.avg_loss], // Single final loss value
                initial_pose,
                icp_stats: IcpStatistics {
                    iterations: state.iteration,
                    final_loss: state.avg_loss,
                    min_loss: state.avg_loss, // In our implementation, final loss is the minimum we reached
                    successful: true, // We only reach this point if detection was successful
                    initial_loss: f64::INFINITY, // We don't track initial loss in this implementation
                    convergence_reason: iterator.termination_reason(&state),
                },
            })
        } else {
            log_warn!(
                LOGGER_NAME,
                "Board detection failed: final_loss={:.6}, inliers={}, threshold={:.6}, min_inliers={}",
                state.avg_loss,
                state.inlier_points.len(),
                config.icp_good_fit_threshold,
                config.icp_min_inlier_points
            );
            None
        }
    }

    /// Compute initial board pose using PCA-based alignment
    fn compute_initial_pose_pca(
        plane_inlier_points: &[na::Point3<f64>],
        header: &Header,
        debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Option<na::Isometry3<f64>> {
        if plane_inlier_points.is_empty() {
            log_warn!(LOGGER_NAME, "Cannot compute PCA pose with empty point set");
            return None;
        }

        if plane_inlier_points.len() < 3 {
            log_warn!(
                LOGGER_NAME,
                "Need at least 3 points for PCA, got {}",
                plane_inlier_points.len()
            );
            return None;
        }

        // Step 1: Compute centroid
        let centroid = plane_inlier_points
            .iter()
            .fold(na::Vector3::zeros(), |acc, point| acc + point.coords)
            / (plane_inlier_points.len() as f64);

        log_debug!(
            LOGGER_NAME,
            "PCA pose initialization: centroid=({:.3}, {:.3}, {:.3}), {} points",
            centroid.x,
            centroid.y,
            centroid.z,
            plane_inlier_points.len()
        );

        // Step 2: Create data matrix for PCA using petal-decomposition
        // petal-decomposition expects data as (n_samples, n_features) = (n_points, 3)
        let n_points = plane_inlier_points.len();
        let mut data_array = Array2::<f64>::zeros((n_points, 3));

        for (row_idx, point) in plane_inlier_points.iter().enumerate() {
            data_array[[row_idx, 0]] = point.x;
            data_array[[row_idx, 1]] = point.y;
            data_array[[row_idx, 2]] = point.z;
        }

        // Step 3: Perform PCA using petal-decomposition (keeps all 3 components)
        let mut pca = PcaBuilder::new(3).build();
        if let Err(e) = pca.fit(&data_array.view()) {
            log_warn!(LOGGER_NAME, "PCA fit failed: {}", e);
            return None;
        }

        // Step 4: Get singular values and components
        let singular_values = pca.singular_values();
        let explained_variance = pca.explained_variance_ratio();

        log_debug!(
            LOGGER_NAME,
            "PCA singular values: [{:.6}, {:.6}, {:.6}]",
            singular_values[0],
            singular_values[1],
            singular_values[2]
        );
        log_debug!(
            LOGGER_NAME,
            "Explained variance ratio: [{:.6}, {:.6}, {:.6}]",
            explained_variance[0],
            explained_variance[1],
            explained_variance[2]
        );

        // Step 5: Extract principal components (eigenvectors)
        // petal-decomposition returns components as (n_components, n_features) = (3, 3)
        // Each row is a principal component
        let components = pca.components();

        // Extract the three principal components
        // PC0 has largest variance (lies in plane), PC2 has smallest variance (normal to plane)
        let mut v1 = na::Vector3::new(components[[0, 0]], components[[0, 1]], components[[0, 2]]); // 1st PC - largest variance (in plane)
        let mut v2 = na::Vector3::new(components[[1, 0]], components[[1, 1]], components[[1, 2]]); // 2nd PC - middle variance (in plane)
        let mut v3 = na::Vector3::new(components[[2, 0]], components[[2, 1]], components[[2, 2]]); // 3rd PC - smallest variance (normal to plane)

        log_debug!(
            LOGGER_NAME,
            "PCA component assignment: v1=PC0({:.6}), v2=PC1({:.6}), v3=PC2({:.6})",
            singular_values[0],
            singular_values[1],
            singular_values[2]
        );

        log_debug!(
            LOGGER_NAME,
            "Initial eigenvectors: v1=({:.3}, {:.3}, {:.3}), v2=({:.3}, {:.3}, {:.3}), v3=({:.3}, {:.3}, {:.3})",
            v1.x, v1.y, v1.z,
            v2.x, v2.y, v2.z,
            v3.x, v3.y, v3.z
        );

        // Publish raw eigenvectors for debugging (before any orientation constraints)
        if let Some(debug_pubs) = debug_publishers {
            if let Ok(eigenvector_markers) =
                Self::create_pca_eigenvector_markers(&centroid, &v1, &v2, &v3, header)
            {
                let _ = debug_pubs.pca_eigenvectors.publish(eigenvector_markers);
                log_debug!(LOGGER_NAME, "Published raw PCA eigenvectors");
            }
        }

        // Step 6: Apply orientation constraints
        // Ensure v3 (normal) points toward camera (positive z in camera frame)
        // Assuming camera is above the calibration board
        if v3.z < 0.0 {
            v3 = -v3;
            log_debug!(LOGGER_NAME, "Flipped v3 to point toward camera");
        }

        // Ensure v1 and v2 have positive z components (point generally upward)
        if v1.z < 0.0 {
            v1 = -v1;
            log_debug!(LOGGER_NAME, "Flipped v1 for positive z component");
        }
        if v2.z < 0.0 {
            v2 = -v2;
            log_debug!(LOGGER_NAME, "Flipped v2 for positive z component");
        }

        // Ensure right-hand rule: v3 = v1 × v2
        let cross_product = v1.cross(&v2);
        if cross_product.dot(&v3) < 0.0 {
            std::mem::swap(&mut v1, &mut v2); // Swap v1 and v2 to maintain right-hand rule
            log_debug!(LOGGER_NAME, "Swapped v1 and v2 to maintain right-hand rule");
        }

        // Step 7: Create rotation from eigenvectors using UnitQuaternion
        // v1 -> x-axis, v2 -> y-axis, v3 -> z-axis
        let pca_rotation_matrix = na::Matrix3::from_columns(&[v1, v2, v3]);
        let pca_rotation = UnitQuaternion::from_matrix(&pca_rotation_matrix);

        log_debug!(
            LOGGER_NAME,
            "Final eigenvectors: v1=({:.3}, {:.3}, {:.3}), v2=({:.3}, {:.3}, {:.3}), v3=({:.3}, {:.3}, {:.3})",
            v1.x, v1.y, v1.z,
            v2.x, v2.y, v2.z,
            v3.x, v3.y, v3.z
        );

        // Step 7.5: Apply -45 degree rotation around plane normal to align with board borders
        // PCA eigenvectors align with diagonal directions, but we want x/y axes aligned with board edges
        let rotation_angle = std::f64::consts::FRAC_PI_4; // -45 degrees
        let plane_normal_unit = na::Unit::new_normalize(v3);
        let normal_rotation = UnitQuaternion::from_axis_angle(&plane_normal_unit, rotation_angle);

        // Compose rotations: first PCA, then rotate around plane normal
        let final_rotation = normal_rotation * pca_rotation;

        log_debug!(
            LOGGER_NAME,
            "Applied -45° rotation around plane normal to align with board borders"
        );

        // Add assertions to verify correctness (debug builds only)
        #[cfg(debug_assertions)]
        {
            let rotation_matrix_obj = final_rotation.to_rotation_matrix();
            let rotation_matrix = rotation_matrix_obj.matrix();
            let det = rotation_matrix.determinant();
            log_debug!(LOGGER_NAME, "Rotation matrix determinant: {:.6}", det);
            assert!(
                (det - 1.0).abs() < 1e-6,
                "Rotation matrix determinant should be 1.0, got {}",
                det
            );

            // Check orthogonality
            let should_be_identity = rotation_matrix * rotation_matrix.transpose();
            let identity = na::Matrix3::<f64>::identity();
            let diff_norm = (&should_be_identity - &identity).norm();
            log_debug!(
                LOGGER_NAME,
                "Orthogonality check (should be ~0): {:.6}",
                diff_norm
            );
            assert!(
                diff_norm < 1e-6,
                "Rotation matrix should be orthogonal, difference norm: {}",
                diff_norm
            );
        }

        // Check right-hand rule (debug builds only)
        #[cfg(debug_assertions)]
        {
            let computed_v3 = v1.cross(&v2);
            let v3_alignment = computed_v3.dot(&v3);
            log_debug!(
                LOGGER_NAME,
                "Right-hand rule check (v1 × v2 · v3, should be ~1): {:.6}",
                v3_alignment
            );
            assert!(
                v3_alignment > 0.9,
                "Right-hand rule violated, alignment: {}",
                v3_alignment
            );
        }

        // Step 8: Use the final composed rotation
        let rotation = final_rotation;

        let pose = na::Isometry3::from_parts(Translation3::from(centroid), rotation);

        log_info!(
            LOGGER_NAME,
            "PCA initial pose: translation=({:.3}, {:.3}, {:.3}), rotation=({:.3}, {:.3}, {:.3}, {:.3})",
            pose.translation.x,
            pose.translation.y,
            pose.translation.z,
            pose.rotation.i,
            pose.rotation.j,
            pose.rotation.k,
            pose.rotation.w
        );

        Some(pose)
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
            size: GeomVector3 {
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
        let q = bbox.pose.rotation.quaternion();

        let marker = Marker {
            header: header.clone(),
            ns: "bbox".to_string(),
            id: 0,
            type_: 1,  // CUBE
            action: 0, // ADD
            pose: geometry_msgs::msg::Pose {
                position: Point {
                    x: bbox.pose.translation.x,
                    y: bbox.pose.translation.y,
                    z: bbox.pose.translation.z,
                },
                orientation: Quaternion {
                    x: q.i,
                    y: q.j,
                    z: q.k,
                    w: q.w,
                },
            },
            scale: GeomVector3 {
                x: bbox.size_xyz[0],
                y: bbox.size_xyz[1],
                z: bbox.size_xyz[2],
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.2,
            },
            ..Default::default()
        };

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
        let board_model = &board_detection.board_model;

        // Base pose
        let base_translation = &board_model.pose.translation;
        let base_rotation = &board_model.pose.rotation;

        // Board cube marker (id 0)
        let board_cube = {
            let q = base_rotation.quaternion();
            Marker {
                header: header.clone(),
                ns: "board".to_string(),
                id: 0,
                type_: 1,  // CUBE
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: base_translation.x,
                        y: base_translation.y,
                        z: base_translation.z,
                    },
                    orientation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
                scale: GeomVector3 {
                    x: board_model.board_shape.board_width.as_meters(),
                    y: board_model.board_shape.board_width.as_meters(),
                    z: 0.02, // 2 cm thickness
                },
                color: ColorRGBA {
                    r: 0.0,
                    g: 1.0,
                    b: 0.0,
                    a: 0.6,
                },
                ..Default::default()
            }
        };

        // Helper to build an arrow marker oriented along the board frame's X axis, then rotated
        let make_axis_arrow =
            |id: i32, rot_after_x: na::UnitQuaternion<f64>, r: f32, g: f32, b: f32| -> Marker {
                let rot = base_rotation * rot_after_x;
                let q = rot.quaternion();
                let len = (board_model.board_shape.board_width.as_meters() * 0.5) as f64;

                Marker {
                    header: header.clone(),
                    ns: "board_axes".to_string(),
                    id,
                    type_: 0,  // ARROW
                    action: 0, // ADD
                    pose: geometry_msgs::msg::Pose {
                        position: Point {
                            x: base_translation.x,
                            y: base_translation.y,
                            z: base_translation.z,
                        },
                        orientation: Quaternion {
                            x: q.i,
                            y: q.j,
                            z: q.k,
                            w: q.w,
                        },
                    },
                    scale: GeomVector3 {
                        x: len,  // shaft length
                        y: 0.02, // shaft diameter
                        z: 0.04, // head diameter
                    },
                    color: ColorRGBA { r, g, b, a: 1.0 },
                    ..Default::default()
                }
            };

        // Rotations to map X axis to Y/Z in the board frame
        let rot_x = na::UnitQuaternion::identity();
        let rot_y = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), FRAC_PI_2);
        let rot_z = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -FRAC_PI_2);

        let x_arrow = make_axis_arrow(1, rot_x, 1.0, 0.0, 0.0); // Red X
        let y_arrow = make_axis_arrow(2, rot_y, 0.0, 1.0, 0.0); // Green Y
        let z_arrow = make_axis_arrow(3, rot_z, 0.0, 0.0, 1.0); // Blue Z

        let arr = MarkerArray {
            markers: vec![board_cube, x_arrow, y_arrow, z_arrow],
        };
        Ok(arr)
    }

    /// Create a circular plane marker to visualize the RANSAC-detected plane
    fn create_plane_marker(
        plane_model: &PlaneModel,
        plane_inlier_points: &[na::Point3<f64>],
        header: &Header,
    ) -> Result<MarkerArray> {
        // Compute centroid of inlier points
        let centroid = plane_inlier_points
            .iter()
            .fold(na::Vector3::zeros(), |acc, point| acc + point.coords)
            / (plane_inlier_points.len() as f64);

        // Simply use the plane normal directly - no rotation corrections
        let normal = plane_model.normal;
        log_debug!(
            LOGGER_NAME,
            "Plane normal (RANSAC): ({:.3}, {:.3}, {:.3})",
            normal.x,
            normal.y,
            normal.z
        );

        // Create rotation to align z-axis with plane normal
        let z_axis = na::Vector3::new(0.0, 0.0, 1.0);
        let rotation_quat = if normal.dot(&z_axis).abs() > 0.999 {
            na::UnitQuaternion::identity()
        } else {
            na::UnitQuaternion::rotation_between(&z_axis, &normal)
                .unwrap_or(na::UnitQuaternion::identity())
        };

        // Create a circular plane marker
        let marker = Marker {
            header: header.clone(),
            ns: "ransac_plane".to_string(),
            id: 0,
            type_: 3,  // CYLINDER for a circular plane
            action: 0, // ADD
            pose: geometry_msgs::msg::Pose {
                position: Point {
                    x: centroid.x,
                    y: centroid.y,
                    z: centroid.z,
                },
                orientation: Quaternion {
                    x: rotation_quat.i,
                    y: rotation_quat.j,
                    z: rotation_quat.k,
                    w: rotation_quat.w,
                },
            },
            scale: GeomVector3 {
                x: 1.0,  // diameter
                y: 1.0,  // diameter
                z: 0.01, // thin disk
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 1.0,
                a: 0.5, // Semi-transparent cyan for RANSAC plane
            },
            ..Default::default()
        };

        Ok(MarkerArray {
            markers: vec![marker],
        })
    }

    fn create_board_markers_from_model(
        board_model: &hollow_board_config::BoardModel,
        header: &Header,
    ) -> Result<MarkerArray> {
        let base_translation = &board_model.pose.translation;
        let base_rotation = &board_model.pose.rotation;

        let board_cube = {
            let q = base_rotation.quaternion();
            Marker {
                header: header.clone(),
                ns: "board_icp".to_string(),
                id: 1000,
                type_: 1,  // CUBE
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: base_translation.x,
                        y: base_translation.y,
                        z: base_translation.z,
                    },
                    orientation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
                scale: GeomVector3 {
                    x: board_model.board_shape.board_width.as_meters(),
                    y: board_model.board_shape.board_width.as_meters(),
                    z: 0.02,
                },
                color: ColorRGBA {
                    r: 1.0,
                    g: 0.5,
                    b: 0.0,
                    a: 0.3,
                },
                ..Default::default()
            }
        };

        let make_axis_arrow =
            |id: i32, rot_after_x: na::UnitQuaternion<f64>, r: f32, g: f32, b: f32| -> Marker {
                let rot = base_rotation * rot_after_x;
                let q = rot.quaternion();
                let len = (board_model.board_shape.board_width.as_meters() * 0.5) as f64;

                Marker {
                    header: header.clone(),
                    ns: "board_axes_icp".to_string(),
                    id,
                    type_: 0,  // ARROW
                    action: 0, // ADD
                    pose: geometry_msgs::msg::Pose {
                        position: Point {
                            x: base_translation.x,
                            y: base_translation.y,
                            z: base_translation.z,
                        },
                        orientation: Quaternion {
                            x: q.i,
                            y: q.j,
                            z: q.k,
                            w: q.w,
                        },
                    },
                    scale: GeomVector3 {
                        x: len,
                        y: 0.02,
                        z: 0.04,
                    },
                    color: ColorRGBA { r, g, b, a: 0.9 },
                    ..Default::default()
                }
            };

        let rot_x = na::UnitQuaternion::identity();
        let rot_y = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), FRAC_PI_2);
        let rot_z = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -FRAC_PI_2);

        let x_arrow = make_axis_arrow(1001, rot_x, 1.0, 0.2, 0.2);
        let y_arrow = make_axis_arrow(1002, rot_y, 0.2, 1.0, 0.2);
        let z_arrow = make_axis_arrow(1003, rot_z, 0.2, 0.2, 1.0);

        let arr = MarkerArray {
            markers: vec![board_cube, x_arrow, y_arrow, z_arrow],
        };
        Ok(arr)
    }

    // Helper functions for ICP iteration debug publishing debug
    // publishing function using IcpDebugPublishers struct
    fn publish_icp_iteration(
        state: &BoardIcpState,
        board_model_params: &BoardModelParams,
        header: &Header,
        debug_publishers: &IcpDebugPublishers,
    ) {
        // Publish iteration pose
        if let Ok(pose_msg) = Self::board_state_to_pose(state, header) {
            let _ = debug_publishers.iteration_pose.publish(pose_msg);
        }

        // Publish board model points
        if let Ok(points_msg) = Self::board_state_to_pointcloud(state, board_model_params, header) {
            let _ = debug_publishers.board_points.publish(points_msg);
        }

        // Publish correspondences as line markers
        if let Ok(markers_msg) = Self::correspondences_to_markers(state, header) {
            let _ = debug_publishers.correspondences.publish(markers_msg);
        }

        // Publish current loss value
        let loss_msg = Float64 {
            data: state.avg_loss,
        };
        let _ = debug_publishers.loss.publish(loss_msg);

        // Publish iteration statistics
        let stats_text = format!(
            "Iteration: {}, Loss: {:.6}, Correspondences: {}/{}, Threshold: {:.6}",
            state.iteration,
            state.avg_loss,
            state.good_correspondences,
            state.total_correspondences,
            state.adaptive_threshold
        );
        let stats_msg = StringMsg { data: stats_text };
        let _ = debug_publishers.stats.publish(stats_msg);
    }

    fn board_state_to_pose(state: &BoardIcpState, header: &Header) -> Result<PoseStamped> {
        let pose = Pose {
            position: Point {
                x: state.board_pose.translation.x,
                y: state.board_pose.translation.y,
                z: state.board_pose.translation.z,
            },
            orientation: Quaternion {
                x: state.board_pose.rotation.i,
                y: state.board_pose.rotation.j,
                z: state.board_pose.rotation.k,
                w: state.board_pose.rotation.w,
            },
        };

        Ok(PoseStamped {
            header: header.clone(),
            pose,
        })
    }

    fn board_state_to_pointcloud(
        state: &BoardIcpState,
        board_model_params: &BoardModelParams,
        header: &Header,
    ) -> Result<PointCloud2> {
        // Create board model using current pose
        let board_model = BoardModel {
            pose: state.board_pose,
            board_shape: board_model_params.board_shape.clone(),
            marker_paper_size: board_model_params.marker_paper_size,
        };

        // Generate board model points (corners and hole centers for visualization)
        let mut points = Vec::new();

        // Add board corners
        points.push(board_model.top_corner());
        points.push(board_model.bottom_corner());
        points.push(board_model.left_corner());
        points.push(board_model.right_corner());

        // Add hole centers
        points.push(board_model.left_circle_center());
        points.push(board_model.right_circle_center());
        points.push(board_model.top_circle_center());

        // Add marker corners for more detail
        points.push(board_model.marker_bottom_corner());
        points.push(board_model.marker_top_corner());
        points.push(board_model.marker_left_corner());
        points.push(board_model.marker_right_corner());
        points.push(board_model.marker_center());

        Self::create_debug_pointcloud(&points, header)
    }

    /// Create arrow markers for raw PCA eigenvectors before any orientation constraints
    fn create_pca_eigenvector_markers(
        centroid: &na::Vector3<f64>,
        v1: &na::Vector3<f64>,
        v2: &na::Vector3<f64>,
        v3: &na::Vector3<f64>,
        header: &Header,
    ) -> Result<MarkerArray> {
        let mut markers = Vec::new();

        // Scale factor for eigenvector arrows
        let scale = 0.3;

        // V1 (1st PC - largest variance) - RED
        let v1_marker = {
            // Direction from centroid along v1
            let direction = v1.normalize();
            let q = na::UnitQuaternion::rotation_between(&na::Vector3::x(), &direction)
                .unwrap_or(na::UnitQuaternion::identity());

            Marker {
                header: header.clone(),
                ns: "pca_eigenvectors".to_string(),
                id: 0,
                type_: 0,  // ARROW
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: centroid.x,
                        y: centroid.y,
                        z: centroid.z,
                    },
                    orientation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
                scale: GeomVector3 {
                    x: scale, // shaft length
                    y: 0.02,  // shaft diameter
                    z: 0.04,  // head diameter
                },
                color: ColorRGBA {
                    r: 1.0, // RED
                    g: 0.0,
                    b: 0.0,
                    a: 1.0,
                },
                ..Default::default()
            }
        };
        markers.push(v1_marker);

        // V2 (2nd PC) - GREEN
        let v2_marker = {
            let direction = v2.normalize();
            let q = na::UnitQuaternion::rotation_between(&na::Vector3::x(), &direction)
                .unwrap_or(na::UnitQuaternion::identity());

            Marker {
                header: header.clone(),
                ns: "pca_eigenvectors".to_string(),
                id: 1,
                type_: 0,  // ARROW
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: centroid.x,
                        y: centroid.y,
                        z: centroid.z,
                    },
                    orientation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
                scale: GeomVector3 {
                    x: scale,
                    y: 0.02,
                    z: 0.04,
                },
                color: ColorRGBA {
                    r: 0.0,
                    g: 1.0, // GREEN
                    b: 0.0,
                    a: 1.0,
                },
                ..Default::default()
            }
        };
        markers.push(v2_marker);

        // V3 (3rd PC - smallest variance, normal) - BLUE
        let v3_marker = {
            let direction = v3.normalize();
            let q = na::UnitQuaternion::rotation_between(&na::Vector3::x(), &direction)
                .unwrap_or(na::UnitQuaternion::identity());

            Marker {
                header: header.clone(),
                ns: "pca_eigenvectors".to_string(),
                id: 2,
                type_: 0,  // ARROW
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: centroid.x,
                        y: centroid.y,
                        z: centroid.z,
                    },
                    orientation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
                scale: GeomVector3 {
                    x: scale,
                    y: 0.02,
                    z: 0.04,
                },
                color: ColorRGBA {
                    r: 0.0,
                    g: 0.0,
                    b: 1.0, // BLUE
                    a: 1.0,
                },
                ..Default::default()
            }
        };
        markers.push(v3_marker);

        Ok(MarkerArray { markers })
    }

    fn correspondences_to_markers(state: &BoardIcpState, header: &Header) -> Result<MarkerArray> {
        let mut markers = Vec::new();

        // Create line markers for each correspondence
        for (i, (data_point, model_point)) in state.correspondences.iter().enumerate() {
            // Create line from data point to model point
            let start_point = Point {
                x: data_point.x,
                y: data_point.y,
                z: data_point.z,
            };
            let end_point = Point {
                x: model_point.x,
                y: model_point.y,
                z: model_point.z,
            };

            let marker = Marker {
                header: header.clone(),
                ns: "icp_correspondences".to_string(),
                id: i as i32,
                type_: 4,  // LINE_STRIP
                action: 0, // ADD
                points: vec![start_point, end_point],
                scale: GeomVector3 {
                    x: 0.002, // Line width
                    y: 0.0,
                    z: 0.0,
                },
                color: ColorRGBA {
                    r: 1.0, // Red lines
                    g: 0.0,
                    b: 0.0,
                    a: 0.8, // Semi-transparent
                },
                ..Default::default()
            };

            markers.push(marker);
        }

        Ok(MarkerArray { markers })
    }
}

fn main() -> Result<()> {
    // Initialize logging for the hollow-board-detector library
    init_logging();

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("lidar_board_detector")?;
    let _lidar_board_detector_node = CalibrationBoardLocatorNode::new(node)?;

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
