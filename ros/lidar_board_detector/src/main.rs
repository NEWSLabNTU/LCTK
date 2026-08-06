mod bbox;
mod bbox_free;

use crate::bbox::BBox;
use anyhow::{anyhow, Result};
use arc_swap::ArcSwap;
use aruco_config::MultiArucoPattern;
use geometry_msgs::msg::{
    Point, Pose, PoseStamped, PoseWithCovariance, Quaternion, Vector3 as GeomVector3,
};
use hollow_board_config::{BoardModel, BoardShape};
use hollow_board_detector::{
    algo::{fit_plane_ransac, voxel_downsample, BoardIcpIterator},
    detection::{BoardIcpState, BoardModelParams, IcpStatistics, PlaneRansacData},
    config::SensorUpAxis, init_logging, Config as BoardDetectorConfig,
    Detection as BoardDetection, Detector as BoardDetector,
};
use nalgebra as na;
use plane_estimator::PlaneModel;
use rclrs::{MandatoryParameter, ParameterRange, PublisherOptions, SubscriptionOptions, *};
use sensor_msgs::msg::{PointCloud2, PointField};
use std::{
    f64::consts::FRAC_PI_2,
    fs,
    path::PathBuf,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc,
    },
    thread::JoinHandle,
    time::Instant,
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
    /// bbox_free only: the RAW Method-E / plane-strip foreground (points before
    /// clustering/merge/gate), on the downsampled cloud. Silent in bbox mode.
    foreground_points: Arc<Publisher<PointCloud2>>,
    /// bbox_free background_subtraction only: cell centers of the finalized
    /// warmup background model — what "static scene" got baked in. Silent in
    /// bbox mode and during warmup.
    background_voxels: Arc<Publisher<PointCloud2>>,
    plane_inliers: Arc<Publisher<PointCloud2>>,
    downsampled_points: Arc<Publisher<PointCloud2>>,
    plane_marker: Arc<Publisher<MarkerArray>>,
    bbox_marker: Arc<Publisher<MarkerArray>>,
    board_marker: Arc<Publisher<MarkerArray>>,
    board_marker_icp: Arc<Publisher<MarkerArray>>,
    initial_board_marker: Arc<Publisher<MarkerArray>>,
    icp_stats: Arc<Publisher<StringMsg>>,
    #[allow(dead_code)]
    pca_eigenvectors: Arc<Publisher<MarkerArray>>,
}

/// ROS parameters for bounding box filter configuration.
/// These parameters can be changed at runtime via `ros2 param set`.
///
/// Example usage:
/// ```bash
/// ros2 param set /lidar_board_detector bbox_center_x 2.5
/// ros2 param set /lidar_board_detector bbox_size_x 1.5
/// ```
pub struct BBoxParameters {
    // Position (center of bounding box)
    center_x: Arc<MandatoryParameter<f64>>,
    center_y: Arc<MandatoryParameter<f64>>,
    center_z: Arc<MandatoryParameter<f64>>,
    // Rotation (quaternion: w, x, y, z)
    rotation_w: Arc<MandatoryParameter<f64>>,
    rotation_x: Arc<MandatoryParameter<f64>>,
    rotation_y: Arc<MandatoryParameter<f64>>,
    rotation_z: Arc<MandatoryParameter<f64>>,
    // Size (dimensions in x, y, z)
    size_x: Arc<MandatoryParameter<f64>>,
    size_y: Arc<MandatoryParameter<f64>>,
    size_z: Arc<MandatoryParameter<f64>>,
}

impl Clone for BBoxParameters {
    fn clone(&self) -> Self {
        Self {
            center_x: Arc::clone(&self.center_x),
            center_y: Arc::clone(&self.center_y),
            center_z: Arc::clone(&self.center_z),
            rotation_w: Arc::clone(&self.rotation_w),
            rotation_x: Arc::clone(&self.rotation_x),
            rotation_y: Arc::clone(&self.rotation_y),
            rotation_z: Arc::clone(&self.rotation_z),
            size_x: Arc::clone(&self.size_x),
            size_y: Arc::clone(&self.size_y),
            size_z: Arc::clone(&self.size_z),
        }
    }
}

impl BBoxParameters {
    /// Declare all bbox parameters on the node with defaults from the given BBox.
    pub fn declare(node: &Node, defaults: &BBox) -> Result<Self> {
        let translation = &defaults.pose.translation;
        let quaternion = defaults.pose.rotation.quaternion();

        let center_x = node
            .declare_parameter::<f64>("bbox_center_x")
            .default(translation.x)
            .description("BBox center position X (meters)")
            .range(ParameterRange {
                lower: None,
                upper: None,
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_center_x: {e}"))?;

        let center_y = node
            .declare_parameter::<f64>("bbox_center_y")
            .default(translation.y)
            .description("BBox center position Y (meters)")
            .range(ParameterRange {
                lower: None,
                upper: None,
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_center_y: {e}"))?;

        let center_z = node
            .declare_parameter::<f64>("bbox_center_z")
            .default(translation.z)
            .description("BBox center position Z (meters)")
            .range(ParameterRange {
                lower: None,
                upper: None,
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_center_z: {e}"))?;

        let rotation_w = node
            .declare_parameter::<f64>("bbox_rotation_w")
            .default(quaternion.w)
            .description("BBox rotation quaternion W component")
            .range(ParameterRange {
                lower: Some(-1.0),
                upper: Some(1.0),
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_rotation_w: {e}"))?;

        let rotation_x = node
            .declare_parameter::<f64>("bbox_rotation_x")
            .default(quaternion.i)
            .description("BBox rotation quaternion X component")
            .range(ParameterRange {
                lower: Some(-1.0),
                upper: Some(1.0),
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_rotation_x: {e}"))?;

        let rotation_y = node
            .declare_parameter::<f64>("bbox_rotation_y")
            .default(quaternion.j)
            .description("BBox rotation quaternion Y component")
            .range(ParameterRange {
                lower: Some(-1.0),
                upper: Some(1.0),
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_rotation_y: {e}"))?;

        let rotation_z = node
            .declare_parameter::<f64>("bbox_rotation_z")
            .default(quaternion.k)
            .description("BBox rotation quaternion Z component")
            .range(ParameterRange {
                lower: Some(-1.0),
                upper: Some(1.0),
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_rotation_z: {e}"))?;

        let size_x = node
            .declare_parameter::<f64>("bbox_size_x")
            .default(defaults.size_xyz[0])
            .description("BBox size in X direction (meters)")
            .range(ParameterRange {
                lower: Some(0.0),
                upper: None,
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_size_x: {e}"))?;

        let size_y = node
            .declare_parameter::<f64>("bbox_size_y")
            .default(defaults.size_xyz[1])
            .description("BBox size in Y direction (meters)")
            .range(ParameterRange {
                lower: Some(0.0),
                upper: None,
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_size_y: {e}"))?;

        let size_z = node
            .declare_parameter::<f64>("bbox_size_z")
            .default(defaults.size_xyz[2])
            .description("BBox size in Z direction (meters)")
            .range(ParameterRange {
                lower: Some(0.0),
                upper: None,
                step: None,
            })
            .mandatory()
            .map_err(|e| anyhow!("Failed to declare bbox_size_z: {e}"))?;

        Ok(Self {
            center_x: Arc::new(center_x),
            center_y: Arc::new(center_y),
            center_z: Arc::new(center_z),
            rotation_w: Arc::new(rotation_w),
            rotation_x: Arc::new(rotation_x),
            rotation_y: Arc::new(rotation_y),
            rotation_z: Arc::new(rotation_z),
            size_x: Arc::new(size_x),
            size_y: Arc::new(size_y),
            size_z: Arc::new(size_z),
        })
    }

    /// Read current parameter values and construct a BBox.
    /// This method reads the latest values, reflecting any runtime parameter changes.
    pub fn to_bbox(&self) -> BBox {
        let translation = na::Translation3::new(
            self.center_x.get(),
            self.center_y.get(),
            self.center_z.get(),
        );

        let quaternion = na::UnitQuaternion::new_normalize(na::Quaternion::new(
            self.rotation_w.get(),
            self.rotation_x.get(),
            self.rotation_y.get(),
            self.rotation_z.get(),
        ));

        let pose = na::Isometry3::from_parts(translation, quaternion);
        let size_xyz = [self.size_x.get(), self.size_y.get(), self.size_z.get()];

        BBox { pose, size_xyz }
    }

    /// Log current parameter values.
    pub fn log_values(&self) {
        log_info!(
            LOGGER_NAME,
            "BBox parameters: center=({:.3}, {:.3}, {:.3}), rotation=({:.3}, {:.3}, {:.3}, {:.3}), size=({:.1}, {:.1}, {:.1})",
            self.center_x.get(),
            self.center_y.get(),
            self.center_z.get(),
            self.rotation_w.get(),
            self.rotation_x.get(),
            self.rotation_y.get(),
            self.rotation_z.get(),
            self.size_x.get(),
            self.size_y.get(),
            self.size_z.get()
        );
    }
}

pub struct CalibrationBoardLocatorNode {
    _node: Node,
    _detection_publisher: Publisher<Detection3DArray>,
    _pointcloud_subscription: Subscription<PointCloud2>,
    // Board debug publishers - grouped into a single struct
    _board_debug_publishers: Option<BoardDebugPublishers>,
    // ICP iteration debug publishers - grouped into a single struct
    _icp_debug_publishers: Option<IcpDebugPublishers>,
    // BBox parameters (dynamically reconfigurable via ROS parameters)
    _bbox_params: BBoxParameters,
    // Processing thread that handles point cloud processing
    _processing_thread: JoinHandle<()>,
    // Shared Method-E background state (kept alive for the processing thread; mutated by
    // reset_background in Task 6)
    _background_state: Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
    // Task 6: reset_background service handle — kept alive so it isn't dropped.
    // Only created (Some) when the bbox_free background-subtraction path is active;
    // `None` in default bbox mode so the service isn't registered unnecessarily.
    _reset_service: Option<rclrs::Service<std_srvs::srv::Empty>>,
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

        // Load initial bbox config from file
        log_info!(LOGGER_NAME, "Loading bbox config from: {}", bbox_file_param);
        let initial_bbox = Self::load_bbox_config(&bbox_file_param)?;

        // Declare bbox parameters with defaults from the config file
        // These can be changed at runtime via `ros2 param set`
        let bbox_params = BBoxParameters::declare(&node, &initial_bbox)?;
        bbox_params.log_values();
        log_info!(
            LOGGER_NAME,
            "Dynamic bbox params available: bbox_center_x, bbox_center_y, bbox_center_z, bbox_rotation_w, bbox_rotation_x, bbox_rotation_y, bbox_rotation_z, bbox_size_x, bbox_size_y, bbox_size_z"
        );
        log_info!(
            LOGGER_NAME,
            "Change at runtime with: ros2 param set /lidar_board_detector bbox_size_x <value>"
        );

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
        log_info!(
            LOGGER_NAME,
            "Loading board detector config from: {}",
            board_detector_file_param
        );
        let board_detector_config = Self::load_board_detector_config(&board_detector_file_param)?;
        log_info!(
            LOGGER_NAME,
            "Loaded board detector config: skip_ransac={}, ransac_threshold={:.3}m, ransac_iterations={}, icp_iterations={}",
            board_detector_config.skip_ransac,
            board_detector_config.plane_ransac_inlier_threshold,
            board_detector_config.plane_ransac_max_iterations,
            board_detector_config.max_icp_iterations
        );

        // Log voxel downsampling configuration
        if board_detector_config.voxel_downsample_enabled {
            log_info!(
                LOGGER_NAME,
                "Voxel downsampling ENABLED: size={:.3}m, use_centroid={}, parallel_threshold={}",
                board_detector_config.voxel_downsample_size,
                board_detector_config.voxel_downsample_use_centroid,
                board_detector_config.voxel_parallel_threshold
            );
        } else {
            log_info!(
                LOGGER_NAME,
                "Voxel downsampling DISABLED (preserving all points for ICP)"
            );
        }

        // Crop-box-free detection config (same file, separate typed view).
        let detection_text = fs::read_to_string(PathBuf::from(&*board_detector_file_param))?;
        let detection_cfg = bbox_free::parse_detection_config(&detection_text)?;
        let detection_mode_str = detection_cfg.detection_mode.clone();
        let detection_mode = bbox_free::DetectionMode::parse(&detection_mode_str)?;
        let bbox_free_cfg: Option<Arc<bbox_free::BboxFreeRaw>> = match detection_mode {
            bbox_free::DetectionMode::BboxFree => {
                // Detector params are flat in the config; bundle them.
                let bf = detection_cfg.into_bbox_free();
                bf.method()?; // validate foreground_method early
                Some(Arc::new(bf))
            }
            bbox_free::DetectionMode::Bbox => None,
        };
        log_info!(LOGGER_NAME, "detection_mode = {detection_mode_str}");

        // Shared background-subtraction state. `BackgroundState` is observed per frame by the single
        // processing thread; `reset_background` (Task 6) mutates it from a service/param callback,
        // so an `Arc<Mutex<Option<..>>>` is sufficient (the design's `ArcSwap<BackgroundState>` is
        // simplified to this — there is only one observer thread). `None` when the bbox_free path
        // is off or uses plane_strip (no background).
        let background_active = matches!(
            bbox_free_cfg.as_ref(),
            Some(bf) if bf.method()? == board_projection_detector::config::ForegroundMethod::BackgroundSubtraction
        );
        let background_state: Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>> =
            Arc::new(std::sync::Mutex::new(match bbox_free_cfg.as_ref() {
                Some(bf) if background_active => {
                    Some(bbox_free::BackgroundState::new(bf.voxel, &bf.background))
                }
                _ => None,
            }));

        // Task 6: runtime `reset_background` control (service path — rclrs 0.7's
        // `node.create_service::<T, _>(name, callback)` with `Fn(Request) -> Response` is
        // available and `std_srvs` resolves, so no fallback to a watched parameter was needed).
        // Re-enters warmup so an operator can re-capture the empty scene after moving the rig.
        // Only registered when background subtraction is active (`background_state` is Some) —
        // in default bbox mode it would be a permanent no-op, so skip the registration entirely.
        let reset_service = if background_active {
            let reset_bg = Arc::clone(&background_state);
            Some(node.create_service::<std_srvs::srv::Empty, _>(
                "~/reset_background",
                move |_request: std_srvs::srv::Empty_Request| {
                    if let Some(state) = reset_bg
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .as_mut()
                    {
                        state.reset();
                        log_info!(LOGGER_NAME, "background reset — re-entering warmup");
                    }
                    std_srvs::srv::Empty_Response::default()
                },
            )?)
        } else {
            None
        };

        log_info!(
            LOGGER_NAME,
            "Loading ArUco pattern config from: {}",
            aruco_pattern_file_param
        );
        let aruco_pattern_config = Self::load_aruco_pattern_config(&aruco_pattern_file_param)?;

        // Create detector
        let detector = Arc::new(BoardDetector::new(
            board_detector_config,
            aruco_pattern_config,
        ));

        // Create publisher for detections with QoS matching the mode
        // - BEST_EFFORT (realtime): Low latency, may drop messages
        // - RELIABLE (offline): No message drops, suitable for rosbag playback
        let mut detection_pub_opts = PublisherOptions::new("calibration_board_detections");
        detection_pub_opts.qos = if use_best_effort_qos {
            QoSProfile {
                history: QoSHistoryPolicy::KeepLast { depth: 1 },
                ..QoSProfile::sensor_data_default() // BEST_EFFORT
            }
        } else {
            QoSProfile {
                history: QoSHistoryPolicy::KeepLast { depth: 10 },
                ..QoSProfile::default() // RELIABLE
            }
        };
        let detection_publisher = node.create_publisher(detection_pub_opts)?;
        let detection_publisher_shared = Arc::clone(&detection_publisher);

        // Create board debug publishers if debug mode is enabled
        let board_debug_publishers = if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Debug mode enabled - creating debug publishers with best-effort QoS"
            );

            // Create best-effort QoS profile with depth=1 (latest only, no queue buildup)
            let mut debug_qos = QoSProfile::sensor_data_default();
            debug_qos.history = rclrs::QoSHistoryPolicy::KeepLast { depth: 1 };

            let mut all_points_opts = PublisherOptions::new("debug/all_points");
            all_points_opts.qos = debug_qos;

            let mut filtered_points_opts = PublisherOptions::new("debug/filtered_points");
            filtered_points_opts.qos = debug_qos;

            let mut foreground_points_opts = PublisherOptions::new("debug/foreground_points");
            foreground_points_opts.qos = debug_qos;

            let mut background_voxels_opts = PublisherOptions::new("debug/background_voxels");
            background_voxels_opts.qos = debug_qos;

            let mut plane_inliers_opts = PublisherOptions::new("debug/plane_inliers");
            plane_inliers_opts.qos = debug_qos;

            let mut downsampled_points_opts = PublisherOptions::new("debug/downsampled_points");
            downsampled_points_opts.qos = debug_qos;

            let mut plane_marker_opts = PublisherOptions::new("debug/plane_marker");
            plane_marker_opts.qos = debug_qos;

            let mut bbox_marker_opts = PublisherOptions::new("debug/bbox_marker");
            bbox_marker_opts.qos = debug_qos;

            let mut board_marker_opts = PublisherOptions::new("debug/final_board_pose");
            board_marker_opts.qos = debug_qos;

            let mut board_marker_icp_opts = PublisherOptions::new("debug/icp_iterations");
            board_marker_icp_opts.qos = debug_qos;

            let mut initial_board_marker_opts = PublisherOptions::new("debug/initial_board_marker");
            initial_board_marker_opts.qos = debug_qos;

            let mut icp_stats_opts = PublisherOptions::new("debug/icp_stats");
            icp_stats_opts.qos = debug_qos;

            let mut pca_eigenvectors_opts = PublisherOptions::new("debug/pca_eigenvectors");
            pca_eigenvectors_opts.qos = debug_qos;

            Some(BoardDebugPublishers {
                all_points: Arc::new(node.create_publisher(all_points_opts)?),
                filtered_points: Arc::new(node.create_publisher(filtered_points_opts)?),
                foreground_points: Arc::new(node.create_publisher(foreground_points_opts)?),
                background_voxels: Arc::new(node.create_publisher(background_voxels_opts)?),
                plane_inliers: Arc::new(node.create_publisher(plane_inliers_opts)?),
                downsampled_points: Arc::new(node.create_publisher(downsampled_points_opts)?),
                plane_marker: Arc::new(node.create_publisher(plane_marker_opts)?),
                bbox_marker: Arc::new(node.create_publisher(bbox_marker_opts)?),
                board_marker: Arc::new(node.create_publisher(board_marker_opts)?),
                board_marker_icp: Arc::new(node.create_publisher(board_marker_icp_opts)?),
                initial_board_marker: Arc::new(node.create_publisher(initial_board_marker_opts)?),
                icp_stats: Arc::new(node.create_publisher(icp_stats_opts)?),
                pca_eigenvectors: Arc::new(node.create_publisher(pca_eigenvectors_opts)?),
            })
        } else {
            None
        };
        let board_debug_shared = board_debug_publishers.clone();

        // ICP iteration debug publishers - grouped into single struct
        let icp_debug_publishers = if enable_icp_iteration_debug {
            log_info!(
                LOGGER_NAME,
                "ICP iteration debug mode enabled - creating iteration debug publishers with best-effort QoS"
            );

            // Create best-effort QoS profile with depth=1 (latest only, no queue buildup)
            let mut icp_debug_qos = QoSProfile::sensor_data_default();
            icp_debug_qos.history = rclrs::QoSHistoryPolicy::KeepLast { depth: 1 };

            let mut iteration_pose_opts =
                PublisherOptions::new("/calibration/icp_debug/iteration_pose");
            iteration_pose_opts.qos = icp_debug_qos;

            let mut board_points_opts =
                PublisherOptions::new("/calibration/icp_debug/board_points");
            board_points_opts.qos = icp_debug_qos;

            let mut correspondences_opts =
                PublisherOptions::new("/calibration/icp_debug/correspondences");
            correspondences_opts.qos = icp_debug_qos;

            let mut loss_opts = PublisherOptions::new("/calibration/icp_debug/loss");
            loss_opts.qos = icp_debug_qos;

            let mut stats_opts = PublisherOptions::new("/calibration/icp_debug/stats");
            stats_opts.qos = icp_debug_qos;

            Some(IcpDebugPublishers {
                iteration_pose: Arc::new(node.create_publisher(iteration_pose_opts)?),
                board_points: Arc::new(node.create_publisher(board_points_opts)?),
                correspondences: Arc::new(node.create_publisher(correspondences_opts)?),
                loss: Arc::new(node.create_publisher(loss_opts)?),
                stats: Arc::new(node.create_publisher(stats_opts)?),
            })
        } else {
            None
        };
        let icp_debug_shared = icp_debug_publishers.clone();

        // Configure QoS for sensor input topics
        let qos_profile = if use_best_effort_qos {
            let mut qos = QoSProfile::sensor_data_default();
            qos.history = rclrs::QoSHistoryPolicy::KeepLast { depth: 1 }; // Prevent buffering delays
            qos
        } else {
            QoSProfile::default() // Reliable for rosbag playback
        };

        // Counter for debugging message reception
        let message_counter = Arc::new(AtomicU64::new(0));
        let counter_clone = Arc::clone(&message_counter);

        // Clone bbox params for processing thread
        let bbox_params_for_callback = bbox_params.clone();

        // Clone crop-box-free config/state for processing thread
        let bbox_free_for_thread = bbox_free_cfg.clone();
        let background_for_thread = Arc::clone(&background_state);

        // Use ArcSwap to store only the latest message - subscription callback just updates this
        // Processing happens in a separate thread to avoid blocking the executor
        let latest_msg: Arc<ArcSwap<Option<Arc<PointCloud2>>>> =
            Arc::new(ArcSwap::new(Arc::new(None)));
        let latest_msg_for_callback = Arc::clone(&latest_msg);
        let latest_msg_for_processing = Arc::clone(&latest_msg);

        // Create subscription to PointCloud2 - callback just stores the latest message
        let mut pointcloud_options = SubscriptionOptions::new("input_pointcloud");
        pointcloud_options.qos = qos_profile;
        let pointcloud_subscription =
            node.create_subscription(pointcloud_options, move |msg: PointCloud2| {
                let count = counter_clone.fetch_add(1, Ordering::Relaxed);
                log_debug!(
                    LOGGER_NAME,
                    "Received msg #{} (ts: {}.{:09})",
                    count + 1,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec
                );
                // Store the latest message (overwrites any previous unprocessed message)
                latest_msg_for_callback.store(Arc::new(Some(Arc::new(msg))));
            })?;

        // Spawn processing thread that processes the latest message when available
        let processing_thread = std::thread::spawn(move || {
            let mut processed_count: u64 = 0;

            loop {
                // Take the latest message (replace with None)
                // ArcSwap pattern ensures we always process the most recent message
                // and skip any intermediate messages that arrived during processing
                let msg_opt = latest_msg_for_processing.swap(Arc::new(None));

                if let Some(msg) = msg_opt.as_ref() {
                    let callback_start = Instant::now();
                    processed_count += 1;

                    log_debug!(
                        LOGGER_NAME,
                        "PROCESS: ts {}.{:09}, count {}",
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                        processed_count
                    );

                    // Clone the message for processing (msg is Arc<PointCloud2>)
                    let msg_clone: PointCloud2 = (**msg).clone();

                    // M-06: guard against panics in the detection pipeline (e.g.
                    // partial_cmp().unwrap() on a NaN point). Without this a single
                    // panic kills only this detached thread, leaving the node alive
                    // but permanently silent. Catch, log, and continue.
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        Self::pointcloud_callback(
                            msg_clone,
                            &detector,
                            &detection_publisher_shared,
                            &bbox_params_for_callback,
                            &board_debug_shared,
                            &icp_debug_shared,
                            &bbox_free_for_thread,
                            &background_for_thread,
                        );
                    }));
                    if result.is_err() {
                        log_error!(
                            LOGGER_NAME,
                            "Board detection panicked while processing a cloud; \
                             skipping it and continuing"
                        );
                    }

                    let processing_time = callback_start.elapsed();
                    log_debug!(
                        LOGGER_NAME,
                        "DONE: processed in {}ms",
                        processing_time.as_millis()
                    );
                } else {
                    // No message available, sleep briefly to avoid busy-waiting
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
            }
        });

        if enable_debug {
            log_info!(
                LOGGER_NAME,
                "Calibration board locator node initialized with debug mode"
            );
            log_info!(
                LOGGER_NAME,
                "Debug topics: debug/all_points, debug/filtered_points, debug/foreground_points (bbox_free), debug/background_voxels (bbox_free), debug/plane_inliers, debug/plane_marker, debug/bbox_marker, debug/final_board_pose, debug/icp_iterations, debug/initial_board_marker, debug/icp_stats, debug/pca_eigenvectors"
            );
        }

        if enable_icp_iteration_debug {
            log_info!(
                LOGGER_NAME,
                "ICP iteration debug topics: /calibration/icp_debug/iteration_pose, /calibration/icp_debug/board_points, /calibration/icp_debug/correspondences, /calibration/icp_debug/loss, /calibration/icp_debug/stats"
            );
        }

        Ok(Self {
            _node: node,
            _detection_publisher: detection_publisher,
            _pointcloud_subscription: pointcloud_subscription,
            _board_debug_publishers: board_debug_publishers,
            _icp_debug_publishers: icp_debug_publishers,
            _bbox_params: bbox_params,
            _processing_thread: processing_thread,
            _background_state: background_state,
            _reset_service: reset_service,
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
        bbox_params: &BBoxParameters,
        board_debug_publishers: &Option<BoardDebugPublishers>,
        icp_debug_publishers: &Option<IcpDebugPublishers>,
        bbox_free_cfg: &Option<Arc<bbox_free::BboxFreeRaw>>,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
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
            bbox_params,
            board_debug_publishers,
            icp_debug_publishers,
            bbox_free_cfg,
            background_state,
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
        bbox_params: &BBoxParameters,
        board_debug_publishers: &Option<BoardDebugPublishers>,
        icp_debug_publishers: &Option<IcpDebugPublishers>,
        bbox_free_cfg: &Option<Arc<bbox_free::BboxFreeRaw>>,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
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

        // Stage 1: select the board cluster — bbox crop (default) or crop-box-free detector.
        let active_points = match bbox_free_cfg {
            None => {
                // Existing bounding-box path, unchanged.
                Self::filter_points_by_bbox(&points, bbox_params, &msg.header, board_debug_publishers)?
            }
            Some(bf) => {
                match Self::select_board_cluster(
                    &points,
                    bf,
                    background_state,
                    &msg.header,
                    board_debug_publishers,
                )? {
                    Some(pts) => pts,
                    None => {
                        return Ok(Detection3DArray {
                            header: msg.header.clone(),
                            detections: Vec::new(),
                        });
                    }
                }
            }
        };

        if active_points.is_empty() {
            log_debug!(
                LOGGER_NAME,
                "Stage 1 produced no points - continuing with empty detection"
            );
            return Ok(Detection3DArray {
                header: msg.header.clone(),
                detections: Vec::new(),
            });
        }

        // Stage 2: RANSAC plane detection (or skip if configured)
        let (plane_model, plane_inlier_points) = if detector.config().skip_ransac {
            // Skip RANSAC and use all bbox-filtered points directly
            log_debug!(
                LOGGER_NAME,
                "RANSAC skipped (skip_ransac=true), using all {} bbox-filtered points for ICP",
                active_points.len()
            );

            // Create a simple plane model using PCA on all points
            let plane_model = Self::compute_plane_from_points(&active_points)?;

            // Publish debug info showing we're using all points
            if let Some(debug_pubs) = board_debug_publishers {
                log_debug!(
                    LOGGER_NAME,
                    "Publishing {} bbox-filtered points to debug/plane_inliers (RANSAC skipped)",
                    active_points.len()
                );
                let debug_cloud = Self::create_debug_pointcloud(&active_points, &msg.header)?;
                if let Err(e) = debug_pubs.plane_inliers.publish(debug_cloud) {
                    log_warn!(LOGGER_NAME, "Failed to publish debug plane inliers: {e}");
                }
            }

            (plane_model, active_points.clone())
        } else {
            // Normal RANSAC plane detection
            match Self::detect_plane_ransac(
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
            }
        };

        // Stage 3a: Voxel downsampling (optional preprocessing)
        log_debug!(
            LOGGER_NAME,
            "Plane inlier points before voxel downsampling: {} points",
            plane_inlier_points.len()
        );

        let config = detector.config();
        let downsampled_points = if config.voxel_downsample_enabled {
            let downsampled = voxel_downsample(
                &plane_inlier_points,
                config.voxel_downsample_size,
                config.voxel_downsample_use_centroid,
                config.voxel_parallel_threshold,
            );

            let reduction_pct =
                (1.0 - downsampled.len() as f64 / plane_inlier_points.len() as f64) * 100.0;
            log_debug!(
                LOGGER_NAME,
                "Voxel downsampling: {} → {} points ({:.1}% reduction)",
                plane_inlier_points.len(),
                downsampled.len(),
                reduction_pct
            );

            // Publish downsampled points for visualization
            if let Some(debug_pubs) = board_debug_publishers {
                match Self::create_debug_pointcloud(&downsampled, &msg.header) {
                    Ok(downsampled_cloud) => {
                        if let Err(e) = debug_pubs.downsampled_points.publish(downsampled_cloud) {
                            log_warn!(LOGGER_NAME, "Failed to publish downsampled points: {e}");
                        }
                    }
                    Err(e) => {
                        log_warn!(LOGGER_NAME, "Failed to create downsampled pointcloud: {e}");
                    }
                }
            }

            downsampled
        } else {
            log_debug!(
                LOGGER_NAME,
                "Voxel downsampling disabled - using all {} plane inlier points",
                plane_inlier_points.len()
            );
            plane_inlier_points.clone()
        };

        log_debug!(
            LOGGER_NAME,
            "Points for ICP: {} points",
            downsampled_points.len()
        );

        // Stage 3b: ICP board pose refinement
        log_debug!(
            LOGGER_NAME,
            "Starting ICP board detection with {} points",
            downsampled_points.len()
        );

        let detection: Option<BoardDetection> = Self::detect_icp(
            detector,
            &plane_model,
            &downsampled_points,
            PlaneRansacData {
                plane_model: plane_model.clone(),
                inlier_points: downsampled_points.clone(),
            },
            &msg.header,
            icp_debug_publishers,
            board_debug_publishers,
        );

        let mut detections = Vec::new();
        if let Some(det) = detection {
            // Log ICP result to service logs
            let final_loss = det
                .icp_losses
                .iter()
                .copied()
                .min_by(|a, b| a.total_cmp(b))
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
            log_debug!(LOGGER_NAME, "Detection returned None - board not found");

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

    /// Crop-box-free Stage 1: returns the detector's selected board-cluster points,
    /// or `None` to publish an empty detection (still warming, or nothing selected).
    fn select_board_cluster(
        points: &[na::Point3<f64>],
        bf: &bbox_free::BboxFreeRaw,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
        header: &Header,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Result<Option<Vec<na::Point3<f64>>>> {
        use board_projection_detector::{config::ForegroundMethod, detector::detect};

        let method = bf.method()?;

        // Cell centers of the finalized background model, captured under the lock
        // for debug publishing once the guard is released. None for plane_strip.
        let mut background_centers: Option<Vec<na::Point3<f64>>> = None;

        // Method E: run the warmup lifecycle to obtain a finalized background.
        let outcome = if method == ForegroundMethod::BackgroundSubtraction {
            let mut guard = background_state
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            let state = guard.as_mut().ok_or_else(|| {
                anyhow!("bbox_free background_subtraction selected but no BackgroundState initialized")
            })?;
            match state.observe_frame(points) {
                bbox_free::WarmupOutcome::Warming { seen, needed } => {
                    log_info!(LOGGER_NAME, "background warmup {seen}/{needed}");
                    return Ok(None);
                }
                bbox_free::WarmupOutcome::Ready => {
                    let model = state.model().expect("Ready implies a finalized model");
                    background_centers = Some(model.voxel_centers());
                    detect(points, &bf.board, method, bf.voxel, Some(model))
                }
            }
        } else {
            // Method B (plane_strip): no background.
            detect(points, &bf.board, method, bf.voxel, None)
        };

        // Publish the finalized background voxel centers so the "static scene"
        // baked in during warmup is visible in RViz (diagnoses a warmup that
        // absorbed the board). Static once Ready; republished each frame so the
        // display is populated whenever the operator adds it.
        if let (Some(debug_pubs), Some(centers)) = (board_debug_publishers, &background_centers) {
            match Self::create_debug_pointcloud(centers, header) {
                Ok(cloud) => {
                    if let Err(e) = debug_pubs.background_voxels.publish(cloud) {
                        log_warn!(LOGGER_NAME, "Failed to publish background voxels: {e}");
                    }
                }
                Err(e) => log_warn!(LOGGER_NAME, "Failed to build background voxel cloud: {e}"),
            }
        }

        // Publish the RAW Method-E / plane-strip foreground (points before
        // clustering/merge/gate) so it is visible in RViz independently of the
        // downstream RANSAC plane_inliers, which only appears AFTER a board is
        // selected. Shows everything background subtraction kept, not just clusters.
        if let Some(debug_pubs) = board_debug_publishers {
            match Self::create_debug_pointcloud(&outcome.foreground_points, header) {
                Ok(cloud) => {
                    if let Err(e) = debug_pubs.foreground_points.publish(cloud) {
                        log_warn!(LOGGER_NAME, "Failed to publish foreground points: {e}");
                    }
                }
                Err(e) => log_warn!(LOGGER_NAME, "Failed to build foreground debug cloud: {e}"),
            }
        }

        match outcome.selected_points {
            Some(pts) if !pts.is_empty() => Ok(Some(pts)),
            _ => {
                match &outcome.reject {
                    // Include the measured value, the threshold it missed, and the unit,
                    // so tuning is data-driven instead of trial-and-error.
                    Some(reason) => match outcome.reject_detail {
                        Some(d) => log_info!(
                            LOGGER_NAME,
                            "bbox_free: no board selected — {}; measured={:.4} vs threshold={:.4} [{}]; candidates={}, foreground_pts={}",
                            bbox_free::describe_reject(reason),
                            d.measured,
                            d.threshold,
                            bbox_free::reject_unit(reason),
                            outcome.n_candidates,
                            outcome.foreground_points.len()
                        ),
                        None => log_info!(
                            LOGGER_NAME,
                            "bbox_free: no board selected — {}; candidates={}, foreground_pts={}",
                            bbox_free::describe_reject(reason),
                            outcome.n_candidates,
                            outcome.foreground_points.len()
                        ),
                    },
                    None => log_info!(LOGGER_NAME, "bbox_free: no board selected (no reject reason)"),
                }
                Ok(None)
            }
        }
    }

    // Stage 1: Bounding box filter
    fn filter_points_by_bbox(
        points: &[na::Point3<f64>],
        bbox_params: &BBoxParameters,
        header: &Header,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Result<Vec<na::Point3<f64>>> {
        // Read current bbox parameter values (reflects runtime changes)
        let bbox = bbox_params.to_bbox();

        // Log bbox values at INFO level for debugging parameter updates
        // Use a static to track last logged values and only log when changed
        use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
        static LAST_SIZE_HASH: AtomicU64 = AtomicU64::new(0);

        let size_hash =
            (bbox.size_xyz[0].to_bits() ^ bbox.size_xyz[1].to_bits() ^ bbox.size_xyz[2].to_bits())
                .wrapping_add(bbox.pose.translation.x.to_bits())
                .wrapping_add(bbox.pose.translation.y.to_bits())
                .wrapping_add(bbox.pose.translation.z.to_bits());
        let prev_hash = LAST_SIZE_HASH.swap(size_hash, AtomicOrdering::Relaxed);
        if size_hash != prev_hash {
            log_info!(
                LOGGER_NAME,
                "BBox UPDATED: center=[{:.2}, {:.2}, {:.2}], size=[{:.2}, {:.2}, {:.2}]",
                bbox.pose.translation.x,
                bbox.pose.translation.y,
                bbox.pose.translation.z,
                bbox.size_xyz[0],
                bbox.size_xyz[1],
                bbox.size_xyz[2]
            );
        }

        log_debug!(
            LOGGER_NAME,
            "Bounding box filter: center=[{:.2}, {:.2}, {:.2}], size=[{:.2}, {:.2}, {:.2}]",
            bbox.pose.translation.x,
            bbox.pose.translation.y,
            bbox.pose.translation.z,
            bbox.size_xyz[0],
            bbox.size_xyz[1],
            bbox.size_xyz[2]
        );

        // Publish bbox marker for visualization in RViz
        if let Some(debug_pubs) = board_debug_publishers {
            let bbox_marker = Self::create_bbox_marker(&bbox, header)?;
            let marker_array = MarkerArray {
                markers: vec![bbox_marker],
            };
            if let Err(e) = debug_pubs.bbox_marker.publish(marker_array) {
                log_warn!(LOGGER_NAME, "Failed to publish bbox marker: {e}");
            }
        }

        let active_points: Vec<_> = points
            .iter()
            .filter(|pt| bbox.contains_point(pt))
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
                log_warn!(LOGGER_NAME, "Plane fitting error: {e}");
                return Err(e);
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
        log_debug!(LOGGER_NAME, "Starting ICP board pose refinement");

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

        // Step 3: Create initial pose using plane normal-based alignment
        let initial_pose = Self::compute_initial_pose_from_plane(
            plane_model,
            plane_inlier_points,
            board_width.as_meters(),
            &config.sensor_up_axis,
            config.initial_inplane_rotation_deg,
            header,
            board_debug_publishers,
        )?;

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

        // Note: plane_inlier_points are already downsampled (if enabled) in process_pointcloud()
        let icp_points: Vec<na::Point3<f64>> = plane_inlier_points.to_vec();

        log_debug!(LOGGER_NAME, "Starting ICP with {} points", icp_points.len());

        // Step 4: Create BoardIcpIterator
        let mut iterator = BoardIcpIterator::new(
            config,
            board_model_params.clone(),
            None, // No progress callback as we handle debug publishing ourselves
        );

        // Step 5: Create initial ICP state
        let mut state = iterator.initial_state(initial_pose, icp_points);

        log_debug!(
            LOGGER_NAME,
            "Starting ICP iterations with initial pose: {:?}",
            state.board_pose
        );

        // Step 7: Iterate with optional debug publishing
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

            // Note: Removed 50ms sleep between ICP iterations as it caused severe lag.
            // The ICP debug visualization now updates at full speed.

            // Check termination condition AFTER the step
            if iterator.should_terminate(&state) {
                let reason = iterator.termination_reason(&state);
                log_debug!(LOGGER_NAME, "ICP iteration terminated: {}", reason);
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

            // Apply post-fixup to ensure pose origin is at the lowest corner
            let corrected_pose = {
                // Create temporary board model to evaluate corners
                let temp_board_model = BoardModel {
                    pose: state.board_pose,
                    board_shape: board_model_params.board_shape.clone(),
                    marker_paper_size: board_model_params.marker_paper_size,
                };

                let board_normal = temp_board_model.board_z_axis();

                let corners = [
                    temp_board_model.bottom_corner(),
                    temp_board_model.left_corner(),
                    temp_board_model.top_corner(),
                    temp_board_model.right_corner(),
                ];

                // Find the vertically lowest corner along the configured sensor "up"
                // axis (Seyond: X=up, so gravity-down is -X, not -Z). Projecting onto
                // up_vector generalizes the old hardcoded ".z" comparison. total_cmp is
                // NaN-safe; partial_cmp().unwrap() would panic on a NaN corner.
                let up_vector = config.sensor_up_axis.as_vector();
                let height = |c: &na::Point3<f64>| c.coords.dot(&up_vector);
                let (lowest_index, lowest_corner) = corners
                    .iter()
                    .enumerate()
                    .min_by(|a, b| height(a.1).total_cmp(&height(b.1)))
                    .unwrap();

                // M-14: the board origin corner is disambiguated purely by "lowest
                // height along the sensor up axis" -- an unstated assumption that exactly
                // one corner is clearly lowest. Near a 45deg roll or a diamond-mounted
                // board, the two lowest corners are nearly tied and the wrong one can win,
                // rotating the board frame 90deg and silently biasing the extrinsic for
                // this pose. Warn when the disambiguation is marginal instead of failing
                // silently.
                {
                    let mut hs: Vec<f64> = corners.iter().map(|c| height(c)).collect();
                    hs.sort_by(|a, b| a.total_cmp(b));
                    let h_spread = hs[3] - hs[0];
                    if h_spread > 1e-9 && (hs[1] - hs[0]) < 0.15 * h_spread {
                        log_warn!(
                            LOGGER_NAME,
                            "Board origin corner is ambiguous: the two lowest corners are \
                             within {:.0}% of the corner height-spread. The board may be near a \
                             45deg roll or the LiDAR frame may not be gravity-aligned; the \
                             extrinsic could be off by a 90deg in-plane rotation for this pose.",
                            100.0 * (hs[1] - hs[0]) / h_spread
                        );
                    }
                }

                log_debug!(
                    LOGGER_NAME,
                    "Post-fixup: lowest corner index={}, position=({:.3}, {:.3}, {:.3})",
                    lowest_index,
                    lowest_corner.x,
                    lowest_corner.y,
                    lowest_corner.z
                );

                // Rotate by 90° × index around the board normal to bring lowest corner to "bottom" position
                let fixup_rotation = {
                    let angle = FRAC_PI_2 * lowest_index as f64;
                    na::UnitQuaternion::from_axis_angle(&board_normal, angle)
                };

                // Move the pose origin to the lowest corner
                let fixup_translation =
                    { na::Translation3::new(lowest_corner.x, lowest_corner.y, lowest_corner.z) };

                // Compose the corrected pose: translation * rotation * original_rotation
                let corrected = fixup_translation * fixup_rotation * state.board_pose.rotation;

                log_info!(
                    LOGGER_NAME,
                    "Post-fixup applied: rotation_angle={:.1}°, new_origin=({:.3}, {:.3}, {:.3})",
                    (FRAC_PI_2 * lowest_index as f64).to_degrees(),
                    corrected.translation.x,
                    corrected.translation.y,
                    corrected.translation.z
                );

                corrected
            };

            // Create final board model and detection
            let board_model = BoardModel {
                pose: corrected_pose,
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
            // INFO, not debug: this is the actual reject reason once a board has
            // reached the ICP stage (initial pose published, no final result). At
            // debug level it was invisible at the default log_level, so the only
            // visible reject was the unrelated bbox_free "no candidate clusters"
            // from sparser frames — misleading. Name which gate failed.
            let why = if state.inlier_points.len() < config.icp_min_inlier_points {
                "inliers < icp_min_inlier_points"
            } else {
                "final_loss > icp_good_fit_threshold"
            };
            log_info!(
                LOGGER_NAME,
                "Board detection failed at ICP ({why}): final_loss={:.6}, inliers={}, icp_good_fit_threshold={:.6}, icp_min_inlier_points={}",
                state.avg_loss,
                state.inlier_points.len(),
                config.icp_good_fit_threshold,
                config.icp_min_inlier_points
            );
            None
        }
    }

    /// Compute initial board pose using plane normal alignment (from wayside-portal)
    /// Build the initial board pose from the fitted plane.
    ///
    /// The board's local frame: Z = board normal (toward sensor), Y ≈ world "up" projected
    /// onto the board plane, X = cross(Y, Z). `sensor_up_axis` selects which world axis
    /// is "up" — must match the LiDAR's coordinate convention (Seyond: X=up, Z=forward).
    fn compute_initial_pose_from_plane(
        plane_model: &PlaneModel,
        plane_inlier_points: &[na::Point3<f64>],
        board_width_meters: f64,
        sensor_up_axis: &SensorUpAxis,
        initial_inplane_rotation_deg: f64,
        _header: &Header,
        _debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Option<na::Isometry3<f64>> {
        if plane_inlier_points.is_empty() {
            log_warn!(
                LOGGER_NAME,
                "Cannot compute initial pose with empty point set"
            );
            return None;
        }

        // Step 1: Compute centroid of plane inlier points
        let inlier_centroid = plane_inlier_points
            .iter()
            .fold(na::Vector3::zeros(), |acc, point| acc + point.coords)
            / (plane_inlier_points.len() as f64);

        log_debug!(
            LOGGER_NAME,
            "Initial pose from plane: centroid=({:.3}, {:.3}, {:.3}), {} points",
            inlier_centroid.x,
            inlier_centroid.y,
            inlier_centroid.z,
            plane_inlier_points.len()
        );

        // Step 2: Obtain the plane normal vector that points towards the origin
        let plane_normal = {
            let normal = plane_model.normal.into_inner();
            if (na::Point3::origin().coords - inlier_centroid).dot(&normal) < 0.0 {
                -normal
            } else {
                normal
            }
        };

        log_debug!(
            LOGGER_NAME,
            "Plane normal (toward origin): ({:.3}, {:.3}, {:.3})",
            plane_normal.x,
            plane_normal.y,
            plane_normal.z
        );

        // Step 3: Build initial rotation from plane geometry.
        //
        // Board local frame: Z → plane_normal (toward sensor),
        //                    Y → world "up" projected onto board plane,
        //                    X → cross(Y, Z)
        //
        // This avoids the hardcoded XY projection that breaks when the sensor's
        // up axis is not Z (e.g. Seyond LiDAR: X=up, Z=forward).
        let rotation = {
            let board_z = plane_normal;
            let up = sensor_up_axis.as_vector();

            // Project world "up" onto the board plane (remove component along board_z)
            let up_projected = up - up.dot(&board_z) * board_z;

            let board_y = if up_projected.norm() > 1e-6 {
                up_projected.normalize()
            } else {
                // "up" is parallel to the board normal — pick an arbitrary perpendicular
                let alt = if board_z.x.abs() < 0.9 {
                    na::Vector3::x()
                } else {
                    na::Vector3::y()
                };
                (alt - alt.dot(&board_z) * board_z).normalize()
            };

            let board_x = board_y.cross(&board_z);

            log_debug!(
                LOGGER_NAME,
                "Board axes — X: ({:.3},{:.3},{:.3}), Y: ({:.3},{:.3},{:.3}), Z: ({:.3},{:.3},{:.3})",
                board_x.x, board_x.y, board_x.z,
                board_y.x, board_y.y, board_y.z,
                board_z.x, board_z.y, board_z.z,
            );

            let base_rotation = na::UnitQuaternion::from_matrix(&na::Matrix3::from_columns(&[
                board_x, board_y, board_z,
            ]));

            // Apply configurable in-plane rotation offset around the board normal.
            // Corrects a fixed rotational bias visible in RViz (set via initial_inplane_rotation_deg).
            if initial_inplane_rotation_deg.abs() > 1e-6 {
                let angle_rad = initial_inplane_rotation_deg.to_radians();
                let offset = na::UnitQuaternion::from_axis_angle(
                    &na::Unit::new_normalize(board_z),
                    angle_rad,
                );
                offset * base_rotation
            } else {
                base_rotation
            }
        };

        // Step 4: Create initial pose with board center at inlier centroid
        // We want: board_center_world = inlier_centroid
        // The board center in board coordinates is at (board_width/2, board_width/2, 0)
        // board_center_world = pose.translation + rotation * board_center_board
        // Therefore: pose.translation = inlier_centroid - rotation * board_center_board
        let board_center_board =
            na::Vector3::new(board_width_meters / 2.0, board_width_meters / 2.0, 0.0);
        let board_center_offset = rotation * board_center_board;
        let corner_position = inlier_centroid - board_center_offset;

        let pose = na::Isometry3::from_parts(na::Translation3::from(corner_position), rotation);

        log_debug!(
            LOGGER_NAME,
            "Initial pose from plane: centroid=({:.3}, {:.3}, {:.3}), corner=({:.3}, {:.3}, {:.3}), rotation=({:.3}, {:.3}, {:.3}, {:.3})",
            inlier_centroid.x,
            inlier_centroid.y,
            inlier_centroid.z,
            pose.translation.x,
            pose.translation.y,
            pose.translation.z,
            rotation.w,
            rotation.i,
            rotation.j,
            rotation.k
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

        // Get field offsets and datatypes. H-03: honor the declared datatype and
        // endianness instead of assuming little-endian FLOAT32 -- a FLOAT64 or
        // big-endian producer would otherwise be decoded from the wrong bytes,
        // yielding garbage points and a silently wrong calibration.
        let x_offset = x_field.offset as usize;
        let y_offset = y_field.offset as usize;
        let z_offset = z_field.offset as usize;
        let x_datatype = x_field.datatype;
        let y_datatype = y_field.datatype;
        let z_datatype = z_field.datatype;
        let is_bigendian = msg.is_bigendian;

        // Parse points (use usize arithmetic to avoid u32 overflow on large clouds)
        let point_step = msg.point_step as usize;
        let num_points = (msg.width as usize) * (msg.height as usize);

        let mut points = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let base_offset = i * point_step;

            // Ensure we don't read beyond the data buffer
            if base_offset + point_step > msg.data.len() {
                log_warn!(LOGGER_NAME, "Point data truncated at point {}", i);
                break;
            }

            // Read x, y, z according to the field datatype and endianness
            let x = Self::read_coord(&msg.data, base_offset + x_offset, x_datatype, is_bigendian)?;
            let y = Self::read_coord(&msg.data, base_offset + y_offset, y_datatype, is_bigendian)?;
            let z = Self::read_coord(&msg.data, base_offset + z_offset, z_datatype, is_bigendian)?;

            // Skip points with invalid coordinates (NaN or infinity)
            if x.is_finite() && y.is_finite() && z.is_finite() {
                points.push(na::Point3::new(x as f64, y as f64, z as f64));
            }
        }

        Ok(points)
    }

    /// Read a single XYZ coordinate honoring the PointField datatype and the
    /// cloud's endianness. Supports FLOAT32 (7) and FLOAT64 (8); returns an
    /// error for any other datatype so an unsupported layout fails loudly rather
    /// than being silently misdecoded.
    fn read_coord(data: &[u8], offset: usize, datatype: u8, is_bigendian: bool) -> Result<f32> {
        const PF_FLOAT32: u8 = 7;
        const PF_FLOAT64: u8 = 8;
        match datatype {
            PF_FLOAT32 => {
                if offset + 4 > data.len() {
                    return Err(anyhow!(
                        "Buffer overflow when reading f32 at offset {offset}"
                    ));
                }
                let bytes: [u8; 4] = data[offset..offset + 4].try_into().unwrap();
                Ok(if is_bigendian {
                    f32::from_be_bytes(bytes)
                } else {
                    f32::from_le_bytes(bytes)
                })
            }
            PF_FLOAT64 => {
                if offset + 8 > data.len() {
                    return Err(anyhow!(
                        "Buffer overflow when reading f64 at offset {offset}"
                    ));
                }
                let bytes: [u8; 8] = data[offset..offset + 8].try_into().unwrap();
                let value = if is_bigendian {
                    f64::from_be_bytes(bytes)
                } else {
                    f64::from_le_bytes(bytes)
                };
                Ok(value as f32)
            }
            other => Err(anyhow!(
                "Unsupported PointField datatype {other} for XYZ (expected FLOAT32=7 or FLOAT64=8)"
            )),
        }
    }

    /// Compute a plane model from points using PCA (for skip_ransac mode)
    fn compute_plane_from_points(points: &[na::Point3<f64>]) -> Result<PlaneModel> {
        if points.len() < 3 {
            return Err(anyhow!(
                "Need at least 3 points to compute plane, got {}",
                points.len()
            ));
        }

        // Compute centroid
        let centroid = points
            .iter()
            .fold(na::Vector3::zeros(), |acc, point| acc + point.coords)
            / (points.len() as f64);

        // Compute covariance matrix
        let mut covariance = na::Matrix3::<f64>::zeros();
        for point in points {
            let diff = point.coords - centroid;
            covariance += diff * diff.transpose();
        }
        covariance /= points.len() as f64;

        // Compute eigendecomposition to find plane normal
        let eigen = covariance.symmetric_eigen();

        // The eigenvector with the smallest eigenvalue is the plane normal
        let mut eigenvalues_indexed: Vec<(usize, f64)> =
            (0..3).map(|i| (i, eigen.eigenvalues[i])).collect();
        eigenvalues_indexed.sort_by(|a, b| a.1.total_cmp(&b.1));

        let normal_idx = eigenvalues_indexed[0].0; // Smallest eigenvalue
        let normal_vec = eigen.eigenvectors.column(normal_idx).into_owned();

        // Ensure normal points toward positive X (sensor direction)
        let normal = if normal_vec.x >= 0.0 {
            na::Unit::new_normalize(normal_vec)
        } else {
            na::Unit::new_normalize(-normal_vec)
        };

        let plane_model = PlaneModel {
            center: na::Point3::from(centroid),
            normal,
        };

        log_debug!(
            LOGGER_NAME,
            "Computed plane from {} points using PCA: normal=[{:.3}, {:.3}, {:.3}], center=[{:.3}, {:.3}, {:.3}]",
            points.len(),
            plane_model.normal.x,
            plane_model.normal.y,
            plane_model.normal.z,
            plane_model.center.x,
            plane_model.center.y,
            plane_model.center.z
        );

        Ok(plane_model)
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

    /// M-13: the 6x6 covariance of the fitted board pose, from the converged ICP correspondences.
    ///
    /// The solver used to treat every board pose as exact and equally trustworthy. It is neither.
    /// The error is strongly **anisotropic**, and that falls straight out of the correspondence
    /// model (`hollow-board-config`):
    ///
    /// - an **interior** point's model correspondence is its own projection onto the board plane,
    ///   so its residual is purely along the plane normal. It says nothing at all about where the
    ///   board sits *within* its own plane.
    /// - only points **outside the square** (clamped to the border) and points **inside a hole**
    ///   (snapped to the rim) have an in-plane residual, and so only they constrain in-plane
    ///   position and the rotation about the normal.
    ///
    /// So a board with few edge and hole-rim returns is tightly determined out-of-plane and almost
    /// free in-plane — and a naive isotropic covariance would hide exactly that.
    ///
    /// The linearisation therefore projects each residual onto the direction the model actually
    /// constrains (at a closest-point correspondence, that is the surface normal, i.e. the residual
    /// direction):
    ///
    /// ```text
    ///   e_i = n_i . (p_i - q_i)                     scalar residual
    ///   J_i = [ n_i^T , (d_i x n_i)^T ]             d_i = q_i - c   (c = the POSE ORIGIN)
    ///   H   = sum J_i^T J_i                         sigma^2 = sum e_i^2 / (N - 6)
    ///   Cov = sigma^2 * H^-1
    /// ```
    ///
    /// Interior points contribute `n_i` = the board normal, so they load only the out-of-plane
    /// block; if the edges and rims are sparse, `H` goes near-singular in-plane and the covariance
    /// blows up precisely where the fit is genuinely weak. That is the point.
    ///
    /// **Frame.** `c` is the *published* pose origin, not the raw ICP one. The post-ICP fixup
    /// (origin moved to the lowest corner, frame rotated by 90°·k about the normal) is a pure
    /// re-parameterisation of the same physical plate — the model points `q_i` are unchanged world
    /// points — so building `J` about the published origin yields the covariance directly in the
    /// published parameterisation. No adjoint, and no chance of silently transposing the x/y and
    /// rx/ry blocks.
    ///
    /// Returns row-major 6x6 in the ROS `PoseWithCovariance` order `[x, y, z, rx, ry, rz]`.
    fn compute_pose_covariance(
        correspondences: &[(na::Point3<f64>, na::Point3<f64>)],
        pose_origin: &na::Point3<f64>,
        board_normal: &na::Vector3<f64>,
    ) -> [f64; 36] {
        const MIN_CORRESPONDENCES: usize = 10;
        // Below this the residual direction is numerically meaningless; fall back to the plane
        // normal, which is the correct constraint direction for an interior point.
        const MIN_RESIDUAL_M: f64 = 1e-4;

        let n_points = correspondences.len();
        if n_points < MIN_CORRESPONDENCES {
            return [0.0; 36];
        }

        let mut hessian = na::Matrix6::<f64>::zeros();
        let mut sum_sq_residual = 0.0;

        for (data_point, model_point) in correspondences {
            let residual = data_point - model_point;
            let norm = residual.norm();

            let direction = if norm > MIN_RESIDUAL_M {
                residual / norm
            } else {
                *board_normal
            };

            // Scalar residual along the constrained direction.
            let e = direction.dot(&residual);
            sum_sq_residual += e * e;

            // A model point rigidly attached to the board moves by  dt + dtheta x (q - c).
            // The residual's sensitivity is therefore  [ n , (q - c) x n ].
            let lever = model_point - pose_origin;
            let rot_part = lever.cross(&direction);

            let mut jacobian = na::SVector::<f64, 6>::zeros();
            jacobian[0] = direction.x;
            jacobian[1] = direction.y;
            jacobian[2] = direction.z;
            jacobian[3] = rot_part.x;
            jacobian[4] = rot_part.y;
            jacobian[5] = rot_part.z;

            hessian += jacobian * jacobian.transpose();
        }

        let dof = (n_points as f64 - 6.0).max(1.0);
        let sigma_sq = sum_sq_residual / dof;

        // H is routinely SINGULAR here, and that is a result, not an error: a board seen with only
        // interior returns has no in-plane information at all, so H has rank 3 (z, rx, ry) and no
        // inverse exists.
        //
        // A plain `try_inverse()` would bail on exactly the case this covariance exists to
        // describe, and any single fallback for the whole matrix would throw away the DoF that ARE
        // well determined. So invert per eigendirection instead: each mode gets sigma^2 / lambda,
        // and an unobservable mode (lambda -> 0) saturates at MAX_VARIANCE rather than exploding
        // to infinity or collapsing to zero.
        //
        // Zero would be the dangerous one -- a zeroed covariance reads downstream as "this pose is
        // exact", which is precisely the lie M-13 is about.
        const MAX_VARIANCE: f64 = 1e6;
        let lambda_floor = sigma_sq / MAX_VARIANCE;

        let eigen = na::SymmetricEigen::new(hessian);
        let mut inverse = na::Matrix6::<f64>::zeros();
        for i in 0..6 {
            let lambda = eigen.eigenvalues[i].max(lambda_floor);
            let v = eigen.eigenvectors.column(i);
            inverse += (v * v.transpose()) / lambda;
        }

        let covariance = inverse * sigma_sq;

        let mut out = [0.0; 36];
        for row in 0..6 {
            for col in 0..6 {
                out[6 * row + col] = covariance[(row, col)];
            }
        }
        out
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
        // M-13: publish the REAL board-pose covariance, computed from the converged ICP
        // correspondences about the published pose origin. It used to be `[0.0; 36]` -- which the
        // solver read as "this pose is exact", so every board pose was weighted equally and the
        // errors-in-variables bias went unmodelled.
        let covariance = Self::compute_pose_covariance(
            &board_detection.icp_data.correspondences,
            &board_model.pose.translation.vector.into(),
            &board_model.board_z_axis(),
        );

        // The score used to be a hardcoded 1.0. Report the inlier ratio instead -- a quantity the
        // detector actually measured.
        let stats = &board_detection.icp_stats;
        let score = if stats.final_loss.is_finite() && stats.final_loss > 0.0 {
            // Bounded, monotonically decreasing in the fit error. `icp_good_fit_threshold` is the
            // acceptance bar (C-04), so a fit exactly at the bar scores 0.5.
            (1.0 / (1.0 + stats.final_loss / 0.035)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let hypothesis = ObjectHypothesisWithPose {
            hypothesis: vision_msgs::msg::ObjectHypothesis {
                class_id: "calibration_board".to_string(),
                score,
            },
            pose: PoseWithCovariance { pose, covariance },
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

    #[allow(dead_code)]
    fn create_board_marker(board_detection: &BoardDetection, header: &Header) -> Result<Marker> {
        // Use the pose returned by algo.rs (embedded in board_detection.board_model.pose)
        let board_model = &board_detection.board_model;

        let mut marker = Marker {
            header: header.clone(),
            ns: "board".to_string(),
            id: 0,
            type_: 1,  // CUBE to approximate board plane
            action: 0, // ADD
            ..Default::default()
        };

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

    /// Create board visualization markers with customizable colors and namespaces
    #[allow(clippy::too_many_arguments)]
    fn create_board_visualization(
        board_model: &hollow_board_config::BoardModel,
        header: &Header,
        namespace_suffix: &str,
        id_offset: i32,
        board_color: ColorRGBA,
        axes_alpha: f32,
        marker_area_alpha: f32,
        hole_alpha: f32,
    ) -> Result<MarkerArray> {
        let base_translation = &board_model.pose.translation;
        let base_rotation = &board_model.pose.rotation;

        // Board cube marker
        // NOTE: The cube is centered at board_center(), not pose.translation
        // because BoardModel expects pose.translation to be at the bottom-left corner (0,0)
        let board_cube = {
            let board_center = board_model.board_center();
            let q = base_rotation.quaternion();
            Marker {
                header: header.clone(),
                ns: format!("board{}", namespace_suffix),
                id: id_offset,
                type_: 1,  // CUBE
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: board_center.x,
                        y: board_center.y,
                        z: board_center.z,
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
                color: board_color,
                ..Default::default()
            }
        };

        // Helper to build axis arrow markers
        let make_axis_arrow =
            |id: i32, rot_after_x: na::UnitQuaternion<f64>, r: f32, g: f32, b: f32| -> Marker {
                let rot = base_rotation * rot_after_x;
                let q = rot.quaternion();
                let len = board_model.board_shape.board_width.as_meters() * 0.5;

                Marker {
                    header: header.clone(),
                    ns: format!("board_axes{}", namespace_suffix),
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
                    color: ColorRGBA {
                        r,
                        g,
                        b,
                        a: axes_alpha,
                    },
                    ..Default::default()
                }
            };

        let rot_x = na::UnitQuaternion::identity();
        let rot_y = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), FRAC_PI_2);
        let rot_z = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), -FRAC_PI_2);

        let x_arrow = make_axis_arrow(id_offset + 1, rot_x, 1.0, 0.0, 0.0);
        let y_arrow = make_axis_arrow(id_offset + 2, rot_y, 0.0, 1.0, 0.0);
        let z_arrow = make_axis_arrow(id_offset + 3, rot_z, 0.0, 0.0, 1.0);

        // ArUco marker area border
        let marker_border = {
            let marker_top = board_model.marker_top_corner();
            let marker_bottom = board_model.marker_bottom_corner();
            let marker_left = board_model.marker_left_corner();
            let marker_right = board_model.marker_right_corner();

            Marker {
                header: header.clone(),
                ns: format!("board_marker_area{}", namespace_suffix),
                id: id_offset + 4,
                type_: 5,  // LINE_LIST
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose::default(),
                points: vec![
                    Point {
                        x: marker_bottom.x,
                        y: marker_bottom.y,
                        z: marker_bottom.z,
                    },
                    Point {
                        x: marker_left.x,
                        y: marker_left.y,
                        z: marker_left.z,
                    },
                    Point {
                        x: marker_left.x,
                        y: marker_left.y,
                        z: marker_left.z,
                    },
                    Point {
                        x: marker_top.x,
                        y: marker_top.y,
                        z: marker_top.z,
                    },
                    Point {
                        x: marker_top.x,
                        y: marker_top.y,
                        z: marker_top.z,
                    },
                    Point {
                        x: marker_right.x,
                        y: marker_right.y,
                        z: marker_right.z,
                    },
                    Point {
                        x: marker_right.x,
                        y: marker_right.y,
                        z: marker_right.z,
                    },
                    Point {
                        x: marker_bottom.x,
                        y: marker_bottom.y,
                        z: marker_bottom.z,
                    },
                ],
                scale: GeomVector3 {
                    x: 0.01,
                    y: 0.0,
                    z: 0.0,
                },
                color: ColorRGBA {
                    r: 1.0,
                    g: 0.7,
                    b: 0.0,
                    a: marker_area_alpha,
                },
                ..Default::default()
            }
        };

        // Three circular holes
        let hole_radius = board_model.board_shape.hole_radius.as_meters();

        let make_hole = |id: i32, center: na::Point3<f64>| -> Marker {
            let q = base_rotation.quaternion();
            Marker {
                header: header.clone(),
                ns: format!("board_holes{}", namespace_suffix),
                id,
                type_: 3,  // CYLINDER
                action: 0, // ADD
                pose: geometry_msgs::msg::Pose {
                    position: Point {
                        x: center.x,
                        y: center.y,
                        z: center.z,
                    },
                    orientation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
                scale: GeomVector3 {
                    x: hole_radius * 2.0,
                    y: hole_radius * 2.0,
                    z: 0.005,
                },
                color: ColorRGBA {
                    r: 0.3,
                    g: 0.3,
                    b: 0.3,
                    a: hole_alpha,
                },
                ..Default::default()
            }
        };

        let left_hole = make_hole(id_offset + 5, board_model.left_circle_center());
        let right_hole = make_hole(id_offset + 6, board_model.right_circle_center());
        let top_hole = make_hole(id_offset + 7, board_model.top_circle_center());

        Ok(MarkerArray {
            markers: vec![
                board_cube,
                x_arrow,
                y_arrow,
                z_arrow,
                marker_border,
                left_hole,
                right_hole,
                top_hole,
            ],
        })
    }

    fn create_board_markers(
        board_detection: &BoardDetection,
        header: &Header,
    ) -> Result<MarkerArray> {
        Self::create_board_visualization(
            &board_detection.board_model,
            header,
            "", // No namespace suffix for final board
            0,  // ID offset 0
            ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.6,
            }, // Green board
            1.0, // Full opacity for axes
            1.0, // Full opacity for marker area
            0.8, // Semi-transparent holes
        )
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
        Self::create_board_visualization(
            board_model,
            header,
            "_icp", // "_icp" namespace suffix for ICP iterations
            1000,   // ID offset 1000
            ColorRGBA {
                r: 1.0,
                g: 0.5,
                b: 0.0,
                a: 0.3,
            }, // Orange board for ICP
            0.9,    // Slightly transparent axes
            0.8,    // Slightly transparent marker area
            0.6,    // More transparent holes for ICP
        )
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
            "Iteration: {}, Loss: {:.6}, Correspondences: {}/{}",
            state.iteration,
            state.avg_loss,
            state.good_correspondences,
            state.total_correspondences
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
        let points = vec![
            // Board corners
            board_model.top_corner(),
            board_model.bottom_corner(),
            board_model.left_corner(),
            board_model.right_corner(),
            // Hole centers
            board_model.left_circle_center(),
            board_model.right_circle_center(),
            board_model.top_circle_center(),
            // Marker corners for more detail
            board_model.marker_bottom_corner(),
            board_model.marker_top_corner(),
            board_model.marker_left_corner(),
            board_model.marker_right_corner(),
            board_model.marker_center(),
        ];

        Self::create_debug_pointcloud(&points, header)
    }

    /// Create arrow markers for raw PCA eigenvectors before any orientation constraints
    #[allow(dead_code)]
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

        // Collect source (data) and target (model) points
        let mut source_points = Vec::new();
        let mut target_points = Vec::new();

        for (data_point, model_point) in state.correspondences.iter() {
            source_points.push(Point {
                x: data_point.x,
                y: data_point.y,
                z: data_point.z,
            });
            target_points.push(Point {
                x: model_point.x,
                y: model_point.y,
                z: model_point.z,
            });
        }

        // Create source points marker (red spheres)
        markers.push(Marker {
            header: header.clone(),
            ns: "correspondence_source".to_string(),
            id: 0,
            type_: 7,  // SPHERE_LIST
            action: 0, // ADD
            points: source_points.clone(),
            scale: GeomVector3 {
                x: 0.015, // Sphere diameter
                y: 0.015,
                z: 0.015,
            },
            color: ColorRGBA {
                r: 1.0, // Red
                g: 0.0,
                b: 0.0,
                a: 1.0,
            },
            ..Default::default()
        });

        // Create target points marker (green spheres)
        markers.push(Marker {
            header: header.clone(),
            ns: "correspondence_target".to_string(),
            id: 0,
            type_: 7,  // SPHERE_LIST
            action: 0, // ADD
            points: target_points.clone(),
            scale: GeomVector3 {
                x: 0.015, // Sphere diameter
                y: 0.015,
                z: 0.015,
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0, // Green
                b: 0.0,
                a: 1.0,
            },
            ..Default::default()
        });

        // Create line markers connecting each correspondence pair (yellow lines)
        for (i, (source, target)) in source_points.iter().zip(target_points.iter()).enumerate() {
            let marker = Marker {
                header: header.clone(),
                ns: "correspondence_lines".to_string(),
                id: i as i32,
                type_: 4,  // LINE_STRIP
                action: 0, // ADD
                points: vec![source.clone(), target.clone()],
                scale: GeomVector3 {
                    x: 0.003, // Line width
                    y: 0.0,
                    z: 0.0,
                },
                color: ColorRGBA {
                    r: 1.0, // Yellow
                    g: 1.0,
                    b: 0.0,
                    a: 0.6, // Semi-transparent
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

#[cfg(test)]
mod covariance_tests {
    use super::*;

    /// Build correspondences for a board lying in the world XY plane (normal = +Z), origin at c.
    ///
    /// `interior` points get a residual purely along the normal — that is what the correspondence
    /// model produces for a point whose plane projection lands inside the square and outside every
    /// hole. `in_plane` points get a residual in the plane, as a border-clamped or hole-rim point
    /// does.
    fn make_correspondences(
        n_interior: usize,
        n_in_plane: usize,
    ) -> (
        Vec<(na::Point3<f64>, na::Point3<f64>)>,
        na::Point3<f64>,
        na::Vector3<f64>,
    ) {
        let origin = na::Point3::new(0.0, 0.0, 0.0);
        let normal = na::Vector3::new(0.0, 0.0, 1.0);
        let mut corr = Vec::new();

        // Spread the points over a 1 m board so the rotation lever arms are realistic.
        for i in 0..n_interior {
            let t = i as f64 / n_interior.max(1) as f64;
            let x = -0.5 + t;
            let y = -0.5 + (t * 7.0).fract();
            let model = na::Point3::new(x, y, 0.0);
            // residual along the normal only
            let noise = if i % 2 == 0 { 0.003 } else { -0.003 };
            let data = na::Point3::new(x, y, noise);
            corr.push((data, model));
        }

        for i in 0..n_in_plane {
            let t = i as f64 / n_in_plane.max(1) as f64;
            // On the border, residual pointing in-plane (outward in +x).
            let model = na::Point3::new(0.5, -0.5 + t, 0.0);
            let data = na::Point3::new(0.5 + 0.003, -0.5 + t, 0.0);
            corr.push((data, model));
        }

        (corr, origin, normal)
    }

    fn diag(cov: &[f64; 36]) -> [f64; 6] {
        let mut d = [0.0; 6];
        for i in 0..6 {
            d[i] = cov[6 * i + i];
        }
        d
    }

    /// M-13's core claim: interior points constrain only the out-of-plane DoF, so a board seen
    /// with no edge or hole-rim returns is almost free to slide within its own plane. The
    /// covariance must SAY that, instead of reporting a confident zero.
    #[test]
    fn interior_only_correspondences_are_unobservable_in_plane() {
        let (corr, origin, normal) = make_correspondences(200, 0);

        let cov = CalibrationBoardLocatorNode::compute_pose_covariance(&corr, &origin, &normal);
        let d = diag(&cov);

        let (var_x, var_y, var_z) = (d[0], d[1], d[2]);
        let var_rz = d[5];

        assert!(
            var_z < 1e-4,
            "out-of-plane translation should be well determined by interior points, got {var_z}"
        );
        assert!(
            var_x > var_z * 1e3 && var_y > var_z * 1e3,
            "in-plane translation must be reported as poorly determined when only interior points \
             are present (interior residuals are purely along the normal and say nothing about \
             where the board sits within its plane); got var_x={var_x}, var_y={var_y}, var_z={var_z}"
        );
        assert!(
            var_rz > 1e-2,
            "rotation about the board normal is unobservable from interior points alone; got {var_rz}"
        );
    }

    /// Adding border / hole-rim points — the ones with an in-plane residual — must actually
    /// constrain the in-plane DoF. Otherwise the anisotropy above is an artefact, not a measurement.
    #[test]
    fn in_plane_correspondences_constrain_the_in_plane_dofs() {
        let (interior_only, origin, normal) = make_correspondences(200, 0);
        let (with_edges, _, _) = make_correspondences(200, 60);

        let before = diag(&CalibrationBoardLocatorNode::compute_pose_covariance(
            &interior_only,
            &origin,
            &normal,
        ));
        let after = diag(&CalibrationBoardLocatorNode::compute_pose_covariance(
            &with_edges,
            &origin,
            &normal,
        ));

        assert!(
            after[0] < before[0] / 100.0,
            "border points must sharply reduce the in-plane x variance: {} -> {}",
            before[0],
            after[0]
        );
        assert!(
            after[5] < before[5] / 10.0,
            "border points must reduce the yaw-about-normal variance: {} -> {}",
            before[5],
            after[5]
        );
    }

    /// Too few points: report nothing rather than a fabricated number.
    #[test]
    fn too_few_correspondences_yields_zero_covariance() {
        let (corr, origin, normal) = make_correspondences(4, 0);
        let cov = CalibrationBoardLocatorNode::compute_pose_covariance(&corr, &origin, &normal);
        assert_eq!(cov, [0.0; 36]);
    }

    /// The published order is ROS's: row-major 6x6, [x, y, z, rx, ry, rz]. Symmetry is a cheap
    /// guard that the row-major flattening is not transposed.
    #[test]
    fn covariance_is_symmetric_and_row_major() {
        let (corr, origin, normal) = make_correspondences(200, 60);
        let cov = CalibrationBoardLocatorNode::compute_pose_covariance(&corr, &origin, &normal);

        for row in 0..6 {
            for col in 0..6 {
                let a = cov[6 * row + col];
                let b = cov[6 * col + row];
                assert!(
                    (a - b).abs() < 1e-12 * (1.0 + a.abs()),
                    "covariance must be symmetric at ({row},{col}): {a} vs {b}"
                );
            }
        }
    }

    #[test]
    fn shipped_config_is_bbox_mode_by_default() {
        let text = include_str!("../../lctk_launch/config/board/board_detector.json5");
        let cfg = crate::bbox_free::parse_detection_config(text).unwrap();
        assert_eq!(
            crate::bbox_free::DetectionMode::parse(&cfg.detection_mode).unwrap(),
            crate::bbox_free::DetectionMode::Bbox
        );
    }
}
