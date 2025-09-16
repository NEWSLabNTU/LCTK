use anyhow::{anyhow, ensure, Context as AnyhowContext, Result};
use aruco_config::MultiArucoPattern;
use aruco_detector::multi_aruco::ImageMarker;
use calibration_quality::{
    metrics::{GeometricError, QualityComponents, StatisticalMetrics},
    CalibrationMetrics, ConvergenceMonitor, QualityAssessor, ValidationConfig,
};
use cv_convert::prelude::*;
use dynamic_calibration::{
    AdjustmentStrategy, CalibrationParameters, DynamicCalibrationController,
};
use geometry_msgs::msg::{Transform, TransformStamped, Vector3};
use hollow_board_config::BoardModel;
use itertools::izip;
use nalgebra as na;
use once_cell::sync::Lazy;
use opencv::core::{Point2d, Point2f, Point3d};
use pnp_solver::{PnpMethod, PnpSolver};
use rclrs::*;
use sensor_msgs::msg::CameraInfo;
use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use vision_msgs::msg::{Detection2DArray, Detection3DArray};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

static DEFAULT_ARUCO_PATTERN: Lazy<MultiArucoPattern> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/aruco_pattern.json5"
    ));
    json5::from_str(text).unwrap()
});

#[derive(Debug, Clone)]
struct DetectionPair {
    aruco_detection: Detection2DArray,
    board_detection: Detection3DArray,
    _timestamp: u64, // nanoseconds since epoch
}

struct ExtrinsicSolverState {
    // Cache for most recent non-empty detections
    latest_aruco_detection: Mutex<Option<Detection2DArray>>,
    latest_board_detection: Mutex<Option<Detection3DArray>>,
    aruco_pattern: MultiArucoPattern,
    camera_info: Mutex<Option<CameraInfo>>,
    pnp_method: PnpMethod,
    parent_frame: Arc<str>,
    child_frame: Arc<str>,
    // Quality assessment components
    quality_assessment: Mutex<QualityAssessor>,
    convergence_monitor: Mutex<ConvergenceMonitor>,
    // Dynamic calibration controller
    dynamic_controller: Mutex<DynamicCalibrationController>,
    // Enable quality assessment flag
    enable_quality_assessment: bool,
    // Enable dynamic adjustment flag
    enable_dynamic_adjustment: bool,
}

pub struct ExtrinsicSolverNode {
    _state: Arc<ExtrinsicSolverState>,
    _node: Node,
    _aruco_subscription: Subscription<Detection2DArray>,
    _board_subscription: Subscription<Detection3DArray>,
    _camera_info_subscription: Subscription<CameraInfo>,
    _transform_publisher: Publisher<TransformStamped>,
    _quality_publisher: Publisher<std_msgs::msg::String>,
    _debug_aruco_publisher: Publisher<Detection2DArray>,
    _debug_board_publisher: Publisher<Detection3DArray>,
}

impl ExtrinsicSolverNode {
    pub fn new(node: Node) -> Result<Self> {
        // Declare parameters with defaults
        let aruco_pattern_file_param: Arc<str> = node
            .declare_parameter("aruco_pattern_file")
            .default(Arc::<str>::from(""))
            .mandatory()?
            .get();
        let method_param: Arc<str> = node
            .declare_parameter("method")
            .default(Arc::<str>::from("SQPNP"))
            .mandatory()?
            .get();
        let parent_frame_param: Arc<str> = node
            .declare_parameter("parent_frame")
            .default(Arc::<str>::from("lidar"))
            .mandatory()?
            .get();
        let child_frame_param: Arc<str> = node
            .declare_parameter("child_frame")
            .default(Arc::<str>::from("camera"))
            .mandatory()?
            .get();
        let enable_quality_assessment_param: bool = node
            .declare_parameter("enable_quality_assessment")
            .default(true)
            .mandatory()?
            .get();
        let enable_dynamic_adjustment_param: bool = node
            .declare_parameter("enable_dynamic_adjustment")
            .default(false)
            .mandatory()?
            .get();
        let adjustment_strategy_param: Arc<str> = node
            .declare_parameter("adjustment_strategy")
            .default(Arc::<str>::from("Balanced"))
            .mandatory()?
            .get();

        // Load configurations
        let aruco_pattern = Self::load_aruco_pattern(&aruco_pattern_file_param)?;
        let method: PnpMethod = method_param.parse()?;

        // Parse adjustment strategy
        let adjustment_strategy = match adjustment_strategy_param.as_ref() {
            "Conservative" => AdjustmentStrategy::Conservative,
            "Balanced" => AdjustmentStrategy::Balanced,
            "Aggressive" => AdjustmentStrategy::Aggressive,
            "Adaptive" => AdjustmentStrategy::Adaptive,
            _ => AdjustmentStrategy::Balanced,
        };

        // Create state
        let state = Arc::new(ExtrinsicSolverState {
            latest_aruco_detection: Mutex::new(None),
            latest_board_detection: Mutex::new(None),
            aruco_pattern,
            camera_info: Mutex::new(None),
            pnp_method: method,
            parent_frame: parent_frame_param,
            child_frame: child_frame_param,
            quality_assessment: Mutex::new(QualityAssessor::new(ValidationConfig::default())),
            convergence_monitor: Mutex::new(ConvergenceMonitor::new()),
            dynamic_controller: Mutex::new(DynamicCalibrationController::with_strategy(
                adjustment_strategy,
            )),
            enable_quality_assessment: enable_quality_assessment_param,
            enable_dynamic_adjustment: enable_dynamic_adjustment_param,
        });

        // Create publisher for extrinsic transforms
        let transform_publisher = node.create_publisher("extrinsic_transform")?;

        // Create publisher for calibration quality metrics
        let quality_publisher = node.create_publisher("calibration_quality")?;

        // Create debug publishers for most recent detections
        let debug_aruco_publisher = node.create_publisher("debug/recent_aruco_detections")?;
        let debug_board_publisher = node.create_publisher("debug/recent_board_detections")?;

        // Create subscribers
        let aruco_subscription = {
            let state = Arc::clone(&state);
            let transform_publisher = Arc::clone(&transform_publisher);
            let quality_publisher = Arc::clone(&quality_publisher);
            let debug_aruco_publisher = Arc::clone(&debug_aruco_publisher);

            node.create_subscription("aruco_detections", move |msg: Detection2DArray| {
                Self::aruco_callback(
                    msg,
                    &state,
                    &transform_publisher,
                    &quality_publisher,
                    &debug_aruco_publisher,
                );
            })?
        };

        let board_subscription = {
            let state = Arc::clone(&state);
            let transform_publisher = Arc::clone(&transform_publisher);
            let quality_publisher = Arc::clone(&quality_publisher);
            let debug_board_publisher = Arc::clone(&debug_board_publisher);

            node.create_subscription(
                "calibration_board_detections",
                move |msg: Detection3DArray| {
                    Self::board_callback(
                        msg,
                        &state,
                        &transform_publisher,
                        &quality_publisher,
                        &debug_board_publisher,
                    );
                },
            )?
        };

        let camera_info_subscription = {
            let state = Arc::clone(&state);

            node.create_subscription("camera_info", move |msg: CameraInfo| {
                Self::camera_info_callback(msg, &state);
            })?
        };

        log_info!(
            LOGGER_NAME,
            "Solve extrinsic params node initialized. Subscribing to: aruco_detections, calibration_board_detections, camera_info. Publishing to: extrinsic_transform, calibration_quality, debug/recent_aruco_detections, debug/recent_board_detections"
        );

        Ok(Self {
            _state: state,
            _node: node,
            _aruco_subscription: aruco_subscription,
            _board_subscription: board_subscription,
            _camera_info_subscription: camera_info_subscription,
            _transform_publisher: transform_publisher,
            _quality_publisher: quality_publisher,
            _debug_aruco_publisher: debug_aruco_publisher,
            _debug_board_publisher: debug_board_publisher,
        })
    }

    fn camera_info_callback(msg: CameraInfo, state: &Arc<ExtrinsicSolverState>) {
        // Store the CameraInfo directly
        if let Ok(mut camera_info_guard) = state.camera_info.lock() {
            *camera_info_guard = Some(msg.clone());
            log_debug!(
                LOGGER_NAME,
                "Extrinsic Solver: Camera info received - {}x{} resolution, distortion model: {}",
                msg.width,
                msg.height,
                msg.distortion_model
            );
        } else {
            log_warn!(LOGGER_NAME, "Failed to lock camera info mutex");
        }
    }

    fn load_aruco_pattern(aruco_pattern_file: &str) -> Result<MultiArucoPattern> {
        if aruco_pattern_file.is_empty() {
            log_info!(LOGGER_NAME, "Using default ArUco pattern configuration");
            return Ok(DEFAULT_ARUCO_PATTERN.clone());
        }

        let path = PathBuf::from(aruco_pattern_file);
        let text = fs::read_to_string(&path)
            .with_context(|| format!("unable to open file '{}'", path.display()))?;
        let pattern = json5::from_str(&text)?;
        Ok(pattern)
    }

    fn get_timestamp_nanos(header: &std_msgs::msg::Header) -> u64 {
        let sec = header.stamp.sec as u64;
        let nanosec = header.stamp.nanosec as u64;
        sec * 1_000_000_000 + nanosec
    }

    fn aruco_callback(
        msg: Detection2DArray,
        state: &Arc<ExtrinsicSolverState>,
        publisher: &Publisher<TransformStamped>,
        quality_publisher: &Publisher<std_msgs::msg::String>,
        _debug_publisher: &Publisher<Detection2DArray>,
    ) {
        log_info!(
            LOGGER_NAME,
            "Extrinsic Solver: ArUco callback ENTRY - {} detections at timestamp {}.{}",
            msg.detections.len(),
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        );

        // Only cache non-empty detections
        if !msg.detections.is_empty() {
            log_info!(
                LOGGER_NAME,
                "Extrinsic Solver: ✓ Caching non-empty ArUco detection with {} markers",
                msg.detections.len()
            );

            // Log detailed info about detection
            for (i, detection) in msg.detections.iter().enumerate() {
                log_debug!(
                    LOGGER_NAME,
                    "  Marker {}: bbox center=({:.2}, {:.2}), size=({:.2}, {:.2}), ID: {}",
                    i,
                    detection.bbox.center.position.x,
                    detection.bbox.center.position.y,
                    detection.bbox.size_x,
                    detection.bbox.size_y,
                    detection.id
                );
            }

            // Update the cached ArUco detection
            if let Ok(mut aruco_cache) = state.latest_aruco_detection.lock() {
                *aruco_cache = Some(msg.clone());

                // Immediately publish to debug topic when caching non-empty detection
                log_debug!(
                    LOGGER_NAME,
                    "Publishing NON-EMPTY ArUco detection to debug topic with {} markers",
                    msg.detections.len()
                );
                if let Err(e) = _debug_publisher.publish(msg.clone()) {
                    log_warn!(LOGGER_NAME, "Failed to publish debug ArUco detection: {e}");
                } else {
                    log_debug!(
                        LOGGER_NAME,
                        "Successfully published NON-EMPTY ArUco detection to debug topic"
                    );
                }
            } else {
                log_warn!(LOGGER_NAME, "Failed to lock ArUco detection cache");
                return;
            }
        } else {
            log_debug!(
                LOGGER_NAME,
                "Extrinsic Solver: Ignoring empty ArUco detection"
            );
        }

        // Try to process if we have both cached detections
        Self::try_process_cached_detections(state, publisher, quality_publisher, None, None);
    }

    fn board_callback(
        msg: Detection3DArray,
        state: &Arc<ExtrinsicSolverState>,
        publisher: &Publisher<TransformStamped>,
        quality_publisher: &Publisher<std_msgs::msg::String>,
        _debug_publisher: &Publisher<Detection3DArray>,
    ) {
        log_info!(
            LOGGER_NAME,
            "Extrinsic Solver: Board callback ENTRY - {} detections at timestamp {}.{}",
            msg.detections.len(),
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        );

        // Only cache non-empty detections
        if !msg.detections.is_empty() {
            log_info!(
                LOGGER_NAME,
                "Extrinsic Solver: ✓ Caching non-empty board detection with {} boards",
                msg.detections.len()
            );

            // Log detailed info about board detection
            for (i, detection) in msg.detections.iter().enumerate() {
                if !detection.results.is_empty() {
                    let pose = &detection.results[0].pose.pose;
                    log_debug!(
                        LOGGER_NAME,
                        "  Board {}: position=({:.3}, {:.3}, {:.3}), orientation=({:.3}, {:.3}, {:.3}, {:.3})",
                        i,
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w
                    );
                }
            }

            // Update the cached board detection
            if let Ok(mut board_cache) = state.latest_board_detection.lock() {
                *board_cache = Some(msg.clone());

                // Immediately publish to debug topic when caching non-empty detection
                log_debug!(
                    LOGGER_NAME,
                    "Publishing NON-EMPTY board detection to debug topic with {} boards",
                    msg.detections.len()
                );
                if let Err(e) = _debug_publisher.publish(msg.clone()) {
                    log_warn!(LOGGER_NAME, "Failed to publish debug board detection: {e}");
                } else {
                    log_debug!(
                        LOGGER_NAME,
                        "Successfully published NON-EMPTY board detection to debug topic"
                    );
                }
            } else {
                log_warn!(LOGGER_NAME, "Failed to lock board detection cache");
                return;
            }
        } else {
            log_error!(
                LOGGER_NAME,
                "🔥 DEBUG: Extrinsic Solver received EMPTY board detection - NOT caching, NOT publishing to debug"
            );
        }

        // Try to process if we have both cached detections
        Self::try_process_cached_detections(state, publisher, quality_publisher, None, None);
    }

    fn publish_cached_detections_to_debug(
        state: &Arc<ExtrinsicSolverState>,
        debug_aruco_publisher: &Publisher<Detection2DArray>,
        debug_board_publisher: &Publisher<Detection3DArray>,
    ) {
        // Publish cached ArUco detection if available
        if let Ok(aruco_cache) = state.latest_aruco_detection.lock() {
            if let Some(ref cached_msg) = *aruco_cache {
                if let Err(e) = debug_aruco_publisher.publish(cached_msg.clone()) {
                    log_debug!(
                        LOGGER_NAME,
                        "Failed to publish cached ArUco detection to debug: {e}"
                    );
                }
            }
        }

        // Publish cached board detection if available
        if let Ok(board_cache) = state.latest_board_detection.lock() {
            if let Some(ref cached_msg) = *board_cache {
                if let Err(e) = debug_board_publisher.publish(cached_msg.clone()) {
                    log_debug!(
                        LOGGER_NAME,
                        "Failed to publish cached board detection to debug: {e}"
                    );
                }
            }
        }
    }

    fn try_process_cached_detections(
        state: &Arc<ExtrinsicSolverState>,
        publisher: &Publisher<TransformStamped>,
        quality_publisher: &Publisher<std_msgs::msg::String>,
        _debug_aruco_publisher: Option<&Publisher<Detection2DArray>>,
        _debug_board_publisher: Option<&Publisher<Detection3DArray>>,
    ) {
        // Try to get both cached detections
        let aruco_detection = {
            match state.latest_aruco_detection.lock() {
                Ok(cache) => cache.clone(),
                Err(e) => {
                    log_warn!(LOGGER_NAME, "Failed to lock ArUco detection cache: {e}");
                    return;
                }
            }
        };

        let board_detection = {
            match state.latest_board_detection.lock() {
                Ok(cache) => cache.clone(),
                Err(e) => {
                    log_warn!(LOGGER_NAME, "Failed to lock board detection cache: {e}");
                    return;
                }
            }
        };

        // Debug publishing removed - now done immediately when caching non-empty detections

        // Check if we have both non-empty cached detections
        match (aruco_detection, board_detection) {
            (Some(aruco_msg), Some(board_msg)) => {
                log_info!(
                    LOGGER_NAME,
                    "Extrinsic Solver: ✓✓✓ BOTH cached detections available - ArUco: {} markers, Board: {} boards",
                    aruco_msg.detections.len(),
                    board_msg.detections.len()
                );

                // Create detection pair from cached detections
                let pair = DetectionPair {
                    aruco_detection: aruco_msg,
                    board_detection: board_msg,
                    _timestamp: 0, // Not used in new approach
                };

                // Process the detection pair
                if let Err(e) =
                    Self::process_detection_pair(pair, publisher, quality_publisher, state)
                {
                    log_error!(
                        LOGGER_NAME,
                        "Extrinsic Solver: ✗ Failed to process cached detection pair: {e}"
                    );
                } else {
                    log_info!(LOGGER_NAME, "Extrinsic Solver: ✓✓✓ Successfully processed cached detection pair - SOLUTION COMPUTED!");
                }
            }
            (aruco_opt, board_opt) => {
                log_debug!(
                    LOGGER_NAME,
                    "Extrinsic Solver: Waiting for both detections - ArUco cached: {}, Board cached: {}",
                    aruco_opt.is_some(),
                    board_opt.is_some()
                );
            }
        }
    }

    fn process_detection_pair(
        pair: DetectionPair,
        publisher: &Publisher<TransformStamped>,
        quality_publisher: &Publisher<std_msgs::msg::String>,
        state: &ExtrinsicSolverState,
    ) -> Result<()> {
        log_info!(
            LOGGER_NAME,
            "Extrinsic Solver: >>> PROCESSING DETECTION PAIR - ArUco: {} detections, Board: {} detections",
            pair.aruco_detection.detections.len(),
            pair.board_detection.detections.len()
        );

        // Check if both detections are present
        if pair.aruco_detection.detections.is_empty() || pair.board_detection.detections.is_empty()
        {
            log_warn!(
                LOGGER_NAME,
                "Extrinsic Solver: ✗ Skipping pair - ArUco empty: {}, Board empty: {}",
                pair.aruco_detection.detections.is_empty(),
                pair.board_detection.detections.is_empty()
            );
            return Ok(()); // Skip if either detection is missing
        }

        log_info!(
            LOGGER_NAME,
            "Extrinsic Solver: ✓ Both ArUco and Board detections present - proceeding"
        );

        // Convert ROS messages to internal types
        log_debug!(
            LOGGER_NAME,
            "Extrinsic Solver: Converting board detection to board model..."
        );
        let board_model = Self::detection3d_to_board_model(&pair.board_detection.detections[0])?;
        log_debug!(LOGGER_NAME, "Extrinsic Solver: ✓ Board model created");

        log_debug!(
            LOGGER_NAME,
            "Extrinsic Solver: Converting ArUco detections to image markers..."
        );
        let image_markers = Self::detection2d_array_to_image_markers(&pair.aruco_detection)?;
        log_debug!(
            LOGGER_NAME,
            "Extrinsic Solver: ✓ Image markers created: {} markers",
            image_markers.len()
        );

        // Check if camera info is available
        log_debug!(
            LOGGER_NAME,
            "Extrinsic Solver: Checking camera info availability..."
        );
        let camera_info = {
            let camera_info_guard = state
                .camera_info
                .lock()
                .map_err(|e| anyhow!("Failed to lock camera info mutex: {}", e))?;
            match camera_info_guard.as_ref() {
                Some(info) => {
                    log_info!(
                        LOGGER_NAME,
                        "Extrinsic Solver: ✓ Camera info available - {}x{} resolution, distortion: {}",
                        info.width,
                        info.height,
                        info.distortion_model
                    );
                    info.clone()
                }
                None => {
                    log_error!(
                        LOGGER_NAME,
                        "Extrinsic Solver: ✗ Camera info not available - cannot proceed with calibration"
                    );
                    return Ok(());
                }
            }
        };

        // Create PnP solver with current camera info
        log_debug!(
            LOGGER_NAME,
            "Extrinsic Solver: Creating PnP solver with method: {:?}",
            state.pnp_method
        );
        let pnp_solver = PnpSolver::new(&camera_info, state.pnp_method);
        log_debug!(LOGGER_NAME, "Extrinsic Solver: ✓ PnP solver created");

        // Get dynamic parameters if enabled
        let current_params = if state.enable_dynamic_adjustment {
            let controller = state
                .dynamic_controller
                .lock()
                .map_err(|e| anyhow!("Failed to lock dynamic controller: {}", e))?;
            controller.parameters().clone()
        } else {
            CalibrationParameters::default()
        };

        // Solve PnP problem
        log_debug!(
            LOGGER_NAME,
            "Extrinsic Solver: Creating point pairs for PnP solving..."
        );
        let point_pairs =
            Self::create_point_pairs(board_model, image_markers, &state.aruco_pattern)?;

        log_info!(
            LOGGER_NAME,
            "Extrinsic Solver: ✓ Created {} point pairs for PnP solving",
            point_pairs.len()
        );

        if point_pairs.is_empty() {
            log_error!(
                LOGGER_NAME,
                "Extrinsic Solver: ✗ No point pairs created - cannot solve PnP"
            );
            return Ok(());
        }

        log_info!(
            LOGGER_NAME,
            "Extrinsic Solver: Attempting to solve PnP with {} point pairs...",
            point_pairs.len()
        );
        if let Some(transform) = pnp_solver.solve(point_pairs.clone()) {
            log_info!(
                LOGGER_NAME,
                "Extrinsic Solver: ✓✓✓ PnP solver SUCCESS! - transform: translation=({:.3}, {:.3}, {:.3}), rotation=({:.3}, {:.3}, {:.3}, {:.3})",
                transform.translation.x, transform.translation.y, transform.translation.z,
                transform.rotation.i, transform.rotation.j, transform.rotation.k, transform.rotation.w
            );
            // Quality assessment if enabled
            if state.enable_quality_assessment {
                let metrics = Self::compute_calibration_metrics(
                    &point_pairs,
                    &transform,
                    &pair.aruco_detection,
                    &pair.board_detection,
                    &current_params,
                )?;

                // Assess quality
                let mut quality_assessment = state
                    .quality_assessment
                    .lock()
                    .map_err(|e| anyhow!("Failed to lock quality assessment: {}", e))?;
                let quality = quality_assessment.assess(
                    &transform,
                    &point_pairs
                        .iter()
                        .map(|(obj, img)| {
                            (
                                nalgebra::Point3::new(obj.x, obj.y, obj.z),
                                nalgebra::Point3::new(img.x, img.y, 0.0),
                            )
                        })
                        .collect::<Vec<_>>(),
                    metrics.detection_confidence,
                )?;

                // Monitor convergence
                let mut convergence_monitor = state
                    .convergence_monitor
                    .lock()
                    .map_err(|e| anyhow!("Failed to lock convergence monitor: {}", e))?;
                convergence_monitor.update(&transform, &metrics);
                let convergence_status = convergence_monitor.status();

                // Create quality report
                let quality_report = serde_json::json!({
                    "overall_quality": quality.overall_score,
                    "metrics": {
                        "reprojection_error": metrics.reprojection_error,
                        "inlier_ratio": metrics.inlier_ratio,
                        "detection_confidence": metrics.detection_confidence,
                        "consistency_score": metrics.consistency_score,
                    },
                    "validation": quality.validation,
                    "convergence": {
                        "is_converged": convergence_status.is_converged,
                        "iterations": convergence_status.iterations,
                        "convergence_rate": convergence_status.convergence_rate,
                    },
                    "parameters": if state.enable_dynamic_adjustment {
                        Some(current_params.summary())
                    } else {
                        None
                    }
                });

                // Publish quality metrics
                let quality_msg = std_msgs::msg::String {
                    data: quality_report.to_string(),
                };
                if let Err(e) = quality_publisher.publish(quality_msg) {
                    log_warn!(LOGGER_NAME, "Failed to publish quality metrics: {e}");
                }
                // Also print the entire calibration quality message for debugging/inspection
                log_info!(
                    LOGGER_NAME,
                    "Calibration quality message: {}",
                    quality_report.to_string()
                );

                // Dynamic parameter adjustment if enabled
                if state.enable_dynamic_adjustment {
                    let mut controller = state
                        .dynamic_controller
                        .lock()
                        .map_err(|e| anyhow!("Failed to lock dynamic controller: {}", e))?;

                    let quality_score = calibration_quality::QualityScore {
                        overall: quality.overall_score,
                        components: QualityComponents::default(),
                    };

                    let updated_params = controller.update(&metrics, &quality_score)?;
                    log_info!(
                        LOGGER_NAME,
                        "Updated calibration parameters: {}",
                        updated_params.summary()
                    );
                }

                // Log quality information
                log_info!(
                    LOGGER_NAME,
                    "Calibration quality: {:.2}%, Convergence: {}, Inliers: {}/{}",
                    quality.overall_score * 100.0,
                    if convergence_status.is_converged {
                        "Yes"
                    } else {
                        "No"
                    },
                    metrics.num_inliers,
                    metrics.num_correspondences
                );
            }

            let transform_msg = Self::isometry_to_transform_stamped(
                transform,
                &pair.aruco_detection.header,
                &state.parent_frame,
                &state.child_frame,
            )?;

            if let Err(e) = publisher.publish(transform_msg) {
                log_warn!(LOGGER_NAME, "Failed to publish transform: {e}");
            } else {
                log_info!(
                    LOGGER_NAME,
                    "Extrinsic Solver: Published extrinsic transform"
                );
                let rotation_matrix = transform.rotation.to_rotation_matrix();
                let r = rotation_matrix.matrix();
                let t = &transform.translation;
                log_info!(
                    LOGGER_NAME,
                    "Extrinsic T (4x4):\n[ {:.6} {:.6} {:.6} {:.6} ]\n[ {:.6} {:.6} {:.6} {:.6} ]\n[ {:.6} {:.6} {:.6} {:.6} ]\n[ {:.6} {:.6} {:.6} {:.6} ]",
                    r[(0, 0)], r[(0, 1)], r[(0, 2)], t.x,
                    r[(1, 0)], r[(1, 1)], r[(1, 2)], t.y,
                    r[(2, 0)], r[(2, 1)], r[(2, 2)], t.z,
                    0.0f64,    0.0f64,    0.0f64,    1.0f64
                );
            }
        } else {
            log_error!(
                LOGGER_NAME,
                "Extrinsic Solver: ✗✗✗ PnP solver FAILED to find solution - check point correspondences and camera calibration"
            );
        }

        Ok(())
    }

    fn detection3d_to_board_model(detection: &vision_msgs::msg::Detection3D) -> Result<BoardModel> {
        // Extract pose from detection
        if detection.results.is_empty() {
            return Err(anyhow!("No detection results available"));
        }

        let pose = &detection.results[0].pose.pose;
        let translation = na::Vector3::new(pose.position.x, pose.position.y, pose.position.z);
        let rotation = na::UnitQuaternion::new_normalize(na::Quaternion::new(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ));
        let pose_isometry = na::Isometry3::from_parts(translation.into(), rotation);

        // Create a basic board model
        // Note: This is simplified - you may need to extract more information
        // from the detection or have additional board configuration
        use measurements::Length;

        Ok(BoardModel {
            pose: pose_isometry,
            marker_paper_size: Length::from_millimeters(100.0), // Default 100mm
            board_shape: hollow_board_config::BoardShape {
                board_width: Length::from_millimeters(1000.0), // Default 1m board width
                hole_radius: Length::from_millimeters(150.0),  // Default 150mm hole radius
                hole_center_shift: Length::from_millimeters(200.0), // Default 200mm hole center shift
            },
        })
    }

    fn detection2d_array_to_image_markers(
        detection_array: &Detection2DArray,
    ) -> Result<Vec<ImageMarker>> {
        // Convert Detection2DArray to ImageMarker format
        // This is a simplified conversion - you may need to adjust based on
        // how the ArUco detection data is structured
        let mut markers = Vec::new();

        for detection in &detection_array.detections {
            // Extract corner points from bounding box
            // Note: This is a simplified approach - real ArUco markers have 4 corners
            let bbox = &detection.bbox;
            let center_x = bbox.center.position.x;
            let center_y = bbox.center.position.y;
            let size_x = bbox.size_x;
            let size_y = bbox.size_y;

            // Create 4 corners from bounding box
            let corners = [
                na::Point2::new(
                    (center_x - size_x / 2.0) as f32,
                    (center_y - size_y / 2.0) as f32,
                ),
                na::Point2::new(
                    (center_x + size_x / 2.0) as f32,
                    (center_y - size_y / 2.0) as f32,
                ),
                na::Point2::new(
                    (center_x + size_x / 2.0) as f32,
                    (center_y + size_y / 2.0) as f32,
                ),
                na::Point2::new(
                    (center_x - size_x / 2.0) as f32,
                    (center_y + size_y / 2.0) as f32,
                ),
            ];

            // Extract marker ID from detection results
            let marker_id = if !detection.results.is_empty() {
                detection.results[0]
                    .hypothesis
                    .class_id
                    .parse::<i32>()
                    .unwrap_or(0)
            } else {
                0
            };

            markers.push(ImageMarker {
                id: marker_id,
                corners,
            });
        }

        Ok(markers)
    }

    fn create_point_pairs(
        board: BoardModel,
        markers: Vec<ImageMarker>,
        aruco_pattern: &MultiArucoPattern,
    ) -> Result<Vec<(Point3d, Point2d)>> {
        let object_points: Vec<Point3d> = board
            .multi_marker_corners(aruco_pattern)
            .into_iter()
            .flatten()
            .map(|p| -> Point3d { p.to_cv() })
            .collect();

        let image_points: Vec<Point2d> = markers
            .into_iter()
            .flat_map(|marker| {
                let corners: Vec<_> = marker
                    .corners
                    .iter()
                    .map(|corner| {
                        let point2f: Point2f = corner.to_cv();
                        let point2d: Point2d = point2f.to().unwrap();
                        point2d
                    })
                    .collect();
                corners
            })
            .collect();

        ensure!(
            object_points.len() == image_points.len(),
            "the number of object points ({object_points_len}) \
	     does not match the number of image points ({image_points_len})",
            object_points_len = object_points.len(),
            image_points_len = image_points.len()
        );

        let point_pairs = izip!(object_points, image_points).collect();
        Ok(point_pairs)
    }

    fn compute_calibration_metrics(
        point_pairs: &[(Point3d, Point2d)],
        transform: &na::Isometry3<f64>,
        aruco_detection: &Detection2DArray,
        _board_detection: &Detection3DArray,
        params: &CalibrationParameters,
    ) -> Result<CalibrationMetrics> {
        // Compute reprojection errors
        let mut reprojection_errors = Vec::new();
        let mut num_inliers = 0;

        for (object_point, image_point) in point_pairs {
            // Transform object point to camera frame
            let pt_3d = na::Point3::new(object_point.x, object_point.y, object_point.z);
            let transformed = transform * pt_3d;

            // Project to image plane (simplified projection, assumes normalized camera)
            if transformed.z > 0.0 {
                let projected_x = transformed.x / transformed.z;
                let projected_y = transformed.y / transformed.z;

                let error = ((projected_x - image_point.x).powi(2)
                    + (projected_y - image_point.y).powi(2))
                .sqrt();
                reprojection_errors.push(error);

                if error < params.outlier_threshold * 100.0 {
                    // Convert to pixels
                    num_inliers += 1;
                }
            }
        }

        // Compute statistics
        let mean_error = if !reprojection_errors.is_empty() {
            reprojection_errors.iter().sum::<f64>() / reprojection_errors.len() as f64
        } else {
            f64::MAX
        };

        let error_std_dev = if reprojection_errors.len() > 1 {
            let variance = reprojection_errors
                .iter()
                .map(|&e| (e - mean_error).powi(2))
                .sum::<f64>()
                / (reprojection_errors.len() - 1) as f64;
            variance.sqrt()
        } else {
            0.0
        };

        // Sort errors for percentile calculations
        let mut sorted_errors = reprojection_errors.clone();
        sorted_errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let median_error = if !sorted_errors.is_empty() {
            sorted_errors[sorted_errors.len() / 2]
        } else {
            0.0
        };

        let percentile_95_error = if !sorted_errors.is_empty() {
            let idx = ((sorted_errors.len() as f64 * 0.95) as usize).min(sorted_errors.len() - 1);
            sorted_errors[idx]
        } else {
            0.0
        };

        // Detection confidence (simplified - based on detection count)
        let expected_detections = 4; // Assuming 4 ArUco markers
        let detection_confidence =
            (aruco_detection.detections.len() as f64 / expected_detections as f64).min(1.0);

        // Create metrics
        Ok(CalibrationMetrics {
            reprojection_error: mean_error,
            consistency_score: 0.8, // Placeholder - would compute from multiple frames
            detection_confidence,
            num_inliers,
            num_correspondences: point_pairs.len(),
            inlier_ratio: num_inliers as f64 / point_pairs.len().max(1) as f64,
            geometric_error: GeometricError {
                mean_translation_error: mean_error * 0.01, // Convert to meters (approximation)
                mean_rotation_error: 0.0,                  // Would need ground truth to compute
                max_translation_error: sorted_errors.last().copied().unwrap_or(0.0) * 0.01,
                max_rotation_error: 0.0,
            },
            statistical_metrics: StatisticalMetrics {
                error_std_dev,
                median_error,
                percentile_95_error,
                outlier_count: point_pairs.len() - num_inliers,
            },
        })
    }

    fn isometry_to_transform_stamped(
        isometry: na::Isometry3<f64>,
        header: &std_msgs::msg::Header,
        parent_frame: &str,
        child_frame: &str,
    ) -> Result<TransformStamped> {
        let translation = Vector3 {
            x: isometry.translation.x,
            y: isometry.translation.y,
            z: isometry.translation.z,
        };

        let rotation = geometry_msgs::msg::Quaternion {
            x: isometry.rotation.i,
            y: isometry.rotation.j,
            z: isometry.rotation.k,
            w: isometry.rotation.w,
        };

        let transform = Transform {
            translation,
            rotation,
        };

        Ok(TransformStamped {
            header: std_msgs::msg::Header {
                stamp: header.stamp.clone(),
                frame_id: parent_frame.to_string(),
            },
            child_frame_id: child_frame.to_string(),
            transform,
        })
    }
}

fn main() -> Result<()> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("solve_extrinsic_params")?;
    let _solve_extrinsic_params_node = ExtrinsicSolverNode::new(node)?;

    log_info!(LOGGER_NAME, "Solve extrinsic params node started");
    log_info!(
        LOGGER_NAME,
        "Extrinsic Solver: Waiting for synchronized detection messages..."
    );

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
