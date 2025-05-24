use anyhow::{anyhow, bail, ensure, Context as AnyhowContext, Result};
use aruco_config::MultiArucoPattern;
use aruco_detector::multi_aruco::ImageMarker;
use cv_convert::prelude::*;
use geometry_msgs::msg::{Transform, TransformStamped, Vector3};
use hollow_board_config::BoardModel;
use itertools::izip;
use nalgebra as na;
use once_cell::sync::Lazy;
use opencv::core::{Point2d, Point2f, Point3d};
use pnp_solver::{PnpMethod, PnpSolver};
use rclrs::{
    log_info, log_warn, Context, CreateBasicExecutor, InitOptions, Node, Publisher,
    RclrsErrorFilter, SpinOptions, Subscription, ToLogParams,
};
use serde_types::{CameraIntrinsics, DistortionCoefs, MrptCalibration};
use std::{
    collections::HashMap,
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
    pending_detections: Mutex<HashMap<u64, DetectionPair>>,
    aruco_pattern: MultiArucoPattern,
    pnp_solver: PnpSolver,
    parent_frame: Arc<str>,
    child_frame: Arc<str>,
    sync_timeout: i64,
}

pub struct ExtrinsicSolverNode {
    state: Arc<ExtrinsicSolverState>,
    _node: Node,
    _aruco_subscription: Subscription<Detection2DArray>,
    _board_subscription: Subscription<Detection3DArray>,
    _transform_publisher: Publisher<TransformStamped>,
}

impl ExtrinsicSolverNode {
    pub fn new(node: Node) -> Result<Self> {
        // Declare parameters with defaults
        let intrinsics_file_param: Arc<str> =
            node.declare_parameter("intrinsics_file").mandatory()?.get();
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
        let sync_timeout_ms_param: i64 = node
            .declare_parameter("sync_timeout_ms")
            .default(100i64)
            .mandatory()?
            .get();

        // Load configurations
        let camera_intrinsics = Self::load_camera_intrinsics(&intrinsics_file_param)?;
        let aruco_pattern = Self::load_aruco_pattern(&aruco_pattern_file_param)?;
        let method: PnpMethod = method_param.parse()?;

        // Create PnP solver
        let pnp_solver = PnpSolver::new(&camera_intrinsics, method);

        // Create state
        let state = Arc::new(ExtrinsicSolverState {
            pending_detections: Mutex::new(HashMap::<u64, DetectionPair>::new()),
            aruco_pattern,
            pnp_solver,
            parent_frame: parent_frame_param,
            child_frame: child_frame_param,
            sync_timeout: sync_timeout_ms_param,
        });

        // Create publisher for extrinsic transforms
        let transform_publisher =
            node.create_publisher::<TransformStamped>("extrinsic_transform")?;

        // Create subscribers
        let aruco_subscription = {
            let state = Arc::clone(&state);
            let transform_publisher = Arc::clone(&transform_publisher);

            node.create_subscription::<Detection2DArray, _>(
                "aruco_detections",
                move |msg: Detection2DArray| {
                    Self::aruco_callback(msg, &state, &transform_publisher);
                },
            )?
        };

        let board_subscription = {
            let state = Arc::clone(&state);
            let transform_publisher = Arc::clone(&transform_publisher);

            node.create_subscription::<Detection3DArray, _>(
                "calibration_board_detections",
                move |msg: Detection3DArray| {
                    Self::board_callback(msg, &state, &transform_publisher);
                },
            )?
        };

        log_info!(
            LOGGER_NAME,
            "Solve extrinsic params node initialized. Subscribing to: aruco_detections, calibration_board_detections. Publishing to: extrinsic_transform"
        );

        Ok(Self {
            state,
            _node: node,
            _aruco_subscription: aruco_subscription,
            _board_subscription: board_subscription,
            _transform_publisher: transform_publisher,
        })
    }

    fn load_camera_intrinsics(intrinsics_file: &str) -> Result<CameraIntrinsics> {
        let path = PathBuf::from(intrinsics_file);
        let yaml_text = fs::read_to_string(&path)
            .with_context(|| format!("unable to open file '{}'", path.display()))?;
        let mrpt_calib: MrptCalibration = serde_yaml::from_str(&yaml_text)?;
        let camera_intrinsics = CameraIntrinsics {
            distortion_coefs: DistortionCoefs::zeros(),
            ..mrpt_calib.intrinsic_params()?
        };
        Ok(camera_intrinsics)
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
    ) {
        let timestamp = Self::get_timestamp_nanos(&msg.header);

        let mut pending = match state.pending_detections.lock() {
            Ok(guard) => guard,
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to lock pending detections: {e}");
                return;
            }
        };

        // Clean up old entries
        Self::cleanup_old_detections(&mut pending, timestamp, state.sync_timeout);

        // Look for matching board detection
        if let Some(mut pair) = pending.remove(&timestamp) {
            pair.aruco_detection = msg;
            drop(pending); // Release lock before processing

            if let Err(e) = Self::process_detection_pair(pair, publisher, state) {
                log_warn!(LOGGER_NAME, "Failed to process detection pair: {e}");
            }
        } else {
            // Store ArUco detection for future matching
            pending.insert(
                timestamp,
                DetectionPair {
                    aruco_detection: msg,
                    board_detection: Detection3DArray::default(),
                    _timestamp: timestamp,
                },
            );
        }
    }

    fn board_callback(
        msg: Detection3DArray,
        state: &Arc<ExtrinsicSolverState>,
        publisher: &Publisher<TransformStamped>,
    ) {
        let timestamp = Self::get_timestamp_nanos(&msg.header);

        let mut pending = match state.pending_detections.lock() {
            Ok(guard) => guard,
            Err(e) => {
                log_warn!(LOGGER_NAME, "Failed to lock pending detections: {e}");
                return;
            }
        };

        // Clean up old entries
        Self::cleanup_old_detections(&mut pending, timestamp, state.sync_timeout);

        // Look for matching ArUco detection
        if let Some(mut pair) = pending.remove(&timestamp) {
            pair.board_detection = msg;
            drop(pending); // Release lock before processing

            if let Err(e) = Self::process_detection_pair(pair, publisher, state) {
                log_warn!(LOGGER_NAME, "Failed to process detection pair: {e}");
            }
        } else {
            // Store board detection for future matching
            pending.insert(
                timestamp,
                DetectionPair {
                    aruco_detection: Detection2DArray::default(),
                    board_detection: msg,
                    _timestamp: timestamp,
                },
            );
        }
    }

    fn cleanup_old_detections(
        pending: &mut HashMap<u64, DetectionPair>,
        current_timestamp: u64,
        timeout_ms: i64,
    ) {
        let timeout_nanos = (timeout_ms * 1_000_000) as u64;
        pending.retain(|&timestamp, _| current_timestamp.saturating_sub(timestamp) < timeout_nanos);
    }

    fn process_detection_pair(
        pair: DetectionPair,
        publisher: &Publisher<TransformStamped>,
        state: &ExtrinsicSolverState,
    ) -> Result<()> {
        // Check if both detections are present
        if pair.aruco_detection.detections.is_empty() || pair.board_detection.detections.is_empty()
        {
            return Ok(()); // Skip if either detection is missing
        }

        // Convert ROS messages to internal types
        let board_model = Self::detection3d_to_board_model(&pair.board_detection.detections[0])?;
        let image_markers = Self::detection2d_array_to_image_markers(&pair.aruco_detection)?;

        // Solve PnP problem
        let point_pairs =
            Self::create_point_pairs(board_model, image_markers, &state.aruco_pattern)?;

        if let Some(transform) = state.pnp_solver.solve(point_pairs) {
            let transform_msg = Self::isometry_to_transform_stamped(
                transform,
                &pair.aruco_detection.header,
                &state.parent_frame,
                &state.child_frame,
            )?;

            if let Err(e) = publisher.publish(transform_msg) {
                log_warn!(LOGGER_NAME, "Failed to publish transform: {e}");
            } else {
                log_info!(LOGGER_NAME, "Published extrinsic transform");
            }
        } else {
            log_warn!(LOGGER_NAME, "PnP solver failed to find solution");
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
    let context = Context::new(std::env::args(), InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("solve_extrinsic_params")?;
    let _solve_extrinsic_params_node = ExtrinsicSolverNode::new(node)?;

    log_info!(LOGGER_NAME, "Solve extrinsic params node started");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| anyhow!("Failed to spin executor: {err}"))
}
