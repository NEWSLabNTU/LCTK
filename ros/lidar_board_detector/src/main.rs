mod bbox;
mod bbox_free;

use crate::bbox::BBox;
use anyhow::{anyhow, Result};
use arc_swap::ArcSwap;
use board_cluster_detector::{
    config::{DetectorTuning, ForegroundMethod, TargetSide},
    detector::detect_for_target,
    geometry::{project_to_plane, PlaneModel},
    square_fit::fit_fixed_square,
};
use calibration_target::{Surface, TargetIdentity, ValidatedTarget};
use calibration_target_detector::{
    PerforatedIcpConfig, SolidRefinementTuning, TargetDetection, TargetDetectionDiagnostics,
    TargetPoseEstimate, TargetPoseEstimator, TargetPoseEstimatorTuning, TargetRejectReason,
    TargetSquarePlaneObservation,
};
use geometry_msgs::msg::{Point, Pose, PoseWithCovariance, Quaternion, Vector3 as GeomVector3};
use lctk_interfaces::msg::CalibrationTargetIdentity;
use nalgebra as na;
use rclrs::{MandatoryParameter, ParameterRange, PublisherOptions, SubscriptionOptions, *};
use sensor_msgs::msg::{PointCloud2, PointField};
use std::{
    fs,
    path::PathBuf,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc,
    },
    thread::JoinHandle,
    time::Instant,
};
use std_msgs::msg::{ColorRGBA, Header};
use vision_msgs::msg::{BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose};
use visualization_msgs::msg::{Marker, MarkerArray};

#[cfg(test)]
use board_cluster_detector::detector::SquarePlaneObservation;
#[cfg(test)]
use calibration_target_detector::{CutoutIcpEvidence, EdgeCoverageEvidence, IcpTermination};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

const LEGACY_HOLLOW_TARGET: &[u8] =
    include_bytes!("../../lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5");

/// Every generated detector has one identity in its own namespace.  The
/// publisher is latched below so a solver may join after node startup.
const TARGET_IDENTITY_TOPIC: &str = "target_identity";

/// Sensor axis used by the physical mounting-up orientation reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Deserialize, Default)]
#[serde(rename_all = "lowercase")]
enum SensorUpAxis {
    X,
    Y,
    #[default]
    Z,
}

impl SensorUpAxis {
    fn as_vector(self) -> na::Vector3<f64> {
        match self {
            Self::X => na::Vector3::x(),
            Self::Y => na::Vector3::y(),
            Self::Z => na::Vector3::z(),
        }
    }
}

fn default_max_icp_iterations() -> usize {
    50
}

fn default_foreground_method() -> ForegroundMethod {
    ForegroundMethod::BackgroundSubtraction
}

fn default_bbf_voxel() -> f64 {
    0.05
}

fn default_dilation_radius() -> i64 {
    1
}

fn default_warmup_frames() -> usize {
    20
}

fn default_icp_pose_weight_threshold() -> f64 {
    1e-4
}

fn default_icp_rejection_threshold() -> f64 {
    0.008
}

fn default_icp_good_fit_threshold() -> f64 {
    0.035
}

fn default_icp_outlier_threshold() -> f64 {
    0.05
}

fn default_icp_damping_factor() -> f64 {
    0.5
}

fn default_icp_min_inlier_points() -> usize {
    100
}

fn default_plane_ransac_max_iterations() -> usize {
    2_000
}

fn default_plane_ransac_inlier_threshold() -> f64 {
    0.05
}

fn default_voxel_downsample_size() -> f64 {
    0.015
}

fn default_voxel_parallel_threshold() -> usize {
    50_000
}

fn default_perforated_min_hypothesis_loss_separation_m() -> f64 {
    0.0001
}

fn default_perforated_min_cutout_rim_correspondences() -> usize {
    1
}

fn default_perforated_cutout_rim_tolerance_m() -> f64 {
    0.03
}

/// Detector Tuning plus the deployment-owned processing stages.  Target
/// geometry is intentionally absent: `target_config` supplies it separately.
/// The old `board_detector_file` remains readable through the same shape while
/// the launch graph migrates; its board dimensions are ignored.
#[derive(Debug, Clone, serde::Deserialize)]
struct DetectorConfig {
    #[serde(default)]
    detection_mode: bbox_free::DetectionMode,
    #[serde(default = "default_foreground_method")]
    foreground_method: ForegroundMethod,
    #[serde(default = "default_bbf_voxel")]
    bbf_voxel: f64,
    #[serde(default = "default_dilation_radius")]
    bg_dilation_radius: i64,
    #[serde(default = "default_warmup_frames")]
    bg_warmup_frames: usize,

    #[serde(default)]
    skip_ransac: bool,
    #[serde(default = "default_plane_ransac_max_iterations")]
    plane_ransac_max_iterations: usize,
    #[serde(default = "default_plane_ransac_inlier_threshold")]
    plane_ransac_inlier_threshold: f64,

    #[serde(default)]
    voxel_downsample_enabled: bool,
    #[serde(default = "default_voxel_downsample_size")]
    voxel_downsample_size: f64,
    #[serde(default)]
    voxel_downsample_use_centroid: bool,
    #[serde(default = "default_voxel_parallel_threshold")]
    voxel_parallel_threshold: usize,

    #[serde(default)]
    sensor_up_axis: SensorUpAxis,
    #[serde(default = "default_max_icp_iterations")]
    max_icp_iterations: usize,
    #[serde(default = "default_icp_pose_weight_threshold")]
    icp_pose_weight_threshold: f64,
    #[serde(default = "default_icp_rejection_threshold")]
    icp_rejection_threshold: f64,
    #[serde(default = "default_icp_good_fit_threshold")]
    icp_good_fit_threshold: f64,
    #[serde(default = "default_icp_outlier_threshold")]
    icp_outlier_threshold: f64,
    #[serde(default = "default_icp_damping_factor")]
    icp_damping_factor: f64,
    #[serde(default = "default_icp_min_inlier_points")]
    icp_min_inlier_points: usize,

    // Solid adapter tuning.  Aliases make the flat deployment config readable
    // without creating a second target-specific parser.
    #[serde(alias = "edge_band_m")]
    solid_edge_band_m: Option<f64>,
    #[serde(alias = "minimum_edge_points")]
    solid_minimum_edge_points: Option<usize>,
    #[serde(alias = "minimum_points_per_covered_edge")]
    solid_minimum_points_per_covered_edge: Option<usize>,
    #[serde(alias = "minimum_covered_edges")]
    solid_minimum_covered_edges: Option<usize>,
    #[serde(alias = "longitudinal_bins_per_edge")]
    solid_longitudinal_bins_per_edge: Option<usize>,
    #[serde(alias = "minimum_occupied_longitudinal_bins")]
    solid_minimum_occupied_longitudinal_bins: Option<usize>,

    #[serde(default = "default_perforated_min_hypothesis_loss_separation_m")]
    min_hypothesis_loss_separation_m: f64,
    #[serde(default = "default_perforated_min_cutout_rim_correspondences")]
    min_cutout_rim_correspondences: usize,
    #[serde(default = "default_perforated_cutout_rim_tolerance_m")]
    cutout_rim_tolerance_m: f64,

    #[serde(flatten)]
    tuning: DetectorTuning,
}

impl DetectorConfig {
    fn target_side(&self, target: &ValidatedTarget) -> Result<TargetSide> {
        TargetSide::metres(target.plate().side_um as f64 / 1_000_000.0)
    }

    fn estimator_tuning(&self, target: &ValidatedTarget) -> Result<TargetPoseEstimatorTuning> {
        match target.plate().surface {
            Surface::Solid => Ok(TargetPoseEstimatorTuning::for_solid(
                SolidRefinementTuning::new(
                    required_config("solid_edge_band_m", self.solid_edge_band_m)?,
                    required_config("solid_minimum_edge_points", self.solid_minimum_edge_points)?,
                    required_config(
                        "solid_minimum_points_per_covered_edge",
                        self.solid_minimum_points_per_covered_edge,
                    )?,
                    required_config(
                        "solid_minimum_covered_edges",
                        self.solid_minimum_covered_edges,
                    )?,
                    required_config(
                        "solid_longitudinal_bins_per_edge",
                        self.solid_longitudinal_bins_per_edge,
                    )?,
                    required_config(
                        "solid_minimum_occupied_longitudinal_bins",
                        self.solid_minimum_occupied_longitudinal_bins,
                    )?,
                ),
            )),
            Surface::Perforated { .. } => Ok(TargetPoseEstimatorTuning::for_perforated(
                PerforatedIcpConfig::new(
                    self.max_icp_iterations,
                    self.icp_outlier_threshold,
                    self.icp_damping_factor,
                    self.icp_pose_weight_threshold,
                    self.icp_rejection_threshold,
                    self.icp_good_fit_threshold,
                    self.icp_min_inlier_points,
                    self.min_hypothesis_loss_separation_m,
                    self.min_cutout_rim_correspondences,
                    self.cutout_rim_tolerance_m,
                ),
            )),
        }
    }

    fn bbox_free_config(&self) -> bbox_free::BboxFreeRaw {
        bbox_free::BboxFreeRaw {
            method: self.foreground_method,
            voxel: self.bbf_voxel,
            board: self.tuning.clone(),
            background: bbox_free::BackgroundParams {
                dilation_radius: self.bg_dilation_radius,
                warmup_frames: self.bg_warmup_frames,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConfigSourceKind {
    New,
    Legacy,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ConfigSource {
    kind: ConfigSourceKind,
    target_config: Option<String>,
    detector_config: String,
}

fn nonempty(value: Option<&str>) -> Option<&str> {
    value.filter(|value| !value.trim().is_empty())
}

/// Keep candidate extraction pose-neutral.  In particular, the old stance
/// floor is not allowed to reject bbox-free evidence before the target-aware
/// estimator applies its own board-up gate.
fn neutral_detector_tuning(tuning: &DetectorTuning) -> DetectorTuning {
    let mut neutral = tuning.clone();
    neutral.stance_floor = 0.0;
    neutral
}

fn required_config<T>(name: &str, value: Option<T>) -> Result<T> {
    value
        .ok_or_else(|| anyhow!("Detector Tuning must explicitly provide {name} for a solid target"))
}

/// Select exactly one configuration contract.  A legacy source always binds
/// the explicit hollow Target Definition; a legacy pattern file is only a
/// migration marker and never determines physical geometry.
fn select_config_source(
    target_config: Option<&str>,
    detector_config: Option<&str>,
    legacy_board_detector_file: Option<&str>,
    legacy_aruco_pattern_file: Option<&str>,
) -> Result<ConfigSource> {
    let target_config = nonempty(target_config);
    let detector_config = nonempty(detector_config);
    let legacy_board_detector_file = nonempty(legacy_board_detector_file);
    let legacy_aruco_pattern_file = nonempty(legacy_aruco_pattern_file);
    let new_any = target_config.is_some() || detector_config.is_some();
    let legacy_any = legacy_board_detector_file.is_some() || legacy_aruco_pattern_file.is_some();
    let new_complete = target_config.is_some() && detector_config.is_some();
    let legacy_complete =
        legacy_board_detector_file.is_some() && legacy_aruco_pattern_file.is_some();

    if new_any && legacy_any {
        return Err(anyhow!(
            "target_config/detector_config and legacy board_detector_file/aruco_pattern_file cannot be mixed; select one source"
        ));
    }
    if new_any {
        if !new_complete {
            return Err(anyhow!(
                "target_config and detector_config must be supplied together"
            ));
        }
        return Ok(ConfigSource {
            kind: ConfigSourceKind::New,
            target_config: Some(
                target_config
                    .ok_or_else(|| anyhow!("target_config is required with detector_config"))?
                    .to_owned(),
            ),
            detector_config: detector_config
                .ok_or_else(|| anyhow!("detector_config is required with target_config"))?
                .to_owned(),
        });
    }
    if legacy_any {
        if !legacy_complete {
            return Err(anyhow!(
                "board_detector_file and aruco_pattern_file must be supplied together"
            ));
        }
        return Ok(ConfigSource {
            kind: ConfigSourceKind::Legacy,
            target_config: None,
            detector_config: legacy_board_detector_file
                .ok_or_else(|| anyhow!("board_detector_file is required with aruco_pattern_file"))?
                .to_owned(),
        });
    }
    Err(anyhow!(
        "target_config and detector_config are required (or the temporary legacy board_detector_file and aruco_pattern_file pair)"
    ))
}

fn target_identity_publisher_options() -> PublisherOptions<'static> {
    let mut options = PublisherOptions::new(TARGET_IDENTITY_TOPIC);
    options.qos = QoSProfile {
        history: QoSHistoryPolicy::KeepLast { depth: 1 },
        ..QoSProfile::default().reliable().transient_local()
    };
    options
}

fn identity_message(identity: &TargetIdentity) -> CalibrationTargetIdentity {
    CalibrationTargetIdentity {
        schema_version: identity.schema_version,
        target_id: identity.target_id.clone(),
        revision: identity.revision,
        semantic_sha256: identity.semantic_sha256.clone(),
        board_frame_convention: identity.board_frame_convention.clone(),
    }
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
    /// bbox_free only: points of the furthest-progressed REJECTED candidate on a
    /// failed frame — the cluster that came closest to passing. Silent in bbox
    /// mode and on a successful detection.
    rejected_cluster: Arc<Publisher<PointCloud2>>,
    plane_inliers: Arc<Publisher<PointCloud2>>,
    downsampled_points: Arc<Publisher<PointCloud2>>,
    plane_marker: Arc<Publisher<MarkerArray>>,
    bbox_marker: Arc<Publisher<MarkerArray>>,
    board_marker: Arc<Publisher<MarkerArray>>,
    #[allow(dead_code)]
    pca_eigenvectors: Arc<Publisher<MarkerArray>>,
}

/// Everything `pointcloud_callback` needs besides the cloud itself.
///
/// All of it is built once at node start and borrowed for the life of the
/// processing thread, so this is a plain bundle of shared references.
#[derive(Clone, Copy)]
struct CallbackContext<'a> {
    target: &'a Arc<ValidatedTarget>,
    estimator: &'a Arc<TargetPoseEstimator>,
    detector_config: &'a DetectorConfig,
    publisher: &'a Publisher<Detection3DArray>,
    bbox_params: &'a Option<BBoxParameters>,
    board_debug_publishers: &'a Option<BoardDebugPublishers>,
    bbox_free_cfg: &'a Option<Arc<bbox_free::BboxFreeRaw>>,
    background_state: &'a Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
}

/// Common evidence handoff for bbox and bbox-free selection.
///
/// Selection owns which returns are admitted; the estimator owns every pose
/// interpretation.  Keeping the fitted observation and the raw selected
/// returns together makes both paths use the same estimator call.
#[derive(Debug, Clone)]
struct SelectedEvidence {
    observation: TargetSquarePlaneObservation,
    points: Vec<na::Point3<f64>>,
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
    // BBox parameters (dynamically reconfigurable via ROS parameters)
    _bbox_params: Option<BBoxParameters>,
    // Processing thread that handles point cloud processing
    _processing_thread: JoinHandle<()>,
    // Shared Method-E background state (kept alive for the processing thread; mutated by
    // reset_background in Task 6)
    _background_state: Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
    // Task 6: reset_background service handle — kept alive so it isn't dropped.
    // Only created (Some) when the bbox_free background-subtraction path is active;
    // `None` in default bbox mode so the service isn't registered unnecessarily.
    _reset_service: Option<rclrs::Service<std_srvs::srv::Empty>>,
    // Latched Target Identity. Held so the publisher outlives `new()` — dropping
    // it would drop the transient-local sample and a late solver would see nothing.
    _target_identity_publisher: Publisher<CalibrationTargetIdentity>,
}

impl CalibrationBoardLocatorNode {
    pub fn new(node: Node) -> Result<Self> {
        // W5-E1 removes the four compatibility parameters. Until then, exactly
        // one pair is accepted: target_config + detector_config, or the old
        // board_detector_file + aruco_pattern_file pair. The latter always means
        // the explicit hollow target below.
        let target_config_param: Option<Arc<str>> =
            node.declare_parameter("target_config").optional()?.get();
        let detector_config_param: Option<Arc<str>> =
            node.declare_parameter("detector_config").optional()?.get();
        let board_detector_file_param: Option<Arc<str>> = node
            .declare_parameter("board_detector_file")
            .optional()?
            .get();
        let aruco_pattern_file_param: Option<Arc<str>> = node
            .declare_parameter("aruco_pattern_file")
            .optional()?
            .get();
        let source = select_config_source(
            target_config_param.as_deref(),
            detector_config_param.as_deref(),
            board_detector_file_param.as_deref(),
            aruco_pattern_file_param.as_deref(),
        )?;
        // Bbox-free mode has no crop-box input.  Keep the parameter optional so
        // a target/detector pair can run without an otherwise meaningless file.
        let bbox_file_param: Option<Arc<str>> =
            node.declare_parameter("bbox_file").optional()?.get();

        let target = Arc::new(match source.kind {
            ConfigSourceKind::New => {
                let path = source
                    .target_config
                    .as_deref()
                    .expect("new source always carries target_config");
                log_info!(LOGGER_NAME, "Loading Target Definition from: {path}");
                Self::load_target(path)?
            }
            ConfigSourceKind::Legacy => {
                log_warn!(
                    LOGGER_NAME,
                    "legacy board_detector_file/aruco_pattern_file is temporary and selects the explicit hollow_1000_aruco_4 target; migrate to target_config/detector_config"
                );
                Self::load_legacy_hollow_target()?
            }
        });

        // Load and declare bbox parameters only when a crop-box file was
        // supplied. Bbox-free mode remains independent of this state.
        let bbox_params = if let Some(file_path) = bbox_file_param.as_deref() {
            log_info!(LOGGER_NAME, "Loading bbox config from: {file_path}");
            let initial_bbox = Self::load_bbox_config(file_path)?;
            let params = BBoxParameters::declare(&node, &initial_bbox)?;
            params.log_values();
            log_info!(
                LOGGER_NAME,
                "Dynamic bbox params available: bbox_center_x, bbox_center_y, bbox_center_z, bbox_rotation_w, bbox_rotation_x, bbox_rotation_y, bbox_rotation_z, bbox_size_x, bbox_size_y, bbox_size_z"
            );
            log_info!(
                LOGGER_NAME,
                "Change at runtime with: ros2 param set /lidar_board_detector bbox_size_x <value>"
            );
            Some(params)
        } else {
            None
        };

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

        // Load Detector Tuning independently from the Target Definition.
        log_info!(
            LOGGER_NAME,
            "Loading Detector Tuning from: {}",
            source.detector_config
        );
        let detector_config = Arc::new(Self::load_detector_config(&source.detector_config)?);
        log_info!(
            LOGGER_NAME,
            "Loaded Detector Tuning: target={}@{}, skip_ransac={}, ransac_threshold={:.3}m, ransac_iterations={}, detection_mode={}",
            target.identity().target_id,
            target.identity().revision,
            detector_config.skip_ransac,
            detector_config.plane_ransac_inlier_threshold,
            detector_config.plane_ransac_max_iterations,
            detector_config.detection_mode.as_str()
        );

        // Log voxel downsampling configuration
        if detector_config.voxel_downsample_enabled {
            log_info!(
                LOGGER_NAME,
                "Voxel downsampling ENABLED: size={:.3}m, use_centroid={}, parallel_threshold={}",
                detector_config.voxel_downsample_size,
                detector_config.voxel_downsample_use_centroid,
                detector_config.voxel_parallel_threshold
            );
        } else {
            log_info!(
                LOGGER_NAME,
                "Voxel downsampling DISABLED (preserving all points for ICP)"
            );
        }

        // Crop-box-free selection uses the same Detector Tuning values, but owns
        // its foreground/background lifecycle in this ROS adapter.
        let detection_mode = detector_config.detection_mode;
        if detection_mode == bbox_free::DetectionMode::Bbox && bbox_params.is_none() {
            return Err(anyhow!(
                "bbox_file is required when detector_config selects detection_mode=bbox"
            ));
        }
        let bbox_free_cfg: Option<Arc<bbox_free::BboxFreeRaw>> = match detection_mode {
            bbox_free::DetectionMode::BboxFree => {
                Some(Arc::new(detector_config.bbox_free_config()))
            }
            bbox_free::DetectionMode::Bbox => None,
        };
        log_info!(LOGGER_NAME, "detection_mode = {}", detection_mode.as_str());

        // Shared background-subtraction state. `BackgroundState` is observed per frame by the single
        // processing thread; `reset_background` (Task 6) mutates it from a service/param callback,
        // so an `Arc<Mutex<Option<..>>>` is sufficient (the design's `ArcSwap<BackgroundState>` is
        // simplified to this — there is only one observer thread). `None` when the bbox_free path
        // is off or uses plane_strip (no background).
        let background_active = matches!(
            bbox_free_cfg.as_ref(),
            Some(bf) if bf.method == board_cluster_detector::config::ForegroundMethod::BackgroundSubtraction
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
                    if let Some(state) = reset_bg.lock().unwrap_or_else(|e| e.into_inner()).as_mut()
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

        let estimator = Arc::new(TargetPoseEstimator::new(
            &target,
            detector_config.estimator_tuning(&target)?,
        )?);

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

        // Announce the complete Target Identity, not a process-global frame
        // convention. Relative routing lets each generated observer have one
        // unambiguous identity topic.
        let target_identity_publisher =
            node.create_publisher(target_identity_publisher_options())?;
        target_identity_publisher.publish(identity_message(target.identity()))?;
        log_info!(
            LOGGER_NAME,
            "Publishing target identity {}@{} (latched on {TARGET_IDENTITY_TOPIC})",
            target.identity().target_id,
            target.identity().revision
        );

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

            let mut rejected_cluster_opts = PublisherOptions::new("debug/rejected_cluster");
            rejected_cluster_opts.qos = debug_qos;

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

            let mut pca_eigenvectors_opts = PublisherOptions::new("debug/pca_eigenvectors");
            pca_eigenvectors_opts.qos = debug_qos;

            Some(BoardDebugPublishers {
                all_points: Arc::new(node.create_publisher(all_points_opts)?),
                filtered_points: Arc::new(node.create_publisher(filtered_points_opts)?),
                foreground_points: Arc::new(node.create_publisher(foreground_points_opts)?),
                background_voxels: Arc::new(node.create_publisher(background_voxels_opts)?),
                rejected_cluster: Arc::new(node.create_publisher(rejected_cluster_opts)?),
                plane_inliers: Arc::new(node.create_publisher(plane_inliers_opts)?),
                downsampled_points: Arc::new(node.create_publisher(downsampled_points_opts)?),
                plane_marker: Arc::new(node.create_publisher(plane_marker_opts)?),
                bbox_marker: Arc::new(node.create_publisher(bbox_marker_opts)?),
                board_marker: Arc::new(node.create_publisher(board_marker_opts)?),
                pca_eigenvectors: Arc::new(node.create_publisher(pca_eigenvectors_opts)?),
            })
        } else {
            None
        };
        let board_debug_shared = board_debug_publishers.clone();

        if enable_icp_iteration_debug {
            log_warn!(
                LOGGER_NAME,
                "enable_icp_iteration_debug is ignored: target pose diagnostics are owned by the neutral estimator"
            );
        }

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
        let target_for_thread = Arc::clone(&target);
        let estimator_for_thread = Arc::clone(&estimator);
        let detector_config_for_thread = Arc::clone(&detector_config);

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
            let ctx = CallbackContext {
                target: &target_for_thread,
                estimator: &estimator_for_thread,
                detector_config: &detector_config_for_thread,
                publisher: &detection_publisher_shared,
                bbox_params: &bbox_params_for_callback,
                board_debug_publishers: &board_debug_shared,
                bbox_free_cfg: &bbox_free_for_thread,
                background_state: &background_for_thread,
            };

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
                        Self::pointcloud_callback(msg_clone, &ctx);
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
                "Debug topics: debug/all_points, debug/filtered_points, debug/foreground_points (bbox_free), debug/background_voxels (bbox_free), debug/rejected_cluster (bbox_free), debug/plane_inliers, debug/plane_marker, debug/bbox_marker, debug/final_board_pose, debug/pca_eigenvectors"
            );
        }

        Ok(Self {
            _node: node,
            _detection_publisher: detection_publisher,
            _pointcloud_subscription: pointcloud_subscription,
            _board_debug_publishers: board_debug_publishers,
            _bbox_params: bbox_params,
            _processing_thread: processing_thread,
            _background_state: background_state,
            _reset_service: reset_service,
            _target_identity_publisher: target_identity_publisher,
        })
    }

    fn load_detector_config(file_path: &str) -> Result<DetectorConfig> {
        if file_path.is_empty() {
            return Err(anyhow!(
                "detector_config (or legacy board_detector_file) was empty"
            ));
        }

        let path = PathBuf::from(file_path);
        Self::load_json5_file(&path)
    }

    fn load_target(file_path: &str) -> Result<ValidatedTarget> {
        if file_path.is_empty() {
            return Err(anyhow!("target_config parameter was empty"));
        }
        let bytes = fs::read(file_path)?;
        ValidatedTarget::parse_json5(&bytes)
    }

    fn load_legacy_hollow_target() -> Result<ValidatedTarget> {
        ValidatedTarget::parse_json5(LEGACY_HOLLOW_TARGET)
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

    fn pointcloud_callback(msg: PointCloud2, ctx: &CallbackContext<'_>) {
        let CallbackContext {
            target,
            estimator,
            detector_config,
            publisher,
            bbox_params,
            board_debug_publishers,
            bbox_free_cfg,
            background_state,
        } = *ctx;

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
            target,
            estimator,
            detector_config,
            bbox_params,
            board_debug_publishers,
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

    #[allow(clippy::too_many_arguments)]
    fn process_pointcloud(
        msg: &PointCloud2,
        target: &Arc<ValidatedTarget>,
        estimator: &Arc<TargetPoseEstimator>,
        detector_config: &DetectorConfig,
        bbox_params: &Option<BBoxParameters>,
        board_debug_publishers: &Option<BoardDebugPublishers>,
        bbox_free_cfg: &Option<Arc<bbox_free::BboxFreeRaw>>,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
    ) -> Result<Detection3DArray> {
        let points = Self::convert_pointcloud2_to_points(msg)?;
        log_debug!(
            LOGGER_NAME,
            "Converted {} points from PointCloud2",
            points.len()
        );

        if let Some(debug_pubs) = board_debug_publishers {
            Self::publish_debug_cloud(&debug_pubs.all_points, &points, &msg.header, "all points");
        }

        // Both selectors now hand the estimator one neutral square/plane
        // observation. The bbox path performs the same known-size square fit
        // explicitly; bbox-free obtains it from detect_for_target.
        let selected = match bbox_free_cfg {
            None => {
                let bbox = bbox_params
                    .as_ref()
                    .ok_or_else(|| anyhow!("bbox_file is required for bbox detection mode"))?;
                let active_points = Self::filter_points_by_bbox(
                    &points,
                    bbox,
                    &msg.header,
                    board_debug_publishers,
                )?;
                Self::select_bbox_evidence(
                    &active_points,
                    target,
                    detector_config,
                    &msg.header,
                    board_debug_publishers,
                )?
            }
            Some(bf) => Self::select_board_cluster(
                &points,
                target,
                bf,
                background_state,
                detector_config.sensor_up_axis.as_vector(),
                &msg.header,
                board_debug_publishers,
            )?,
        };
        let Some(selected) = selected else {
            return Ok(Detection3DArray {
                header: msg.header.clone(),
                detections: Vec::new(),
            });
        };

        let evidence_points = if detector_config.voxel_downsample_enabled {
            let downsampled = Self::voxel_downsample(
                &selected.points,
                detector_config.voxel_downsample_size,
                detector_config.voxel_downsample_use_centroid,
            );
            if let Some(debug_pubs) = board_debug_publishers {
                Self::publish_debug_cloud(
                    &debug_pubs.downsampled_points,
                    &downsampled,
                    &msg.header,
                    "downsampled points",
                );
            }
            downsampled
        } else {
            selected.points
        };

        if evidence_points.is_empty() {
            return Ok(Detection3DArray {
                header: msg.header.clone(),
                detections: Vec::new(),
            });
        }

        let estimate = estimator.estimate(selected.observation, evidence_points.clone());
        let detection = match estimate {
            TargetPoseEstimate::Detected(detection) => {
                log_info!(
                    LOGGER_NAME,
                    "Target detection successful: target={}@{}, pose=({:.6}, {:.6}, {:.6})",
                    detection.target_identity.target_id,
                    detection.target_identity.revision,
                    detection.pose.translation.x,
                    detection.pose.translation.y,
                    detection.pose.translation.z
                );
                if let Some(debug_pubs) = board_debug_publishers {
                    let markers =
                        Self::create_target_markers(target, detection.pose, &msg.header, "", 0)?;
                    debug_pubs.board_marker.publish(markers)?;
                }
                Some(Self::convert_target_detection_to_detection3d(
                    target,
                    &detection,
                    &evidence_points,
                    &msg.header,
                )?)
            }
            TargetPoseEstimate::Rejected(rejection) => {
                Self::log_rejection(&rejection.reason, &rejection.target_identity);
                if let Some(debug_pubs) = board_debug_publishers {
                    debug_pubs.board_marker.publish(MarkerArray::default())?;
                }
                None
            }
        };

        Ok(Detection3DArray {
            header: msg.header.clone(),
            detections: detection.into_iter().collect(),
        })
    }

    /// Crop-box-free Stage 1: return one target-neutral square/plane
    /// observation, or `None` while warming/no candidate was selected.
    fn select_board_cluster(
        points: &[na::Point3<f64>],
        target: &ValidatedTarget,
        bf: &bbox_free::BboxFreeRaw,
        background_state: &Arc<std::sync::Mutex<Option<bbox_free::BackgroundState>>>,
        sensor_up: na::Vector3<f64>,
        header: &Header,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Result<Option<SelectedEvidence>> {
        let method = bf.method;
        let target_side = TargetSide::metres(target.plate().side_um as f64 / 1_000_000.0)?;

        // `detect_for_target` must return neutral square/plane evidence.  A
        // stance threshold is a legacy pose gate and belongs to
        // `TargetPoseEstimator`, where the selected target's board-up
        // convention is available; it must not make bbox-free selection differ
        // from bbox selection.
        let neutral_tuning = neutral_detector_tuning(&bf.board);

        // Cell centers of the finalized background model, captured under the lock
        // for debug publishing once the guard is released. None for plane_strip.
        let mut background_centers: Option<Vec<na::Point3<f64>>> = None;

        // Method E: run the warmup lifecycle to obtain a finalized background.
        let outcome = if method == ForegroundMethod::BackgroundSubtraction {
            let mut guard = background_state.lock().unwrap_or_else(|e| e.into_inner());
            let state = guard.as_mut().ok_or_else(|| {
                anyhow!(
                    "bbox_free background_subtraction selected but no BackgroundState initialized"
                )
            })?;
            match state.observe_frame(points) {
                bbox_free::WarmupOutcome::Warming { seen, needed } => {
                    log_info!(LOGGER_NAME, "background warmup {seen}/{needed}");
                    return Ok(None);
                }
                bbox_free::WarmupOutcome::Ready => {
                    let model = state.model().expect("Ready implies a finalized model");
                    background_centers = Some(model.voxel_centers());
                    detect_for_target(
                        points,
                        target_side,
                        &neutral_tuning,
                        method,
                        bf.voxel,
                        Some(model),
                    )
                }
            }
        } else {
            // Method B (plane_strip): no background.
            detect_for_target(points, target_side, &neutral_tuning, method, bf.voxel, None)
        };

        // Publish the finalized background voxel centers so the "static scene"
        // baked in during warmup is visible in RViz.
        if let (Some(debug_pubs), Some(centers)) = (board_debug_publishers, &background_centers) {
            Self::publish_debug_cloud(
                &debug_pubs.background_voxels,
                centers,
                header,
                "background voxels",
            );
        }

        if let Some(debug_pubs) = board_debug_publishers {
            Self::publish_debug_cloud(
                &debug_pubs.foreground_points,
                &outcome.foreground_points,
                header,
                "foreground points",
            );
            Self::publish_debug_cloud(
                &debug_pubs.rejected_cluster,
                &outcome.rejected_cluster,
                header,
                "rejected cluster",
            );
        }

        let Some(square_plane) = outcome.observation else {
            match &outcome.reject {
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
            return Ok(None);
        };

        if square_plane.points.is_empty() {
            return Ok(None);
        }
        let observation =
            TargetSquarePlaneObservation::from_square_plane(&square_plane, sensor_up)?;
        if let Some(debug_pubs) = board_debug_publishers {
            Self::publish_debug_cloud(
                &debug_pubs.plane_inliers,
                &square_plane.points,
                header,
                "plane inliers",
            );
        }
        Ok(Some(SelectedEvidence {
            points: square_plane.points.clone(),
            observation,
        }))
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

    /// Bbox-selected Stage 1 after the crop-box selector.  Bbox filtering is
    /// only a selector; this adapter must still produce the same fixed-size
    /// square/plane observation as the bbox-free detector before entering the
    /// estimator seam.  Keeping this pure over already-selected points also
    /// makes the legacy hollow point-cloud handoff regression-testable without
    /// constructing a ROS node just to hold dynamic bbox parameters.
    fn select_bbox_evidence(
        active_points: &[na::Point3<f64>],
        target: &ValidatedTarget,
        detector_config: &DetectorConfig,
        header: &Header,
        board_debug_publishers: &Option<BoardDebugPublishers>,
    ) -> Result<Option<SelectedEvidence>> {
        if active_points.len() < 3 {
            log_info!(
                LOGGER_NAME,
                "bbox: no board selected — only {} finite points in the configured box",
                active_points.len()
            );
            return Ok(None);
        }

        // A frame without a valid plane is a normal selector miss, not a
        // callback failure.  The callback must publish its stable empty array
        // for that frame, preserving the old observer behavior.
        let (plane, plane_points) = match Self::fit_bbox_plane(active_points, detector_config) {
            Ok(result) => result,
            Err(error) => {
                log_info!(
                    LOGGER_NAME,
                    "bbox: no board selected — plane fit failed: {error}"
                );
                return Ok(None);
            }
        };
        if let Some(debug_pubs) = board_debug_publishers {
            Self::publish_debug_cloud(
                &debug_pubs.plane_inliers,
                &plane_points,
                header,
                "plane inliers",
            );
            if let Ok(markers) = Self::create_plane_marker(&plane, &plane_points, header) {
                let _ = debug_pubs.plane_marker.publish(markers);
            }
        }

        let target_side = detector_config.target_side(target)?.as_metres();
        let coords = project_to_plane(&plane_points, &plane);
        let Some(square_fit) = fit_fixed_square(&coords, target_side, None, None) else {
            log_info!(
                LOGGER_NAME,
                "bbox: no board selected — fixed square fit requires at least 20 points, got {}",
                coords.len()
            );
            return Ok(None);
        };
        if square_fit.residual > detector_config.tuning.square_icp_residual_max {
            log_info!(
                LOGGER_NAME,
                "bbox: no board selected — square residual {:.4} exceeds {:.4}",
                square_fit.residual,
                detector_config.tuning.square_icp_residual_max
            );
            return Ok(None);
        }

        let observation = TargetSquarePlaneObservation::from_fitted_square(
            &plane,
            &square_fit,
            detector_config.sensor_up_axis.as_vector(),
        )?;
        Ok(Some(SelectedEvidence {
            observation,
            points: plane_points,
        }))
    }

    /// Deterministic voxel reduction for estimator evidence.  The board-cluster
    /// crate owns the bbox-free selector's voxel stage; bbox mode uses this
    /// small adapter so both paths apply the same deployment tuning.
    fn voxel_downsample(
        points: &[na::Point3<f64>],
        voxel: f64,
        use_centroid: bool,
    ) -> Vec<na::Point3<f64>> {
        if !voxel.is_finite() || voxel <= 0.0 {
            return points.to_vec();
        }
        type VoxelCell = (na::Vector3<f64>, usize, na::Point3<f64>);
        let mut cells: std::collections::BTreeMap<(i64, i64, i64), VoxelCell> =
            std::collections::BTreeMap::new();
        for point in points {
            let key = (
                (point.x / voxel).floor() as i64,
                (point.y / voxel).floor() as i64,
                (point.z / voxel).floor() as i64,
            );
            cells
                .entry(key)
                .and_modify(|(sum, count, _first)| {
                    *sum += point.coords;
                    *count += 1;
                })
                .or_insert((point.coords, 1, *point));
        }
        cells
            .into_values()
            .map(|(sum, count, first)| {
                if use_centroid {
                    na::Point3::from(sum / count as f64)
                } else {
                    first
                }
            })
            .collect()
    }

    fn log_rejection(reason: &TargetRejectReason, identity: &TargetIdentity) {
        match reason {
            TargetRejectReason::BoardUpAlignment {
                alignment,
                required_minimum,
            } => log_info!(
                LOGGER_NAME,
                "target rejected: target={}@{} reason=board_up_alignment alignment={:.4} required_minimum={:.4}",
                identity.target_id,
                identity.revision,
                alignment,
                required_minimum
            ),
            TargetRejectReason::InsufficientOuterEdgeEvidence { evidence } => log_info!(
                LOGGER_NAME,
                "target rejected: target={}@{} reason=insufficient_outer_edge_evidence edge_points={} edge_counts={:?} covered_edges={} required_edges={} occupied_bins={:?}",
                identity.target_id,
                identity.revision,
                evidence.edge_point_count,
                evidence.edge_point_counts,
                evidence.covered_edge_count,
                evidence.minimum_covered_edges,
                evidence.occupied_longitudinal_bins
            ),
            TargetRejectReason::AmbiguousCutoutEvidence {
                evidence,
                required_separation_m,
            } => log_info!(
                LOGGER_NAME,
                "target rejected: target={}@{} reason=ambiguous_cutout_evidence best_loss_m={:.6} second_best_loss_m={:.6} separation_m={:.6} required_separation_m={:.6}",
                identity.target_id,
                identity.revision,
                evidence.best_loss_m,
                evidence.second_best_loss_m,
                evidence.loss_separation_m,
                required_separation_m
            ),
            TargetRejectReason::WeakCutoutEvidence {
                evidence,
                required_rim_correspondences,
            } => log_info!(
                LOGGER_NAME,
                "target rejected: target={}@{} reason=weak_cutout_evidence rim_correspondences={} required_rim_correspondences={} best_loss_m={:.6}",
                identity.target_id,
                identity.revision,
                evidence.cutout_rim_correspondences,
                required_rim_correspondences,
                evidence.best_loss_m
            ),
            TargetRejectReason::PerforatedIcpFailure { evidence } => log_info!(
                LOGGER_NAME,
                "target rejected: target={}@{} reason=perforated_icp_failure rim_correspondences={} iterations={} total_correspondences={} best_loss_m={:.6}",
                identity.target_id,
                identity.revision,
                evidence.cutout_rim_correspondences,
                evidence.iteration_count,
                evidence.total_correspondences,
                evidence.best_loss_m
            ),
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
    /// Preserve the legacy bbox contract: callers may opt into RANSAC and its
    /// configured inlier threshold, otherwise all bbox-selected points are
    /// retained for the shared plane fit.
    fn fit_bbox_plane(
        points: &[na::Point3<f64>],
        detector_config: &DetectorConfig,
    ) -> Result<(PlaneModel, Vec<na::Point3<f64>>)> {
        if detector_config.skip_ransac {
            return Ok((Self::compute_plane_from_points(points)?, points.to_vec()));
        }
        Self::fit_plane_ransac(
            points,
            detector_config.plane_ransac_max_iterations,
            detector_config.plane_ransac_inlier_threshold,
        )
    }

    /// Deterministic three-point RANSAC for the bbox adapter.  The previous
    /// observer used a plane RANSAC before its hollow ICP stage; retaining that
    /// boundary avoids letting crop-box clutter bias the common square fit.
    fn fit_plane_ransac(
        points: &[na::Point3<f64>],
        max_iterations: usize,
        inlier_threshold: f64,
    ) -> Result<(PlaneModel, Vec<na::Point3<f64>>)> {
        if points.len() < 3 {
            return Err(anyhow!(
                "RANSAC needs at least 3 points, got {}",
                points.len()
            ));
        }
        if max_iterations == 0 {
            return Err(anyhow!(
                "plane_ransac_max_iterations must be greater than zero"
            ));
        }
        if !inlier_threshold.is_finite() || inlier_threshold <= 0.0 {
            return Err(anyhow!(
                "plane_ransac_inlier_threshold must be finite and greater than zero"
            ));
        }

        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        let mut best_indices: Vec<usize> = Vec::new();
        let mut best_error = f64::INFINITY;
        for _ in 0..max_iterations {
            let next_index = |state: &mut u64| {
                // xorshift64* keeps this adapter deterministic across runs and
                // avoids making a global RNG part of the ROS callback state.
                *state ^= *state >> 12;
                *state ^= *state << 25;
                *state ^= *state >> 27;
                ((*state).wrapping_mul(0x2545_f491_4f6c_dd1d) as usize) % points.len()
            };
            let a = next_index(&mut state);
            let mut b = next_index(&mut state);
            let mut c = next_index(&mut state);
            if b == a {
                b = (b + 1) % points.len();
            }
            if c == a || c == b {
                c = (c + 1) % points.len();
                if c == a || c == b {
                    c = (c + 1) % points.len();
                }
            }

            let ab = points[b] - points[a];
            let ac = points[c] - points[a];
            let cross = ab.cross(&ac);
            let norm = cross.norm();
            if !norm.is_finite() || norm <= 1e-12 {
                continue;
            }
            let normal = cross / norm;
            let origin = points[a];
            let mut indices = Vec::new();
            let mut error = 0.0;
            for (index, point) in points.iter().enumerate() {
                let residual = (point - origin).dot(&normal).abs();
                if residual <= inlier_threshold {
                    indices.push(index);
                    error += residual;
                }
            }
            if indices.len() > best_indices.len()
                || (indices.len() == best_indices.len() && error < best_error)
            {
                best_indices = indices;
                best_error = error;
            }
        }

        if best_indices.len() < 3 {
            return Err(anyhow!(
                "RANSAC failed to find a plane with three inliers (threshold={inlier_threshold:.6}m)"
            ));
        }
        let inliers: Vec<_> = best_indices
            .into_iter()
            .map(|index| points[index])
            .collect();
        Ok((Self::compute_plane_from_points(&inliers)?, inliers))
    }

    /// Fit the common board-cluster plane representation for bbox evidence.
    fn compute_plane_from_points(points: &[na::Point3<f64>]) -> Result<PlaneModel> {
        if points.len() < 3 {
            return Err(anyhow!(
                "Need at least 3 points to compute plane, got {}",
                points.len()
            ));
        }
        Ok(board_cluster_detector::geometry::fit_plane(points))
    }

    fn publish_debug_cloud(
        publisher: &Publisher<PointCloud2>,
        points: &[na::Point3<f64>],
        header: &std_msgs::msg::Header,
        what: &str,
    ) {
        match Self::create_debug_pointcloud(points, header) {
            Ok(cloud) => {
                if let Err(e) = publisher.publish(cloud) {
                    log_warn!(LOGGER_NAME, "Failed to publish {what}: {e}");
                }
            }
            Err(e) => log_warn!(LOGGER_NAME, "Failed to build {what} cloud: {e}"),
        }
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

    fn convert_target_detection_to_detection3d(
        target: &ValidatedTarget,
        detection: &TargetDetection,
        evidence_points: &[na::Point3<f64>],
        header: &std_msgs::msg::Header,
    ) -> Result<Detection3D> {
        let pose = Pose {
            position: Point {
                x: detection.pose.translation.x,
                y: detection.pose.translation.y,
                z: detection.pose.translation.z,
            },
            orientation: Quaternion {
                x: detection.pose.rotation.i,
                y: detection.pose.rotation.j,
                z: detection.pose.rotation.k,
                w: detection.pose.rotation.w,
            },
        };

        // Detection3D's oriented box follows the target pose and uses the
        // physical plate side from the selected Target Definition.  The old
        // observer hard-coded 1 m for every target.
        let side_m = target.plate().side_um as f64 / 1_000_000.0;
        let bbox = BoundingBox3D {
            center: pose.clone(),
            size: GeomVector3 {
                x: side_m,
                y: side_m,
                // Target Definition describes the face, not its backing
                // thickness; keep a small finite depth for RViz consumers.
                z: 0.01,
            },
        };

        let posed = target.posed(detection.pose);
        let correspondences: Vec<(na::Point3<f64>, na::Point3<f64>)> = posed
            .closest_points(evidence_points.iter())
            .into_iter()
            .map(|correspondence| (*correspondence.input, correspondence.closest))
            .collect();
        let pose_origin = na::Point3::from(detection.pose.translation.vector);
        let covariance = Self::compute_pose_covariance(
            &correspondences,
            &pose_origin,
            &posed.z_axis().into_inner(),
        );

        let score = match &detection.diagnostics {
            TargetDetectionDiagnostics::Solid(evidence) => {
                (evidence.covered_edge_count as f64 / 4.0).clamp(0.0, 1.0)
            }
            TargetDetectionDiagnostics::CutoutIcp(evidence) => {
                if evidence.best_loss_m.is_finite() && evidence.best_loss_m >= 0.0 {
                    (1.0 / (1.0 + evidence.best_loss_m / 0.035)).clamp(0.0, 1.0)
                } else {
                    0.0
                }
            }
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
            id: target.identity().target_id.clone(),
        })
    }

    fn create_bbox_marker(bbox: &BBox, header: &Header) -> Result<Marker> {
        let q = bbox.pose.rotation.quaternion();
        Ok(Marker {
            header: header.clone(),
            ns: "bbox".to_string(),
            id: 0,
            type_: 1, // CUBE
            action: 0,
            pose: Pose {
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
        })
    }

    /// Create target-sized RViz markers from the neutral estimator pose.
    ///
    /// Points are transformed exactly once into the cloud frame and the marker
    /// pose is identity.  A solid Target Definition therefore cannot acquire
    /// hollow-board cutout markers by accident.
    fn create_target_markers(
        target: &ValidatedTarget,
        pose: na::Isometry3<f64>,
        header: &Header,
        namespace_suffix: &str,
        id_offset: i32,
    ) -> Result<MarkerArray> {
        let posed = target.posed(pose);
        let as_point = |point: na::Point3<f64>| Point {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let outline = [
            posed.top_corner(),
            posed.left_corner(),
            posed.bottom_corner(),
            posed.right_corner(),
            posed.top_corner(),
        ];
        let mut markers = vec![Marker {
            header: header.clone(),
            ns: format!("target_plate{namespace_suffix}"),
            id: id_offset,
            type_: 4, // LINE_STRIP
            action: 0,
            points: outline.into_iter().map(as_point).collect(),
            scale: GeomVector3 {
                x: target.plate().side_um as f64 / 1_000_000.0 * 0.015,
                y: 0.0,
                z: 0.0,
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.9,
            },
            ..Default::default()
        }];

        let axis_len = target.plate().side_um as f64 / 2_000_000.0;
        for (axis_id, axis, (r, g, b)) in [
            (1, na::Vector3::x(), (1.0, 0.0, 0.0)),
            (2, na::Vector3::y(), (0.0, 1.0, 0.0)),
            (3, na::Vector3::z(), (0.0, 0.0, 1.0)),
        ] {
            let world_axis = pose.rotation * axis;
            let axis_rotation =
                na::UnitQuaternion::rotation_between(&na::Vector3::x(), &world_axis)
                    .unwrap_or_else(na::UnitQuaternion::identity);
            let axis_q = axis_rotation.quaternion();
            markers.push(Marker {
                header: header.clone(),
                ns: format!("target_axes{namespace_suffix}"),
                id: id_offset + axis_id,
                type_: 0, // ARROW; local +X follows target pose rotation.
                action: 0,
                pose: Pose {
                    position: as_point(posed.center()),
                    orientation: Quaternion {
                        x: axis_q.i,
                        y: axis_q.j,
                        z: axis_q.k,
                        w: axis_q.w,
                    },
                },
                scale: GeomVector3 {
                    x: axis_len,
                    y: 0.02,
                    z: 0.04,
                },
                color: ColorRGBA { r, g, b, a: 1.0 },
                ..Default::default()
            });
        }

        for (index, (_marker_id, corners)) in posed.marker_corners_by_id().into_iter().enumerate() {
            let mut points = Vec::with_capacity(8);
            for edge in 0..4 {
                points.push(as_point(corners[edge]));
                points.push(as_point(corners[(edge + 1) % 4]));
            }
            markers.push(Marker {
                header: header.clone(),
                ns: format!("target_fiducials{namespace_suffix}"),
                id: id_offset + 10 + index as i32,
                type_: 5, // LINE_LIST
                action: 0,
                points,
                scale: GeomVector3 {
                    x: 0.008,
                    y: 0.0,
                    z: 0.0,
                },
                color: ColorRGBA {
                    r: 1.0,
                    g: 0.7,
                    b: 0.0,
                    a: 1.0,
                },
                ..Default::default()
            });
        }

        if let Surface::Perforated { circular_cutouts } = &target.plate().surface {
            let q = pose.rotation.quaternion();
            for (index, cutout) in circular_cutouts.iter().enumerate() {
                let center = pose.transform_point(&na::Point3::new(
                    cutout.x_um as f64 / 1_000_000.0,
                    cutout.y_um as f64 / 1_000_000.0,
                    0.0,
                ));
                markers.push(Marker {
                    header: header.clone(),
                    ns: format!("target_cutouts{namespace_suffix}"),
                    id: id_offset + 100 + index as i32,
                    type_: 3, // CYLINDER
                    action: 0,
                    pose: Pose {
                        position: as_point(center),
                        orientation: Quaternion {
                            x: q.i,
                            y: q.j,
                            z: q.k,
                            w: q.w,
                        },
                    },
                    scale: GeomVector3 {
                        x: 2.0 * cutout.radius_um as f64 / 1_000_000.0,
                        y: 2.0 * cutout.radius_um as f64 / 1_000_000.0,
                        z: 0.005,
                    },
                    color: ColorRGBA {
                        r: 0.3,
                        g: 0.3,
                        b: 0.3,
                        a: 0.8,
                    },
                    ..Default::default()
                });
            }
        }

        Ok(MarkerArray { markers })
    }

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
}

fn main() -> Result<()> {
    env_logger::Builder::from_default_env()
        .format_timestamp_millis()
        .init();

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

    /// (model↔measured point pairs, board origin, board normal).
    type Correspondences = (
        Vec<(na::Point3<f64>, na::Point3<f64>)>,
        na::Point3<f64>,
        na::Vector3<f64>,
    );

    /// Build correspondences for a board lying in the world XY plane (normal = +Z), origin at c.
    ///
    /// `interior` points get a residual purely along the normal — that is what the correspondence
    /// model produces for a point whose plane projection lands inside the square and outside every
    /// hole. `in_plane` points get a residual in the plane, as a border-clamped or hole-rim point
    /// does.
    fn make_correspondences(n_interior: usize, n_in_plane: usize) -> Correspondences {
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
        assert_eq!(cfg.detection_mode, crate::bbox_free::DetectionMode::Bbox);
    }

    #[test]
    fn config_source_requires_exactly_one_complete_pair() {
        let new_source =
            select_config_source(Some("target.json5"), Some("tuning.json5"), None, None).unwrap();
        assert_eq!(new_source.kind, ConfigSourceKind::New);
        assert_eq!(new_source.target_config.as_deref(), Some("target.json5"));
        assert_eq!(new_source.detector_config, "tuning.json5");

        let legacy_source =
            select_config_source(None, None, Some("board.json5"), Some("aruco.json5")).unwrap();
        assert_eq!(legacy_source.kind, ConfigSourceKind::Legacy);
        assert_eq!(legacy_source.target_config, None);
        assert_eq!(legacy_source.detector_config, "board.json5");

        for args in [
            (Some("target.json5"), None, None, None),
            (None, Some("tuning.json5"), None, None),
            (None, None, Some("board.json5"), None),
            (None, None, None, Some("aruco.json5")),
            (
                Some("target.json5"),
                Some("tuning.json5"),
                Some("board.json5"),
                Some("aruco.json5"),
            ),
        ] {
            assert!(
                select_config_source(args.0, args.1, args.2, args.3).is_err(),
                "{args:?}"
            );
        }
    }

    #[test]
    fn neutral_selector_tuning_does_not_apply_pose_stance_gate() {
        let tuning: DetectorTuning =
            json5::from_str(r#"{ "stance_floor": 0.9, "up_axis": [1.0, 0.0, 0.0] }"#).unwrap();
        let neutral = neutral_detector_tuning(&tuning);
        assert_eq!(neutral.stance_floor, 0.0);
        assert_eq!(neutral.up_axis, tuning.up_axis);
    }

    #[test]
    fn solid_estimator_tuning_must_be_explicit_in_detector_config() {
        let target = ValidatedTarget::parse_json5(include_bytes!(
            "../../lctk_launch/config/targets/solid_600_aruco_1_v1.json5"
        ))
        .unwrap();
        let missing: DetectorConfig = json5::from_str("{}").unwrap();
        let error = missing.estimator_tuning(&target).unwrap_err().to_string();
        assert!(error.contains("solid_edge_band_m"));

        let explicit: DetectorConfig = json5::from_str(
            r#"{
                solid_edge_band_m: 0.015,
                solid_minimum_edge_points: 8,
                solid_minimum_points_per_covered_edge: 1,
                solid_minimum_covered_edges: 3,
                solid_longitudinal_bins_per_edge: 4,
                solid_minimum_occupied_longitudinal_bins: 2
            }"#,
        )
        .unwrap();
        assert!(explicit.estimator_tuning(&target).is_ok());
    }

    #[test]
    fn bbox_ransac_preserves_plane_inliers_and_rejects_clutter() {
        let mut points = Vec::new();
        for i in 0..120 {
            let x = -0.5 + (i % 12) as f64 * 0.08;
            let y = -0.5 + (i / 12) as f64 * 0.08;
            points.push(na::Point3::new(x, y, 2.0));
        }
        for i in 0..20 {
            points.push(na::Point3::new(i as f64, -i as f64, 4.0));
        }
        let (plane, inliers) =
            CalibrationBoardLocatorNode::fit_plane_ransac(&points, 500, 0.01).unwrap();
        assert!(
            inliers.len() >= 110,
            "expected most plane points, got {}",
            inliers.len()
        );
        assert!(inliers.iter().all(|point| (point.z - 2.0).abs() <= 1e-12));
        assert!(plane.center.z.is_finite());
        assert!(plane.normal.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn identity_publisher_is_relative_reliable_latched_depth_one() {
        let options = target_identity_publisher_options();
        assert_eq!(options.topic, TARGET_IDENTITY_TOPIC);
        assert!(!options.topic.starts_with('/'));
        assert_eq!(options.qos.history, QoSHistoryPolicy::KeepLast { depth: 1 });
        assert_eq!(options.qos.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(options.qos.durability, QoSDurabilityPolicy::TransientLocal);
    }

    #[test]
    fn bbox_and_bbox_free_adapters_have_identical_observation_semantics() {
        let plane = PlaneModel {
            center: na::Point3::new(0.0, 0.0, 2.0),
            normal: na::Vector3::z(),
            u: na::Vector3::x(),
            v: na::Vector3::y(),
        };
        let square_fit = board_cluster_detector::square_fit::SquareFit {
            center: [0.0, 0.0],
            theta: 0.0,
            residual: 0.01,
            corners_2d: [[0.3, 0.3], [-0.3, 0.3], [-0.3, -0.3], [0.3, -0.3]],
        };
        let bbox_observation =
            TargetSquarePlaneObservation::from_fitted_square(&plane, &square_fit, na::Vector3::z())
                .unwrap();
        let square_plane = SquarePlaneObservation {
            points: vec![na::Point3::new(0.0, 0.0, 2.0)],
            plane,
            square_fit,
        };
        let bbox_free_observation =
            TargetSquarePlaneObservation::from_square_plane(&square_plane, na::Vector3::z())
                .unwrap();

        assert_eq!(bbox_observation.center, bbox_free_observation.center);
        assert_eq!(
            bbox_observation.fitted_corners,
            bbox_free_observation.fitted_corners
        );
        assert_eq!(
            bbox_observation.orientation,
            bbox_free_observation.orientation
        );
        assert_eq!(
            bbox_observation.sensor_facing_normal,
            bbox_free_observation.sensor_facing_normal
        );
    }

    #[test]
    fn solid_target_output_uses_target_size_and_no_cutout_markers() {
        let target = ValidatedTarget::parse_json5(include_bytes!(
            "../../lctk_launch/config/targets/solid_600_aruco_1_v1.json5"
        ))
        .unwrap();
        let pose = na::Isometry3::translation(0.0, 0.0, 2.0);
        let header = Header::default();
        let markers =
            CalibrationBoardLocatorNode::create_target_markers(&target, pose, &header, "", 0)
                .unwrap();
        assert!(markers
            .markers
            .iter()
            .all(|marker| !marker.ns.contains("cutout")));

        let detection = TargetDetection {
            pose,
            target_identity: target.identity().clone(),
            selected_quadrant: 0,
            diagnostics: TargetDetectionDiagnostics::Solid(EdgeCoverageEvidence {
                edge_point_count: 32,
                edge_point_counts: [8; 4],
                covered_edge_count: 4,
                occupied_longitudinal_bins: [2; 4],
                weak_in_plane_center: false,
                weak_yaw: false,
                board_up_alignment: 1.0,
                edge_band_m: 0.02,
                minimum_edge_points: 8,
                minimum_points_per_covered_edge: 1,
                minimum_covered_edges: 3,
                longitudinal_bins_per_edge: 4,
                minimum_occupied_longitudinal_bins: 2,
            }),
        };
        let ros_detection = CalibrationBoardLocatorNode::convert_target_detection_to_detection3d(
            &target,
            &detection,
            &[],
            &header,
        )
        .unwrap();
        assert!((ros_detection.bbox.size.x - 0.6).abs() < 1e-12);
        assert!((ros_detection.bbox.size.y - 0.6).abs() < 1e-12);
        assert_eq!(ros_detection.id, "solid_600_aruco_1");
    }

    #[test]
    fn legacy_hollow_source_keeps_identity_and_one_metre_output_geometry() {
        let source = select_config_source(
            None,
            None,
            Some("legacy-board.json5"),
            Some("legacy-aruco.json5"),
        )
        .unwrap();
        assert_eq!(source.kind, ConfigSourceKind::Legacy);

        let target = CalibrationBoardLocatorNode::load_legacy_hollow_target().unwrap();
        assert_eq!(target.identity().target_id, "hollow_1000_aruco_4");
        assert_eq!(target.identity().revision, 1);
        assert_eq!(target.plate().side_um, 1_000_000);
        assert_eq!(target.identity().semantic_sha256.len(), 64);
        assert!(target
            .identity()
            .semantic_sha256
            .chars()
            .all(|character| character.is_ascii_hexdigit() && !character.is_ascii_uppercase()));

        let pose = na::Isometry3::translation(0.0, 0.0, 2.0);
        let detection = TargetDetection {
            pose,
            target_identity: target.identity().clone(),
            selected_quadrant: 0,
            diagnostics: TargetDetectionDiagnostics::CutoutIcp(CutoutIcpEvidence {
                best_loss_m: 0.001,
                second_best_loss_m: 0.01,
                loss_separation_m: 0.009,
                cutout_rim_correspondences: 12,
                iteration_count: 3,
                total_correspondences: 20,
                termination: IcpTermination::GoodFit,
            }),
        };
        let ros_detection = CalibrationBoardLocatorNode::convert_target_detection_to_detection3d(
            &target,
            &detection,
            &[],
            &Header::default(),
        )
        .unwrap();
        assert!((ros_detection.bbox.size.x - 1.0).abs() < 1e-12);
        assert!((ros_detection.bbox.size.y - 1.0).abs() < 1e-12);
        assert_eq!(ros_detection.id, "hollow_1000_aruco_4");
    }

    #[test]
    fn legacy_hollow_point_cloud_runs_bbox_adapter_and_estimator() {
        let source = select_config_source(
            None,
            None,
            Some("legacy-board.json5"),
            Some("legacy-aruco.json5"),
        )
        .unwrap();
        let target = CalibrationBoardLocatorNode::load_legacy_hollow_target().unwrap();
        assert_eq!(source.kind, ConfigSourceKind::Legacy);

        // This is the representative hollow sample used by the perforated
        // estimator facade: a diamond plate surface plus every cutout rim,
        // translated into the sensor frame at z=2 m.  Keeping it as a point
        // cloud exercises the observer adapter's plane/square handoff rather
        // than fabricating a TargetDetection for the regression.
        let Surface::Perforated { circular_cutouts } = &target.plate().surface else {
            panic!("legacy compatibility target must be perforated");
        };
        let mut points = Vec::new();
        for xi in -16..=16 {
            for yi in -16..=16 {
                let x = xi as f64 * 0.04;
                let y = yi as f64 * 0.04;
                if x.abs() + y.abs() > target.half_diagonal_m() - 0.01 {
                    continue;
                }
                if circular_cutouts.iter().any(|cutout| {
                    let dx = x - cutout.x_um as f64 / 1_000_000.0;
                    let dy = y - cutout.y_um as f64 / 1_000_000.0;
                    dx.hypot(dy) < cutout.radius_um as f64 / 1_000_000.0 + 0.002
                }) {
                    continue;
                }
                points.push(na::Point3::new(x, y, 2.0));
            }
        }
        for cutout in circular_cutouts {
            let center_x = cutout.x_um as f64 / 1_000_000.0;
            let center_y = cutout.y_um as f64 / 1_000_000.0;
            let radius = cutout.radius_um as f64 / 1_000_000.0;
            for sample in 0..32 {
                let angle = sample as f64 * std::f64::consts::TAU / 32.0;
                points.push(na::Point3::new(
                    center_x + radius * angle.cos(),
                    center_y + radius * angle.sin(),
                    2.0,
                ));
            }
        }

        let detector_config: DetectorConfig = json5::from_str(include_str!(
            "../../lctk_launch/config/board/board_detector.json5"
        ))
        .unwrap();
        let selected = CalibrationBoardLocatorNode::select_bbox_evidence(
            &points,
            &target,
            &detector_config,
            &Header::default(),
            &None,
        )
        .unwrap()
        .expect("representative hollow cloud should produce bbox evidence");
        assert!((selected.observation.center.z - 2.0).abs() < 1e-9);

        let evidence_points = selected.points.clone();
        let estimator =
            TargetPoseEstimator::new(&target, detector_config.estimator_tuning(&target).unwrap())
                .unwrap();
        let TargetPoseEstimate::Detected(detection) =
            estimator.estimate(selected.observation, selected.points)
        else {
            panic!("representative hollow cloud should be detected");
        };
        assert_eq!(detection.target_identity.target_id, "hollow_1000_aruco_4");
        assert!(matches!(
            detection.diagnostics,
            TargetDetectionDiagnostics::CutoutIcp(_)
        ));
        let ros_detection = CalibrationBoardLocatorNode::convert_target_detection_to_detection3d(
            &target,
            &detection,
            &evidence_points,
            &Header::default(),
        )
        .unwrap();
        assert!((ros_detection.bbox.size.x - 1.0).abs() < 1e-12);
        assert!((ros_detection.bbox.size.y - 1.0).abs() < 1e-12);
        assert_eq!(ros_detection.id, "hollow_1000_aruco_4");
    }

    #[test]
    fn bbox_ransac_plane_miss_is_an_empty_selection() {
        let target = ValidatedTarget::parse_json5(include_bytes!(
            "../../lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5"
        ))
        .unwrap();
        let detector_config: DetectorConfig = json5::from_str(
            r#"{
                skip_ransac: false,
                plane_ransac_max_iterations: 100,
                plane_ransac_inlier_threshold: 0.01
            }"#,
        )
        .unwrap();
        let collinear_points = vec![
            na::Point3::new(-0.2, 0.0, 2.0),
            na::Point3::new(0.0, 0.0, 2.0),
            na::Point3::new(0.2, 0.0, 2.0),
        ];
        assert!(CalibrationBoardLocatorNode::select_bbox_evidence(
            &collinear_points,
            &target,
            &detector_config,
            &Header::default(),
            &None,
        )
        .unwrap()
        .is_none());
    }

    #[test]
    fn solid_estimator_rejects_with_structured_reason() {
        let target = ValidatedTarget::parse_json5(include_bytes!(
            "../../lctk_launch/config/targets/solid_600_aruco_1_v1.json5"
        ))
        .unwrap();
        let estimator = TargetPoseEstimator::new(
            &target,
            TargetPoseEstimatorTuning::for_solid(SolidRefinementTuning::new(0.02, 8, 1, 3, 4, 2)),
        )
        .unwrap();
        let plane = PlaneModel {
            center: na::Point3::new(0.0, 0.0, 2.0),
            normal: na::Vector3::z(),
            u: na::Vector3::x(),
            v: na::Vector3::y(),
        };
        let square_fit = board_cluster_detector::square_fit::SquareFit {
            center: [0.0, 0.0],
            theta: 0.0,
            residual: 0.0,
            corners_2d: [[0.3, 0.3], [-0.3, 0.3], [-0.3, -0.3], [0.3, -0.3]],
        };
        let observation =
            TargetSquarePlaneObservation::from_fitted_square(&plane, &square_fit, na::Vector3::z())
                .unwrap();
        let result = estimator.estimate(observation, Vec::new());
        match result {
            TargetPoseEstimate::Rejected(rejection) => {
                assert_eq!(rejection.target_identity, *target.identity());
                assert!(matches!(
                    rejection.reason,
                    TargetRejectReason::InsufficientOuterEdgeEvidence { .. }
                        | TargetRejectReason::BoardUpAlignment { .. }
                ));
            }
            TargetPoseEstimate::Detected(_) => panic!("empty edge evidence must reject"),
        }
    }
}
