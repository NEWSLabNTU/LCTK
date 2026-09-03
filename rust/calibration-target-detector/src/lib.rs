//! Calibration-target pose estimation.
//!
//! [`TargetPoseEstimator`] is the only production estimator seam.  It accepts
//! neutral square-and-plane evidence, dispatches to the physical surface adapter
//! internally, and returns either one target-neutral detection or a structured
//! rejection.  The surface adapters are deliberately private: callers select a
//! validated target definition and detector tuning, never an estimator class.

use anyhow::{bail, Result};
use board_cluster_detector::{
    detector::SquarePlaneObservation,
    geometry::{unproject, PlaneModel},
    square_fit::SquareFit,
};
use calibration_target::{Surface, TargetIdentity, ValidatedTarget};
use nalgebra::{Isometry3, Point3, UnitVector3, Vector3};

mod perforated;
mod solid;

pub use perforated::PerforatedIcpConfig;
pub use solid::SolidRefinementTuning;

/// Reusable, deployment-owned estimator tuning.
///
/// This contains no plate dimensions, cutout positions, marker layout, or target
/// identity.  Those physical facts are read exclusively from [`ValidatedTarget`].
/// A preset supplies the one field relevant to its selected target; construction
/// rejects a preset that omits or supplies tuning for the wrong surface.
#[derive(Debug, Clone, Default)]
pub struct TargetPoseEstimatorTuning {
    solid: Option<SolidRefinementTuning>,
    perforated: Option<PerforatedIcpConfig>,
}

impl TargetPoseEstimatorTuning {
    pub fn for_solid(solid: SolidRefinementTuning) -> Self {
        Self {
            solid: Some(solid),
            perforated: None,
        }
    }

    pub fn for_perforated(perforated: PerforatedIcpConfig) -> Self {
        Self {
            solid: None,
            perforated: Some(perforated),
        }
    }
}

/// The deep, target-neutral estimator facade.
#[derive(Debug, Clone)]
pub struct TargetPoseEstimator {
    target: ValidatedTarget,
    tuning: TargetPoseEstimatorTuning,
}

impl TargetPoseEstimator {
    /// Bind reusable detector tuning to one immutable physical target.
    pub fn new(target: &ValidatedTarget, tuning: TargetPoseEstimatorTuning) -> Result<Self> {
        match (&target.plate().surface, &tuning.solid, &tuning.perforated) {
            (Surface::Solid, Some(solid), None) => solid::validate_tuning(*solid)?,
            (Surface::Perforated { .. }, None, Some(perforated)) => perforated.validate()?,
            (Surface::Solid, _, _) => bail!(
                "solid target requires exactly solid detector tuning and no perforated tuning"
            ),
            (Surface::Perforated { .. }, _, _) => bail!(
                "perforated target requires exactly perforated detector tuning and no solid tuning"
            ),
        }
        Ok(Self {
            target: target.clone(),
            tuning,
        })
    }

    pub fn target_identity(&self) -> &TargetIdentity {
        self.target.identity()
    }

    /// Estimate one pose from shared square/plane evidence and selected target
    /// returns.  Rejections are data, not exceptional control flow: observers can
    /// publish actionable diagnostics without parsing error strings.
    ///
    /// This deliberately does not accept raw point clouds.  W4-A owns bbox and
    /// bbox-free point selection plus background state; accepting raw points here
    /// would silently choose detector/background policy in the facade.  Both
    /// selectors instead hand their common observation and selected evidence to
    /// this entry point.
    pub fn estimate(
        &self,
        observation: TargetSquarePlaneObservation,
        evidence_points: Vec<Point3<f64>>,
    ) -> TargetPoseEstimate {
        match &self.target.plate().surface {
            Surface::Solid => self.estimate_solid(observation, evidence_points),
            Surface::Perforated { .. } => self.estimate_perforated(observation, evidence_points),
        }
    }

    fn estimate_solid(
        &self,
        observation: TargetSquarePlaneObservation,
        evidence_points: Vec<Point3<f64>>,
    ) -> TargetPoseEstimate {
        let tuning = self.tuning.solid.expect("validated solid tuning");
        match solid::refine_solid_target(&self.target, &observation, &evidence_points, tuning) {
            Ok(result) => TargetPoseEstimate::Detected(Box::new(TargetDetection {
                pose: result.pose,
                target_identity: self.target.identity().clone(),
                selected_quadrant: result.selected_corner_index,
                diagnostics: TargetDetectionDiagnostics::Solid(EdgeCoverageEvidence {
                    edge_point_count: result.diagnostics.edge_point_count,
                    edge_point_counts: result.diagnostics.edge_point_counts,
                    covered_edge_count: result.diagnostics.covered_edge_count,
                    occupied_longitudinal_bins: result.diagnostics.occupied_longitudinal_bins,
                    weak_in_plane_center: result.diagnostics.weak_in_plane_center,
                    weak_yaw: result.diagnostics.weak_yaw,
                    board_up_alignment: result.final_board_up_alignment,
                    edge_band_m: result.diagnostics.edge_band_m,
                    minimum_edge_points: result.diagnostics.minimum_edge_points,
                    minimum_points_per_covered_edge: result
                        .diagnostics
                        .minimum_points_per_covered_edge,
                    minimum_covered_edges: result.diagnostics.minimum_covered_edges,
                    longitudinal_bins_per_edge: result.diagnostics.longitudinal_bins_per_edge,
                    minimum_occupied_longitudinal_bins: result
                        .diagnostics
                        .minimum_occupied_longitudinal_bins,
                }),
            })),
            Err(error) => TargetPoseEstimate::Rejected(Box::new(TargetRejection {
                target_identity: self.target.identity().clone(),
                reason: solid_reject_reason(*error),
                observation,
            })),
        }
    }

    fn estimate_perforated(
        &self,
        observation: TargetSquarePlaneObservation,
        evidence_points: Vec<Point3<f64>>,
    ) -> TargetPoseEstimate {
        let tuning = self.tuning.perforated.expect("validated perforated tuning");
        match perforated::estimate_perforated_pose(
            &self.target,
            &observation,
            evidence_points,
            tuning,
        ) {
            Ok(result) => TargetPoseEstimate::Detected(Box::new(TargetDetection {
                pose: result.pose,
                target_identity: self.target.identity().clone(),
                selected_quadrant: result.winning_candidate_index,
                diagnostics: TargetDetectionDiagnostics::CutoutIcp(CutoutIcpEvidence {
                    best_loss_m: result.best_loss_m,
                    second_best_loss_m: result.second_best_loss_m,
                    loss_separation_m: result.loss_separation_m,
                    cutout_rim_correspondences: result.cutout_rim_correspondences,
                    iteration_count: result.iteration_count,
                    total_correspondences: result.total_correspondences,
                    termination: result.termination,
                }),
            })),
            Err(error) => TargetPoseEstimate::Rejected(Box::new(TargetRejection {
                target_identity: self.target.identity().clone(),
                reason: perforated_reject_reason(error),
                observation,
            })),
        }
    }
}

fn perforated_reject_reason(error: perforated::PerforatedRejection) -> TargetRejectReason {
    use perforated::PerforatedRejection;
    match error {
        PerforatedRejection::AmbiguousCutoutEvidence {
            evidence,
            required_separation_m,
        } => TargetRejectReason::AmbiguousCutoutEvidence {
            evidence: cutout_evidence(evidence),
            required_separation_m,
        },
        PerforatedRejection::WeakCutoutEvidence {
            evidence,
            required_rim_correspondences,
        } => TargetRejectReason::WeakCutoutEvidence {
            evidence: cutout_evidence(evidence),
            required_rim_correspondences,
        },
        PerforatedRejection::IcpFailure { evidence } => TargetRejectReason::PerforatedIcpFailure {
            evidence: cutout_evidence(evidence),
        },
    }
}

fn cutout_evidence(evidence: perforated::PerforatedEvidence) -> CutoutIcpEvidence {
    CutoutIcpEvidence {
        best_loss_m: evidence.best_loss_m,
        second_best_loss_m: evidence.second_best_loss_m,
        loss_separation_m: evidence.loss_separation_m,
        cutout_rim_correspondences: evidence.cutout_rim_correspondences,
        iteration_count: evidence.iteration_count,
        total_correspondences: evidence.total_correspondences,
        termination: evidence.termination,
    }
}

/// Successful target-neutral pose output.
#[derive(Debug, Clone)]
pub struct TargetDetection {
    pub pose: Isometry3<f64>,
    pub target_identity: TargetIdentity,
    /// The named target corner chosen by physical evidence, in cyclic fitted-corner order.
    pub selected_quadrant: usize,
    pub diagnostics: TargetDetectionDiagnostics,
}

/// Structured acceptance diagnostics without exposing a surface estimator.
#[derive(Debug, Clone)]
pub enum TargetDetectionDiagnostics {
    /// Solid targets report the outer-edge evidence that constrained pose.
    Solid(EdgeCoverageEvidence),
    /// Perforated targets report cutout evidence and ICP quality.
    CutoutIcp(CutoutIcpEvidence),
}

#[derive(Debug, Clone, PartialEq)]
pub struct CutoutIcpEvidence {
    pub best_loss_m: f64,
    /// `None` when no successful runner-up hypothesis existed, never a
    /// synthetic sentinel.
    pub second_best_loss_m: Option<f64>,
    /// `None` exactly when `second_best_loss_m` is `None`.
    pub loss_separation_m: Option<f64>,
    pub cutout_rim_correspondences: usize,
    pub iteration_count: usize,
    pub total_correspondences: usize,
    pub termination: IcpTermination,
}

/// Structured reason the cutout-aware ICP loop stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IcpTermination {
    GoodFit,
    StablePose,
    MaxIterations,
    TooFewInliers,
    TooFewKabschPoints,
    NoCorrespondences,
}

/// Target-neutral report of the plate-perimeter evidence used for a pose.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeCoverageEvidence {
    pub edge_point_count: usize,
    pub edge_point_counts: [usize; 4],
    pub covered_edge_count: usize,
    pub occupied_longitudinal_bins: [usize; 4],
    pub weak_in_plane_center: bool,
    pub weak_yaw: bool,
    pub board_up_alignment: f64,
    pub edge_band_m: f64,
    pub minimum_edge_points: usize,
    pub minimum_points_per_covered_edge: usize,
    pub minimum_covered_edges: usize,
    pub longitudinal_bins_per_edge: usize,
    pub minimum_occupied_longitudinal_bins: usize,
}

/// Estimate outcome.  A rejected observation carries its common evidence so a
/// ROS adapter can report/debug it with exactly the same semantics as acceptance.
#[derive(Debug, Clone)]
pub enum TargetPoseEstimate {
    Detected(Box<TargetDetection>),
    Rejected(Box<TargetRejection>),
}

#[derive(Debug, Clone)]
pub struct TargetRejection {
    pub target_identity: TargetIdentity,
    pub reason: TargetRejectReason,
    pub observation: TargetSquarePlaneObservation,
}

/// Stable rejection categories for logging, metrics and operator diagnosis.
#[derive(Debug, Clone, PartialEq)]
pub enum TargetRejectReason {
    BoardUpAlignment {
        alignment: f64,
        required_minimum: f64,
    },
    InsufficientOuterEdgeEvidence {
        evidence: EdgeCoverageEvidence,
    },
    AmbiguousCutoutEvidence {
        evidence: CutoutIcpEvidence,
        required_separation_m: f64,
    },
    WeakCutoutEvidence {
        evidence: CutoutIcpEvidence,
        required_rim_correspondences: usize,
    },
    PerforatedIcpFailure {
        evidence: CutoutIcpEvidence,
    },
}

fn solid_reject_reason(error: solid::SolidRejection) -> TargetRejectReason {
    match error {
        solid::SolidRejection::BoardUpAlignment { evidence } => {
            TargetRejectReason::BoardUpAlignment {
                alignment: evidence.board_up_alignment,
                required_minimum: solid::MIN_FINAL_BOARD_UP_ALIGNMENT,
            }
        }
        solid::SolidRejection::InsufficientOuterEdgeEvidence { evidence } => {
            TargetRejectReason::InsufficientOuterEdgeEvidence {
                evidence: edge_evidence(evidence),
            }
        }
    }
}

fn edge_evidence(evidence: solid::SolidRejectionEvidence) -> EdgeCoverageEvidence {
    EdgeCoverageEvidence {
        edge_point_count: evidence.perimeter.point_counts.iter().sum(),
        edge_point_counts: evidence.perimeter.point_counts,
        covered_edge_count: evidence.covered_edge_count,
        occupied_longitudinal_bins: evidence.perimeter.occupied_longitudinal_bins,
        weak_in_plane_center: evidence.weak_in_plane_center,
        weak_yaw: evidence.weak_yaw,
        board_up_alignment: evidence.board_up_alignment,
        edge_band_m: evidence.tuning.edge_band_m,
        minimum_edge_points: evidence.tuning.minimum_edge_points,
        minimum_points_per_covered_edge: evidence.tuning.minimum_points_per_covered_edge,
        minimum_covered_edges: evidence.tuning.minimum_covered_edges,
        longitudinal_bins_per_edge: evidence.tuning.longitudinal_bins_per_edge,
        minimum_occupied_longitudinal_bins: evidence.tuning.minimum_occupied_longitudinal_bins,
    }
}

/// Numerical equality used only to report a geometric tie.  Acceptance thresholds
/// belong to target-specific adapters, not this observation module.
pub const ALIGNMENT_TIE_EPSILON: f64 = 1e-12;

/// A sensor-facing fitted square plus all physically possible corner-up frames.
///
/// `fitted_corners` keeps `SquareFit::corners_2d` cyclic order.  No candidate is
/// a final target pose: a target-specific adapter owns physical-evidence checks,
/// acceptance thresholds, and refinement.
#[derive(Debug, Clone)]
pub struct TargetSquarePlaneObservation {
    pub center: Point3<f64>,
    pub fitted_corners: [Point3<f64>; 4],
    /// Unit plane normal facing the sensor at the sensor-frame origin.
    pub sensor_facing_normal: UnitVector3<f64>,
    pub board_up_candidates: [BoardUpCandidate; 4],
    pub orientation: OrientationSelection,
}

/// One quarter-turn interpretation derived from one fitted plate corner.
#[derive(Debug, Clone)]
pub struct BoardUpCandidate {
    /// Index into [`TargetSquarePlaneObservation::fitted_corners`].
    pub corner_index: usize,
    /// Fitted plate corner from which `board_up` was measured.
    pub corner: Point3<f64>,
    /// Unit vector from fitted square centre toward `corner`.
    pub board_up: UnitVector3<f64>,
    /// Canonical target +X candidate: `board_up × sensor_facing_normal`.
    pub x_axis: UnitVector3<f64>,
    /// Canonical target +Z candidate, facing the sensor.
    pub z_axis: UnitVector3<f64>,
    /// `board_up · sensor_up`; deliberately signed, never absolute.
    pub sensor_up_alignment: f64,
}

/// Deterministic ranking of four corner-up candidates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrientationSelection {
    /// Winning candidate index.  Ties select the lowest fitted-corner index so
    /// diagnostics are deterministic; `ambiguous` preserves that it is not unique.
    pub best_candidate_index: usize,
    pub best_alignment: f64,
    pub second_best_alignment: f64,
    pub alignment_gap: f64,
    pub ambiguous: bool,
}

impl TargetSquarePlaneObservation {
    /// Construct target-neutral observation from bbox-free detector evidence.
    ///
    /// `detect_for_target` supplies this `SquarePlaneObservation`.  The bbox-free
    /// adapter delegates to [`Self::from_fitted_square`], the same semantic
    /// constructor used by bbox-selected plane and square evidence.
    pub fn from_square_plane(
        evidence: &SquarePlaneObservation,
        sensor_up: Vector3<f64>,
    ) -> Result<Self> {
        Self::from_fitted_square(&evidence.plane, &evidence.square_fit, sensor_up)
    }

    /// Construct from a bbox-selected plane and known-size square fit.
    ///
    /// This is the explicit bbox handoff seam: bbox selection may bypass
    /// `detect_for_target`, but it must still produce the same plane-and-square
    /// evidence before target pose estimation.  Wiring the current ROS bbox path
    /// to this constructor belongs to Phase 8 W4-A.  The sensor is the origin of
    /// the input frame, and `sensor_up` must be finite and non-zero.
    pub fn from_fitted_square(
        plane: &PlaneModel,
        square_fit: &SquareFit,
        sensor_up: Vector3<f64>,
    ) -> Result<Self> {
        let sensor_up = finite_unit(sensor_up, "sensor_up")?;
        let center = unproject(&[square_fit.center], plane)[0];
        let fitted: Vec<Point3<f64>> = unproject(&square_fit.corners_2d, plane);
        let fitted_corners: [Point3<f64>; 4] = fitted
            .try_into()
            .expect("SquareFit always contains four corners");

        let mut normal = finite_unit(plane.normal, "plane.normal")?;
        let center_to_sensor = -center.coords;
        if !is_finite_nonzero(center_to_sensor) {
            bail!("fitted square center coincides with sensor origin");
        }
        if normal.dot(&center_to_sensor) < 0.0 {
            normal = UnitVector3::new_unchecked(-normal.into_inner());
        }

        let board_up_candidates = std::array::from_fn(|corner_index| {
            let corner = fitted_corners[corner_index];
            let board_up = UnitVector3::new_normalize(corner - center);
            let x_axis = UnitVector3::new_normalize(board_up.cross(&normal));
            BoardUpCandidate {
                corner_index,
                corner,
                board_up,
                x_axis,
                z_axis: normal,
                sensor_up_alignment: board_up.dot(&sensor_up),
            }
        });
        let orientation = select_orientation(&board_up_candidates);

        Ok(Self {
            center,
            fitted_corners,
            sensor_facing_normal: normal,
            board_up_candidates,
            orientation,
        })
    }

    /// Deterministically ranked best candidate.  Callers must inspect
    /// `orientation.ambiguous` and apply their target-specific gate.
    pub fn best_candidate(&self) -> &BoardUpCandidate {
        &self.board_up_candidates[self.orientation.best_candidate_index]
    }
}

fn finite_unit(vector: Vector3<f64>, name: &str) -> Result<UnitVector3<f64>> {
    if !is_finite_nonzero(vector) {
        bail!("{name} must be finite and non-zero");
    }
    Ok(UnitVector3::new_normalize(vector))
}

fn is_finite_nonzero(vector: Vector3<f64>) -> bool {
    vector.iter().all(|value| value.is_finite()) && vector.norm_squared() > 1e-24
}

fn select_orientation(candidates: &[BoardUpCandidate; 4]) -> OrientationSelection {
    let mut ranked = [0usize, 1, 2, 3];
    ranked.sort_by(|&left, &right| {
        candidates[right]
            .sensor_up_alignment
            .total_cmp(&candidates[left].sensor_up_alignment)
            .then_with(|| left.cmp(&right))
    });
    let best_candidate_index = ranked[0];
    let best_alignment = candidates[ranked[0]].sensor_up_alignment;
    let second_best_alignment = candidates[ranked[1]].sensor_up_alignment;
    let alignment_gap = best_alignment - second_best_alignment;
    OrientationSelection {
        best_candidate_index,
        best_alignment,
        second_best_alignment,
        alignment_gap,
        ambiguous: alignment_gap <= ALIGNMENT_TIE_EPSILON,
    }
}
