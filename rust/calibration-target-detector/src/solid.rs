//! Solid-board refinement adapter.
//!
//! The square fit is the sole source of in-plane centre and yaw.  The fitted
//! plane is the sole source of normal translation and tilt.  This separation is
//! intentional: interior plane points may improve plane geometry, but can never
//! manufacture the edge evidence required to accept an in-plane square pose.

use anyhow::{bail, Result};
use calibration_target::{Surface, ValidatedTarget};
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, UnitVector3};

use crate::TargetSquarePlaneObservation;

/// The one fixed solid-board orientation acceptance threshold.
pub const MIN_FINAL_BOARD_UP_ALIGNMENT: f64 = 0.90;

/// Explicit, deployment-owned evidence tuning for the solid-board adapter.
///
/// These values intentionally have no implicit production default: a caller must
/// name the edge band and the minimum number of supporting returns for its sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidRefinementTuning {
    /// A return supports an edge when its projected point is within this distance
    /// of a fitted-square perimeter segment.
    pub(crate) edge_band_m: f64,
    /// Minimum count of supporting returns required before accepting in-plane pose.
    pub(crate) minimum_edge_points: usize,
    /// Minimum returns assigned to each covered perimeter edge.
    pub(crate) minimum_points_per_covered_edge: usize,
    /// Number of distinct perimeter edges that must have the required support.
    /// Three is the least coverage that observes both in-plane translations and
    /// yaw; deployments that need all four edges must say so explicitly.
    pub(crate) minimum_covered_edges: usize,
    /// Number of equal longitudinal bins used to test spread along each edge.
    pub(crate) longitudinal_bins_per_edge: usize,
    /// Minimum occupied longitudinal bins on every edge counted as covered.
    pub(crate) minimum_occupied_longitudinal_bins: usize,
}

impl SolidRefinementTuning {
    /// Creates explicit deployment tuning for the solid adapter.
    pub fn new(
        edge_band_m: f64,
        minimum_edge_points: usize,
        minimum_points_per_covered_edge: usize,
        minimum_covered_edges: usize,
        longitudinal_bins_per_edge: usize,
        minimum_occupied_longitudinal_bins: usize,
    ) -> Self {
        Self {
            edge_band_m,
            minimum_edge_points,
            minimum_points_per_covered_edge,
            minimum_covered_edges,
            longitudinal_bins_per_edge,
            minimum_occupied_longitudinal_bins,
        }
    }
}

/// Honest uncertainty status for the in-plane square estimate.
///
/// Edge coverage establishes that the square parameters are observable enough to
/// accept, but this adapter has no residual/noise model from which to estimate a
/// numeric covariance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InPlaneUncertainty {
    NotEstimatedFromGeometricEdgeEvidence,
}

/// Which evidence is allowed to determine each independent pose component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseEvidenceOwner {
    SquareFit,
    PlaneFit,
}

/// Observability and evidence-ownership report for a solid-board refinement.
#[derive(Debug, Clone, PartialEq)]
pub struct SolidRefinementDiagnostics {
    pub edge_point_count: usize,
    /// Returns assigned to each cyclic fitted perimeter edge.  A return is
    /// assigned once, even when it lies inside the bands of two adjacent edges.
    pub edge_point_counts: [usize; 4],
    pub covered_edge_count: usize,
    /// Number of independently occupied longitudinal bins for each edge.
    pub occupied_longitudinal_bins: [usize; 4],
    /// Bitmask of occupied longitudinal bins for each edge; bit 0 is adjacent
    /// to the edge's cyclic start corner.
    pub longitudinal_bin_masks: [u64; 4],
    pub edge_band_m: f64,
    pub minimum_edge_points: usize,
    pub minimum_points_per_covered_edge: usize,
    pub minimum_covered_edges: usize,
    pub longitudinal_bins_per_edge: usize,
    pub minimum_occupied_longitudinal_bins: usize,
    /// True unless each of the two in-plane directions has support on both
    /// opposing square edges.
    pub weak_in_plane_center: bool,
    /// True unless all four perimeter edges support the fit.
    pub weak_yaw: bool,
    pub in_plane_uncertainty: InPlaneUncertainty,
    pub in_plane_center_owner: PoseEvidenceOwner,
    pub yaw_owner: PoseEvidenceOwner,
    pub normal_translation_owner: PoseEvidenceOwner,
    pub tilt_owner: PoseEvidenceOwner,
    /// Signed distance of the fitted square centre along the sensor-facing normal.
    pub normal_translation_m: f64,
}

/// Accepted solid-board pose.  `selected_corner_index` is frozen from the W3-A
/// orientation ranking; this adapter does not re-rank it during refinement.
#[derive(Debug, Clone)]
pub struct SolidRefinementResult {
    pub pose: Isometry3<f64>,
    pub selected_corner_index: usize,
    pub final_board_up_alignment: f64,
    pub diagnostics: SolidRefinementDiagnostics,
}

/// Adapter-local failures.  The facade maps every case into its stable public
/// rejection vocabulary; no caller needs to interpret formatted errors.
#[derive(Debug, Clone)]
pub(crate) enum SolidRejection {
    BoardUpAlignment { evidence: SolidRejectionEvidence },
    InsufficientOuterEdgeEvidence { evidence: SolidRejectionEvidence },
}

#[derive(Debug, Clone)]
pub(crate) struct SolidRejectionEvidence {
    pub(crate) perimeter: PerimeterEvidence,
    pub(crate) covered_edge_count: usize,
    pub(crate) weak_in_plane_center: bool,
    pub(crate) weak_yaw: bool,
    pub(crate) board_up_alignment: f64,
    pub(crate) tuning: SolidRefinementTuning,
}

/// Refine an already fitted solid square without allowing interior plane evidence
/// to change its in-plane interpretation.
pub fn refine_solid_target(
    target: &ValidatedTarget,
    observation: &TargetSquarePlaneObservation,
    evidence_points: &[Point3<f64>],
    tuning: SolidRefinementTuning,
) -> std::result::Result<SolidRefinementResult, Box<SolidRejection>> {
    if !matches!(target.plate().surface, Surface::Solid) {
        unreachable!("TargetPoseEstimator dispatches solid refinement only for solid targets");
    }

    // Do not choose again after any fitting step.  The candidate ranking is W3-A's
    // deterministic orientation decision, and this index is part of the result.
    let selected = observation.best_candidate();
    let edge_evidence = fitted_perimeter_evidence_by_edge(
        evidence_points,
        &observation.fitted_corners,
        tuning.edge_band_m,
        tuning.longitudinal_bins_per_edge,
    );
    let edge_point_counts = edge_evidence.point_counts;
    let edge_point_count = edge_point_counts.iter().sum();
    let covered_edge_count = covered_edge_count(&edge_evidence, tuning);
    let rejection_evidence = || SolidRejectionEvidence {
        weak_in_plane_center: !has_full_perimeter_evidence(
            &edge_evidence,
            tuning.minimum_points_per_covered_edge,
            tuning.minimum_occupied_longitudinal_bins,
        ),
        weak_yaw: !has_full_perimeter_evidence(
            &edge_evidence,
            tuning.minimum_points_per_covered_edge,
            tuning.minimum_occupied_longitudinal_bins,
        ),
        perimeter: edge_evidence.clone(),
        covered_edge_count,
        board_up_alignment: selected.sensor_up_alignment,
        tuning,
    };
    if selected.sensor_up_alignment < MIN_FINAL_BOARD_UP_ALIGNMENT {
        return Err(Box::new(SolidRejection::BoardUpAlignment {
            evidence: rejection_evidence(),
        }));
    }
    if edge_point_count < tuning.minimum_edge_points {
        return Err(Box::new(SolidRejection::InsufficientOuterEdgeEvidence {
            evidence: rejection_evidence(),
        }));
    }
    if covered_edge_count < tuning.minimum_covered_edges {
        return Err(Box::new(SolidRejection::InsufficientOuterEdgeEvidence {
            evidence: rejection_evidence(),
        }));
    }

    let pose = pose_from_selected_candidate(
        observation,
        selected.x_axis,
        selected.board_up,
        selected.z_axis,
    );
    let diagnostics = SolidRefinementDiagnostics {
        edge_point_count,
        edge_point_counts,
        covered_edge_count,
        occupied_longitudinal_bins: edge_evidence.occupied_longitudinal_bins,
        longitudinal_bin_masks: edge_evidence.longitudinal_bin_masks,
        edge_band_m: tuning.edge_band_m,
        minimum_edge_points: tuning.minimum_edge_points,
        minimum_points_per_covered_edge: tuning.minimum_points_per_covered_edge,
        minimum_covered_edges: tuning.minimum_covered_edges,
        longitudinal_bins_per_edge: tuning.longitudinal_bins_per_edge,
        minimum_occupied_longitudinal_bins: tuning.minimum_occupied_longitudinal_bins,
        weak_in_plane_center: !has_full_perimeter_evidence(
            &edge_evidence,
            tuning.minimum_points_per_covered_edge,
            tuning.minimum_occupied_longitudinal_bins,
        ),
        weak_yaw: !has_full_perimeter_evidence(
            &edge_evidence,
            tuning.minimum_points_per_covered_edge,
            tuning.minimum_occupied_longitudinal_bins,
        ),
        in_plane_uncertainty: InPlaneUncertainty::NotEstimatedFromGeometricEdgeEvidence,
        in_plane_center_owner: PoseEvidenceOwner::SquareFit,
        yaw_owner: PoseEvidenceOwner::SquareFit,
        normal_translation_owner: PoseEvidenceOwner::PlaneFit,
        tilt_owner: PoseEvidenceOwner::PlaneFit,
        normal_translation_m: observation
            .center
            .coords
            .dot(&observation.sensor_facing_normal),
    };

    Ok(SolidRefinementResult {
        pose,
        selected_corner_index: selected.corner_index,
        final_board_up_alignment: selected.sensor_up_alignment,
        diagnostics,
    })
}

fn covered_edge_count(evidence: &PerimeterEvidence, tuning: SolidRefinementTuning) -> usize {
    evidence
        .point_counts
        .iter()
        .zip(evidence.occupied_longitudinal_bins.iter())
        .filter(|(&count, &occupied_bins)| {
            count >= tuning.minimum_points_per_covered_edge
                && occupied_bins >= tuning.minimum_occupied_longitudinal_bins
        })
        .count()
}

pub(crate) fn validate_tuning(tuning: SolidRefinementTuning) -> Result<()> {
    if !tuning.edge_band_m.is_finite() || tuning.edge_band_m <= 0.0 {
        bail!("solid refinement edge_band_m must be finite and greater than zero");
    }
    if tuning.minimum_edge_points == 0 {
        bail!("solid refinement minimum_edge_points must be greater than zero");
    }
    if tuning.minimum_points_per_covered_edge == 0 {
        bail!("solid refinement minimum_points_per_covered_edge must be greater than zero");
    }
    if !(3..=4).contains(&tuning.minimum_covered_edges) {
        bail!("solid refinement minimum_covered_edges must be between 3 and 4");
    }
    if !(2..=64).contains(&tuning.longitudinal_bins_per_edge) {
        bail!("solid refinement longitudinal_bins_per_edge must be between 2 and 64");
    }
    if !(2..=tuning.longitudinal_bins_per_edge).contains(&tuning.minimum_occupied_longitudinal_bins)
    {
        bail!(
            "solid refinement minimum_occupied_longitudinal_bins must be between 2 and longitudinal_bins_per_edge"
        );
    }
    Ok(())
}

fn pose_from_selected_candidate(
    observation: &TargetSquarePlaneObservation,
    x_axis: UnitVector3<f64>,
    y_axis: UnitVector3<f64>,
    z_axis: UnitVector3<f64>,
) -> Isometry3<f64> {
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
        x_axis.into_inner(),
        y_axis.into_inner(),
        z_axis.into_inner(),
    ]));
    Isometry3::from_parts(
        Translation3::from(observation.center.coords),
        UnitQuaternion::from_rotation_matrix(&rotation),
    )
}

fn fitted_perimeter_evidence_by_edge(
    points: &[Point3<f64>],
    corners: &[Point3<f64>; 4],
    edge_band_m: f64,
    longitudinal_bins_per_edge: usize,
) -> PerimeterEvidence {
    let mut point_counts = [0; 4];
    let mut occupied_bins = [0u64; 4];
    for point in points
        .iter()
        .filter(|point| point.coords.iter().all(|value| value.is_finite()))
    {
        let (edge_index, distance, fraction) = (0..4)
            .map(|index| {
                let (distance, fraction) = distance_and_fraction_to_segment(
                    *point,
                    corners[index],
                    corners[(index + 1) % 4],
                );
                (index, distance, fraction)
            })
            .min_by(|left, right| left.1.total_cmp(&right.1))
            .expect("a square has four perimeter edges");
        if distance <= edge_band_m {
            point_counts[edge_index] += 1;
            let bin = ((fraction * longitudinal_bins_per_edge as f64) as usize)
                .min(longitudinal_bins_per_edge - 1);
            occupied_bins[edge_index] |= 1 << bin;
        }
    }
    PerimeterEvidence {
        point_counts,
        occupied_longitudinal_bins: occupied_bins.map(|bins| bins.count_ones() as usize),
        longitudinal_bin_masks: occupied_bins,
    }
}

#[derive(Debug, Clone)]
pub(crate) struct PerimeterEvidence {
    pub(crate) point_counts: [usize; 4],
    pub(crate) occupied_longitudinal_bins: [usize; 4],
    longitudinal_bin_masks: [u64; 4],
}

fn has_full_perimeter_evidence(
    evidence: &PerimeterEvidence,
    minimum_points: usize,
    minimum_occupied_longitudinal_bins: usize,
) -> bool {
    evidence
        .point_counts
        .iter()
        .zip(evidence.occupied_longitudinal_bins.iter())
        .all(|(&count, &occupied_bins)| {
            count >= minimum_points && occupied_bins >= minimum_occupied_longitudinal_bins
        })
}

fn distance_and_fraction_to_segment(
    point: Point3<f64>,
    start: Point3<f64>,
    end: Point3<f64>,
) -> (f64, f64) {
    let segment = end - start;
    let length_squared = segment.norm_squared();
    if length_squared == 0.0 {
        return ((point - start).norm(), 0.0);
    }
    let fraction = ((point - start).dot(&segment) / length_squared).clamp(0.0, 1.0);
    ((point - (start + segment * fraction)).norm(), fraction)
}
