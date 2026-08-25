//! Target-neutral square-and-plane observation for calibration-target pose estimation.
//!
//! This crate deliberately stops before target pose estimation.  It converts the
//! fitted square evidence from `board-cluster-detector` into sensor-facing plane
//! geometry and four possible named board-up axes.  Surface adapters choose and
//! refine a candidate later; this module knows neither solid nor perforated targets.

use anyhow::{bail, Result};
use board_cluster_detector::{
    detector::SquarePlaneObservation,
    geometry::{unproject, PlaneModel},
    square_fit::SquareFit,
};
use nalgebra::{Point3, UnitVector3, Vector3};

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
