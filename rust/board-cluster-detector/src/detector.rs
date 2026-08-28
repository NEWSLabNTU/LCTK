//! Detection orchestration: downsample -> foreground candidates -> square-fit
//! discriminator -> best pose. Port of the `square_icp=True` branch of
//! `experiments/board-detection-2d/src/boarddet/detector.py:detect`.
//!
//! Only the square-icp branch is ported (the production path). The non-icp
//! scoring branch is out of scope.

#![allow(deprecated)] // This module owns the temporary legacy facade.

use nalgebra::Point3;

use crate::{
    background::BackgroundModel,
    candidates::foreground_and_candidates,
    config::{DetectorTuning, ForegroundMethod, TargetDetectionParams, TargetSide},
    geometry::{self, project_to_plane, PlaneModel},
    pose::{board_pose, isolation_density, stance_3d, BoardDetection},
    scorer::seed_center,
    square_fit::{fit_fixed_square, SquareFit},
};

/// Target-neutral evidence produced by board clustering. It deliberately does
/// not name a board-local axis or orientation: those belong to the target pose
/// estimator, which has the selected target's orientation reference.
#[derive(Debug, Clone)]
pub struct SquarePlaneObservation {
    pub points: Vec<Point3<f64>>,
    pub plane: PlaneModel,
    pub square_fit: SquareFit,
}

/// Why no board was detected. The first three (`Flatness`, `Extent`,
/// `SizeGate`) are raised inside candidate generation
/// (`plausible_board_patch`) and are not surfaced by the current generator
/// ports; `NoClusters`/`SquareResidual`/`Stance`/`Isolation` are the reasons
/// `detect` itself can report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectReason {
    NoClusters,
    Flatness,
    Extent,
    SizeGate,
    SquareResidual,
    Stance,
    Isolation,
}

impl RejectReason {
    /// Stage rank (later stage = further along the pipeline), used to pick the
    /// "furthest" reject, mirroring `reject.py:furthest`.
    fn rank(self) -> u8 {
        match self {
            RejectReason::NoClusters => 0,
            RejectReason::Flatness => 1,
            RejectReason::Extent => 2,
            RejectReason::SizeGate => 3,
            RejectReason::SquareResidual => 4,
            RejectReason::Stance => 5,
            RejectReason::Isolation => 6,
        }
    }
}

/// Measured value vs threshold for the furthest-progressed rejected candidate.
/// Lets a caller log HOW NARROWLY a frame missed each gate instead of just the
/// reason. `measured`/`threshold` are in the gate's own units (see `detect`).
#[derive(Debug, Clone, Copy)]
pub struct RejectDetail {
    pub measured: f64,
    pub threshold: f64,
}

/// Result of one `detect` call.
///
/// `selected_points` / `selected_plane` are the winning candidate's member
/// points and fitted plane — the sub-project-2 output fed to RANSAC+ICP.
/// `detection` (the square-fit pose) is used here only for gating/selection.
#[derive(Debug, Clone)]
pub struct DetectOutcome {
    pub detection: Option<BoardDetection>,
    pub selected_points: Option<Vec<Point3<f64>>>,
    pub selected_plane: Option<PlaneModel>,
    /// Target-neutral square/plane evidence retained for downstream target pose
    /// estimation. `detection` remains the deprecated legacy pose facade.
    pub observation: Option<SquarePlaneObservation>,
    pub n_candidates: usize,
    pub reject: Option<RejectReason>,
    /// Numbers behind `reject` for the furthest candidate (None if accepted or
    /// no candidate reached any gate). Units by reason:
    /// SquareResidual = coverage residual (unitless); Stance = normalized
    /// diagonal·up (0-1); Isolation = points per metre of quad perimeter.
    pub reject_detail: Option<RejectDetail>,
    /// The RAW per-method foreground — the point set fed to `cluster_and_gate`
    /// BEFORE clustering / coplanar merge / board-patch gating (Method E:
    /// background-subtracted points; Method B: non-big-plane remainder), on the
    /// voxel-downsampled cloud. Published as a debug cloud so the true foreground
    /// is visible, independent of surviving clusters or downstream `plane_inliers`.
    pub foreground_points: Vec<Point3<f64>>,
    /// Member points of the furthest-progressed REJECTED candidate — the cluster
    /// that got closest to passing before failing `reject`. Empty on an accepted
    /// frame or when no candidate reached a gate. Published so the operator can
    /// see the shape that failed square-fit / stance / isolation.
    pub rejected_cluster: Vec<Point3<f64>>,
}

/// Result of the target-side detector interface. Its public evidence is only a
/// fitted known-size square and plane; no target-frame pose is constructed
/// here. Diagnostics preserve the established bbox-free surface.
#[derive(Debug, Clone)]
pub struct TargetDetectOutcome {
    pub observation: Option<SquarePlaneObservation>,
    pub n_candidates: usize,
    pub reject: Option<RejectReason>,
    pub reject_detail: Option<RejectDetail>,
    pub foreground_points: Vec<Point3<f64>>,
    pub rejected_cluster: Vec<Point3<f64>>,
    candidates: Vec<NeutralCandidate>,
    square_furthest: Option<FurthestReject>,
}

#[derive(Debug, Clone)]
struct NeutralCandidate {
    observation: SquarePlaneObservation,
    isolation_reject: Option<RejectDetail>,
}

#[derive(Debug, Clone)]
struct FurthestReject {
    reason: RejectReason,
    detail: RejectDetail,
    points: Vec<Point3<f64>>,
}

fn consider_reject(
    furthest: &mut Option<FurthestReject>,
    reason: RejectReason,
    detail: RejectDetail,
    points: &[Point3<f64>],
) {
    let take = match furthest.as_ref() {
        None => true,
        Some(current) if reason.rank() > current.reason.rank() => true,
        Some(current) if reason.rank() == current.reason.rank() => {
            (detail.measured - detail.threshold).abs()
                < (current.detail.measured - current.detail.threshold).abs()
        }
        Some(_) => false,
    };
    if take {
        *furthest = Some(FurthestReject {
            reason,
            detail,
            points: points.to_vec(),
        });
    }
}

/// Detect the calibration board in one frame, producing a board *pose*.
///
/// Ports `detector.py:detect` (square_icp branch): `finite_only` ->
/// `voxel_downsample` -> per-method candidate generation -> per candidate
/// {project, seed, fixed-square fit, pose, stance gate, isolation gate} ->
/// keep the lowest-residual survivor.
///
/// Production detection goes through [`detect_for_target`], which stops at
/// neutral square/plane evidence and never constructs target-frame axes.  This
/// function is the one place that still runs the Python pipeline's pose
/// construction and its gate *ordering* -- stance before isolation -- and it
/// exists because the recorded-fixture parity suite compares against Python's
/// pose output, which neutral evidence alone cannot reproduce.  It is not a
/// compatibility shim for a serialized config: W5-E2 removed the `side_m`
/// adapter it used to take, so the physical side now enters the same way it
/// does everywhere else, through [`TargetSide`].
pub fn detect(
    points: &[Point3<f64>],
    target_side: TargetSide,
    tuning: &DetectorTuning,
    method: ForegroundMethod,
    voxel: f64,
    background: Option<&BackgroundModel>,
) -> DetectOutcome {
    let target = detect_for_target(points, target_side, tuning, method, voxel, background);

    let mut best: Option<(BoardDetection, SquarePlaneObservation)> = None;
    let mut furthest = target.square_furthest.clone();

    // Deliberately preserve the legacy gate order: stance precedes isolation.
    // Neutral detection computed isolation without constructing axes, but the
    // facade delays applying that result until after its legacy stance gate.
    for candidate in &target.candidates {
        let observation = &candidate.observation;
        let fit = observation.square_fit;
        let det = board_pose(
            &observation.plane,
            &fit.corners_2d,
            1.0 / (1.0 + fit.residual),
            tuning.up_axis,
        );

        if tuning.stance_floor > 0.0 {
            let stance = stance_3d(&det.corners_3d, tuning.up_axis);
            if stance <= tuning.stance_floor {
                consider_reject(
                    &mut furthest,
                    RejectReason::Stance,
                    RejectDetail {
                        measured: stance,
                        threshold: tuning.stance_floor,
                    },
                    &observation.points,
                );
                continue;
            }
        }

        if let Some(detail) = candidate.isolation_reject {
            consider_reject(
                &mut furthest,
                RejectReason::Isolation,
                detail,
                &observation.points,
            );
            continue;
        }

        let replace = best
            .as_ref()
            .is_none_or(|(_, current)| fit.residual < current.square_fit.residual);
        if replace {
            best = Some((det, observation.clone()));
        }
    }

    let accepted = best.is_some();
    let (detection, observation) = best
        .map(|(detection, observation)| (Some(detection), Some(observation)))
        .unwrap_or((None, None));
    let (reject, reject_detail, rejected_cluster) = legacy_reject(accepted, furthest);
    let (selected_points, selected_plane) = observation
        .as_ref()
        .map(|observation| (Some(observation.points.clone()), Some(observation.plane)))
        .unwrap_or((None, None));

    DetectOutcome {
        detection,
        selected_points,
        selected_plane,
        observation,
        n_candidates: target.n_candidates,
        reject,
        reject_detail,
        foreground_points: target.foreground_points,
        rejected_cluster,
    }
}

fn legacy_reject(
    accepted: bool,
    furthest: Option<FurthestReject>,
) -> (Option<RejectReason>, Option<RejectDetail>, Vec<Point3<f64>>) {
    match (accepted, furthest) {
        (true, _) => (None, None, Vec::new()),
        (false, Some(f)) => (Some(f.reason), Some(f.detail), f.points),
        (false, None) => (Some(RejectReason::NoClusters), None, Vec::new()),
    }
}

/// Detect a selected calibration target's square face.
///
/// The physical side enters only through [`TargetSide`]. `DetectorTuning`
/// contains sensor/range operating knobs, not target geometry. The returned
/// observation is intentionally axis-neutral for W3's target pose estimator.
pub fn detect_for_target(
    points: &[Point3<f64>],
    target_side: TargetSide,
    tuning: &DetectorTuning,
    method: ForegroundMethod,
    voxel: f64,
    background: Option<&BackgroundModel>,
) -> TargetDetectOutcome {
    let params = TargetDetectionParams::new(target_side, tuning);
    let pts = geometry::finite_only(points);
    let dn = geometry::voxel_downsample(&pts, voxel);

    // No background yet for Method E: nothing to diff against.
    if method == ForegroundMethod::BackgroundSubtraction && background.is_none() {
        return TargetDetectOutcome {
            observation: None,
            n_candidates: 0,
            reject: Some(RejectReason::NoClusters),
            reject_detail: None,
            foreground_points: Vec::new(),
            rejected_cluster: Vec::new(),
            candidates: Vec::new(),
            square_furthest: None,
        };
    }

    // `foreground_points` is the RAW per-method foreground (Method E:
    // background-subtracted points; Method B: non-big-plane remainder), captured
    // BEFORE clustering/merge/gate — the true foreground, not surviving clusters.
    let (foreground_points, cands) = foreground_and_candidates(&dn, &params, method, background);
    let n_candidates = cands.len();

    let mut best_observation: Option<SquarePlaneObservation> = None;
    let mut square_furthest: Option<FurthestReject> = None;
    let mut neutral_furthest: Option<FurthestReject> = None;
    let mut candidates = Vec::new();

    for cand in &cands {
        let coords = project_to_plane(&cand.points, &cand.plane);
        let seed = seed_center(&coords, &params);
        let fit = fit_fixed_square(&coords, target_side.as_metres(), Some(seed), None);
        let fit = match fit {
            Some(f) if f.residual < tuning.square_icp_residual_max => f,
            // Failed square fit: report the residual (NaN when the fit itself
            // returned nothing, e.g. too few points) against its threshold.
            Some(f) => {
                consider_reject(
                    &mut square_furthest,
                    RejectReason::SquareResidual,
                    RejectDetail {
                        measured: f.residual,
                        threshold: tuning.square_icp_residual_max,
                    },
                    &cand.points,
                );
                continue;
            }
            None => {
                consider_reject(
                    &mut square_furthest,
                    RejectReason::SquareResidual,
                    RejectDetail {
                        measured: f64::NAN,
                        threshold: tuning.square_icp_residual_max,
                    },
                    &cand.points,
                );
                continue;
            }
        };

        let isolation_reject = if tuning.isolation {
            let density =
                isolation_density(&dn, &cand.plane, &fit.corners_2d, &tuning.isolation_band());
            if density > tuning.isolation_max_density {
                Some(RejectDetail {
                    measured: density,
                    threshold: tuning.isolation_max_density,
                })
            } else {
                None
            }
        } else {
            None
        };

        let observation = SquarePlaneObservation {
            points: cand.points.clone(),
            plane: cand.plane,
            square_fit: fit,
        };
        if let Some(detail) = isolation_reject {
            consider_reject(
                &mut neutral_furthest,
                RejectReason::Isolation,
                detail,
                &cand.points,
            );
        } else if best_observation
            .as_ref()
            .is_none_or(|current| fit.residual < current.square_fit.residual)
        {
            best_observation = Some(observation.clone());
        }
        candidates.push(NeutralCandidate {
            observation,
            isolation_reject,
        });
    }

    if let Some(square) = square_furthest.clone() {
        consider_reject(
            &mut neutral_furthest,
            square.reason,
            square.detail,
            &square.points,
        );
    }
    let (reject, reject_detail, rejected_cluster) =
        match (best_observation.is_some(), neutral_furthest) {
            (true, _) => (None, None, Vec::new()),
            (false, Some(f)) => (Some(f.reason), Some(f.detail), f.points),
            (false, None) => (Some(RejectReason::NoClusters), None, Vec::new()),
        };

    TargetDetectOutcome {
        observation: best_observation,
        n_candidates,
        reject,
        reject_detail,
        foreground_points,
        rejected_cluster,
        candidates,
        square_furthest,
    }
}
