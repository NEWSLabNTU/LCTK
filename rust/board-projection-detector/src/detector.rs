//! Detection orchestration: downsample -> foreground candidates -> square-fit
//! discriminator -> best pose. Port of the `square_icp=True` branch of
//! `experiments/board-detection-2d/src/boarddet/detector.py:detect`.
//!
//! Only the square-icp branch is ported (the production path). The non-icp
//! scoring branch is out of scope.

use nalgebra::Point3;

use crate::background::BackgroundModel;
use crate::candidates::foreground_and_candidates;
use crate::config::{BoardConfig, ForegroundMethod};
use crate::geometry::{self, project_to_plane, PlaneModel};
use crate::pose::{board_pose, isolation_density, stance_3d, BoardDetection};
use crate::scorer::seed_center;
use crate::square_fit::fit_fixed_square;

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

/// Result of one `detect` call.
///
/// `selected_points` / `selected_plane` are the winning candidate's member
/// points and fitted plane — the sub-project-2 output fed to RANSAC+ICP.
/// `detection` (the square-fit pose) is used here only for gating/selection.
/// Measured value vs threshold for the furthest-progressed rejected candidate.
/// Lets a caller log HOW NARROWLY a frame missed each gate instead of just the
/// reason. `measured`/`threshold` are in the gate's own units (see `detect`).
#[derive(Debug, Clone, Copy)]
pub struct RejectDetail {
    pub measured: f64,
    pub threshold: f64,
}

#[derive(Debug, Clone)]
pub struct DetectOutcome {
    pub detection: Option<BoardDetection>,
    pub selected_points: Option<Vec<Point3<f64>>>,
    pub selected_plane: Option<PlaneModel>,
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
}

/// Detect the calibration board in one frame.
///
/// Ports `detector.py:detect` (square_icp branch): `finite_only` ->
/// `voxel_downsample` -> per-method candidate generation -> per candidate
/// {project, seed, fixed-square fit, pose, stance gate, isolation gate} ->
/// keep the lowest-residual survivor.
pub fn detect(
    points: &[Point3<f64>],
    board: &BoardConfig,
    method: ForegroundMethod,
    voxel: f64,
    background: Option<&BackgroundModel>,
) -> DetectOutcome {
    let pts = geometry::finite_only(points);
    let dn = geometry::voxel_downsample(&pts, voxel);

    // No background yet for Method E: nothing to diff against.
    if method == ForegroundMethod::BackgroundSubtraction && background.is_none() {
        return DetectOutcome {
            detection: None,
            selected_points: None,
            selected_plane: None,
            n_candidates: 0,
            reject: Some(RejectReason::NoClusters),
            reject_detail: None,
            foreground_points: Vec::new(),
        };
    }

    // `foreground_points` is the RAW per-method foreground (Method E:
    // background-subtracted points; Method B: non-big-plane remainder), captured
    // BEFORE clustering/merge/gate — the true foreground, not surviving clusters.
    let (foreground_points, cands) =
        foreground_and_candidates(&dn, board, method, background);
    let n_candidates = cands.len();

    let mut best_residual = f64::INFINITY;
    let mut best_det: Option<BoardDetection> = None;
    let mut best_points: Option<Vec<Point3<f64>>> = None;
    let mut best_plane: Option<PlaneModel> = None;
    // Track the furthest-progressed reject AND its measured/threshold numbers.
    // Furthest wins by rank; ties (same gate) keep the candidate that missed by
    // the smallest margin — the most informative "how close was it" reading.
    let mut furthest: Option<(RejectReason, RejectDetail)> = None;
    let consider =
        |r: RejectReason, measured: f64, threshold: f64, cur: &mut Option<(RejectReason, RejectDetail)>| {
            let take = match cur {
                None => true,
                Some((cr, cd)) => {
                    if r.rank() > cr.rank() {
                        true
                    } else if r.rank() == cr.rank() {
                        (measured - threshold).abs() < (cd.measured - cd.threshold).abs()
                    } else {
                        false
                    }
                }
            };
            if take {
                *cur = Some((r, RejectDetail { measured, threshold }));
            }
        };

    for cand in &cands {
        let coords = project_to_plane(&cand.points, &cand.plane);
        let seed = seed_center(&coords, board);
        let fit = fit_fixed_square(&coords, board.side_m, Some(seed), None);
        let fit = match fit {
            Some(f) if f.residual < board.square_icp_residual_max => f,
            // Failed square fit: report the residual (NaN when the fit itself
            // returned nothing, e.g. too few points) against its threshold.
            Some(f) => {
                consider(
                    RejectReason::SquareResidual,
                    f.residual,
                    board.square_icp_residual_max,
                    &mut furthest,
                );
                continue;
            }
            None => {
                consider(
                    RejectReason::SquareResidual,
                    f64::NAN,
                    board.square_icp_residual_max,
                    &mut furthest,
                );
                continue;
            }
        };

        let det = board_pose(
            &cand.plane,
            &fit.corners_2d,
            1.0 / (1.0 + fit.residual),
            board.up_axis,
        );

        if board.stance_floor > 0.0 {
            let stance = stance_3d(&det.corners_3d, board.up_axis);
            if stance <= board.stance_floor {
                consider(RejectReason::Stance, stance, board.stance_floor, &mut furthest);
                continue;
            }
        }

        if board.isolation {
            let density = isolation_density(
                &dn,
                &cand.plane,
                &fit.corners_2d,
                board.isolation_coplanar_tol,
                board.isolation_band_lo,
                board.isolation_band_hi,
            );
            if density > board.isolation_max_density {
                consider(
                    RejectReason::Isolation,
                    density,
                    board.isolation_max_density,
                    &mut furthest,
                );
                continue;
            }
        }

        if fit.residual < best_residual {
            best_residual = fit.residual;
            best_det = Some(det);
            best_points = Some(cand.points.clone());
            best_plane = Some(cand.plane);
        }
    }

    let (reject, reject_detail) = if best_det.is_some() {
        (None, None)
    } else {
        match furthest {
            Some((r, d)) => (Some(r), Some(d)),
            None => (Some(RejectReason::NoClusters), None),
        }
    };

    DetectOutcome {
        detection: best_det,
        selected_points: best_points,
        selected_plane: best_plane,
        n_candidates,
        reject,
        reject_detail,
        foreground_points,
    }
}
