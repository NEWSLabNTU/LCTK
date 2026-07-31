//! Detection orchestration: downsample -> foreground candidates -> square-fit
//! discriminator -> best pose. Port of the `square_icp=True` branch of
//! `experiments/board-detection-2d/src/boarddet/detector.py:detect`.
//!
//! Only the square-icp branch is ported (the production path). The non-icp
//! scoring branch is out of scope.

use nalgebra::Point3;

use crate::background::BackgroundModel;
use crate::candidates::{generate_background_diff, generate_plane_strip};
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
#[derive(Debug, Clone)]
pub struct DetectOutcome {
    pub detection: Option<BoardDetection>,
    pub selected_points: Option<Vec<Point3<f64>>>,
    pub selected_plane: Option<PlaneModel>,
    pub n_candidates: usize,
    pub reject: Option<RejectReason>,
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

    let cands = match method {
        ForegroundMethod::PlaneStrip => generate_plane_strip(&dn, board),
        ForegroundMethod::BackgroundSubtraction => match background {
            Some(bg) => generate_background_diff(&dn, board, bg),
            None => {
                return DetectOutcome {
                    detection: None,
                    selected_points: None,
                    selected_plane: None,
                    n_candidates: 0,
                    reject: Some(RejectReason::NoClusters),
                };
            }
        },
    };
    let n_candidates = cands.len();

    let mut best_residual = f64::INFINITY;
    let mut best_det: Option<BoardDetection> = None;
    let mut best_points: Option<Vec<Point3<f64>>> = None;
    let mut best_plane: Option<PlaneModel> = None;
    let mut furthest_reject: Option<RejectReason> = None;
    let note = |r: RejectReason, cur: &mut Option<RejectReason>| {
        if cur.map(|c| r.rank() > c.rank()).unwrap_or(true) {
            *cur = Some(r);
        }
    };

    for cand in &cands {
        let coords = project_to_plane(&cand.points, &cand.plane);
        let seed = seed_center(&coords, board);
        let fit = fit_fixed_square(&coords, board.side_m, Some(seed), None);
        let fit = match fit {
            Some(f) if f.residual < board.square_icp_residual_max => f,
            _ => {
                note(RejectReason::SquareResidual, &mut furthest_reject);
                continue;
            }
        };

        let det = board_pose(
            &cand.plane,
            &fit.corners_2d,
            1.0 / (1.0 + fit.residual),
            board.up_axis,
        );

        if board.stance_floor > 0.0
            && stance_3d(&det.corners_3d, board.up_axis) <= board.stance_floor
        {
            note(RejectReason::Stance, &mut furthest_reject);
            continue;
        }

        if board.isolation
            && isolation_density(&dn, &cand.plane, &fit.corners_2d) > board.isolation_max_density
        {
            note(RejectReason::Isolation, &mut furthest_reject);
            continue;
        }

        if fit.residual < best_residual {
            best_residual = fit.residual;
            best_det = Some(det);
            best_points = Some(cand.points.clone());
            best_plane = Some(cand.plane);
        }
    }

    let reject = if best_det.is_some() {
        None
    } else {
        Some(furthest_reject.unwrap_or(RejectReason::NoClusters))
    };

    DetectOutcome {
        detection: best_det,
        selected_points: best_points,
        selected_plane: best_plane,
        n_candidates,
        reject,
    }
}
