//! Perforated-target pose refinement.
//!
//! This adapter owns the ICP lifecycle migrated from the former
//! `hollow-board-detector::BoardIcpIterator`, which W5-E2 deleted along with its
//! crate.  It applies the same step-then-termination contract to a
//! [`ValidatedTarget`] surface, whose closest-point query accounts for cutouts.

use anyhow::{bail, Result};
use calibration_target::{CircularCutout, Surface, ValidatedTarget};
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

use crate::{IcpTermination, TargetSquarePlaneObservation};

/// Tuning for cutout-aware ICP.  All values are explicit at the call site; this
/// module deliberately has no production defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PerforatedIcpConfig {
    max_iterations: usize,
    outlier_threshold_m: f64,
    damping_factor: f64,
    pose_weight_threshold: f64,
    /// Consecutive completed updates whose pose weight is at or below
    /// `pose_weight_threshold` before the iterator terminates with
    /// [`IcpTermination::StablePose`] -- a successful termination even when
    /// the residual is still above `good_fit_threshold_m`.
    stable_pose_iterations: usize,
    /// Residual threshold that ends ICP with [`IcpTermination::GoodFit`] once
    /// `state.avg_loss` drops strictly below it.  This mirrors the legacy
    /// caller's `icp_good_fit_threshold`; it is now the iterator's own
    /// termination condition, not a separate gate applied after the loop
    /// exits.
    good_fit_threshold_m: f64,
    min_inlier_points: usize,
    /// The winning quarter-turn must beat the runner up by at least this loss.
    min_hypothesis_loss_separation_m: f64,
    /// Minimum final model points which lie on a cutout rim.  This prevents a
    /// square-only solution from being accepted as perforated evidence.
    min_cutout_rim_correspondences: usize,
    /// Euclidean tolerance for observed input points to a physical cutout rim.
    cutout_rim_tolerance_m: f64,
}

impl PerforatedIcpConfig {
    /// Creates explicit deployment tuning for the perforated adapter.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        max_iterations: usize,
        outlier_threshold_m: f64,
        damping_factor: f64,
        pose_weight_threshold: f64,
        stable_pose_iterations: usize,
        good_fit_threshold_m: f64,
        min_inlier_points: usize,
        min_hypothesis_loss_separation_m: f64,
        min_cutout_rim_correspondences: usize,
        cutout_rim_tolerance_m: f64,
    ) -> Self {
        Self {
            max_iterations,
            outlier_threshold_m,
            damping_factor,
            pose_weight_threshold,
            stable_pose_iterations,
            good_fit_threshold_m,
            min_inlier_points,
            min_hypothesis_loss_separation_m,
            min_cutout_rim_correspondences,
            cutout_rim_tolerance_m,
        }
    }
    pub(crate) fn validate(self) -> Result<()> {
        if self.max_iterations == 0 {
            bail!("perforated ICP max_iterations must be greater than zero");
        }
        // `stable_pose_iterations` is a `usize` count, not one of the finite/
        // non-negative floats validated in the loop below.
        if self.stable_pose_iterations == 0 {
            bail!("perforated ICP stable_pose_iterations must be greater than zero");
        }
        for (name, value) in [
            ("outlier_threshold_m", self.outlier_threshold_m),
            ("damping_factor", self.damping_factor),
            ("pose_weight_threshold", self.pose_weight_threshold),
            ("good_fit_threshold_m", self.good_fit_threshold_m),
            (
                "min_hypothesis_loss_separation_m",
                self.min_hypothesis_loss_separation_m,
            ),
            ("cutout_rim_tolerance_m", self.cutout_rim_tolerance_m),
        ] {
            if !value.is_finite() || value < 0.0 {
                bail!("perforated ICP {name} must be finite and non-negative");
            }
        }
        if !(0.0..=1.0).contains(&self.damping_factor) || self.damping_factor == 0.0 {
            bail!("perforated ICP damping_factor must be in (0, 1]");
        }
        if self.min_inlier_points < 3 {
            bail!("perforated ICP min_inlier_points must be at least three");
        }
        Ok(())
    }
}

/// Complete state after one explicit ICP step.
///
/// Fields and termination ordering deliberately mirror the legacy iterator.  In
/// particular, a new state has zero good correspondences and must be stepped once
/// before a termination check is meaningful.
#[derive(Debug, Clone)]
pub struct PerforatedIcpState {
    pub iteration: usize,
    pub board_pose: Isometry3<f64>,
    pub inlier_points: Vec<Point3<f64>>,
    pub correspondences: Vec<(Point3<f64>, Point3<f64>)>,
    pub avg_loss: f64,
    pub total_correspondences: usize,
    pub good_correspondences: usize,
    pub termination_count: usize,
}

/// A cutout-aware replacement implementation of the legacy BoardIcpIterator.
/// It is intentionally retained as an internal seam, not a production toggle.
pub struct PerforatedBoardIcpIterator<'a> {
    target: &'a ValidatedTarget,
    config: PerforatedIcpConfig,
}

impl<'a> PerforatedBoardIcpIterator<'a> {
    pub fn new(target: &'a ValidatedTarget, config: PerforatedIcpConfig) -> Result<Self> {
        config.validate()?;
        if !matches!(target.plate().surface, Surface::Perforated { .. }) {
            bail!("perforated ICP requires a perforated target");
        }
        Ok(Self { target, config })
    }

    pub fn initial_state(
        &self,
        initial_pose: Isometry3<f64>,
        initial_inlier_points: Vec<Point3<f64>>,
    ) -> PerforatedIcpState {
        PerforatedIcpState {
            iteration: 0,
            board_pose: initial_pose,
            inlier_points: initial_inlier_points,
            correspondences: Vec::new(),
            avg_loss: f64::INFINITY,
            total_correspondences: 0,
            good_correspondences: 0,
            termination_count: 0,
        }
    }

    pub fn step(&self, current: &PerforatedIcpState) -> PerforatedIcpState {
        let posed = self.target.posed(current.board_pose);
        let correspondences: Vec<_> = posed
            .closest_points(current.inlier_points.iter())
            .into_iter()
            .map(|pair| (*pair.input, pair.closest))
            .collect();
        let total_correspondences = correspondences.len();
        if correspondences.is_empty() {
            return PerforatedIcpState {
                iteration: current.iteration + 1,
                correspondences: Vec::new(),
                avg_loss: f64::INFINITY,
                total_correspondences: 0,
                good_correspondences: 0,
                ..current.clone()
            };
        }

        let avg_loss = correspondences
            .iter()
            .map(|(input, closest)| (input - closest).norm())
            .sum::<f64>()
            / total_correspondences as f64;
        let good: Vec<_> = correspondences
            .iter()
            .copied()
            .filter(|(input, closest)| (input - closest).norm() <= self.config.outlier_threshold_m)
            .collect();
        let good_len = good.len();
        if good_len < 3 {
            return PerforatedIcpState {
                iteration: current.iteration + 1,
                correspondences: good,
                avg_loss,
                total_correspondences,
                good_correspondences: good_len,
                ..current.clone()
            };
        }

        let Some(align_pose) = kabsch_transform(
            &good.iter().map(|(_, model)| *model).collect::<Vec<_>>(),
            &good.iter().map(|(input, _)| *input).collect::<Vec<_>>(),
        ) else {
            return PerforatedIcpState {
                iteration: current.iteration + 1,
                correspondences: good,
                avg_loss,
                total_correspondences,
                good_correspondences: good_len,
                ..current.clone()
            };
        };
        let new_pose = align_pose * current.board_pose;
        let damped_translation = Translation3::from(
            current.board_pose.translation.vector
                + (new_pose.translation.vector - current.board_pose.translation.vector)
                    * self.config.damping_factor,
        );
        let damped_rotation = UnitQuaternion::slerp(
            &current.board_pose.rotation,
            &new_pose.rotation,
            self.config.damping_factor,
        );
        let pose_weight = (damped_translation.vector - current.board_pose.translation.vector)
            .norm()
            + damped_rotation
                .rotation_to(&current.board_pose.rotation)
                .angle();
        let termination_count = if pose_weight <= self.config.pose_weight_threshold {
            current.termination_count + 1
        } else {
            0
        };
        PerforatedIcpState {
            iteration: current.iteration + 1,
            board_pose: Isometry3::from_parts(damped_translation, damped_rotation),
            inlier_points: current.inlier_points.clone(),
            correspondences: good,
            avg_loss,
            total_correspondences,
            good_correspondences: good_len,
            termination_count,
        }
    }

    /// New ordering (M-21): hard-invalid state first -- too few inlier points,
    /// too few points for Kabsch, no correspondences -- then `GoodFit`, then
    /// `StablePose`, then the iteration limit.  The hard-invalid checks stay
    /// first because they mean the state itself cannot support a pose
    /// estimate at all, regardless of how the residual or update history
    /// looks; `GoodFit` outranks `StablePose` so a run that is both
    /// well-converged and quiet is reported for the reason a caller actually
    /// cares about.  This is a plain disjunction -- any one condition stops
    /// the loop -- but every disjunct here is exactly the predicate
    /// `termination_kind` uses to classify that same condition, in the same
    /// order, so the two cannot drift apart: whichever disjunct trips first
    /// here is what `termination_kind` reports once the loop has stopped.
    pub fn should_terminate(&self, state: &PerforatedIcpState) -> bool {
        state.inlier_points.len() < self.config.min_inlier_points
            || state.good_correspondences < 3
            || state.correspondences.is_empty()
            || state.avg_loss < self.config.good_fit_threshold_m
            || state.termination_count >= self.config.stable_pose_iterations
            || state.iteration >= self.config.max_iterations
    }

    /// Test-only diagnostic string, ordered exactly like [`should_terminate`]
    /// but -- unlike [`termination_kind`], which assumes its precondition
    /// (the state is already terminal) -- with an explicit `"Unknown"`
    /// fallback for a state where nothing has tripped yet, so a test can
    /// distinguish "still running" from a genuine terminal reason.
    #[cfg(test)]
    pub fn termination_reason(&self, state: &PerforatedIcpState) -> String {
        if state.inlier_points.len() < self.config.min_inlier_points {
            format!(
                "Insufficient inlier points: {} < {}",
                state.inlier_points.len(),
                self.config.min_inlier_points
            )
        } else if state.good_correspondences < 3 {
            format!(
                "Insufficient points for Kabsch: {}",
                state.good_correspondences
            )
        } else if state.correspondences.is_empty() {
            "No correspondences found".to_owned()
        } else if state.avg_loss < self.config.good_fit_threshold_m {
            "Converged (good fit)".to_owned()
        } else if state.termination_count >= self.config.stable_pose_iterations {
            "Converged (stable pose)".to_owned()
        } else if state.iteration >= self.config.max_iterations {
            format!("Max iterations reached: {}", self.config.max_iterations)
        } else {
            "Unknown".to_owned()
        }
    }
}

/// Per-quarter-turn refinement record.  This is deliberately adapter-local until W3-D
/// maps it into the public detection result.
#[derive(Debug, Clone)]
pub struct PerforatedHypothesisResult {
    pub candidate_index: usize,
    pub state: PerforatedIcpState,
    pub cutout_rim_correspondences: usize,
}

/// Accepted perforated pose, after square, cutout, and separation evidence agree.
#[derive(Debug, Clone)]
pub struct PerforatedPoseEstimate {
    pub pose: Isometry3<f64>,
    pub winning_candidate_index: usize,
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

/// Adapter-local ICP evidence carried to the facade on every rejection.
#[derive(Debug, Clone)]
pub(crate) struct PerforatedEvidence {
    pub(crate) best_loss_m: f64,
    /// `None` when no successful runner-up hypothesis existed, never a
    /// synthetic sentinel.
    pub(crate) second_best_loss_m: Option<f64>,
    /// `None` exactly when `second_best_loss_m` is `None`.
    pub(crate) loss_separation_m: Option<f64>,
    pub(crate) cutout_rim_correspondences: usize,
    pub(crate) iteration_count: usize,
    pub(crate) total_correspondences: usize,
    pub(crate) termination: IcpTermination,
}

/// Typed adapter failures.  Formatting remains only in legacy iterator APIs.
#[derive(Debug, Clone)]
pub(crate) enum PerforatedRejection {
    AmbiguousCutoutEvidence {
        evidence: PerforatedEvidence,
        required_separation_m: f64,
    },
    WeakCutoutEvidence {
        evidence: PerforatedEvidence,
        required_rim_correspondences: usize,
    },
    IcpFailure {
        evidence: PerforatedEvidence,
    },
}

impl std::fmt::Display for PerforatedRejection {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

/// No candidate is accepted unless the cutouts, rather than common square evidence,
/// select it.  There is deliberately no fallback to observation.orientation.
pub fn estimate_perforated_pose(
    target: &ValidatedTarget,
    observation: &TargetSquarePlaneObservation,
    inlier_points: Vec<Point3<f64>>,
    config: PerforatedIcpConfig,
) -> std::result::Result<PerforatedPoseEstimate, PerforatedRejection> {
    let iterator = PerforatedBoardIcpIterator::new(target, config)
        .expect("TargetPoseEstimator validates the perforated target and tuning");
    let hypotheses: [PerforatedHypothesisResult; 4] = std::array::from_fn(|candidate_index| {
        let initial = pose_from_candidate(observation, candidate_index);
        let mut state = iterator.initial_state(initial, inlier_points.clone());
        // Preserve the legacy, awkward protocol: do work first, then ask whether to stop.
        loop {
            state = iterator.step(&state);
            if iterator.should_terminate(&state) {
                break;
            }
        }
        PerforatedHypothesisResult {
            candidate_index,
            cutout_rim_correspondences: cutout_rim_correspondences(
                target,
                &state,
                config.cutout_rim_tolerance_m,
            ),
            state,
        }
    });
    // Classify before ranking. Only `GoodFit`/`StablePose` states are
    // publication candidates -- `MaxIterations` and every hard-invalid state
    // are discarded here, not merely outranked. `StablePose` does not itself
    // examine `avg_loss` (it only counts consecutive quiet updates), so a
    // state can reach it with a non-finite loss -- NaN from a degenerate
    // correspondence set, say. Such a state must never win a ranking
    // comparison or reach publication, so the finite check sits right here,
    // alongside the success classification, rather than as a separate gate
    // applied only to whatever `sort` happened to put first.
    let mut successful_indices: Vec<usize> = (0..hypotheses.len())
        .filter(|&index| {
            successful_termination(&hypotheses[index].state, config)
                && hypotheses[index].state.avg_loss.is_finite()
        })
        .collect();
    rank_by_loss(&hypotheses, &mut successful_indices);

    let Some(&best_index) = successful_indices.first() else {
        // Zero successful hypotheses: reject, but keep the lowest-loss failed
        // attempt as diagnostic evidence -- it is never published, and it
        // reports its own real `termination_kind` so an operator can tell
        // `MaxIterations` from `TooFewInliers`.
        let mut all_indices = [0usize, 1, 2, 3];
        rank_by_loss(&hypotheses, &mut all_indices);
        let worst_best = &hypotheses[all_indices[0]];
        return Err(PerforatedRejection::IcpFailure {
            evidence: PerforatedEvidence {
                best_loss_m: worst_best.state.avg_loss,
                second_best_loss_m: None,
                loss_separation_m: None,
                cutout_rim_correspondences: worst_best.cutout_rim_correspondences,
                iteration_count: worst_best.state.iteration,
                total_correspondences: worst_best.state.total_correspondences,
                termination: termination_kind(&worst_best.state, config),
            },
        });
    };
    let best = &hypotheses[best_index];
    let runner_up = successful_indices.get(1).map(|&index| &hypotheses[index]);
    let separation = runner_up.map(|second| second.state.avg_loss - best.state.avg_loss);
    let evidence = || PerforatedEvidence {
        best_loss_m: best.state.avg_loss,
        second_best_loss_m: runner_up.map(|second| second.state.avg_loss),
        loss_separation_m: separation,
        cutout_rim_correspondences: best.cutout_rim_correspondences,
        iteration_count: best.state.iteration,
        total_correspondences: best.state.total_correspondences,
        termination: termination_kind(&best.state, config),
    };
    // Iterator convergence alone is still subject to the legacy production
    // caller's final inlier gate.  There is no separate post-ICP residual
    // check: `good_fit_threshold_m` already decided termination above.
    //
    // This is arguably redundant now: `inlier_points` is cloned unchanged
    // into every one of the four attempts, so an observation with too few
    // inlier points makes all four classify `TooFewInliers` (hard-invalid)
    // and get discarded above, well before this line runs. Kept anyway --
    // it is the caller-facing statement of the requirement, and a future
    // change to how the loop seeds `inlier_points` per attempt must not
    // silently lose this gate.
    if best.state.inlier_points.len() < config.min_inlier_points {
        return Err(PerforatedRejection::IcpFailure {
            evidence: evidence(),
        });
    }
    // Skipped, not defaulted, when there is no successful runner-up: a single
    // successful hypothesis publishes without a separation comparison.
    if let Some(separation) = separation {
        if separation < config.min_hypothesis_loss_separation_m {
            return Err(PerforatedRejection::AmbiguousCutoutEvidence {
                evidence: evidence(),
                required_separation_m: config.min_hypothesis_loss_separation_m,
            });
        }
    }
    if best.cutout_rim_correspondences < config.min_cutout_rim_correspondences {
        return Err(PerforatedRejection::WeakCutoutEvidence {
            evidence: evidence(),
            required_rim_correspondences: config.min_cutout_rim_correspondences,
        });
    }
    Ok(PerforatedPoseEstimate {
        pose: best.state.board_pose,
        winning_candidate_index: best.candidate_index,
        best_loss_m: best.state.avg_loss,
        second_best_loss_m: runner_up.map(|second| second.state.avg_loss),
        loss_separation_m: separation,
        cutout_rim_correspondences: best.cutout_rim_correspondences,
        iteration_count: best.state.iteration,
        total_correspondences: best.state.total_correspondences,
        termination: termination_kind(&best.state, config),
    })
}

/// Sorts hypothesis indices ascending by `avg_loss`, breaking ties by
/// candidate index. The one deterministic ordering used both to rank
/// successful hypotheses for publication and to pick the most-plausible
/// failed attempt for rejection diagnostics.
fn rank_by_loss(hypotheses: &[PerforatedHypothesisResult; 4], indices: &mut [usize]) {
    indices.sort_by(|&left, &right| {
        hypotheses[left]
            .state
            .avg_loss
            .total_cmp(&hypotheses[right].state.avg_loss)
            .then_with(|| left.cmp(&right))
    });
}

/// `GoodFit` and `StablePose` are the only successful outcomes; `MaxIterations`
/// -- reaching the iteration cap without either -- is unconditionally a failed
/// hypothesis, and the hard-invalid states never reach this function's `true`
/// branch at all.  Delegates to [`termination_kind`] so this crate has exactly
/// one place that decides what counts as success.
fn successful_termination(state: &PerforatedIcpState, config: PerforatedIcpConfig) -> bool {
    matches!(
        termination_kind(state, config),
        IcpTermination::GoodFit | IcpTermination::StablePose
    )
}

/// Classifies an already-terminal state (one for which
/// [`PerforatedBoardIcpIterator::should_terminate`] returned `true`) into the
/// single reason it stopped.  Precedence: hard-invalid state first (too few
/// inlier points, too few points for Kabsch, no correspondences), then
/// `GoodFit`, then `StablePose`; the final `else` is only reached once all of
/// those are ruled out, which -- given the precondition -- means the
/// iteration cap was hit.  Keep this in lockstep with `should_terminate`: the
/// two must classify every state the same way.
fn termination_kind(state: &PerforatedIcpState, config: PerforatedIcpConfig) -> IcpTermination {
    if state.inlier_points.len() < config.min_inlier_points {
        IcpTermination::TooFewInliers
    } else if state.good_correspondences < 3 {
        IcpTermination::TooFewKabschPoints
    } else if state.correspondences.is_empty() {
        IcpTermination::NoCorrespondences
    } else if state.avg_loss < config.good_fit_threshold_m {
        IcpTermination::GoodFit
    } else if state.termination_count >= config.stable_pose_iterations {
        IcpTermination::StablePose
    } else {
        IcpTermination::MaxIterations
    }
}

fn pose_from_candidate(observation: &TargetSquarePlaneObservation, index: usize) -> Isometry3<f64> {
    let candidate = &observation.board_up_candidates[index];
    let rotation = Matrix3::from_columns(&[
        candidate.x_axis.into_inner(),
        candidate.board_up.into_inner(),
        candidate.z_axis.into_inner(),
    ]);
    Isometry3::from_parts(
        Translation3::from(observation.center.coords),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
    )
}

fn cutout_rim_correspondences(
    target: &ValidatedTarget,
    state: &PerforatedIcpState,
    tolerance_m: f64,
) -> usize {
    let Surface::Perforated { circular_cutouts } = &target.plate().surface else {
        return 0;
    };
    state
        .correspondences
        .iter()
        .filter(|(input, _)| {
            observed_point_is_on_cutout_rim(state.board_pose, input, circular_cutouts, tolerance_m)
        })
        .count()
}

fn observed_point_is_on_cutout_rim(
    pose: Isometry3<f64>,
    input: &Point3<f64>,
    cutouts: &[CircularCutout],
    tolerance_m: f64,
) -> bool {
    let local = pose.inverse_transform_point(input);
    cutouts.iter().any(|cutout| {
        let center = Point3::new(
            cutout.x_um as f64 / 1_000_000.0,
            cutout.y_um as f64 / 1_000_000.0,
            0.0,
        );
        let radial_error =
            (local.x - center.x).hypot(local.y - center.y) - cutout.radius_um as f64 / 1_000_000.0;
        radial_error.hypot(local.z) <= tolerance_m
    })
}

fn kabsch_transform(input: &[Point3<f64>], target: &[Point3<f64>]) -> Option<Isometry3<f64>> {
    if input.len() != target.len() || input.len() < 3 {
        return None;
    }
    let centroid = |points: &[Point3<f64>]| -> Point3<f64> {
        Point3::from(
            points
                .iter()
                .fold(Vector3::zeros(), |sum, point| sum + point.coords)
                / points.len() as f64,
        )
    };
    let input_centroid = centroid(input);
    let target_centroid = centroid(target);
    let centered_input: Vec<_> = input.iter().map(|point| point - input_centroid).collect();
    let centered_target: Vec<_> = target.iter().map(|point| point - target_centroid).collect();
    let covariance = nalgebra::Matrix3xX::from_columns(&centered_input)
        * nalgebra::Matrix3xX::from_columns(&centered_target).transpose();
    let svd = nalgebra::SVD::new(covariance, true, true);
    let u = svd.u?;
    let v = svd.v_t?.transpose();
    let u_t = u.transpose();
    let determinant = (v * u_t).determinant();
    let correction = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, determinant.signum()));
    let rotation = v * correction * u_t;
    let rotation = UnitQuaternion::from_matrix(&rotation.fixed_view::<3, 3>(0, 0).into_owned());
    let translation = Translation3::from(target_centroid.coords - rotation * input_centroid.coords);
    Some(Isometry3 {
        rotation,
        translation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use board_cluster_detector::{geometry::PlaneModel, square_fit::SquareFit};

    const HOLLOW: &[u8] = include_bytes!("../../../fixtures/targets/hollow_1000_aruco_4_v1.json5");

    fn target() -> ValidatedTarget {
        ValidatedTarget::parse_json5(HOLLOW).unwrap()
    }

    fn config() -> PerforatedIcpConfig {
        PerforatedIcpConfig {
            max_iterations: 40,
            outlier_threshold_m: 0.2,
            damping_factor: 0.5,
            pose_weight_threshold: 1e-8,
            stable_pose_iterations: 3,
            // Tight: this crate's only termination threshold now, so it must
            // do the job the old `rejection_threshold_m` did for these tests
            // (drive the loop to near-exact convergence on noiseless synthetic
            // data) rather than the old, much looser `good_fit_threshold_m`
            // final-gate value, which the loop itself never used to enforce.
            good_fit_threshold_m: 1e-8,
            min_inlier_points: 3,
            min_hypothesis_loss_separation_m: 0.0001,
            min_cutout_rim_correspondences: 1,
            cutout_rim_tolerance_m: 1e-5,
        }
    }

    fn structural_points() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.1, 0.0, 1.0),
            Point3::new(0.0, 0.1, 1.0),
            Point3::new(0.1, 0.1, 1.0),
        ]
    }

    #[test]
    fn initial_state_requires_a_step_before_termination_check() {
        let target = target();
        let iterator = PerforatedBoardIcpIterator::new(&target, config()).unwrap();
        let state = iterator.initial_state(Isometry3::identity(), structural_points());
        assert!(iterator.should_terminate(&state));
        assert_eq!(
            iterator.termination_reason(&state),
            "Insufficient points for Kabsch: 0"
        );
        let next = iterator.step(&state);
        assert_eq!(next.iteration, 1);
        assert_eq!(next.total_correspondences, 4);
        assert_eq!(next.good_correspondences, 0);
        assert!(next.correspondences.is_empty());
        // The new target uses the manifest's 1 m geometry, so these points are
        // exactly 1 m off its physical plane.  The state transitions themselves
        // are the legacy golden contract; this numeric golden pins the new surface.
        assert_relative_eq!(next.avg_loss, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn termination_order_and_stable_pose_boundary_match_legacy_contract() {
        let target = target();
        let mut cfg = config();
        cfg.max_iterations = 10;
        cfg.stable_pose_iterations = 3;
        let iterator = PerforatedBoardIcpIterator::new(&target, cfg).unwrap();
        let mut state = iterator.initial_state(Isometry3::identity(), structural_points());
        state.iteration = 9;
        state.avg_loss = cfg.good_fit_threshold_m;
        state.total_correspondences = 4;
        state.good_correspondences = 4;
        state.correspondences = vec![(Point3::origin(), Point3::origin()); 4];
        state.termination_count = cfg.stable_pose_iterations - 1;
        assert!(!iterator.should_terminate(&state));
        assert_eq!(iterator.termination_reason(&state), "Unknown");
        state.termination_count = cfg.stable_pose_iterations;
        assert_eq!(
            iterator.termination_reason(&state),
            "Converged (stable pose)"
        );
        state.avg_loss = cfg.good_fit_threshold_m / 2.0;
        state.iteration = cfg.max_iterations;
        assert_eq!(iterator.termination_reason(&state), "Converged (good fit)");
    }

    #[test]
    fn manifest_icp_step_keeps_the_legacy_hollow_characterization_golden() {
        // The old/new parity fixture, formerly compared against
        // `hollow-board-detector::BoardIcpIterator`.  That crate is gone as of
        // W5-E2, so this pins the exact legacy scene's public-neutral step
        // metrics directly.
        //
        // Note what this golden does NOT cover: it pins per-step metrics that
        // are computed before the pose update, so it is blind to the direction
        // that update is applied in.  It passed unchanged across the H-14 sign
        // fix.  Convergence from a perturbed seed is covered separately, in
        // tests/perforated_convergence.rs.
        let target = target();
        let mut cfg = config();
        cfg.outlier_threshold_m = 2.0;
        let iterator = PerforatedBoardIcpIterator::new(&target, cfg).unwrap();
        let points = vec![
            Point3::new(0.282_843, 0.03, 0.02),
            Point3::new(0.282_843, 0.10, 0.02),
            Point3::new(0.03, 0.282_843, 0.02),
            Point3::new(-0.282_843, -0.03, 0.02),
            Point3::new(0.0, -0.30, 0.02),
        ];
        let state = iterator.step(&iterator.initial_state(Isometry3::identity(), points));
        assert_relative_eq!(state.avg_loss, 0.087_763_479_977_847_64, epsilon = 2e-6);
        assert_eq!(state.good_correspondences, 5);
        assert_eq!(state.correspondences.len(), 5);
    }

    #[test]
    fn symmetric_or_weak_cutout_evidence_never_falls_back_to_square_orientation() {
        let target = target();
        let observation = observation();
        let pose = pose_from_candidate(&observation, 2);
        let face_only = [
            Point3::new(-0.1, -0.1, 0.0),
            Point3::new(-0.1, 0.1, 0.0),
            Point3::new(0.1, -0.1, 0.0),
            Point3::new(0.1, 0.1, 0.0),
        ]
        .map(|point| pose.transform_point(&point))
        .to_vec();
        assert!(matches!(
            estimate_perforated_pose(&target, &observation, face_only, config()).unwrap_err(),
            PerforatedRejection::AmbiguousCutoutEvidence { .. }
                | PerforatedRejection::IcpFailure { .. }
        ));
    }

    #[test]
    fn asymmetric_cutout_evidence_selects_the_correct_quadrant() {
        let target = target();
        let observation = observation();
        let expected_index = 2;
        let expected_pose = pose_from_candidate(&observation, expected_index);
        let estimate = estimate_perforated_pose(
            &target,
            &observation,
            perforated_samples(&target, expected_pose),
            config(),
        )
        .unwrap();
        assert!(estimate.iteration_count > 0);
        assert_eq!(estimate.winning_candidate_index, expected_index);
        // Under this fixture's tight `good_fit_threshold_m`, only the correct
        // quadrant's initial pose actually converges (`GoodFit`); the other
        // three quarter turns exhaust `max_iterations` without settling and
        // are discarded as failed hypotheses before ranking. That leaves a
        // single successful hypothesis, so per the hypothesis policy it
        // publishes without a separation comparison: `None`, not a
        // synthetic/zero separation.
        assert_eq!(estimate.second_best_loss_m, None);
        assert_eq!(estimate.loss_separation_m, None);
        assert!(estimate.cutout_rim_correspondences >= config().min_cutout_rim_correspondences);
        assert_relative_eq!(
            estimate.pose.translation.vector,
            expected_pose.translation.vector,
            epsilon = 1e-9
        );
        assert_relative_eq!(
            estimate.pose.rotation,
            expected_pose.rotation,
            epsilon = 1e-9
        );
    }

    #[test]
    fn finite_failed_termination_states_do_not_become_pose_authority() {
        let target = target();
        let observation = observation();
        let pose = pose_from_candidate(&observation, 0);
        let too_few = [Point3::new(-0.1, 0.0, 0.0), Point3::new(0.1, 0.0, 0.0)]
            .map(|point| pose.transform_point(&point))
            .to_vec();
        assert!(matches!(
            estimate_perforated_pose(&target, &observation, too_few, config()).unwrap_err(),
            PerforatedRejection::IcpFailure { .. }
        ));

        let mut cfg = config();
        cfg.max_iterations = 1;
        let poor = perforated_samples(&target, pose)
            .into_iter()
            .map(|point| point + (pose * Vector3::z_axis()).into_inner() * 0.02)
            .collect();
        assert!(matches!(
            estimate_perforated_pose(&target, &observation, poor, cfg).unwrap_err(),
            PerforatedRejection::IcpFailure { .. }
        ));
    }

    #[test]
    fn max_iteration_exit_cannot_publish_despite_good_rims_and_separation() {
        let target = target();
        let observation = observation();
        let pose = pose_from_candidate(&observation, 2);
        let normal = (pose * Vector3::z_axis()).into_inner();
        let near_true = perforated_samples(&target, pose)
            .into_iter()
            .map(|point| point + normal * 1e-4)
            .collect();
        let mut cfg = config();
        cfg.max_iterations = 1;
        // The single permitted step's residual is bound below by the ~1e-4
        // normal offset (the pose update that would shrink it only lands on
        // the *next* step, which never runs). Tighter than that offset, so
        // this genuinely exercises a `MaxIterations` exit rather than one
        // that also happens to satisfy `GoodFit` on its last iteration -- per
        // the M-21 contract, a `GoodFit` residual on the final permitted
        // iteration must win over `MaxIterations`, which is exactly NOT what
        // this test wants to exercise.
        cfg.good_fit_threshold_m = 1e-5;
        // Keep observed-rim evidence valid despite the deliberate normal offset;
        // this regression is solely about failed termination becoming authority.
        cfg.cutout_rim_tolerance_m = 2e-4;
        assert!(matches!(
            estimate_perforated_pose(&target, &observation, near_true, cfg).unwrap_err(),
            PerforatedRejection::IcpFailure { .. }
        ));
    }

    #[test]
    fn each_initial_hypothesis_keeps_its_named_quadrant() {
        let observation = observation();
        for index in 0..4 {
            let pose = pose_from_candidate(&observation, index);
            let candidate = &observation.board_up_candidates[index];
            assert_relative_eq!(
                pose * Vector3::y_axis(),
                candidate.board_up,
                epsilon = 1e-12
            );
            assert_relative_eq!(pose * Vector3::x_axis(), candidate.x_axis, epsilon = 1e-12);
            assert_relative_eq!(pose * Vector3::z_axis(), candidate.z_axis, epsilon = 1e-12);
        }
    }

    fn perforated_samples(target: &ValidatedTarget, pose: Isometry3<f64>) -> Vec<Point3<f64>> {
        let Surface::Perforated { circular_cutouts } = &target.plate().surface else {
            unreachable!()
        };
        let half_diagonal = target.half_diagonal_m();
        let mut local = Vec::new();
        for xi in -16..=16 {
            for yi in -16..=16 {
                let x = xi as f64 * 0.04;
                let y = yi as f64 * 0.04;
                if x.abs() + y.abs() > half_diagonal - 0.01 {
                    continue;
                }
                if circular_cutouts.iter().any(|cutout| {
                    let dx = x - cutout.x_um as f64 / 1_000_000.0;
                    let dy = y - cutout.y_um as f64 / 1_000_000.0;
                    dx.hypot(dy) < cutout.radius_um as f64 / 1_000_000.0 + 0.002
                }) {
                    continue;
                }
                local.push(Point3::new(x, y, 0.0));
            }
        }
        for cutout in circular_cutouts {
            let center_x = cutout.x_um as f64 / 1_000_000.0;
            let center_y = cutout.y_um as f64 / 1_000_000.0;
            let radius = cutout.radius_um as f64 / 1_000_000.0;
            for sample in 0..32 {
                let angle = sample as f64 * std::f64::consts::TAU / 32.0;
                local.push(Point3::new(
                    center_x + radius * angle.cos(),
                    center_y + radius * angle.sin(),
                    0.0,
                ));
            }
        }
        local
            .into_iter()
            .map(|point| pose.transform_point(&point))
            .collect()
    }

    fn observation() -> TargetSquarePlaneObservation {
        let plane = PlaneModel {
            center: Point3::new(0.0, 0.0, 3.0),
            normal: Vector3::z(),
            u: Vector3::x(),
            v: Vector3::y(),
        };
        let half = 1.0 / std::f64::consts::SQRT_2;
        let square = SquareFit {
            center: [0.0, 0.0],
            theta: 0.0,
            residual: 0.0,
            corners_2d: [[half, 0.0], [0.0, half], [-half, 0.0], [0.0, -half]],
        };
        TargetSquarePlaneObservation::from_fitted_square(&plane, &square, Vector3::y()).unwrap()
    }
}
