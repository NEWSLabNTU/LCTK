//! ICP *convergence* coverage, migrated from the deleted
//! `hollow-board-detector::tests::test_icp_correctness` (W5-E2, Phase 8). Nothing
//! else in the repository asserts that cutout-aware ICP actually converges to the
//! right answer from a perturbed seed -- only that the iteration loop terminates,
//! or that one fixed step reproduces a golden number. These tests close that gap.
//!
//! `test_convergence_counter_increases` below is load-bearing beyond this crate:
//! its comments are the empirical basis for the open issue
//! `docs/issues/M-21-icp-stable-pose-exit-unreachable.md`, which cites the deleted
//! file by name as its evidence. The reasoning is preserved here; only the
//! deleted crate's specific numbers are not, because they described a different
//! model.
//!
//! **Why this file drives everything through [`TargetPoseEstimator`] rather
//! than a raw ICP iterator.** The deleted crate's tests called
//! `hollow_board_detector::algo::BoardIcpIterator::{initial_state,step,
//! should_terminate}` directly -- that type was `pub`. This crate's equivalent,
//! `PerforatedBoardIcpIterator`, lives in a private `mod perforated` and is not
//! re-exported (only `PerforatedIcpConfig` is, from `lib.rs`), so an integration
//! test in `tests/` cannot reach it -- only unit tests compiled inside the crate
//! can, which is exactly what `perforated.rs`'s own `#[cfg(test)] mod tests`
//! already exercises for the mechanical step/termination contract. So, matching
//! the public-interface style `tests/perforated_facade.rs` and `tests/solid.rs`
//! already use, every test here goes through `TargetPoseEstimator::estimate`,
//! constructing an observation whose fitted-square evidence places one of the
//! four quadrant hypotheses exactly at the desired seed pose (see
//! `observation_seeding_pose` below), and reading the converged result back off
//! `TargetDetection`/`CutoutIcpEvidence` instead of a raw `PerforatedIcpState`.
//!
//! **Why the manifest is 1 m here, not the deleted crate's 0.5 m.** The old tests
//! built their own 0.5 m `BoardShape` from the deleted crate's types. This crate
//! works from a [`ValidatedTarget`] parsed from a real target manifest, and the
//! shipped hollow target
//! (`ros/lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5`, mirrored here
//! as `fixtures/targets/hollow_1000_aruco_4_v1.json5`, same physical target, cutout
//! order shuffled) is 1 m. Every tolerance below was therefore re-measured on this
//! geometry by actually running the test and observing the behaviour; none of the
//! deleted file's numbers are reused, because they describe a plate that no longer
//! exists in this crate. The old prose reasoning for *why* each tolerance is
//! shaped the way it is is kept -- only the numerals changed.
//!
//! # A real defect these tests found (currently un-fixed; see the migration report)
//!
//! Four of the six tests below (all but the fixture guard and the seeded-at-truth
//! fixed-point test) currently **fail against shipped code**, and are left that
//! way deliberately rather than loosened to green. Driving them to failure
//! surfaced a genuine sign error in `PerforatedBoardIcpIterator::step`
//! (`src/perforated.rs`, the `let new_pose = align_pose.inverse() * current.board_pose;`
//! line): it should read `align_pose * current.board_pose`, with no `.inverse()`.
//!
//! As written, every ICP correction moves the pose *away* from the true fit
//! instead of toward it -- confirmed by cross-checking a byte-for-byte
//! reimplementation of `step`'s correspondence/outlier/Kabsch/damping logic
//! (built only from this crate's and `calibration-target`'s public API, so it
//! never touches the private iterator) with the sign flipped: identical inputs,
//! opposite sign, and the pose converges cleanly instead of diverging. The
//! deleted crate's own equivalent line inverts *its* Kabsch result too, but its
//! Kabsch helper is called with the (target, input) argument order swapped
//! relative to this crate's `kabsch_transform(model, observed)` -- so the old
//! crate's inversion was compensating for its own call order and the two are
//! mathematically equivalent, correct formulas that happen to look alike. This
//! crate's `step` calls `kabsch_transform` with the already-correct order and
//! then inverts on top of that, which flips the sign a second time. The
//! `// This inversion is intentional and matches the legacy iterator's
//! input/model ordering exactly.` comment above that line is the reasoning
//! error: the ordering matches, but because it already matches, no extra
//! `.inverse()` is needed.
//!
//! This is invisible to every ICP test that already existed before this file:
//! seeded exactly at the truth pose (as `perforated.rs`'s own
//! `asymmetric_cutout_evidence_selects_the_correct_quadrant` and this file's own
//! `test_identity_transformation_convergence` are), the Kabsch correction is the
//! identity transform, which is its own inverse -- so the sign of the bug never
//! shows up unless the seed is actually perturbed. That is exactly what this
//! packet exists to add.

use board_cluster_detector::{geometry::PlaneModel, square_fit::SquareFit};
use calibration_target::{Surface, ValidatedTarget};
use calibration_target_detector::{
    CutoutIcpEvidence, IcpTermination, PerforatedIcpConfig, TargetDetection,
    TargetDetectionDiagnostics, TargetPoseEstimate, TargetPoseEstimator, TargetPoseEstimatorTuning,
    TargetSquarePlaneObservation,
};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::{FRAC_1_SQRT_2, SQRT_2, TAU};

const HOLLOW: &[u8] = include_bytes!("../../../fixtures/targets/hollow_1000_aruco_4_v1.json5");

fn target() -> ValidatedTarget {
    ValidatedTarget::parse_json5(HOLLOW).unwrap()
}

/// The manifest's physical cutouts, as plain `(x_m, y_m, radius_m)` triples.
fn cutouts_m(target: &ValidatedTarget) -> Vec<(f64, f64, f64)> {
    let Surface::Perforated { circular_cutouts } = &target.plate().surface else {
        panic!("hollow manifest must be perforated");
    };
    circular_cutouts
        .iter()
        .map(|cutout| {
            (
                cutout.x_um as f64 / 1_000_000.0,
                cutout.y_um as f64 / 1_000_000.0,
                cutout.radius_um as f64 / 1_000_000.0,
            )
        })
        .collect()
}

/// Generates a synthetic sensor return for every point of a regular grid covering
/// the physical board, posed at `pose`, expressed in the world (sensor) frame.
///
/// Ported from the deleted crate's `create_grid_points`; the mapping and the
/// reasoning for it are unchanged -- only the plate dimensions now come from
/// `target` instead of a fixed 0.5 m constant.
///
/// **Why the 45-deg map, and why it must not be "simplified" away.** In the
/// target's canonical local frame the axes run corner to corner
/// (`local_left_corner` sits on `+X`, `local_top_corner` on `+Y`), so the plate is
/// not an axis-aligned square -- it is the *diamond* `|x| + |y| <= R`, with
/// `R = target.half_diagonal_m()`. A grid laid out directly over local
/// `[0, W] x [0, W]` leaves roughly half its points off the plate entirely,
/// silently feeding ICP data the target's own closest-point surface can never
/// explain (`SolidSquareSurface::closest_local` would just project them back onto
/// the boundary).
///
/// So the grid is laid out in the plate's **edge** coordinates `(u, v)`, each
/// spanning `[0, W]`, and rotated into the corner-aligned frame:
///
/// ```text
/// x = (u - v) / sqrt(2)
/// y = (u + v) / sqrt(2) - R
/// ```
///
/// Check the two anchors: `u = v = 0` gives `(0, -R)`, the bottom corner, and
/// `u = W, v = 0` gives `(+R, 0)`, the left corner. The map is a rotation
/// followed by a translation, so it covers the diamond exactly and uniformly.
///
/// Points falling inside one of the three cutouts are dropped: the physical
/// board has no material there, so a real sensor returns nothing. Keeping them
/// would both floor the achievable ICP residual at the hole radius and blunt the
/// only feature that resolves the plate's four-fold rotational symmetry.
fn create_grid_points(
    target: &ValidatedTarget,
    pose: &Isometry3<f64>,
    grid_size: usize,
) -> Vec<Point3<f64>> {
    let half_diagonal = target.half_diagonal_m();
    let width = half_diagonal * SQRT_2;
    let step = width / (grid_size - 1) as f64;
    let holes = cutouts_m(target);

    (0..grid_size)
        .flat_map(|i| (0..grid_size).map(move |j| (i as f64 * step, j as f64 * step)))
        .map(|(u, v)| {
            (
                (u - v) * FRAC_1_SQRT_2,
                (u + v) * FRAC_1_SQRT_2 - half_diagonal,
            )
        })
        .filter(|&(x, y)| {
            // The margin drops points sitting *exactly* on a rim as well. A grid
            // step can land there to the last bit, and whether such a point then
            // reads as just inside or just outside depends on which way the pose's
            // round-off falls -- an ambiguity no test should have to reason about.
            const RIM_MARGIN: f64 = 1e-6;
            !holes
                .iter()
                .any(|&(hx, hy, hr)| (x - hx).hypot(y - hy) < hr + RIM_MARGIN)
        })
        .map(|(x, y)| pose.transform_point(&Point3::new(x, y, 0.0)))
        .collect()
}

/// A ring of points sampled exactly on each cutout's physical rim, posed at
/// `pose`. Mirrors the combined grid-plus-rim pattern `tests/perforated_facade.rs`
/// (`samples`) and `perforated.rs`'s own inline test fixture (`perforated_samples`)
/// both already use: `create_grid_points` alone lands close to a rim only by luck
/// of grid alignment, and the cutout-rim evidence gate
/// (`min_cutout_rim_correspondences`) needs correspondences it can actually find.
fn cutout_rim_samples(target: &ValidatedTarget, pose: &Isometry3<f64>) -> Vec<Point3<f64>> {
    const RIM_SAMPLES: usize = 32;
    cutouts_m(target)
        .into_iter()
        .flat_map(|(cx, cy, radius)| {
            (0..RIM_SAMPLES).map(move |sample| {
                let angle = sample as f64 * TAU / RIM_SAMPLES as f64;
                (cx + radius * angle.cos(), cy + radius * angle.sin())
            })
        })
        .map(|(x, y)| pose.transform_point(&Point3::new(x, y, 0.0)))
        .collect()
}

/// Evidence points for the ICP convergence tests below: the diamond grid, plus
/// exact rim samples so cutout evidence is always found.
fn evidence_points(
    target: &ValidatedTarget,
    pose: &Isometry3<f64>,
    grid_size: usize,
) -> Vec<Point3<f64>> {
    let mut points = create_grid_points(target, pose, grid_size);
    points.extend(cutout_rim_samples(target, pose));
    points
}

/// The grid density the convergence tests below sample at: `1.0 m / 80 = 12.5 mm`,
/// the same absolute step the deleted crate's highest-value test used on its
/// 0.5 m board (`W / 40`). Sample spacing is the discretisation floor a
/// point-to-model ICP cannot localise the plate more finely than, so keeping the
/// same absolute density keeps that reasoning intact on the new geometry.
const GRID_SIZE: usize = 81;

/// Worst-case distance between the four corners of the target posed at `a` and at
/// `b`.
///
/// Corners rather than the raw `Isometry3`, because they are what consumers
/// downstream read, and because a rotation about the board normal -- the one
/// degree of freedom only the three-cutout asymmetry witnesses -- moves them
/// while leaving the translation intact.
fn max_corner_error(target: &ValidatedTarget, a: Isometry3<f64>, b: Isometry3<f64>) -> f64 {
    let (posed_a, posed_b) = (target.posed(a), target.posed(b));
    [
        (posed_a.top_corner() - posed_b.top_corner()).norm(),
        (posed_a.bottom_corner() - posed_b.bottom_corner()).norm(),
        (posed_a.left_corner() - posed_b.left_corner()).norm(),
        (posed_a.right_corner() - posed_b.right_corner()).norm(),
    ]
    .into_iter()
    .fold(0.0f64, f64::max)
}

/// Builds a `TargetSquarePlaneObservation` whose quadrant-1 hypothesis
/// (`board_up_candidates[1]`, the fitted corner opposite the diamond's `+Y`
/// anchor) is *exactly* `seed` -- so feeding it plus points generated from a
/// nearby `truth` pose to `TargetPoseEstimator::estimate` drives that hypothesis's
/// ICP from `seed` toward `truth`, the same experiment the deleted crate ran by
/// calling `BoardIcpIterator::initial_state(seed, ...)` directly.
///
/// The derivation: `TargetSquarePlaneObservation::from_fitted_square` builds
/// candidate 1's frame as `x_axis = board_up x normal`, `board_up = seed * +Y`,
/// `normal = plane.normal` (after its sensor-facing flip), translation =
/// `plane.center`. Passing `plane.{center,u,v,normal} = seed * {translation,
/// +X,+Y,+Z}` and `square_fit.corners_2d` built from `target.half_diagonal_m()`
/// therefore reproduces `seed` exactly for candidate 1, **provided the flip does
/// not fire** -- `from_fitted_square` always turns the final normal to face the
/// origin, so `seed`'s own `+Z` must already point back at the origin or the
/// reproduced pose would silently be a different (mirrored) one. `facing_origin`
/// below asserts this so a badly chosen seed/truth pair fails loudly here rather
/// than passing for the wrong quadrant.
fn observation_seeding_pose(
    target: &ValidatedTarget,
    seed: Isometry3<f64>,
) -> TargetSquarePlaneObservation {
    let normal = (seed * Vector3::z_axis()).into_inner();
    let center_to_origin = -seed.translation.vector;
    assert!(
        normal.dot(&center_to_origin) >= 0.0,
        "seed pose's +Z must already face the origin, or from_fitted_square's \
         sensor-facing flip silently reproduces a mirrored quadrant instead of \
         `seed`"
    );
    let half = target.half_diagonal_m();
    let plane = PlaneModel {
        center: Point3::from(seed.translation.vector),
        normal,
        u: (seed * Vector3::x_axis()).into_inner(),
        v: (seed * Vector3::y_axis()).into_inner(),
    };
    let square = SquareFit {
        center: [0.0, 0.0],
        theta: 0.0,
        residual: 0.0,
        corners_2d: [[half, 0.0], [0.0, half], [-half, 0.0], [0.0, -half]],
    };
    // `sensor_up` only feeds `TargetSquarePlaneObservation::orientation`, which
    // `estimate_perforated_pose` never reads (it races all four quadrant
    // hypotheses directly); any finite, non-zero vector is fine here.
    TargetSquarePlaneObservation::from_fitted_square(&plane, &square, Vector3::y())
        .expect("plane/square inputs are finite and non-degenerate by construction")
}

/// Tuning shared by the convergence tests.
///
/// `good_fit_threshold_m` is the parameter to get right. As of the M-21
/// termination cleanup it is also the iterator's own early-exit threshold, not
/// a separate post-ICP gate: `estimate_perforated_pose` refuses to publish a
/// detection at all unless the winning hypothesis stopped via `avg_loss <
/// good_fit_threshold_m` ("good fit") or `termination_count >=
/// stable_pose_iterations` ("stable pose") -- hitting `max_iterations` is
/// never accepted, however good the final residual is
/// (`max_iteration_exit_cannot_publish_despite_good_rims_and_separation` in
/// `perforated.rs` pins exactly this). The stable-pose exit needs on the order
/// of 1800 iterations on this manifest (see `test_convergence_counter_increases`),
/// far more than a seeded-correction test needs to run, so `good_fit_threshold_m`
/// has to be loose enough that the *good-fit* exit is actually reached within
/// `max_iterations` -- set it too tight (as an earlier draft of this file did,
/// mirroring the deleted crate's `create_convergence_config`, which zeroed the
/// equivalent threshold because its crate had no such publish gate at all) and
/// every one of these tests spuriously rejects with `MaxIterations`, regardless
/// of how good the pose actually is.
///
/// `stable_pose_iterations` is fixed here at 100, matching the *magnitude* of
/// the legacy hard-coded `termination_count > 100` boundary M-21 found
/// unreachable at any shipped iteration budget -- not the exact comparison.
/// The new state machine's `termination_count >= stable_pose_iterations`
/// fires one iteration earlier than the legacy `termination_count > 100`
/// would (`>= 100` vs `> 100`), a fixed one-iteration difference immaterial
/// at the ~1800-iteration scale `test_convergence_counter_increases` measures
/// below, so the iteration counts quoted in comments there stay accurate.
/// Only `good_fit_threshold_m` varies per test.
fn convergence_tuning(max_iterations: usize, good_fit_threshold_m: f64) -> PerforatedIcpConfig {
    PerforatedIcpConfig::new(
        max_iterations,
        0.2, // outlier_threshold_m: generous; correspondences from a seed off
        // by centimetres must not be rejected as outliers before ICP gets a
        // chance to pull them in.
        0.5,  // damping_factor
        1e-9, // pose_weight_threshold: tight, so the stable-pose exit is a
        // late-iteration event rather than an early false-positive.
        100, // stable_pose_iterations: see the doc comment above.
        good_fit_threshold_m,
        3,    // min_inlier_points
        1e-6, // min_hypothesis_loss_separation_m
        1,    // min_cutout_rim_correspondences
        1e-4, // cutout_rim_tolerance_m: rim samples are exact by construction;
              // this only needs to survive ICP's residual float error, not grid
              // discretisation.
    )
}

fn estimator(target: &ValidatedTarget, config: PerforatedIcpConfig) -> TargetPoseEstimator {
    TargetPoseEstimator::new(target, TargetPoseEstimatorTuning::for_perforated(config)).unwrap()
}

fn detected(estimate: TargetPoseEstimate) -> TargetDetection {
    match estimate {
        TargetPoseEstimate::Detected(detection) => *detection,
        TargetPoseEstimate::Rejected(rejection) => panic!("expected detection, got {rejection:?}"),
    }
}

fn cutout_diagnostics(detection: &TargetDetection) -> &CutoutIcpEvidence {
    let TargetDetectionDiagnostics::CutoutIcp(evidence) = &detection.diagnostics else {
        panic!("expected cutout ICP diagnostics")
    };
    evidence
}

/// Guards the fixture itself. Every generated grid point must lie on the physical
/// board -- inside the diamond `|x| + |y| <= R` and outside all three cutouts --
/// and the extremes of the `(u, v)` sweep must reach the plate's corners, so the
/// grid genuinely covers the plate rather than a shrunken patch of it. A
/// convergence test whose fixture is wrong proves nothing, so this guard migrates
/// with the generator.
#[test]
fn grid_points_all_lie_on_the_physical_board() {
    let target = target();
    let half_diagonal = target.half_diagonal_m();
    let holes = cutouts_m(&target);
    let pose = Isometry3::from_parts(
        Translation3::new(1.5, -0.4, -2.0),
        UnitQuaternion::from_euler_angles(0.3, -0.5, 0.9),
    );
    let posed = target.posed(pose);
    let points = create_grid_points(&target, &pose, GRID_SIZE);

    assert!(!points.is_empty(), "fixture must produce points");

    // The grid steps by W/80 here, so the corners are hit exactly by
    // construction; the slack absorbs f64 round-off in the 45-deg map only.
    const EPS: f64 = 1e-9;
    let mut max_reach: f64 = 0.0;

    for point in &points {
        let local = pose.inverse_transform_point(point);
        let (x, y) = (local.x, local.y);

        let l1 = x.abs() + y.abs();
        assert!(
            l1 <= half_diagonal + EPS,
            "point at local ({x}, {y}) is off the plate: |x|+|y| = {l1} > R = {half_diagonal}"
        );
        max_reach = max_reach.max(l1);

        for &(hx, hy, hr) in &holes {
            let radius = (x - hx).hypot(y - hy);
            assert!(
                radius >= hr,
                "point at local ({x}, {y}) sits inside the cutout at ({hx}, {hy}), \
                 {radius} m from its centre"
            );
        }

        // The plate is flat, so nothing may leave the board plane.
        let out_of_plane = (point - posed.center()).dot(&posed.z_axis());
        assert!(
            out_of_plane.abs() < EPS,
            "point at local ({x}, {y}) is {out_of_plane} m off the board plane"
        );
    }

    assert!(
        (max_reach - half_diagonal).abs() < EPS,
        "the grid should reach the plate's corners: max |x|+|y| = {max_reach}, R = {half_diagonal}"
    );
}

/// The assertion the deleted crate's file was missing for years: not "the loop
/// terminated" but "the loop terminated *somewhere true*".
///
/// The seed hypothesis is deliberately displaced from the truth pose that
/// generated the points -- in-plane translation plus an in-plane rotation, the
/// degree of freedom the three-cutout asymmetry is the only witness to -- and the
/// converged detection's four corner accessors are then required to land on the
/// true corners.
///
/// Corner positions rather than the pose itself, because they are what every
/// consumer downstream actually reads, and because a convention error that
/// leaves the pose numerically plausible still moves the corners.
#[test]
fn converged_pose_lands_the_corners_on_the_true_corners() {
    let target = target();

    let truth = Isometry3::from_parts(
        Translation3::new(0.6, -0.3, -2.5),
        UnitQuaternion::from_euler_angles(0.15, -0.25, 0.4),
    );
    // Perturb in the board's own frame: 2 cm across the plate and 4 deg of roll
    // about its normal. Both are far larger than the tolerance asserted below, so
    // passing cannot be an artefact of starting close enough.
    let perturbation = Isometry3::from_parts(
        Translation3::new(0.02, -0.015, 0.01),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 4f64.to_radians()),
    );
    let seed = truth * perturbation;

    let seed_error = max_corner_error(&target, seed, truth);

    let points = evidence_points(&target, &truth, GRID_SIZE);
    let observation = observation_seeding_pose(&target, seed);
    // 2000 iterations, `good_fit_threshold_m = 1e-6`: the good-fit exit needs
    // real headroom on this manifest (measured against a corrected reference
    // implementation -- see the top-of-file bug writeup -- the seeded hypothesis
    // needs ~824 iterations to cross `1e-6`; a tighter threshold or a smaller cap
    // makes the run miss the publish gate entirely and report `PerforatedIcpFailure`
    // even though the pose is fine).
    let config = convergence_tuning(2000, 1e-6);
    let detection = detected(estimator(&target, config).estimate(observation, points));

    // Sanity: this must be the seeded hypothesis, not some other quadrant that
    // happened to fit better.
    assert_eq!(detection.selected_quadrant, 1);

    let error = max_corner_error(&target, detection.pose, truth);

    // Tolerance. Measured worst-corner error against the corrected reference
    // implementation on this 1 m manifest, this grid and this seed: 8.9e-5 m,
    // reached after 824 iterations, against a ~7.1 cm seed corner error. 1 mm
    // sits > 10x above the measured value (loose enough not to flake on an
    // incidental property of this exact grid) and ~800x below the seed, so a
    // genuine failure to converge cannot slip through.
    const CORNER_TOL: f64 = 1e-3;
    assert!(
        error < CORNER_TOL,
        "corners ended {error:e} m from truth (seed was {seed_error:e} m off), \
         tolerance {CORNER_TOL} m"
    );

    // A converged fit must also *explain* the points; a pose that happens to sit
    // near the truth while the residual is still large would mean the two agreed
    // by accident.
    let avg_loss = cutout_diagnostics(&detection).best_loss_m;
    assert!(
        avg_loss < CORNER_TOL,
        "average point-to-model residual {avg_loss} m should be within the corner tolerance"
    );
}

/// Seeded *at* the truth, ICP must be a fixed point: the pose must not drift away.
///
/// The failure this guards against is a correspondence or Kabsch convention error
/// that pushes the model off a pose that already explains every point perfectly
/// -- a bug that a "did the loop end?" assertion cannot see, and that a test
/// seeded away from truth can mask by converging to the wrong place from both
/// sides.
#[test]
fn test_identity_transformation_convergence() {
    let target = target();

    // Off the origin and off-axis, so a drift along any single axis shows up.
    let truth = Isometry3::from_parts(
        Translation3::new(0.8, -0.2, -2.2),
        UnitQuaternion::from_euler_angles(0.1, -0.2, 0.35),
    );
    let points = evidence_points(&target, &truth, GRID_SIZE);
    let observation = observation_seeding_pose(&target, truth);
    // Seeded exactly at the truth, the very first correspondence pass already
    // has (near) zero residual, so `good_fit_threshold_m = 1e-9` is reached on
    // the first step regardless of `max_iterations`; 25 is only a safety cap.
    let config = convergence_tuning(25, 1e-9);
    let detection = detected(estimator(&target, config).estimate(observation, points));

    assert_eq!(detection.selected_quadrant, 1);

    // Tolerance. measured worst-corner drift and residual are reported in the
    // assertion messages below on failure; DRIFT_TOL is set well above f64
    // round-off but far below the offsets the recovery/rotation tests below seed,
    // so a genuine "seeded at truth, walks away" bug cannot slip past it while it
    // stays tight enough not to mask one.
    const DRIFT_TOL: f64 = 1e-6;
    let drift = max_corner_error(&target, detection.pose, truth);
    assert!(
        drift < DRIFT_TOL,
        "pose seeded at the truth drifted {drift:e} m (worst corner), tolerance {DRIFT_TOL} m"
    );

    let avg_loss = cutout_diagnostics(&detection).best_loss_m;
    assert!(
        avg_loss < DRIFT_TOL,
        "average point-to-model residual {avg_loss} m should stay at the sampling floor"
    );
}

/// Seeded with a translation offset, ICP must *reduce* it -- the assertion the
/// deleted crate's original version of this test never made.
#[test]
fn test_small_translation_recovery() {
    let target = target();

    let truth = Isometry3::from_parts(
        Translation3::new(0.5, -0.25, -2.0),
        UnitQuaternion::from_euler_angles(0.12, -0.2, 0.3),
    );
    // Perturb in the *board's* frame: 3 cm across the plate and 1 cm along its
    // normal, so both the in-plane degrees of freedom (constrained by the edges
    // and hole rims) and the out-of-plane one (constrained by the plane fit) are
    // exercised.
    let seed = truth
        * Isometry3::from_parts(
            Translation3::new(0.03, -0.02, 0.01),
            UnitQuaternion::identity(),
        );

    let seed_error = (seed.translation.vector - truth.translation.vector).norm();
    // Sanity: the seed must really be displaced, or "reduced the error" is
    // vacuous.
    assert!(
        seed_error > 0.03,
        "seed must start materially off truth, got {seed_error:e} m"
    );

    let points = evidence_points(&target, &truth, GRID_SIZE);
    let observation = observation_seeding_pose(&target, seed);
    // See `converged_pose_lands_the_corners_on_the_true_corners` for why 2000
    // iterations and `1e-6` (not the deleted crate's zeroed threshold): the
    // seeded hypothesis needs real headroom to cross the good-fit publish gate.
    let config = convergence_tuning(2000, 1e-6);
    let detection = detected(estimator(&target, config).estimate(observation, points));
    assert_eq!(detection.selected_quadrant, 1);

    let final_error = (detection.pose.translation.vector - truth.translation.vector).norm();

    // Tolerance. Measured against the corrected reference implementation: seed
    // error 3.74e-2 m -> final error 2.8e-5 m after 682 iterations. 1 mm sits
    // ~36x above the measured value and ~37x below the seed error, so nothing
    // short of genuine convergence clears it.
    const TRANSLATION_TOL: f64 = 1e-3;
    assert!(
        final_error < TRANSLATION_TOL,
        "translation error only went {seed_error:e} m -> {final_error:e} m, \
         tolerance {TRANSLATION_TOL} m"
    );
    // Stated separately so a regression that merely *stops improving* still
    // reads as one.
    assert!(
        final_error < seed_error / 10.0,
        "ICP must materially reduce the seeded offset: {seed_error:e} m -> {final_error:e} m"
    );
}

/// Seeded with an in-plane rotation -- about the board's own normal -- ICP must
/// recover it.
///
/// That axis specifically: the plate is a diamond with four-fold rotational
/// symmetry, so rotation about the normal is invisible to the plate outline and
/// to the plane fit. The three cutouts' asymmetry is its *only* witness, which
/// makes this the degree of freedom most worth asserting on and the one a
/// correspondence bug is likeliest to lose.
#[test]
fn test_small_rotation_handling() {
    let target = target();

    let truth = Isometry3::from_parts(
        Translation3::new(0.4, 0.15, -1.8),
        UnitQuaternion::from_euler_angles(-0.1, 0.22, 0.5),
    );
    // 5 deg about the local +Z (the board normal). Small enough that the cutouts
    // still overlap their true positions -- a larger angle would let the fit
    // slide toward the 90-deg-apart symmetric alias instead of back to truth,
    // which is a separate question from "does it handle a small rotation".
    const SEED_ANGLE_DEG: f64 = 5.0;
    let seed = truth
        * Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), SEED_ANGLE_DEG.to_radians()),
        );

    let seed_angle = seed.rotation.angle_to(&truth.rotation);
    let seed_corner_error = max_corner_error(&target, seed, truth);

    let points = evidence_points(&target, &truth, GRID_SIZE);
    let observation = observation_seeding_pose(&target, seed);
    // See `converged_pose_lands_the_corners_on_the_true_corners` for why 2000
    // iterations and `1e-6`.
    let config = convergence_tuning(2000, 1e-6);
    let detection = detected(estimator(&target, config).estimate(observation, points));
    assert_eq!(detection.selected_quadrant, 1);

    let final_angle = detection.pose.rotation.angle_to(&truth.rotation);
    let final_corner_error = max_corner_error(&target, detection.pose, truth);

    // Tolerance. Measured against the corrected reference implementation: seed
    // angle 8.7e-2 rad (5 deg) -> final residual 1.0e-4 rad after 836 iterations,
    // an ~840x reduction. 1e-3 rad sits ~10x above the measured value (in-plane
    // angle is resolved only by the hole rims, the coarsest evidence in the
    // fixture, and is the slowest-converging degree of freedom here) and ~87x
    // below the seed.
    const ANGLE_TOL: f64 = 1e-3;
    assert!(
        final_angle < ANGLE_TOL,
        "in-plane rotation error only went {seed_angle:e} rad -> {final_angle:e} rad, \
         tolerance {ANGLE_TOL} rad"
    );
    // The angle alone can look small while the board sits elsewhere; corners pin
    // the pose the way consumers see it. Measured 8.9e-5 m against a 6.2e-2 m
    // seed corner error.
    const CORNER_TOL: f64 = 1e-3;
    assert!(
        final_corner_error < CORNER_TOL,
        "corners ended {final_corner_error:e} m from truth (seed was {seed_corner_error:e} m \
         off), tolerance {CORNER_TOL} m"
    );
}

/// The two things the deleted crate's `BoardIcpState` carried -- `iteration` and
/// `termination_count` -- have no public equivalent through this crate's facade:
/// `CutoutIcpEvidence` reports only the *final* `iteration_count` and the
/// structured `termination` reason a hypothesis stopped for. So this test asks
/// the same underlying question a different way: given enough iterations, does
/// the loop ever actually stop via `IcpTermination::StablePose` (which the
/// private state machine reaches at `termination_count > 100`) rather than
/// always hitting the iteration cap or the (here, unreachable) good-fit exit
/// first?
///
/// **Why this matters beyond this crate.** The deleted file measured, by
/// instrumenting the private iterator directly, that the per-step pose weight on
/// its 0.5 m board shrinks by a roughly constant factor per step -- geometric but
/// slow convergence, because interior grid points project onto the plane exactly
/// where they already are and say nothing about in-plane pose, leaving only the
/// edge and hole-rim samples to drive it -- so reaching `pose_weight_threshold`
/// took roughly 538 steps, and the *stable-pose* exit (100 more quiet steps after
/// that) took roughly 639. That observation is `M-21`'s evidence that with any
/// realistic `max_icp_iterations`, the stable-pose exit is unreachable in
/// production: real runs always leave via the residual gate instead. This crate's
/// private iterator cannot be instrumented from here the same way, but the
/// question it answers is unchanged, so this test answers it empirically instead:
/// it raises `max_iterations` until `StablePose` is observed at all, and reports
/// how many iterations that took on this 1 m manifest -- corroborating, on new
/// geometry, the same "several hundred steps" order of magnitude M-21 relies on.
/// Measured against the corrected reference implementation (see the top-of-file
/// bug writeup): on this manifest and seed, the stable-pose exit needs ~1809
/// iterations -- the same order of magnitude as the deleted crate's ~639, and
/// still far beyond any shipped preset's iteration cap, so M-21's conclusion
/// holds on the new geometry too.
#[test]
fn test_convergence_counter_increases() {
    let target = target();

    let truth = Isometry3::from_parts(
        Translation3::new(0.3, -0.1, -1.6),
        UnitQuaternion::from_euler_angles(0.05, -0.15, 0.25),
    );
    // Seeded off truth, so the run genuinely has to move before it can go quiet:
    // starting *at* truth would let the stable-pose exit fire trivially on the
    // first check and prove nothing about a converging run reaching it.
    let seed = truth
        * Isometry3::from_parts(
            Translation3::new(0.02, 0.015, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 3f64.to_radians()),
        );

    let points = evidence_points(&target, &truth, GRID_SIZE);
    let observation = observation_seeding_pose(&target, seed);

    // 5000, not the 2000 the other tests use: reaching the stable-pose exit
    // needs materially more steps than reaching a good corner fit does (see the
    // M-21 reasoning above; measured ~1809 iterations), so the other tests' cap
    // is not enough to observe it. `good_fit_threshold_m = 0.0` is deliberate,
    // not just "small": at any positive threshold this manifest's good-fit exit
    // fires first (measured: `1e-9` reaches good-fit at ~1724 iterations, before
    // the stable-pose exit would), which would test the wrong exit path entirely.
    const MAX_ITERATIONS: usize = 5000;
    let config = convergence_tuning(MAX_ITERATIONS, 0.0);
    let detection = detected(estimator(&target, config).estimate(observation, points));
    assert_eq!(detection.selected_quadrant, 1);

    let evidence = cutout_diagnostics(&detection);
    assert!(
        evidence.iteration_count > 1,
        "the run must take more than one real step, got {}",
        evidence.iteration_count
    );
    assert!(
        evidence.iteration_count < MAX_ITERATIONS,
        "expected the stable-pose exit to fire before the {MAX_ITERATIONS}-iteration cap, \
         but the run used every iteration ({}); M-21's premise -- that this exit is \
         reachable at all, just not at realistic caps -- would not hold on this manifest",
        evidence.iteration_count
    );
    assert_eq!(
        evidence.termination,
        IcpTermination::StablePose,
        "expected the run to leave via the stable-pose exit after {} iterations, got {:?}; \
         M-21 is specifically about this exit being reachable only with an unrealistically \
         high iteration cap",
        evidence.iteration_count,
        evidence.termination
    );

    // Convergence, not just quiescence: a pose that froze somewhere wrong would
    // also stop moving.
    const CORNER_TOL: f64 = 1e-3;
    let corner_error = max_corner_error(&target, detection.pose, truth);
    assert!(
        corner_error < CORNER_TOL,
        "stable-pose exit fired but the pose is {corner_error:e} m off truth \
         (tolerance {CORNER_TOL} m)"
    );
}
