use hollow_board_config::{BoardModel, BoardShape, MarkerPaperPlacement};
use hollow_board_detector::{
    algo::BoardIcpIterator,
    config::{Config, SensorUpAxis},
    detection::{BoardIcpState, BoardModelParams},
};
use measurements::Length;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::{FRAC_1_SQRT_2, SQRT_2};

const BOARD_WIDTH: f64 = 0.5;
const HOLE_RADIUS: f64 = 0.05;
const HOLE_CENTER_SHIFT: f64 = 0.05;

/// `R = W/√2`, the distance from the plate centre to any of its four corners.
const HALF_DIAGONAL: f64 = BOARD_WIDTH * FRAC_1_SQRT_2;

/// `d = s√2`: the hole centres sit on the plate's *diagonals*, so a shift measured
/// along an edge reaches √2 times as far from the centre. Mirrors
/// `BoardModel::hole_center_distance`.
const HOLE_CENTER_DISTANCE: f64 = HOLE_CENTER_SHIFT * SQRT_2;

fn test_board_shape() -> BoardShape {
    BoardShape {
        board_width: Length::from_meters(BOARD_WIDTH),
        hole_radius: Length::from_meters(HOLE_RADIUS),
        hole_center_shift: Length::from_meters(HOLE_CENTER_SHIFT),
    }
}

fn create_test_config() -> Config {
    Config {
        max_icp_iterations: 100,
        icp_pose_weight_threshold: 1e-6,
        icp_rejection_threshold: 0.1,
        plane_ransac_max_iterations: 1000,
        plane_ransac_inlier_threshold: 0.01,
        skip_ransac: false,
        icp_good_fit_threshold: 0.05,
        icp_outlier_threshold: 0.1,
        icp_damping_factor: 0.5,
        icp_min_inlier_points: 3,
        voxel_downsample_enabled: false,
        voxel_downsample_size: 0.02,
        voxel_downsample_use_centroid: true,
        voxel_parallel_threshold: 50_000,
        // These tests seed BoardIcpIterator with an explicit pose, so neither
        // field affects them; they only satisfy the struct literal.
        sensor_up_axis: SensorUpAxis::Z,
        initial_inplane_rotation_deg: 0.0,
        board_shape: test_board_shape(),
    }
}

fn create_board_params() -> BoardModelParams {
    BoardModelParams {
        board_shape: test_board_shape(),
        marker_paper_size: Length::from_meters(0.1),
        marker_paper_placement: MarkerPaperPlacement::flush_with_bottom_corner(
            Length::from_meters(BOARD_WIDTH),
            Length::from_meters(0.1),
        ),
    }
}

/// The board these fixtures model, posed at `pose` — the same model
/// [`BoardIcpIterator::step`] builds internally, so tests can ask where its corners are.
fn board_at(pose: Isometry3<f64>) -> BoardModel {
    let params = create_board_params();
    BoardModel {
        pose,
        board_shape: params.board_shape,
        marker_paper_size: params.marker_paper_size,
        marker_paper_placement: params.marker_paper_placement,
    }
}

/// Generates a synthetic LiDAR return for every point of a regular grid covering the
/// physical board, expressed in the world frame.
///
/// **Why the 45° map, and why it must not be "simplified" away.** In the board's
/// canonical local frame the axes run corner to corner, so the plate is not an
/// axis-aligned square — it is the *diamond* `|x| + |y| ≤ R`, with `R = W/√2`. A grid laid
/// out directly over local `[0, W] × [0, W]`, as this fixture used to do, leaves roughly
/// half its points off the plate entirely, silently feeding ICP data the board model can
/// never explain.
///
/// So the grid is laid out in the plate's **edge** coordinates `(u, v)`, each spanning
/// `[0, W]` — which is the square the old code was really thinking of — and rotated into
/// the corner-aligned frame:
///
/// ```text
/// x = (u − v)/√2
/// y = (u + v)/√2 − R
/// ```
///
/// Check the two anchors: `u = v = 0` gives the bottom corner `(0, −R)`, and `u = W,
/// v = 0` gives the left corner `(+R, 0)`. The map is a rotation followed by a
/// translation, so it covers the diamond exactly and uniformly.
///
/// Points falling inside one of the three holes are dropped: the physical board has no
/// material there, so a real sensor returns nothing. Keeping them would both floor the
/// achievable ICP residual at the hole radius and blunt the *only* feature that resolves
/// the plate's four-fold rotational symmetry.
fn create_grid_points(pose: &Isometry3<f64>, grid_size: usize) -> Vec<Point3<f64>> {
    let step = BOARD_WIDTH / (grid_size - 1) as f64;
    let hole_centers = [
        (HOLE_CENTER_DISTANCE, 0.0),
        (-HOLE_CENTER_DISTANCE, 0.0),
        (0.0, HOLE_CENTER_DISTANCE),
    ];

    (0..grid_size)
        .flat_map(|i| (0..grid_size).map(move |j| (i as f64 * step, j as f64 * step)))
        .map(|(u, v)| {
            (
                (u - v) * FRAC_1_SQRT_2,
                (u + v) * FRAC_1_SQRT_2 - HALF_DIAGONAL,
            )
        })
        .filter(|&(x, y)| {
            // The margin drops points sitting *exactly* on a rim as well. A grid step can
            // land there to the last bit, and whether such a point then reads as just
            // inside or just outside depends on which way the board pose's round-off
            // falls — an ambiguity no test should have to reason about.
            const RIM_MARGIN: f64 = 1e-6;
            !hole_centers
                .iter()
                .any(|&(hx, hy)| (x - hx).hypot(y - hy) < HOLE_RADIUS + RIM_MARGIN)
        })
        .map(|(x, y)| pose.transform_point(&Point3::new(x, y, 0.0)))
        .collect()
}

/// Drives `iterator` from `seed` to a fixed point and returns the final state plus the
/// number of steps actually taken.
///
/// **Why the loop steps first and tests the predicate afterwards.** The state returned by
/// [`BoardIcpIterator::initial_state`] carries `good_correspondences == 0`, and
/// `should_terminate` reads anything below 3 as "stop" — so a freshly built initial state
/// always reports "terminate". The natural-looking `while !should_terminate(&state) { ... }`
/// runs **zero** steps: the body never executes and any assertion after the loop describes
/// the seed, not ICP. Four tests in this file were written that way and asserted nothing
/// for as long as they existed. Every ICP test here therefore goes through this helper.
///
/// The wart is in `should_terminate`, not in the tests, but it is left alone deliberately:
/// production code calls `step` first too, so the predicate is never asked about a virgin
/// state in the real pipeline.
fn run_icp(
    iterator: &mut BoardIcpIterator<'_>,
    seed: Isometry3<f64>,
    points: Vec<Point3<f64>>,
    max_iterations: usize,
) -> (BoardIcpState, usize) {
    let mut state = iterator.initial_state(seed, points);
    let mut iteration_count = 0;
    loop {
        let next_state = iterator.step(&state);
        // `step` owns the iteration counter; a test that trusts its own loop variable would
        // not notice the counter freezing.
        assert_eq!(
            next_state.iteration,
            state.iteration + 1,
            "step() must advance the iteration counter"
        );
        state = next_state;
        iteration_count += 1;
        if iteration_count >= max_iterations || iterator.should_terminate(&state) {
            break;
        }
    }
    assert!(iteration_count >= 1, "the loop must run at least one step");
    (state, iteration_count)
}

/// Worst-case distance between the four corners of the board posed at `a` and at `b`.
///
/// Corners rather than the raw `Isometry3`, because they are what consumers downstream
/// read, and because a rotation about the board normal — the one degree of freedom only
/// the three-hole asymmetry witnesses — moves them while leaving the translation intact.
fn max_corner_error(a: Isometry3<f64>, b: Isometry3<f64>) -> f64 {
    let (a, b) = (board_at(a), board_at(b));
    [
        (a.top_corner() - b.top_corner()).norm(),
        (a.bottom_corner() - b.bottom_corner()).norm(),
        (a.left_corner() - b.left_corner()).norm(),
        (a.right_corner() - b.right_corner()).norm(),
    ]
    .into_iter()
    .fold(0.0f64, f64::max)
}

/// Guards the fixture itself. Every generated point must lie on the physical board —
/// inside the diamond `|x| + |y| ≤ R` and outside all three holes — and the extremes of
/// the `(u, v)` sweep must reach the plate's corners, so the grid genuinely covers the
/// plate rather than a shrunken patch of it.
#[test]
fn grid_points_all_lie_on_the_physical_board() {
    let pose = Isometry3::from_parts(
        Translation3::new(1.5, -0.4, 2.0),
        UnitQuaternion::from_euler_angles(0.3, -0.5, 0.9),
    );
    let board = board_at(pose);
    let points = create_grid_points(&pose, 21);

    assert!(!points.is_empty(), "fixture must produce points");

    // The grid steps by W/20 = 25 mm here, so the corners are hit exactly by construction;
    // the slack absorbs f64 round-off in the 45° map only.
    const EPS: f64 = 1e-9;
    let mut max_reach: f64 = 0.0;

    for point in &points {
        let (x, y) = board.plane_coordinates(point);
        let (x, y) = (x.as_meters(), y.as_meters());

        let l1 = x.abs() + y.abs();
        assert!(
            l1 <= HALF_DIAGONAL + EPS,
            "point at local ({x}, {y}) is off the plate: |x|+|y| = {l1} > R = {HALF_DIAGONAL}"
        );
        max_reach = max_reach.max(l1);

        for (hx, hy) in [
            (HOLE_CENTER_DISTANCE, 0.0),
            (-HOLE_CENTER_DISTANCE, 0.0),
            (0.0, HOLE_CENTER_DISTANCE),
        ] {
            let radius = (x - hx).hypot(y - hy);
            assert!(
                radius >= HOLE_RADIUS,
                "point at local ({x}, {y}) sits inside the hole at ({hx}, {hy}), \
                 {radius} m from its centre"
            );
        }

        // The plate is flat, so nothing may leave the board plane.
        let out_of_plane = (point - board.board_center()).dot(&board.board_z_axis());
        assert!(
            out_of_plane.abs() < EPS,
            "point at local ({x}, {y}) is {out_of_plane} m off the board plane"
        );
    }

    assert!(
        (max_reach - HALF_DIAGONAL).abs() < EPS,
        "the grid should reach the plate's corners: max |x|+|y| = {max_reach}, R = {HALF_DIAGONAL}"
    );
}

/// The assertion the rest of this file was missing: not "the loop terminated" but "the
/// loop terminated *somewhere true*".
///
/// ICP is seeded with a pose deliberately displaced from the one that generated the
/// points — in-plane translation plus an in-plane rotation, the degree of freedom the
/// three-hole asymmetry is the only witness to — and the converged model's four corner
/// accessors are then required to land on the true corners.
///
/// Corner positions rather than the pose itself, because they are what every consumer
/// downstream actually reads, and because a convention error that leaves the pose
/// numerically plausible still moves the corners.
#[test]
fn converged_pose_lands_the_corners_on_the_true_corners() {
    let mut config = create_test_config();
    // `icp_rejection_threshold` is the detector's *accept/reject* gate, not a convergence
    // criterion: at 0.1 m it fires on the very first step here and would end the loop
    // while the pose is still visibly wrong. Drive this loop to a real fixed point
    // instead, and let the iteration cap be the only backstop.
    config.icp_rejection_threshold = 0.0;
    config.max_icp_iterations = 400;

    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let truth = Isometry3::from_parts(
        Translation3::new(0.6, -0.3, 2.5),
        UnitQuaternion::from_euler_angles(0.15, -0.25, 0.4),
    );
    let points = create_grid_points(&truth, 41);

    // Perturb in the board's own frame: 2 cm across the plate and 4° of roll about its
    // normal. Both are far larger than the tolerance asserted below, so passing cannot be
    // an artefact of starting close enough.
    let perturbation = Isometry3::from_parts(
        Translation3::new(0.02, -0.015, 0.01),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 4f64.to_radians()),
    );
    let seed = truth * perturbation;

    let truth_board = board_at(truth);
    let seed_board = board_at(seed);
    let seed_error = [
        (seed_board.top_corner() - truth_board.top_corner()).norm(),
        (seed_board.bottom_corner() - truth_board.bottom_corner()).norm(),
        (seed_board.left_corner() - truth_board.left_corner()).norm(),
        (seed_board.right_corner() - truth_board.right_corner()).norm(),
    ]
    .into_iter()
    .fold(0.0f64, f64::max);

    // Step *before* consulting `should_terminate`, never after: the state returned by
    // `initial_state` carries `good_correspondences == 0`, which that predicate reads as
    // "stop". A `while !should_terminate(...)` loop around this iterator therefore runs
    // zero steps and asserts nothing about ICP at all.
    let mut state = iterator.initial_state(seed, points);
    let mut iteration_count = 0;
    loop {
        state = iterator.step(&state);
        iteration_count += 1;
        if iteration_count >= config.max_icp_iterations || iterator.should_terminate(&state) {
            break;
        }
    }

    let converged_board = board_at(state.board_pose);
    let corners = [
        (
            converged_board.top_corner(),
            truth_board.top_corner(),
            "top",
        ),
        (
            converged_board.bottom_corner(),
            truth_board.bottom_corner(),
            "bottom",
        ),
        (
            converged_board.left_corner(),
            truth_board.left_corner(),
            "left",
        ),
        (
            converged_board.right_corner(),
            truth_board.right_corner(),
            "right",
        ),
    ];

    // Tolerance. A point-to-model ICP cannot resolve the board more finely than the
    // spacing of the evidence it is given: the fixture samples the plate on a 12.5 mm
    // grid (W/40), so the edge and the hole rims are the discretisation floor here, not
    // float round-off. Measured worst-corner error at this fixture is 1.2e-4 m, and 1 mm
    // sits an order of magnitude above that — loose enough not to be a flake, and still
    // 48× tighter than the 4.8e-2 m the seed pose starts out, so a genuine failure to
    // converge cannot slip through. It is not tightened to the measured value, which
    // would pin an incidental property of this exact grid.
    const CORNER_TOL: f64 = 1e-3;

    for (actual, expected, name) in corners {
        let error = (actual - expected).norm();
        assert!(
            error < CORNER_TOL,
            "{name} corner ended {error:e} m from truth after {iteration_count} iterations \
             (seed was {seed_error:e} m off, tolerance {CORNER_TOL} m); \
             got {actual:?}, expected {expected:?}"
        );
    }

    // A converged fit must also *explain* the points; a pose that happens to sit near the
    // truth while the residual is still large would mean the two agreed by accident.
    let avg_loss = state.avg_loss;
    assert!(
        avg_loss < CORNER_TOL,
        "average point-to-model residual {avg_loss} m should be within the corner tolerance"
    );
}

/// A config that makes ICP actually iterate.
///
/// `icp_rejection_threshold` is the detector's accept/reject *gate*, not a convergence
/// criterion. At the shipped 0.1 m it fires on the very first step of these fixtures —
/// `should_terminate` then reports success while the pose is still visibly wrong, and any
/// test that stops there measures one damped half-step rather than convergence. Zeroing it
/// leaves the iteration cap as the only backstop, which is what drives the loop to a real
/// fixed point.
fn create_convergence_config() -> Config {
    Config {
        icp_rejection_threshold: 0.0,
        max_icp_iterations: 400,
        ..create_test_config()
    }
}

/// Seeded *at* the truth, ICP must be a fixed point: the pose must not drift away and the
/// residual must stay at the sampling floor.
///
/// The failure this guards against is a correspondence or Kabsch convention error that
/// pushes the model off a pose that already explains every point perfectly — a bug that a
/// "did the loop end?" assertion cannot see, and that a test seeded away from truth can
/// mask by converging to the wrong place from both sides.
#[test]
fn test_identity_transformation_convergence() {
    let config = create_convergence_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    // Off the origin and off-axis, so a drift along any single axis shows up. The points
    // are generated from this same pose, so it is exactly the pose ICP should hold.
    let truth = Isometry3::from_parts(
        Translation3::new(0.8, -0.2, 2.2),
        UnitQuaternion::from_euler_angles(0.1, -0.2, 0.35),
    );
    let points = create_grid_points(&truth, 41);

    let state = iterator.initial_state(truth, points);
    assert_eq!(
        state.iteration, 0,
        "Initial state should start at iteration 0"
    );
    assert_eq!(
        state.termination_count, 0,
        "Initial termination count should be 0"
    );

    // A fixed count rather than "until it stops": stopping early would hide a slow drift,
    // and with the rejection gate zeroed there is nothing here to stop on anyway.
    const STEPS: usize = 25;
    let (state, iterations) = run_icp(&mut iterator, truth, state.inlier_points, STEPS);
    assert_eq!(
        iterations, STEPS,
        "the fixed-point test must run every step"
    );

    // Tolerance. The truth turns out to be an *exact* fixed point of this iteration:
    // measured worst-corner drift after 25 steps is 0.0 m to the last bit, and the average
    // residual is 1.4e-16 m (f64 round-off in the pose transform, nothing more). 1e-9 m is
    // therefore ~7 orders of magnitude above the observed noise — no risk of a flake — while
    // still being ~3e4 times smaller than the 3.1e-2 m corner displacement the rotation test
    // below seeds, so a genuine "seeded at truth, walks away" bug cannot slip past it.
    // Not asserted as equality: pinning bit-exactness would make any future re-association
    // order or SIMD change read as a regression when it is not one.
    const DRIFT_TOL: f64 = 1e-9;
    let drift = max_corner_error(state.board_pose, truth);
    assert!(
        drift < DRIFT_TOL,
        "pose seeded at the truth drifted {drift:e} m (worst corner) over {iterations} \
         iterations, tolerance {DRIFT_TOL} m"
    );

    // And it must still explain the points: a low residual rules out "held still because it
    // lost its correspondences".
    assert!(
        state.avg_loss < DRIFT_TOL,
        "average point-to-model residual {} m should stay at the sampling floor",
        state.avg_loss
    );
    assert!(
        state.good_correspondences >= 3,
        "ICP must keep real correspondences throughout, got {}",
        state.good_correspondences
    );
}

/// Seeded with a translation offset, ICP must *reduce* it — the assertion the old version
/// of this test never made.
#[test]
fn test_small_translation_recovery() {
    let config = create_convergence_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let truth = Isometry3::from_parts(
        Translation3::new(0.5, -0.25, 2.0),
        UnitQuaternion::from_euler_angles(0.12, -0.2, 0.3),
    );
    let points = create_grid_points(&truth, 41);

    // Perturb in the *board's* frame: 3 cm across the plate and 1 cm along its normal, so
    // both the in-plane degrees of freedom (constrained by the edges and hole rims) and the
    // out-of-plane one (constrained by the plane fit) are exercised.
    let seed = truth
        * Isometry3::from_parts(
            Translation3::new(0.03, -0.02, 0.01),
            UnitQuaternion::identity(),
        );

    let seed_error = (seed.translation.vector - truth.translation.vector).norm();
    // Sanity: the seed must really be displaced, or "reduced the error" is vacuous.
    assert!(
        seed_error > 0.03,
        "seed must start materially off truth, got {seed_error:e} m"
    );

    let (state, iterations) = run_icp(&mut iterator, seed, points, config.max_icp_iterations);

    let final_error = (state.board_pose.translation.vector - truth.translation.vector).norm();

    // Tolerance. Measured final translation error is 9.3e-5 m from a 3.74e-2 m seed — a
    // 400x reduction, reached at the 400-iteration cap (see the note on convergence speed
    // in `test_convergence_counter_increases`: the run is still improving when the cap
    // stops it, so this figure is an upper bound, not a plateau). 1 mm sits ~11x above the
    // measured value — loose enough not to pin an incidental property of this grid — and
    // ~40x below the seed error, so nothing short of genuine convergence clears it.
    // Compare with the 12.5 mm sample spacing: ICP cannot localise the plate more finely
    // than the evidence it is given, and 1 mm is already well inside that.
    const TRANSLATION_TOL: f64 = 1e-3;
    assert!(
        final_error < TRANSLATION_TOL,
        "translation error only went {seed_error:e} m -> {final_error:e} m in {iterations} \
         iterations, tolerance {TRANSLATION_TOL} m"
    );
    // Stated separately so a regression that merely *stops improving* still reads as one.
    assert!(
        final_error < seed_error / 10.0,
        "ICP must materially reduce the seeded offset: {seed_error:e} m -> {final_error:e} m"
    );
}

/// Seeded with an in-plane rotation — about the board's own normal — ICP must recover it.
///
/// That axis specifically: the plate is a diamond with four-fold rotational symmetry, so
/// rotation about the normal is invisible to the plate outline and to the plane fit. The
/// three holes' asymmetry is its *only* witness, which makes this the degree of freedom
/// most worth asserting on and the one a correspondence bug is likeliest to lose.
#[test]
fn test_small_rotation_handling() {
    let config = create_convergence_config();
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let truth = Isometry3::from_parts(
        Translation3::new(0.4, 0.15, 1.8),
        UnitQuaternion::from_euler_angles(-0.1, 0.22, 0.5),
    );
    let points = create_grid_points(&truth, 41);

    // 5° about the local +Z (the board normal). Small enough that the holes still overlap
    // their true positions — a larger angle would let the fit slide toward the 90°-apart
    // symmetric alias instead of back to truth, which is a separate question from "does it
    // handle a small rotation".
    const SEED_ANGLE_DEG: f64 = 5.0;
    let seed = truth
        * Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), SEED_ANGLE_DEG.to_radians()),
        );

    let seed_angle = seed.rotation.angle_to(&truth.rotation);
    let seed_corner_error = max_corner_error(seed, truth);

    let (state, iterations) = run_icp(&mut iterator, seed, points, config.max_icp_iterations);

    let final_angle = state.board_pose.rotation.angle_to(&truth.rotation);
    let final_corner_error = max_corner_error(state.board_pose, truth);

    // Tolerance. Measured residual angle is 7.0e-4 rad (0.04°) from an 8.7e-2 rad (5°)
    // seed — a 125x reduction, at the 400-iteration cap. 1e-2 rad (0.57°) is ~14x above the
    // measured value and ~9x below the seed, so it is neither a flake nor vacuous. It is
    // deliberately loose relative to the measurement: in-plane angle is resolved only by
    // the hole rims, the coarsest evidence in the fixture, and it is the slowest-converging
    // degree of freedom here.
    const ANGLE_TOL: f64 = 1e-2;
    assert!(
        final_angle < ANGLE_TOL,
        "in-plane rotation error only went {seed_angle:e} rad -> {final_angle:e} rad in \
         {iterations} iterations, tolerance {ANGLE_TOL} rad"
    );
    // The angle alone can look small while the board sits elsewhere; corners pin the pose
    // the way consumers see it. Measured 2.54e-4 m against a 3.08e-2 m seed error — the
    // tolerance is ~8x above the measurement and ~12x below the seed.
    const CORNER_TOL: f64 = 2e-3;
    assert!(
        final_corner_error < CORNER_TOL,
        "corners ended {final_corner_error:e} m from truth (seed was {seed_corner_error:e} m \
         off), tolerance {CORNER_TOL} m"
    );
}

#[test]
fn test_termination_on_insufficient_points() {
    let config = create_test_config();
    let board_params = create_board_params();
    let iterator = BoardIcpIterator::new(&config, board_params, None);

    let initial_pose = Isometry3::identity();
    let insufficient_points = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.1, 0.0, 0.0)];

    let state = iterator.initial_state(initial_pose, insufficient_points);

    assert!(
        iterator.should_terminate(&state)
            || state.inlier_points.len() < config.icp_min_inlier_points,
        "Should recognize insufficient points"
    );
}

#[test]
fn test_damping_reduces_step_size() {
    let config = create_test_config();
    let board_params = create_board_params();

    let target_pose =
        Isometry3::from_parts(Translation3::new(0.1, 0.1, 0.0), UnitQuaternion::identity());

    let points = create_grid_points(&target_pose, 10);
    let initial_pose = Isometry3::identity();

    let mut iterator = BoardIcpIterator::new(&config, board_params, None);
    let state0 = iterator.initial_state(initial_pose, points.clone());
    let state1 = iterator.step(&state0);

    if state1.good_correspondences >= 3 {
        let movement =
            (state1.board_pose.translation.vector - state0.board_pose.translation.vector).norm();

        let max_expected_movement = 0.1 * config.icp_damping_factor * 2.0;

        assert!(
            movement <= max_expected_movement,
            "Damping should limit movement: {} <= {}",
            movement,
            max_expected_movement
        );
    }
}

/// The two counters `BoardIcpState` carries must actually advance across real steps:
/// `iteration` once per step, and `termination_count` once ICP has settled.
///
/// `termination_count` is the stable-pose detector — `step` increments it whenever a
/// step moved the pose by less than `icp_pose_weight_threshold` and resets it to zero
/// otherwise — so "it reached a nonzero value, having started at zero" is the observable
/// proof that ICP converged rather than kept wandering.
#[test]
fn test_convergence_counter_increases() {
    // 3000, not the 400 the other tests use, and the reason is worth recording. This
    // iteration converges *geometrically but slowly*: measured on this fixture the
    // per-step pose weight shrinks by a factor of only ~0.987, from 1.6e-3 at step 0 to
    // 7.1e-6 at step 380. Reaching `icp_pose_weight_threshold` (1e-6) therefore takes ~538
    // steps, and `should_terminate` then wants 100 *more* consecutive quiet steps before it
    // will call it "Converged (stable pose)". The rate is inherent to the correspondence
    // model rather than to the damping: points in the plate's interior project onto the
    // plane exactly where they already are and so say nothing about in-plane pose, leaving
    // only the edge and hole-rim samples to drive it.
    //
    // Consequence worth knowing (not a defect this test asserts on): with any realistic
    // `max_icp_iterations`, the stable-pose exit is unreachable — real runs always leave
    // via `avg_loss < icp_rejection_threshold`. Raising the cap here is what lets this test
    // observe the counter mechanism at all.
    let config = Config {
        max_icp_iterations: 3000,
        ..create_convergence_config()
    };
    let board_params = create_board_params();
    let mut iterator = BoardIcpIterator::new(&config, board_params, None);

    let truth = Isometry3::from_parts(
        Translation3::new(0.3, -0.1, 1.6),
        UnitQuaternion::from_euler_angles(0.05, -0.15, 0.25),
    );
    let points = create_grid_points(&truth, 41);

    // Seeded off truth, so the first steps genuinely move the pose: starting *at* truth
    // could bank the counter on step one and prove nothing about convergence.
    let seed = truth
        * Isometry3::from_parts(
            Translation3::new(0.02, 0.015, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 3f64.to_radians()),
        );

    let mut state = iterator.initial_state(seed, points);
    assert_eq!(state.iteration, 0, "state must start at iteration 0");
    assert_eq!(
        state.termination_count, 0,
        "termination count must start at 0"
    );

    let mut steps = 0;
    let mut max_termination_count = 0;
    let mut moving_steps = 0;
    loop {
        let next_state = iterator.step(&state);
        assert_eq!(
            next_state.iteration,
            state.iteration + 1,
            "step {steps} did not advance the iteration counter"
        );
        // A step whose counter is still 0 is a step that moved the pose: proof the run is
        // not a sequence of no-ops that trivially bank `termination_count`.
        if next_state.termination_count == 0 {
            moving_steps += 1;
        }
        max_termination_count = max_termination_count.max(next_state.termination_count);
        state = next_state;
        steps += 1;
        if steps >= config.max_icp_iterations || iterator.should_terminate(&state) {
            break;
        }
    }

    assert!(
        state.iteration == steps && steps > 1,
        "the iteration counter must track {steps} real steps, got {}",
        state.iteration
    );
    // Measured: the run ends after 639 steps — via `should_terminate`, not the cap — of
    // which the first 538 still move the pose and the last 101 bank `termination_count` up
    // to 101, which is exactly the "> 100" the stable-pose exit wants. Requiring only > 0
    // keeps the assertion about the mechanism (the counter leaves zero and stays up) rather
    // than about the exact step at which this particular grid settles.
    assert!(
        max_termination_count > 0,
        "termination_count never left 0 in {steps} steps: ICP never reached a stable pose"
    );
    assert!(
        moving_steps > 0,
        "no step ever moved the pose, so the counter proves nothing"
    );
    // Convergence, not just quiescence: a pose that froze somewhere wrong would also stop
    // moving. 1 mm is the same corner tolerance the recovery tests use; measured error here
    // is 9.8e-6 m (100x inside it) against a 4.15e-2 m seed offset (41x outside it).
    const CORNER_TOL: f64 = 1e-3;
    let corner_error = max_corner_error(state.board_pose, truth);
    assert!(
        corner_error < CORNER_TOL,
        "counter settled but the pose is {corner_error:e} m off truth (tolerance {CORNER_TOL} m)"
    );
}
