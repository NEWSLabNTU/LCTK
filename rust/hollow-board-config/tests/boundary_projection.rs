//! Nearest-point projection onto the physical calibration plate.
//!
//! # Why this file exists
//!
//! The board model's canonical local frame is being redefined (see
//! `docs/superpowers/specs/2026-08-13-corner-aligned-board-frame.md`): its in-plane
//! axes move from the plate's **edges** to its **diagonals**, and its origin from a
//! corner to the plate centre. In the old frame the plate was an axis-aligned box in
//! local coordinates, so membership factorised per coordinate and the nearest point
//! could be found by clamping each coordinate independently. In the new frame the
//! plate is a **diamond**, `|x| + |y| <= R` with `R = board_width / sqrt(2)`, and the
//! clamp is replaced by a true L-1 ball projection.
//!
//! That rewrite is exactly the kind of code where sign errors, wrong vertex snapping
//! and quadrant-folding bugs hide, and where every previously existing assertion was
//! blind: the old geometry checks were rotation-invariant world-frame distances and
//! dot products, which hold identically under any in-plane relabelling. This file is
//! deliberately **not** rotation-invariant.
//!
//! # How the reference is obtained
//!
//! Two independent references, neither of which reimplements the L-1 formula — a test
//! that restates the implementation's own arithmetic agrees with a wrong
//! implementation and proves nothing:
//!
//! 1. **Brute force.** The physical board (plate minus three open discs, boundary
//!    included) is sampled densely and the minimum-distance sample is taken. This
//!    knows only what the board *is*, not how the projection is computed.
//! 2. **The previous frame's projection**, reimplemented from its own definition in
//!    [`old_frame_projection`]. Projection onto the nearest point of an unchanged
//!    physical set is a metric operation, so both conventions must return the
//!    *identical world point* — that is the migration's central claim, and
//!    [`new_frame_projection_is_an_exact_reparameterisation_of_the_old_frame`]
//!    discharges it mechanically.
//!
//! # Every test runs under several poses
//!
//! An identity-pose-only test passes by accident: a convention error is an in-plane
//! rotation, and many quantities of interest are invariant under it when the board
//! sits axis-aligned at the origin. Each test therefore sweeps the identity plus
//! several full 3-D rotations with translation, drawn from the deterministic LCG
//! below so failures reproduce exactly.

use hollow_board_config::{BoardModel, BoardShape, MarkerPaperPlacement};
use measurements::Length;
use nalgebra as na;

// ---------------------------------------------------------------------------
// The board under test
// ---------------------------------------------------------------------------

/// Plate edge length. 1 m keeps every derived quantity legible in the failure
/// messages, and matches the board used by the crate's own inline tests.
const BOARD_WIDTH_M: f64 = 1.0;
const HOLE_RADIUS_M: f64 = 0.15;
const HOLE_CENTER_SHIFT_M: f64 = 0.2;
const MARKER_PAPER_SIZE_M: f64 = 0.3;

/// Half-diagonal `R`: the distance from the plate centre to any of its four corners,
/// and the radius of the L-1 ball the plate becomes in the new local frame.
const HALF_DIAGONAL_M: f64 = BOARD_WIDTH_M / std::f64::consts::SQRT_2;

/// Distance from the plate centre to a hole centre, `d = hole_center_shift * sqrt(2)`.
/// In the old frame each hole sat at `(+-s, +-s)` off the centre along the plate's
/// *edges*; along the diagonals that same physical offset has length `s * sqrt(2)`.
const HOLE_CENTER_DISTANCE_M: f64 = HOLE_CENTER_SHIFT_M * std::f64::consts::SQRT_2;

// ---------------------------------------------------------------------------
// Tolerances
// ---------------------------------------------------------------------------

/// For assertions whose expected value is computed analytically (corner snapping, edge
/// feet, hole rims, and the old-frame cross-check). Both sides are closed-form, so the
/// only error in play is f64 round-off through one rotation and one translation at
/// metre scale — roughly 1e-16 m. 1e-9 m is nine orders of magnitude of headroom and
/// still a nanometre, far below any geometric error this file is hunting.
const TOL_ANALYTIC_M: f64 = 1e-9;

/// Upper bound on how far the returned point may exceed the brute-force minimum.
/// This one is tight *on purpose* and does not need the sampling resolution: every
/// brute-force sample lies on the board, so the sampled minimum is an upper bound on
/// the true nearest distance. A correct projection is therefore never farther than it,
/// whatever the sampling density. Only round-off needs covering.
const TOL_BRUTE_FORCE_UPPER_M: f64 = 1e-9;

/// Lower bound in the same comparison, and the one place the sampling resolution does
/// enter: the sampled minimum can *overshoot* the true nearest distance by up to the
/// gap between samples, so the returned point is legitimately allowed to be that much
/// closer. The interior grid is the coarsest sampling, giving a worst-case gap of
/// `GRID_STEP_M / sqrt(2)` = 7.07 mm; 8 mm covers it.
const TOL_BRUTE_FORCE_LOWER_M: f64 = 8e-3;

/// Slack for "is this point on the board at all". Same round-off budget as
/// [`TOL_ANALYTIC_M`]; it is separate because it guards a set membership predicate
/// rather than a position, and a future tightening of one should not silently move
/// the other.
const TOL_MEMBERSHIP_M: f64 = 1e-9;

/// Interior sampling pitch for the brute-force reference, in metres. Sets the
/// *sensitivity floor* of that test, not its tolerance: errors smaller than the
/// sampling gap can hide. Every bug class this file targets — a flipped sign, a wrong
/// vertex, a mis-folded quadrant — displaces the answer by tens of centimetres on a
/// 1 m plate, so 1 cm is ample and keeps the sweep to a few seconds.
const GRID_STEP_M: f64 = 0.01;

/// Boundary sampling pitch, in metres. The nearest point is on the plate's edge or a
/// hole rim for every query that is not directly over the plate's interior, which is
/// the majority, so the boundary is sampled ten times finer than the interior.
const BOUNDARY_STEP_M: f64 = 0.001;

// ---------------------------------------------------------------------------
// Deterministic pseudo-randomness
// ---------------------------------------------------------------------------

/// A 64-bit LCG (the constants are Knuth's MMIX). Written out inline rather than
/// pulling in `rand`, so the sequence is fixed forever by this file: a failure
/// reported from CI reproduces byte-for-byte locally, and no dependency bump can
/// silently re-roll the point set out from under a passing suite.
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in `[low, high)`.
    fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }
}

/// A full 3-D rotation plus a translation well away from the origin.
fn random_pose(rng: &mut Lcg) -> na::Isometry3<f64> {
    let axis = loop {
        let candidate = na::Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        );
        // Reject near-zero draws: normalising them would amplify round-off into the
        // rotation axis and make the "deterministic" pose depend on FP details.
        if candidate.norm() > 0.25 {
            break na::Unit::new_normalize(candidate);
        }
    };
    let angle = rng.range(0.0, std::f64::consts::TAU);
    let translation = na::Translation3::new(
        rng.range(-2.0, 2.0),
        rng.range(-2.0, 2.0),
        rng.range(-2.0, 2.0),
    );
    na::Isometry3::from_parts(
        translation,
        na::UnitQuaternion::from_axis_angle(&axis, angle),
    )
}

/// The identity plus `random_count` randomised poses.
///
/// The identity is kept because it makes a failure trivially readable; the randomised
/// poses are what actually make the suite convention-sensitive, since a projection bug
/// that happens to commute with the identity's axis alignment cannot survive an
/// arbitrary rotation.
fn test_poses(seed: u64, random_count: usize) -> Vec<na::Isometry3<f64>> {
    let mut rng = Lcg::new(seed);
    let mut poses = vec![na::Isometry3::identity()];
    poses.extend((0..random_count).map(|_| random_pose(&mut rng)));
    poses
}

fn board_with_pose(pose: na::Isometry3<f64>) -> BoardModel {
    let board_shape = BoardShape {
        board_width: Length::from_meters(BOARD_WIDTH_M),
        hole_radius: Length::from_meters(HOLE_RADIUS_M),
        hole_center_shift: Length::from_meters(HOLE_CENTER_SHIFT_M),
    };
    BoardModel {
        pose,
        marker_paper_size: Length::from_meters(MARKER_PAPER_SIZE_M),
        marker_paper_placement: MarkerPaperPlacement::flush_with_bottom_corner(
            board_shape.board_width,
            Length::from_meters(MARKER_PAPER_SIZE_M),
        ),
        board_shape,
    }
}

// ---------------------------------------------------------------------------
// The canonical frame, read straight off the pose
// ---------------------------------------------------------------------------

/// The new canonical frame's world axes, derived from the **pose alone**.
///
/// Deliberately not derived from the corner accessors: doing so would make this file
/// blind to an accessor that is itself wrong by a quarter turn, which is precisely the
/// silent failure mode the frame change risks. The spec fixes the frame as: origin at
/// the plate centre (the pose translation), +Z the board normal, +Y toward the top
/// corner, +X toward the left corner.
struct Frame {
    center: na::Point3<f64>,
    x: na::Vector3<f64>,
    y: na::Vector3<f64>,
    z: na::Vector3<f64>,
}

impl Frame {
    fn of(pose: &na::Isometry3<f64>) -> Self {
        Self {
            center: pose.transform_point(&na::Point3::origin()),
            x: pose.rotation * na::Vector3::x(),
            y: pose.rotation * na::Vector3::y(),
            z: pose.rotation * na::Vector3::z(),
        }
    }

    /// World position of the in-plane local coordinate `(a, b)`.
    fn point(&self, a: f64, b: f64) -> na::Point3<f64> {
        self.center + self.x * a + self.y * b
    }

    /// World position of `(a, b)` lifted `height` along the board normal.
    fn point_at_height(&self, a: f64, b: f64, height: f64) -> na::Point3<f64> {
        self.point(a, b) + self.z * height
    }

    /// Local `(a, b, height)` of a world point.
    fn local(&self, point: &na::Point3<f64>) -> (f64, f64, f64) {
        let offset = point - self.center;
        (
            offset.dot(&self.x),
            offset.dot(&self.y),
            offset.dot(&self.z),
        )
    }
}

/// The three hole centres in local plate coordinates, as `(name, a, b)`.
///
/// The asymmetry is the whole point of a three-hole board: it is the only feature that
/// can resolve the square's 90-degree ambiguity, so a test that got the hole layout
/// wrong would also be unable to detect a quarter-turn.
fn hole_centers_local() -> [(&'static str, f64, f64); 3] {
    let d = HOLE_CENTER_DISTANCE_M;
    [("left", d, 0.0), ("right", -d, 0.0), ("top", 0.0, d)]
}

/// Radius, in metres, within which a query is treated as sitting *on* a hole centre.
///
/// There the nearest board point is genuinely non-unique — the whole rim is equidistant
/// — so any answer is correct and only the distance is assertable. Chosen at 1 micron:
/// far enough out that the radial direction of a non-degenerate query is still
/// determined to well under [`TOL_ANALYTIC_M`] (a 1e-16 m perturbation of a 1e-6 m
/// radial vector moves the rim point by ~1.5e-11 m), and far tighter than any
/// geometric error worth catching.
const HOLE_CENTRE_TIE_RADIUS_M: f64 = 1e-6;

/// The hole whose centre this in-plane local position sits on, if any.
fn hole_centre_tie(a: f64, b: f64) -> Option<&'static str> {
    hole_centers_local()
        .into_iter()
        .find(|&(_, ha, hb)| {
            ((a - ha).powi(2) + (b - hb).powi(2)).sqrt() < HOLE_CENTRE_TIE_RADIUS_M
        })
        .map(|(name, _, _)| name)
}

// ---------------------------------------------------------------------------
// Calling the code under test
// ---------------------------------------------------------------------------

/// Projects `points` and returns just the corresponding board points.
///
/// Also pins the two structural guarantees callers rely on and no other test states:
/// one correspondence per input, in input order, with the input echoed unchanged.
fn project(board: &BoardModel, points: &[na::Point3<f64>]) -> Vec<na::Point3<f64>> {
    let correspondences = board
        .find_correspondences(points.to_vec())
        .expect("find_correspondences returned None for a well-formed board");
    assert_eq!(
        correspondences.len(),
        points.len(),
        "one correspondence per input point"
    );
    for (index, (input, _)) in correspondences.iter().enumerate() {
        assert_eq!(
            *input, points[index],
            "correspondence {index} echoes a different input point than it was given"
        );
    }
    correspondences
        .into_iter()
        .map(|(_, corresponding)| corresponding)
        .collect()
}

// ---------------------------------------------------------------------------
// The brute-force reference
// ---------------------------------------------------------------------------

/// Dense samples of the physical board's surface, in local plate coordinates.
///
/// The board is the closed diamond `|a| + |b| <= R` with three **open** discs removed,
/// so the plate's edges and the hole rims belong to it. Three families:
///
/// - an interior grid at [`GRID_STEP_M`], with points inside a hole dropped;
/// - the four plate edges at [`BOUNDARY_STEP_M`];
/// - the three hole rims at the same arc pitch.
///
/// Boundaries get the fine pitch because that is where the answer lands for every
/// query not sitting directly over the plate's interior.
fn board_surface_samples_local() -> Vec<(f64, f64)> {
    let r = HALF_DIAGONAL_M;
    let holes = hole_centers_local();

    let inside_a_hole = |a: f64, b: f64| {
        holes
            .iter()
            .any(|&(_, ha, hb)| ((a - ha).powi(2) + (b - hb).powi(2)).sqrt() < HOLE_RADIUS_M)
    };

    let mut samples = Vec::new();

    // Interior grid.
    let steps = (2.0 * r / GRID_STEP_M).ceil() as i64;
    for i in 0..=steps {
        let a = -r + GRID_STEP_M * i as f64;
        for j in 0..=steps {
            let b = -r + GRID_STEP_M * j as f64;
            if a.abs() + b.abs() <= r && !inside_a_hole(a, b) {
                samples.push((a, b));
            }
        }
    }

    // Plate edges, corner to corner. Corner order walks the diamond so consecutive
    // pairs are exactly the four edges.
    let corners = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
    let edge_steps = (BOARD_WIDTH_M / BOUNDARY_STEP_M).ceil() as i64;
    for index in 0..corners.len() {
        let (a0, b0) = corners[index];
        let (a1, b1) = corners[(index + 1) % corners.len()];
        for step in 0..=edge_steps {
            let t = step as f64 / edge_steps as f64;
            samples.push((a0 + (a1 - a0) * t, b0 + (b1 - b0) * t));
        }
    }

    // Hole rims.
    let rim_steps = ((std::f64::consts::TAU * HOLE_RADIUS_M) / BOUNDARY_STEP_M).ceil() as i64;
    for (_, ha, hb) in holes {
        for step in 0..rim_steps {
            let theta = std::f64::consts::TAU * step as f64 / rim_steps as f64;
            samples.push((
                ha + HOLE_RADIUS_M * theta.cos(),
                hb + HOLE_RADIUS_M * theta.sin(),
            ));
        }
    }

    samples
}

/// Is `point` a point of the physical board?
///
/// Guards against the failure mode a distance comparison alone cannot see: a
/// projection that returns something impossibly close by landing *off* the board, for
/// instance inside a hole or beyond an edge.
fn is_on_board(frame: &Frame, point: &na::Point3<f64>) -> Result<(), String> {
    let (a, b, height) = frame.local(point);
    if height.abs() > TOL_MEMBERSHIP_M {
        return Err(format!("off the board plane by {height:e} m"));
    }
    if a.abs() + b.abs() > HALF_DIAGONAL_M + TOL_MEMBERSHIP_M {
        return Err(format!(
            "outside the plate: |a|+|b| = {sum} > R = {r}",
            sum = a.abs() + b.abs(),
            r = HALF_DIAGONAL_M
        ));
    }
    for (name, ha, hb) in hole_centers_local() {
        let distance = ((a - ha).powi(2) + (b - hb).powi(2)).sqrt();
        if distance < HOLE_RADIUS_M - TOL_MEMBERSHIP_M {
            return Err(format!(
                "inside the {name} hole: {distance} m from its centre, radius {HOLE_RADIUS_M} m"
            ));
        }
    }
    Ok(())
}

/// Query points in local plate coordinates plus a height off the plane, spanning well
/// inside, just inside, exactly on, just outside and far outside the plate, plus a
/// deliberate sweep over each hole.
///
/// Points are generated as a radius factor times a direction drawn from the unit L-1
/// sphere, so "just outside" means just outside *the diamond* in every direction,
/// including straight past a corner and perpendicular to an edge.
fn query_points_local(rng: &mut Lcg, count: usize) -> Vec<(f64, f64, f64)> {
    let r = HALF_DIAGONAL_M;
    let mut queries = Vec::with_capacity(count + 3 * 12);

    for index in 0..count {
        // A direction on the unit L-1 sphere.
        let (da, db) = loop {
            let a = rng.range(-1.0, 1.0);
            let b = rng.range(-1.0, 1.0);
            let l1 = a.abs() + b.abs();
            if l1 > 1e-6 {
                break (a / l1, b / l1);
            }
        };

        let factor = match index % 5 {
            0 => rng.range(0.0, 0.95), // well inside
            1 => 1.0 - 1e-3,           // just inside
            2 => 1.0,                  // exactly on the boundary
            3 => 1.0 + 1e-3,           // just outside
            _ => rng.range(1.05, 5.0), // far outside
        };

        let height = match index % 3 {
            0 => 0.0, // in the board plane, where the projection is a pure in-plane move
            _ => rng.range(-1.5, 1.5),
        };

        queries.push((r * factor * da, r * factor * db, height));
    }

    // Every hole, sampled from its centre out past its rim. Without this the random
    // sweep would only occasionally land over a hole.
    for (_, ha, hb) in hole_centers_local() {
        for step in 0..12 {
            let theta = std::f64::consts::TAU * step as f64 / 12.0;
            let radius = HOLE_RADIUS_M * (step % 4) as f64 / 3.0;
            queries.push((
                ha + radius * theta.cos(),
                hb + radius * theta.sin(),
                rng.range(-0.5, 0.5),
            ));
        }
    }

    queries
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// The headline test: over many points and several poses, `find_correspondences` must
/// return the genuine nearest point of the physical board, checked against a reference
/// that knows only the board's *shape* — never its projection formula.
///
/// Catches the whole family the L-1 rewrite is exposed to: a sign dropped by
/// `copysign`, a fold into the wrong quadrant, snapping to the wrong vertex, and an
/// "outside" predicate that disagrees with the projection it guards. A test that
/// restated the L-1 formula as its own expectation would agree with every one of them.
#[test]
fn projection_is_the_true_nearest_board_point_against_a_brute_force_reference() {
    let samples_local = board_surface_samples_local();
    let mut rng = Lcg::new(0x5eed_0001);
    let queries_local = query_points_local(&mut rng, 250);

    for (pose_index, pose) in test_poses(0xB0A2_D001, 4).into_iter().enumerate() {
        let board = board_with_pose(pose);
        let frame = Frame::of(&board.pose);

        let samples_world: Vec<na::Point3<f64>> = samples_local
            .iter()
            .map(|&(a, b)| frame.point(a, b))
            .collect();
        let queries: Vec<na::Point3<f64>> = queries_local
            .iter()
            .map(|&(a, b, height)| frame.point_at_height(a, b, height))
            .collect();

        for (query, actual) in queries.iter().zip(project(&board, &queries)) {
            let (qa, qb, qh) = frame.local(query);
            let context =
                format!("pose {pose_index}, query local ({qa:.6}, {qb:.6}) height {qh:.6}");

            if let Err(reason) = is_on_board(&frame, &actual) {
                panic!("{context}: projection is not a point of the board: {reason}");
            }

            let actual_distance = (actual - query).norm();
            let brute_force_distance = samples_world
                .iter()
                .map(|sample| (sample - query).norm())
                .fold(f64::INFINITY, f64::min);

            assert!(
                actual_distance <= brute_force_distance + TOL_BRUTE_FORCE_UPPER_M,
                "{context}: projection is farther than a known board point \
                 ({actual_distance} m vs brute force {brute_force_distance} m)"
            );
            assert!(
                actual_distance >= brute_force_distance - TOL_BRUTE_FORCE_LOWER_M,
                "{context}: projection is closer than any point of the board can be \
                 ({actual_distance} m vs brute force {brute_force_distance} m)"
            );
        }
    }
}

/// Points beyond a corner must snap to that exact corner, and all four corners must be
/// reachable.
///
/// This is the `pa < 0` / `pb < 0` arm of the projection, the only branch where the
/// answer is a vertex rather than a perpendicular foot, and the one a quadrant-folding
/// bug corrupts most quietly: fold the wrong way and the point still lands on *a*
/// corner, just the wrong one, 90 or 180 degrees around the plate. Asserting against
/// the named accessors as well as the frame pins which corner is which — the binding
/// whose corruption produces a silent quarter-turn downstream.
#[test]
fn points_beyond_a_corner_snap_to_that_exact_plate_corner() {
    let r = HALF_DIAGONAL_M;

    for (pose_index, pose) in test_poses(0xB0A2_D002, 4).into_iter().enumerate() {
        let board = board_with_pose(pose);
        let frame = Frame::of(&board.pose);

        // (name, local corner, accessor result)
        let corners = [
            ("left", (r, 0.0), board.left_corner()),
            ("top", (0.0, r), board.top_corner()),
            ("right", (-r, 0.0), board.right_corner()),
            ("bottom", (0.0, -r), board.bottom_corner()),
        ];

        let mut reached = [false; 4];

        for (corner_index, (name, (ca, cb), accessor)) in corners.into_iter().enumerate() {
            let expected = frame.point(ca, cb);
            assert!(
                (accessor - expected).norm() < TOL_ANALYTIC_M,
                "pose {pose_index}: the {name}_corner accessor is not at local \
                 ({ca}, {cb}); got {accessor:?}, expected {expected:?}"
            );

            // Outward direction along the corner's diagonal, and the in-plane
            // perpendicular. A point at `corner + p * outward + q * perpendicular`
            // lies in the corner's normal cone whenever |q| <= p, and must therefore
            // project onto the corner itself.
            let outward = (ca / r, cb / r);
            let perpendicular = (-outward.1, outward.0);

            let mut queries = Vec::new();
            for &p in &[1e-3, 0.05, 0.4, 3.0] {
                for &ratio in &[0.0, 0.5, -0.5, 0.95, -0.95] {
                    let q = ratio * p;
                    let a = ca + outward.0 * p + perpendicular.0 * q;
                    let b = cb + outward.1 * p + perpendicular.1 * q;
                    for &height in &[0.0, 0.7, -0.7] {
                        queries.push(frame.point_at_height(a, b, height));
                    }
                }
            }

            for (query, actual) in queries.iter().zip(project(&board, &queries)) {
                let (qa, qb, qh) = frame.local(query);
                assert!(
                    (actual - expected).norm() < TOL_ANALYTIC_M,
                    "pose {pose_index}: a point in the {name} corner's normal cone \
                     (local ({qa:.6}, {qb:.6}) height {qh:.3}) projected to \
                     {actual:?} instead of the {name} corner {expected:?}; local \
                     result {result:?}",
                    result = frame.local(&actual)
                );
                reached[corner_index] = true;
            }
        }

        assert!(
            reached.iter().all(|&hit| hit),
            "pose {pose_index}: not every plate corner was exercised"
        );
    }
}

/// A point just outside an edge, in the plane or lifted off it, must project onto the
/// perpendicular foot on that edge, at exactly the expected distance.
///
/// This is the `pa >= 0 && pb >= 0` arm. It is where a wrong half-shift `t` shows up:
/// halve it twice, or subtract it from only one coordinate, and the answer slides
/// along the edge or off it entirely while still looking plausible.
#[test]
fn points_just_outside_an_edge_project_perpendicularly_onto_that_edge() {
    let r = HALF_DIAGONAL_M;
    let corners = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
    let edge_names = ["left-top", "top-right", "right-bottom", "bottom-left"];

    for (pose_index, pose) in test_poses(0xB0A2_D003, 4).into_iter().enumerate() {
        let board = board_with_pose(pose);
        let frame = Frame::of(&board.pose);

        for edge_index in 0..corners.len() {
            let name = edge_names[edge_index];
            let (a0, b0) = corners[edge_index];
            let (a1, b1) = corners[(edge_index + 1) % corners.len()];

            // Outward unit normal of this edge: the diamond's edges have local normals
            // (+-1, +-1)/sqrt(2), and the edge's midpoint direction from the centre
            // gives the outward sign.
            let (ma, mb) = ((a0 + a1) / 2.0, (b0 + b1) / 2.0);
            let midpoint_norm = (ma * ma + mb * mb).sqrt();
            let normal = (ma / midpoint_norm, mb / midpoint_norm);

            // Interior parameters only: at t = 0 or 1 the answer is a corner, which is
            // the other test's business.
            for &t in &[0.1, 0.35, 0.5, 0.8] {
                let foot = (a0 + (a1 - a0) * t, b0 + (b1 - b0) * t);
                let expected = frame.point(foot.0, foot.1);

                for &offset in &[1e-4, 0.02, 0.5] {
                    for &height in &[0.0, 0.3, -0.9] {
                        let query = frame.point_at_height(
                            foot.0 + normal.0 * offset,
                            foot.1 + normal.1 * offset,
                            height,
                        );
                        let actual = project(&board, std::slice::from_ref(&query))[0];

                        assert!(
                            (actual - expected).norm() < TOL_ANALYTIC_M,
                            "pose {pose_index}, {name} edge at t = {t}, offset \
                             {offset} m, height {height} m: projected to {actual:?} \
                             instead of the perpendicular foot {expected:?}"
                        );

                        let expected_distance = (offset * offset + height * height).sqrt();
                        let actual_distance = (actual - query).norm();
                        assert!(
                            (actual_distance - expected_distance).abs() < TOL_ANALYTIC_M,
                            "pose {pose_index}, {name} edge at t = {t}: distance to \
                             the board is {actual_distance} m, expected \
                             {expected_distance} m"
                        );
                    }
                }
            }
        }
    }
}

/// A point already on the plate and clear of the holes is its own projection, and a
/// point above it projects straight down the normal.
///
/// The fixed-point set of the projection *is* the board. The old implementation
/// decided "outside" by comparing a componentwise clamp against the original for exact
/// float equality; no componentwise operation has a diamond for its fixed points, so
/// this property is what catches an "outside" predicate that was ported across
/// mechanically and now mis-classifies interior points as boundary ones.
#[test]
fn interior_points_clear_of_the_holes_are_their_own_projection() {
    let r = HALF_DIAGONAL_M;
    let mut rng = Lcg::new(0x5eed_0004);

    // Local interior points, kept a clear margin away from the edges and the holes so
    // this test is about the interior branch and not about boundary tie-breaking.
    let margin = 0.02;
    let mut interior = Vec::new();
    while interior.len() < 200 {
        let a = rng.range(-r, r);
        let b = rng.range(-r, r);
        if a.abs() + b.abs() > r - margin {
            continue;
        }
        let clear_of_holes = hole_centers_local().iter().all(|&(_, ha, hb)| {
            ((a - ha).powi(2) + (b - hb).powi(2)).sqrt() > HOLE_RADIUS_M + margin
        });
        if clear_of_holes {
            interior.push((a, b));
        }
    }

    for (pose_index, pose) in test_poses(0xB0A2_D004, 4).into_iter().enumerate() {
        let board = board_with_pose(pose);
        let frame = Frame::of(&board.pose);

        for &height in &[0.0, 0.4, -1.2] {
            let expected: Vec<na::Point3<f64>> =
                interior.iter().map(|&(a, b)| frame.point(a, b)).collect();
            let queries: Vec<na::Point3<f64>> = interior
                .iter()
                .map(|&(a, b)| frame.point_at_height(a, b, height))
                .collect();

            for ((query, want), actual) in
                queries.iter().zip(&expected).zip(project(&board, &queries))
            {
                let (qa, qb, _) = frame.local(query);
                assert!(
                    (actual - want).norm() < TOL_ANALYTIC_M,
                    "pose {pose_index}: interior point local ({qa:.6}, {qb:.6}) at \
                     height {height} projected to {actual:?} instead of {want:?} \
                     (local result {result:?})",
                    result = frame.local(&actual)
                );
            }
        }
    }
}

/// A point over a hole projects radially onto that hole's rim, at exactly
/// `hole_radius` from the hole's centre.
///
/// Hole handling is *not* what the frame change touches, which is exactly why it needs
/// pinning: the hole centres move in local coordinates (from `(+-s, +-s)` along the
/// edges to `(+-s*sqrt(2), 0)` and `(0, s*sqrt(2))` along the diagonals) even though
/// they do not move physically. Get that conversion wrong and the holes rotate 45
/// degrees about the plate centre while every rotation-invariant check still passes.
#[test]
fn points_over_a_hole_project_radially_onto_that_holes_rim() {
    let d = HOLE_CENTER_DISTANCE_M;

    for (pose_index, pose) in test_poses(0xB0A2_D005, 4).into_iter().enumerate() {
        let board = board_with_pose(pose);
        let frame = Frame::of(&board.pose);

        // The named accessors must agree with the canonical local layout: this is the
        // hole-identity binding, and the three-hole asymmetry is the only thing that
        // can resolve the plate's 90-degree symmetry.
        for (name, accessor, (ha, hb)) in [
            ("left", board.left_circle_center(), (d, 0.0)),
            ("right", board.right_circle_center(), (-d, 0.0)),
            ("top", board.top_circle_center(), (0.0, d)),
        ] {
            let expected_center = frame.point(ha, hb);
            assert!(
                (accessor - expected_center).norm() < TOL_ANALYTIC_M,
                "pose {pose_index}: the {name}_circle_center accessor is not at local \
                 ({ha}, {hb}); got {accessor:?}, expected {expected_center:?}"
            );

            for &fraction in &[0.0, 0.05, 0.5, 0.95] {
                for step in 0..8 {
                    let theta = std::f64::consts::TAU * step as f64 / 8.0;
                    let radius = HOLE_RADIUS_M * fraction;
                    let (oa, ob) = (radius * theta.cos(), radius * theta.sin());

                    for &height in &[0.0, 0.6] {
                        let query = frame.point_at_height(ha + oa, hb + ob, height);
                        let actual = project(&board, std::slice::from_ref(&query))[0];

                        let from_center = (actual - expected_center).norm();
                        assert!(
                            (from_center - HOLE_RADIUS_M).abs() < TOL_ANALYTIC_M,
                            "pose {pose_index}: a point {radius} m into the {name} \
                             hole projected {from_center} m from that hole's centre, \
                             expected exactly the rim at {HOLE_RADIUS_M} m"
                        );

                        // A point exactly at the centre has no radial direction, so
                        // any rim point is correct there; everywhere else the answer
                        // must be radially outward.
                        if radius > 0.0 {
                            let expected = frame.point(ha + oa, hb + ob)
                                + (frame.point(ha + oa, hb + ob) - expected_center).normalize()
                                    * (HOLE_RADIUS_M - radius);
                            assert!(
                                (actual - expected).norm() < TOL_ANALYTIC_M,
                                "pose {pose_index}: a point inside the {name} hole \
                                 projected to {actual:?} instead of the radially \
                                 outward rim point {expected:?}"
                            );
                        }
                    }
                }
            }
        }
    }
}

/// The **previous** (edge-aligned) frame's projection, reimplemented from its own
/// definition rather than shared with the crate.
///
/// Old frame: the origin is the plate's *bottom* corner, +X runs along the edge toward
/// the left corner and +Y along the edge toward the right corner, so the plate is the
/// axis-aligned box `[0, W] x [0, W]` and the nearest point is found by clamping each
/// coordinate independently. Hole centres sit at `(W/2 +- s, W/2 -+ s)`.
///
/// Kept verbatim on purpose. It is the only surviving executable statement of the old
/// convention, and the test below is worthless without it.
fn old_frame_projection(pose_old: &na::Isometry3<f64>, query: &na::Point3<f64>) -> na::Point3<f64> {
    let origin = pose_old.transform_point(&na::Point3::origin());
    let x = pose_old.rotation * na::Vector3::x();
    let y = pose_old.rotation * na::Vector3::y();

    let offset = query - origin;
    let (a, b) = (offset.dot(&x), offset.dot(&y));

    let clamped = (a.clamp(0.0, BOARD_WIDTH_M), b.clamp(0.0, BOARD_WIDTH_M));
    let plane_point = |(a, b): (f64, f64)| origin + x * a + y * b;

    if clamped != (a, b) {
        return plane_point(clamped);
    }

    let half = BOARD_WIDTH_M / 2.0;
    let s = HOLE_CENTER_SHIFT_M;
    let projection = plane_point((a, b));

    // Same order as the implementation being replaced: left, right, top.
    for (ha, hb) in [
        (half + s, half - s),
        (half - s, half + s),
        (half + s, half + s),
    ] {
        let center = plane_point((ha, hb));
        let radial = projection - center;
        if radial.norm() < HOLE_RADIUS_M {
            // The old implementation's degenerate guard, reproduced: a point exactly
            // on a hole's centre has no radial direction, so it picked the rim point
            // along the board's own +X axis. Reproduced rather than tidied because
            // that arbitrary choice is *why* the two conventions may legitimately
            // disagree there, and the caller's tie handling depends on knowing it.
            if radial.norm() < 1e-10 {
                return center + x * HOLE_RADIUS_M;
            }
            return center + radial.normalize() * HOLE_RADIUS_M;
        }
    }

    projection
}

/// **Keep this test permanently.**
///
/// It discharges the migration's central claim mechanically rather than by argument:
/// the new frame is an *exact re-parameterisation* of the old one, not an analogue.
/// The physical plate does not move, and projection onto the nearest point of an
/// unchanged set is a metric operation, so the two conventions must return the
/// **identical world point** for every query — bit-for-bit up to f64 round-off, not
/// merely "close enough".
///
/// The two frames are related by an exact conjugation, which this test is also the
/// executable statement of: the new rotation is the old one composed with a -45
/// degree in-plane rotation, and the new translation is the old frame's plate centre.
/// If that conjugation is ever wrong — the classic failure being +45 instead of -45,
/// which produces a geometrically identical diamond with its corners relabelled — this
/// test fails while every rotation-invariant check in the repository still passes.
///
/// It also outlives the migration: it keeps the old convention executable, so anyone
/// reading a pre-change bag, config or saved detection has a checked reference for
/// what those numbers meant.
#[test]
fn new_frame_projection_is_an_exact_reparameterisation_of_the_old_frame() {
    let mut rng = Lcg::new(0x5eed_0006);
    let queries_local = query_points_local(&mut rng, 300);

    for (pose_index, pose_old) in test_poses(0xB0A2_D006, 5).into_iter().enumerate() {
        // The conjugation: origin to the plate centre, axes rotated -45 degrees in
        // the board plane so +X points at the left corner and +Y at the top corner.
        let plate_center = pose_old.transform_point(&na::Point3::new(
            BOARD_WIDTH_M / 2.0,
            BOARD_WIDTH_M / 2.0,
            0.0,
        ));
        let pose_new = na::Isometry3::from_parts(
            na::Translation3::from(plate_center.coords),
            pose_old.rotation
                * na::UnitQuaternion::from_axis_angle(
                    &na::Vector3::z_axis(),
                    -std::f64::consts::FRAC_PI_4,
                ),
        );

        let board = board_with_pose(pose_new);
        let frame = Frame::of(&board.pose);

        // Sanity on the conjugation itself, so a failure below is attributable to the
        // projection rather than to a mis-stated relationship between the frames.
        let bottom_corner_old = pose_old.transform_point(&na::Point3::origin());
        let bottom_corner_new = frame.point(0.0, -HALF_DIAGONAL_M);
        assert!(
            (bottom_corner_new - bottom_corner_old).norm() < TOL_ANALYTIC_M,
            "pose {pose_index}: the two frames do not describe the same plate — the \
             old origin is at {bottom_corner_old:?} but the new frame's bottom corner \
             is at {bottom_corner_new:?}"
        );

        let queries: Vec<na::Point3<f64>> = queries_local
            .iter()
            .map(|&(a, b, height)| frame.point_at_height(a, b, height))
            .collect();

        for (query, actual) in queries.iter().zip(project(&board, &queries)) {
            let expected = old_frame_projection(&pose_old, query);
            let (qa, qb, qh) = frame.local(query);

            // A query sitting on a hole's centre has no unique nearest board point:
            // the entire rim is equidistant, so the answer is an arbitrary tie-break
            // and the two conventions are free to break it differently (they pick
            // along their own +X axis, which is exactly the 45 degrees this migration
            // is about). Comparing world points there would assert the tie-break, not
            // the geometry. Assert what is actually determined — the distance — and
            // let the dedicated rim test cover the radial direction.
            if let Some(name) = hole_centre_tie(qa, qb) {
                let centre = frame.point_at_height(qa, qb, 0.0);
                for (which, point) in [("new", &actual), ("old", &expected)] {
                    let from_centre = (point - centre).norm();
                    assert!(
                        (from_centre - HOLE_RADIUS_M).abs() < TOL_ANALYTIC_M,
                        "pose {pose_index}: a query on the {name} hole's centre has \
                         every rim point equidistant, but the {which} frame answered \
                         {point:?}, {from_centre} m from that centre rather than the \
                         rim at {HOLE_RADIUS_M} m"
                    );
                }
                continue;
            }

            assert!(
                (actual - expected).norm() < TOL_ANALYTIC_M,
                "pose {pose_index}: query at new-frame local ({qa:.6}, {qb:.6}) \
                 height {qh:.6} projected to {actual:?}, but the old frame's \
                 projection of the same physical board gives {expected:?} \
                 (difference {difference} m)",
                difference = (actual - expected).norm()
            );
        }
    }
}
