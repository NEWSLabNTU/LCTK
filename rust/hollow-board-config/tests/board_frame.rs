//! Pins the calibration board's **canonical local frame** — the one thing the crate's
//! existing geometry assertions cannot see.
//!
//! # Why this file exists
//!
//! `BoardModel` describes a square plate hung as a diamond, and every accessor on it is
//! named for the diamond: top/bottom/left/right *corner*, three hole centres. The frame
//! those names live in is being redefined by
//! `docs/superpowers/specs/2026-08-13-corner-aligned-board-frame.md`: the origin moves
//! from a plate corner to the plate **centre**, and the in-plane axes move from the
//! plate's **edges** to its **diagonals**, so that `+Y` points at the top corner and
//! `+X` at the left corner. `+Z` stays the board normal.
//!
//! The class of bug this file catches is a **45° in-plane relabelling** — the model
//! believing the plate is edge-aligned when it is physically corner-standing. That error
//! is invisible to every assertion the crate carried before: they are all world-frame
//! distances, norms and dot products between accessors, and all of those are
//! *rotation-invariant*. Rotate the whole frame by any angle within the board plane and
//! every one of them still holds, which is precisely why a 45° convention error survived
//! in the model for as long as it did.
//!
//! The cure is to stop comparing accessors only to each other and start comparing each
//! one to the model's **own** advertised axes: `board_x_axis`, `board_y_axis`,
//! `board_z_axis`. A dot product against the model's own frame is *not* invariant under
//! in-plane rotation, so it can tell a diamond from a square.
//!
//! # Why every test sweeps several poses
//!
//! At the identity pose the world frame and the board frame coincide, so a whole family
//! of frame errors — transposed axes, a rotation folded into the wrong side of a
//! product, an origin taken from the wrong operand — cancels out and the test passes for
//! the wrong reason. Every test here therefore runs against a fixed, deterministic set
//! of boards at assorted translations, full 3-D rotations, plate widths, and hole
//! shifts. Randomised but reproducible: a failure names the exact case, and rerunning
//! reproduces it.

use hollow_board_config::{BoardModel, BoardShape, MarkerPaperPlacement};
use measurements::Length;
use nalgebra as na;

/// Positional/directional tolerance, in metres (or in metres² for the dot products,
/// whose operands are all sub-metre). Everything under test is exact rational geometry
/// pushed through one rotation and one translation, so the only error in play is f64
/// round-off at roughly 1e-15 relative on ~10 m coordinates; 1e-9 leaves six orders of
/// margin while still being far tighter than any real geometric mistake.
const TOL: f64 = 1e-9;

// ---------------------------------------------------------------------------
// Deterministic pose generation
// ---------------------------------------------------------------------------

/// A tiny linear congruential generator, written out here rather than pulled in as a
/// dependency: these tests need *reproducible* variety, not statistical quality, and a
/// fixed seed means a failing case can be re-run byte for byte.
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Uniform in `[0, 1)`, taken from the high bits — an LCG's low bits have short
    /// periods and would make the "random" poses quietly repetitive.
    fn unit(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.state >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

/// One board under test, with a label that identifies it in a failure message.
struct Case {
    label: String,
    board: BoardModel,
}

fn board_at(pose: na::Isometry3<f64>, board_width: f64, hole_center_shift: f64) -> BoardModel {
    let board_shape = BoardShape {
        board_width: Length::from_meters(board_width),
        hole_radius: Length::from_meters(0.15),
        hole_center_shift: Length::from_meters(hole_center_shift),
    };
    let marker_paper_size = Length::from_meters(0.3);

    BoardModel {
        pose,
        marker_paper_size,
        marker_paper_placement: MarkerPaperPlacement::flush_with_bottom_corner(
            board_shape.board_width,
            marker_paper_size,
        ),
        board_shape,
    }
}

/// The identity pose plus eight randomised ones, spanning translation, full 3-D
/// rotation, plate width and hole shift.
///
/// The identity case is kept only because it makes a failure easy to read by hand; it
/// proves nothing on its own, which is the whole reason the other eight are here.
fn cases() -> Vec<Case> {
    let mut rng = Lcg::new(0x5EED_B0A2_D_F00_D1E);
    let mut cases = vec![Case {
        label: "identity pose, W=1.0, s=0.2".to_owned(),
        board: board_at(na::Isometry3::identity(), 1.0, 0.2),
    }];

    for index in 0..8 {
        let translation = na::Translation3::new(
            rng.range(-20.0, 20.0),
            rng.range(-20.0, 20.0),
            rng.range(-20.0, 20.0),
        );
        let rotation = na::UnitQuaternion::from_euler_angles(
            rng.range(-std::f64::consts::PI, std::f64::consts::PI),
            rng.range(-std::f64::consts::PI, std::f64::consts::PI),
            rng.range(-std::f64::consts::PI, std::f64::consts::PI),
        );
        let board_width = rng.range(0.4, 1.6);
        let hole_center_shift = rng.range(0.05, board_width / 4.0);

        cases.push(Case {
            label: format!("random pose #{index}, W={board_width:.4}, s={hole_center_shift:.4}"),
            board: board_at(
                na::Isometry3::from_parts(translation, rotation),
                board_width,
                hole_center_shift,
            ),
        });
    }

    cases
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Resolves a world point into the board's **own** frame: the components of
/// `point - board_center()` along the model's advertised x/y/z axes.
///
/// This is the single operation that makes every test in this file
/// convention-sensitive. Anything phrased purely as distances between accessors would
/// hold just as well for a plate rotated 45° within its own plane.
fn local_of(board: &BoardModel, point: &na::Point3<f64>) -> (f64, f64, f64) {
    let offset = point - board.board_center();
    (
        offset.dot(&board.board_x_axis()),
        offset.dot(&board.board_y_axis()),
        offset.dot(&board.board_z_axis()),
    )
}

/// The half-diagonal `R = W/√2`: the distance from the plate centre to any corner.
fn half_diagonal(board: &BoardModel) -> f64 {
    board.board_shape.board_width.as_meters() / 2f64.sqrt()
}

/// The hole offset `d = s√2`: how far each hole centre sits from the plate centre along
/// a diagonal, given a hole shift `s` measured along the plate's edges.
fn hole_offset(board: &BoardModel) -> f64 {
    board.board_shape.hole_center_shift.as_meters() * 2f64.sqrt()
}

fn assert_close(actual: f64, expected: f64, label: &str, what: &str) {
    assert!(
        (actual - expected).abs() < TOL,
        "{label}: {what} was {actual}, expected {expected} (tolerance {TOL})"
    );
}

/// Asserts a point's coordinates in the board's own frame, which is the only phrasing
/// that can detect a rotated convention.
fn assert_local(
    board: &BoardModel,
    label: &str,
    name: &str,
    point: na::Point3<f64>,
    ex: f64,
    ey: f64,
) {
    let (x, y, z) = local_of(board, &point);
    assert_close(x, ex, label, &format!("{name} local x"));
    assert_close(y, ey, label, &format!("{name} local y"));
    assert_close(
        z,
        0.0,
        label,
        &format!("{name} local z (must lie in the board plane)"),
    );
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Every accessor sits at its specified coordinate **in the board's own frame**.
///
/// This is the load-bearing test of the file. It is the only assertion in the crate
/// capable of distinguishing the corner-aligned (diamond) frame from the edge-aligned
/// (square) one: the two differ by an in-plane rotation of exactly 45°, under which all
/// inter-accessor distances, norms and dot products are unchanged. Comparing each
/// accessor against `board_x_axis`/`board_y_axis` breaks that invariance.
///
/// A failure here means the model's coordinates disagree with the model's own accessor
/// names — the exact defect that forced operators to hand-set
/// `initial_inplane_rotation_deg = 45.0` before the detector would find anything.
#[test]
fn accessors_sit_at_their_canonical_coordinates_in_the_boards_own_frame() {
    for Case { label, board } in cases() {
        let r = half_diagonal(&board);
        let d = hole_offset(&board);

        assert_local(
            &board,
            &label,
            "board_center",
            board.board_center(),
            0.0,
            0.0,
        );
        assert_local(&board, &label, "top_corner", board.top_corner(), 0.0, r);
        assert_local(
            &board,
            &label,
            "bottom_corner",
            board.bottom_corner(),
            0.0,
            -r,
        );
        assert_local(&board, &label, "left_corner", board.left_corner(), r, 0.0);
        assert_local(
            &board,
            &label,
            "right_corner",
            board.right_corner(),
            -r,
            0.0,
        );
        assert_local(
            &board,
            &label,
            "left_circle_center",
            board.left_circle_center(),
            d,
            0.0,
        );
        assert_local(
            &board,
            &label,
            "right_circle_center",
            board.right_circle_center(),
            -d,
            0.0,
        );
        assert_local(
            &board,
            &label,
            "top_circle_center",
            board.top_circle_center(),
            0.0,
            d,
        );
    }
}

/// The pose's translation *is* the plate centre.
///
/// The pose is what gets published and what every downstream consumer treats as "where
/// the board is". If the translation were still a corner while the accessors were
/// centre-relative, every reported position would be off by a half-diagonal — roughly
/// 0.7 m on a 1 m plate — and the pose's rotational uncertainty would be inflated by
/// that lever arm. This test is what keeps the origin from drifting back to a corner.
#[test]
fn pose_translation_is_the_plate_centre() {
    for Case { label, board } in cases() {
        let translation: na::Point3<f64> = board.pose.translation.vector.into();
        let center = board.board_center();
        assert!(
            (translation - center).norm() < TOL,
            "{label}: pose.translation {translation:?} is not the plate centre {center:?}"
        );
    }
}

/// `board_plane_point(x, y)` is the inverse of resolving a world point into the board's
/// own frame, over positive **and negative** coordinates.
///
/// Negative inputs are the point of the test. In the old edge-aligned frame the plate
/// occupied `[0, W]²` and every real call site passed non-negative coordinates, so a
/// sign or origin error in the mapping had no way to show itself. The centre-origin
/// frame puts half the plate at negative coordinates, and the boundary projection now
/// depends on that half being right.
#[test]
fn board_plane_point_round_trips_through_the_boards_own_axes() {
    let samples: [(f64, f64); 9] = [
        (0.0, 0.0),
        (0.25, 0.0),
        (0.0, 0.25),
        (-0.25, 0.0),
        (0.0, -0.25),
        (0.3, 0.4),
        (-0.3, 0.4),
        (0.3, -0.4),
        (-0.3, -0.4),
    ];

    for Case { label, board } in cases() {
        for (x, y) in samples {
            let point = board.board_plane_point(Length::from_meters(x), Length::from_meters(y));
            let (rx, ry, rz) = local_of(&board, &point);

            assert_close(rx, x, &label, &format!("round trip x for ({x}, {y})"));
            assert_close(ry, y, &label, &format!("round trip y for ({x}, {y})"));
            assert_close(rz, 0.0, &label, &format!("round trip z for ({x}, {y})"));
        }
    }
}

/// The four corner accessors really do describe a square of side `W` standing on a
/// corner: diagonals `W√2`, edges `W`, right angles at every vertex, all in one plane.
///
/// These are the rotation-invariant facts, and on their own they cannot detect the 45°
/// error — a square is a square whichever way you label its axes. They are here to catch
/// the *other* half of the failure mode: an accessor table edited to satisfy the frame
/// test by moving a corner to a place that is no longer a corner. Together with the
/// frame-pinning test above, the shape and its labelling are both nailed down.
#[test]
fn corner_accessors_describe_a_square_plate_standing_on_a_corner() {
    for Case { label, board } in cases() {
        let width = board.board_shape.board_width.as_meters();
        let diagonal = width * 2f64.sqrt();

        let top = board.top_corner();
        let bottom = board.bottom_corner();
        let left = board.left_corner();
        let right = board.right_corner();

        assert_close(
            (top - bottom).norm(),
            diagonal,
            &label,
            "top-to-bottom diagonal",
        );
        assert_close(
            (left - right).norm(),
            diagonal,
            &label,
            "left-to-right diagonal",
        );

        assert_close((top - left).norm(), width, &label, "top-left edge");
        assert_close((left - bottom).norm(), width, &label, "left-bottom edge");
        assert_close((bottom - right).norm(), width, &label, "bottom-right edge");
        assert_close((right - top).norm(), width, &label, "right-top edge");

        // A right angle at each of the four vertices, between the two edges meeting there.
        assert_close(
            (top - left).dot(&(bottom - left)),
            0.0,
            &label,
            "angle at the left corner",
        );
        assert_close(
            (left - bottom).dot(&(right - bottom)),
            0.0,
            &label,
            "angle at the bottom corner",
        );
        assert_close(
            (bottom - right).dot(&(top - right)),
            0.0,
            &label,
            "angle at the right corner",
        );
        assert_close(
            (right - top).dot(&(left - top)),
            0.0,
            &label,
            "angle at the top corner",
        );

        // Coplanarity: nothing the model exposes may leave the board plane.
        let normal = board.board_z_axis();
        let center = board.board_center();
        for (name, point) in [
            ("board_center", board.board_center()),
            ("top_corner", top),
            ("bottom_corner", bottom),
            ("left_corner", left),
            ("right_corner", right),
            ("left_circle_center", board.left_circle_center()),
            ("right_circle_center", board.right_circle_center()),
            ("top_circle_center", board.top_circle_center()),
        ] {
            assert_close(
                (point - center).dot(&normal),
                0.0,
                &label,
                &format!("{name} offset along the board normal"),
            );
        }
    }
}

/// The advertised axes form a right-handed frame: `X × Y = Z`.
///
/// Handedness is what stops a "fix" to the 45° problem from being applied as a mirror
/// rather than a rotation. A reflected frame would satisfy every distance and angle
/// assertion above and still produce an extrinsic that is wrong in a way no reprojection
/// error would reveal.
#[test]
fn board_axes_form_a_right_handed_frame() {
    for Case { label, board } in cases() {
        let cross = board.board_x_axis().cross(&board.board_y_axis());
        let z = board.board_z_axis();
        assert!(
            (cross - *z).norm() < TOL,
            "{label}: board_x_axis × board_y_axis = {cross:?}, expected board_z_axis {z:?}"
        );
    }
}

/// The three hole centres sit at `d = s√2` from the plate centre, along `+X`, `−X` and
/// `+Y` — and that arrangement is **not** invariant under the plate's own 90° rotations.
///
/// This matters more than it looks. The plate is a square: its outline alone leaves the
/// pose ambiguous to within four indistinguishable quarter turns, and points landing on
/// the plate's interior carry no in-plane information at all. The three-hole pattern is
/// the *only* feature in the entire model capable of resolving that ambiguity, so if the
/// holes were ever laid out symmetrically — three-fold, or mirror-symmetric about a
/// diagonal — ICP would have nothing to lock onto and the silent quarter-turn failure
/// mode would become unreachable by any downstream check.
#[test]
fn three_hole_pattern_breaks_the_plates_four_fold_symmetry() {
    for Case { label, board } in cases() {
        let d = hole_offset(&board);

        let holes = [
            ("left_circle_center", board.left_circle_center()),
            ("right_circle_center", board.right_circle_center()),
            ("top_circle_center", board.top_circle_center()),
        ];

        // Each hole is exactly d from the centre, in the plane...
        for (name, point) in holes {
            assert_close(
                (point - board.board_center()).norm(),
                d,
                &label,
                &format!("{name} distance from the plate centre"),
            );
        }

        // ...and along the diagonal its name claims. Distance alone is invariant under
        // any in-plane rotation, so without these three the asymmetry below would hold
        // for a hole pattern rotated 45° away from where the plate's holes are drilled.
        assert_local(
            &board,
            &label,
            "left_circle_center",
            board.left_circle_center(),
            d,
            0.0,
        );
        assert_local(
            &board,
            &label,
            "right_circle_center",
            board.right_circle_center(),
            -d,
            0.0,
        );
        assert_local(
            &board,
            &label,
            "top_circle_center",
            board.top_circle_center(),
            0.0,
            d,
        );

        let local: Vec<(f64, f64)> = holes
            .iter()
            .map(|(_, point)| {
                let (x, y, _) = local_of(&board, point);
                (x, y)
            })
            .collect();

        // Every non-trivial quarter turn about the board normal must move the hole set
        // to somewhere it was not. (x, y) -> (-y, x) is the +90° in-plane rotation.
        for quarter_turns in 1..4 {
            let rotated: Vec<(f64, f64)> = local
                .iter()
                .map(|&(x, y)| {
                    let mut p = (x, y);
                    for _ in 0..quarter_turns {
                        p = (-p.1, p.0);
                    }
                    p
                })
                .collect();

            let set_preserved = rotated.iter().all(|&(rx, ry)| {
                local
                    .iter()
                    .any(|&(x, y)| (rx - x).abs() < TOL && (ry - y).abs() < TOL)
            });

            assert!(
                !set_preserved,
                "{label}: the hole pattern {local:?} is unchanged by a {}° rotation about the \
                 board normal, so it cannot resolve the square plate's four-fold ambiguity",
                quarter_turns * 90
            );
        }
    }
}
