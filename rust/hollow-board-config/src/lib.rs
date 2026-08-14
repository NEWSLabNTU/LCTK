//! The geometric model of the hollow calibration board, and its canonical local frame.
//!
//! # The canonical board frame
//!
//! The plate is a square hung as a **diamond**: it stands on one corner, so its
//! diagonals run up and across rather than its edges. Every name in this module — top,
//! bottom, left and right *corner*, and the three hole positions — describes that
//! diamond, and the local frame is defined to match it:
//!
//! - **origin** — the plate's centre. This is the model's [`BoardModel::pose`]
//!   translation, so a published board pose is a pose *of the plate centre*.
//! - **+Z** — the board normal, pointing toward the sensor.
//! - **+Y** — from the centre toward the **top** corner.
//! - **+X** — `Y × Z`, which points from the centre toward the **left** corner.
//!
//! With `W` the plate's edge length, `R = W/√2` its half-diagonal, `s` the hole centre
//! shift and `d = s√2`:
//!
//! | accessor | local (x, y) |
//! |---|---|
//! | [`BoardModel::board_center`] | (0, 0) |
//! | [`BoardModel::top_corner`] | (0, +R) |
//! | [`BoardModel::bottom_corner`] | (0, −R) |
//! | [`BoardModel::left_corner`] | (+R, 0) |
//! | [`BoardModel::right_corner`] | (−R, 0) |
//! | [`BoardModel::left_circle_center`] | (+d, 0) |
//! | [`BoardModel::right_circle_center`] | (−d, 0) |
//! | [`BoardModel::top_circle_center`] | (0, +d) |
//!
//! The frame is right-handed: writing `u` for the up-diagonal and `v` for the
//! perpendicular in-plane direction, `Y × Z = u × (u×v) = −v`, and
//! `X × Y = (−v) × u = u × v = Z`.
//!
//! ## Two things that look like mistakes and are not
//!
//! **The left corner appears on an observer's right.** The corner accessors are named
//! from the *board's* point of view, not the sensor's. Renaming them would silently
//! reorder the corner lists every downstream consumer depends on, so the naming is
//! recorded here and deliberately left alone.
//!
//! **Z is the normal, not X.** The board-cluster detector uses the REP-103 convention,
//! where X is the normal. Adopting it here is a separate change: the quality-metric
//! module and the detection publisher both read this rotation's third column as the
//! normal. After the corner alignment above, that change reduces to a column
//! permutation and one sign flip.
//!
//! ## Why the three holes matter
//!
//! A square is symmetric under four 90° rotations, and points landing on the plate's
//! interior say nothing about rotation within the board plane. The **only** feature that
//! can resolve which corner is which is the three-hole asymmetry: two holes sit on the
//! horizontal diagonal at ±d, one sits on the vertical diagonal at +d, and none sits at
//! −d. That single missing fourth hole is what makes the pose observable.

use aruco_config::MultiArucoPattern;
use measurements::Length;
use nalgebra as na;
use serde::{Deserialize, Serialize};

pub use aruco_config::MarkerPaperPlacement;
use std::borrow::Borrow;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoardShape {
    /// The entire board rectangle size.
    #[serde(with = "newslab_serde_measurements::length")]
    pub board_width: Length,
    /// The hole radius.
    #[serde(with = "newslab_serde_measurements::length")]
    pub hole_radius: Length,
    /// The displacement of the hole center from the center of rectangle board.
    #[serde(with = "newslab_serde_measurements::length")]
    pub hole_center_shift: Length,
}

/// The model of a square board with three holes on it.
///
/// See the module documentation for the local frame this model's coordinates are
/// expressed in.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardModel {
    /// Pose of the **plate centre** in the sensor frame.
    #[serde(with = "newslab_serde_nalgebra::isometry3_as_euler_angles")]
    pub pose: na::Isometry3<f64>,
    /// The marker size on the board.
    pub marker_paper_size: Length,
    /// Where that paper sits on the plate. See [`MarkerPaperPlacement`].
    pub marker_paper_placement: MarkerPaperPlacement,
    pub board_shape: BoardShape,
}

impl BoardModel {
    pub fn board_x_axis(&self) -> na::UnitVector3<f64> {
        self.pose * na::Vector3::x_axis()
    }

    pub fn board_y_axis(&self) -> na::UnitVector3<f64> {
        self.pose * na::Vector3::y_axis()
    }

    pub fn board_z_axis(&self) -> na::UnitVector3<f64> {
        self.pose * na::Vector3::z_axis()
    }

    /// Half the plate's diagonal, `W/√2` — the distance from the centre to any corner.
    ///
    /// This is the plate's radius in the local frame's own axes, and it is the `R` the
    /// module documentation's coordinate table is written in terms of.
    pub fn half_diagonal(&self) -> Length {
        self.board_shape.board_width / 2f64.sqrt()
    }

    /// Maps local in-plane coordinates, in **metres**, to a world point.
    ///
    /// The raw-`f64` primitive that [`BoardModel::board_plane_point`] and the
    /// correspondence search are both written in terms of, so that the conversion from
    /// `(x, y)` to a world point lives in exactly one place.
    pub fn board_plane_point_meters(&self, x: f64, y: f64) -> na::Point3<f64> {
        self.pose.transform_point(&na::Point3::origin())
            + self.board_x_axis().scale(x)
            + self.board_y_axis().scale(y)
    }

    pub fn board_plane_point(&self, x: Length, y: Length) -> na::Point3<f64> {
        self.board_plane_point_meters(x.as_meters(), y.as_meters())
    }

    /// The inverse of [`BoardModel::board_plane_point`]: resolves a world point into the
    /// board's local in-plane coordinates, discarding its distance along the normal.
    ///
    /// Exposed because every consumer that wants to ask *where on the board is this* was
    /// otherwise obliged to re-derive the projection from the axis accessors, and each
    /// such re-derivation is a chance to pick the wrong axis.
    pub fn plane_coordinates(&self, point: &na::Point3<f64>) -> (Length, Length) {
        let offset = point - self.pose.transform_point(&na::Point3::origin());
        (
            Length::from_meters(offset.dot(&self.board_x_axis())),
            Length::from_meters(offset.dot(&self.board_y_axis())),
        )
    }

    pub fn board_center(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(0.0, 0.0)
    }

    pub fn top_corner(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(0.0, self.half_diagonal().as_meters())
    }

    pub fn bottom_corner(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(0.0, -self.half_diagonal().as_meters())
    }

    pub fn left_corner(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(self.half_diagonal().as_meters(), 0.0)
    }

    pub fn right_corner(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(-self.half_diagonal().as_meters(), 0.0)
    }

    /// Distance from the plate centre to each hole centre, `s√2`.
    ///
    /// The holes sit on the plate's diagonals, so the configured `hole_center_shift`,
    /// which is measured along the plate's *edges*, reaches `√2` times as far along a
    /// diagonal.
    fn hole_center_distance(&self) -> f64 {
        self.board_shape.hole_center_shift.as_meters() * 2f64.sqrt()
    }

    pub fn left_circle_center(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(self.hole_center_distance(), 0.0)
    }

    pub fn right_circle_center(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(-self.hole_center_distance(), 0.0)
    }

    pub fn top_circle_center(&self) -> na::Point3<f64> {
        self.board_plane_point_meters(0.0, self.hole_center_distance())
    }

    /// Unit vector in the board plane, from the plate centre toward the **top** corner.
    ///
    /// Named for the plate's geometry rather than the model's axes, so callers that care
    /// about the physical board do not have to know which axis currently plays that
    /// role.
    pub fn plate_up_diagonal(&self) -> na::UnitVector3<f64> {
        self.board_y_axis()
    }

    /// Unit vector in the board plane, from the plate centre toward the **left** corner.
    pub fn plate_left_diagonal(&self) -> na::UnitVector3<f64> {
        self.board_x_axis()
    }

    /// Maps a point given in the **marker paper's own coordinates** to the world.
    ///
    /// Paper coordinates run along the paper's edges, which are parallel to the plate's
    /// edges and therefore at 45° to the board frame's axes: the origin is the paper
    /// corner nearest the plate's bottom corner, `u` runs toward the plate's left corner
    /// and `v` toward its right corner, both spanning `[0, marker_paper_size]`.
    ///
    /// This is the single place that knows *where the paper is on the plate*, and the
    /// only place that bridges the paper's edge-aligned coordinates and the plate's
    /// corner-aligned frame. Routing every marker accessor through it means the marker
    /// layout's own arithmetic never has to learn about the plate's frame.
    pub fn marker_paper_point(&self, u: Length, v: Length) -> na::Point3<f64> {
        let up = self.plate_up_diagonal();
        let left = self.plate_left_diagonal();

        let paper_center = self.board_center()
            + left.scale(self.marker_paper_placement.toward_left_corner.as_meters())
            + up.scale(self.marker_paper_placement.toward_top_corner.as_meters());

        // The paper's edge directions, as unit vectors: bisectors of the two diagonals.
        let u_dir = (*up + *left) / 2f64.sqrt();
        let v_dir = (*up - *left) / 2f64.sqrt();

        let half_paper = self.marker_paper_size.as_meters() / 2.0;
        paper_center - (u_dir + v_dir).scale(half_paper)
            + u_dir.scale(u.as_meters())
            + v_dir.scale(v.as_meters())
    }

    pub fn marker_bottom_corner(&self) -> na::Point3<f64> {
        self.marker_paper_point(Length::from_meters(0.0), Length::from_meters(0.0))
    }

    pub fn marker_top_corner(&self) -> na::Point3<f64> {
        self.marker_paper_point(self.marker_paper_size, self.marker_paper_size)
    }

    pub fn marker_left_corner(&self) -> na::Point3<f64> {
        self.marker_paper_point(self.marker_paper_size, Length::from_meters(0.0))
    }

    pub fn marker_right_corner(&self) -> na::Point3<f64> {
        self.marker_paper_point(Length::from_meters(0.0), self.marker_paper_size)
    }

    pub fn marker_center(&self) -> na::Point3<f64> {
        self.marker_paper_point(self.marker_paper_size / 2.0, self.marker_paper_size / 2.0)
    }

    /// Computes the 3D positions of marker corner points
    ///
    /// The returned vector has format `[bottom_corners, left_corners, right_corners, top_corners]`,
    /// where each `*_corners` is a vector of points in order `[right, top, left, bottom]`.
    pub fn multi_marker_corners(&self, pattern: &MultiArucoPattern) -> Vec<Vec<na::Point3<f64>>> {
        let MultiArucoPattern {
            board_border_size,
            marker_square_size_ratio,
            num_squares_per_side,
            ..
        } = *pattern;

        // L-04: this routine hardcodes a 2x2 marker grid (it places exactly 4
        // markers below), so make that assumption explicit instead of baking a bare
        // "/ 2.0" divisor. A config with a different grid would otherwise produce
        // silently wrong object points; assert it loudly in debug builds and derive
        // the divisor from the pattern.
        debug_assert_eq!(
            num_squares_per_side, 2,
            "multi_marker_corners only supports a 2x2 marker grid"
        );
        let square_size =
            (self.marker_paper_size - 2.0 * board_border_size) / num_squares_per_side as f64;
        let marker_size = square_size * marker_square_size_ratio.raw();
        let marker_border = (square_size - marker_size) / 2.0;

        let make_corners = |[base_u, base_v]: [_; 2]| {
            let bottom = self.marker_paper_point(base_u, base_v);
            let left = self.marker_paper_point(base_u + marker_size, base_v);
            let right = self.marker_paper_point(base_u, base_v + marker_size);
            let top = self.marker_paper_point(base_u + marker_size, base_v + marker_size);
            vec![right, top, left, bottom]
        };

        let origin_u = board_border_size + marker_border;
        let origin_v = board_border_size + marker_border;

        let bottom_corners = make_corners([origin_u, origin_v]);
        let left_corners = make_corners([origin_u + square_size, origin_v]);
        let right_corners = make_corners([origin_u, origin_v + square_size]);
        let top_corners = make_corners([origin_u + square_size, origin_v + square_size]);

        vec![bottom_corners, left_corners, right_corners, top_corners]
    }

    // `marker_pose` used to live here. It paired the marker paper's centre with the
    // board model's own rotation — which no longer agrees with the paper's orientation,
    // the two now differing by the 45° between the plate's edges and its diagonals. It
    // had no callers, so it is deleted rather than shipped as an API that is wrong by an
    // eighth of a turn.

    /// Finds, for each input point, the nearest point on the physical board.
    ///
    /// Parallel build: one rayon pass. The per-point body is shared with the serial
    /// build — see [`CorrespondenceContext::corresponding_point`].
    #[cfg(feature = "parallel")]
    pub fn find_correspondences<InputPoint, DataIter>(
        &self,
        points: DataIter,
    ) -> Option<Vec<(InputPoint, na::Point3<f64>)>>
    where
        DataIter:
            IntoIterator<Item = InputPoint> + rayon::iter::IntoParallelIterator<Item = InputPoint>,
        InputPoint: Borrow<na::Point3<f64>> + Send,
    {
        let context = self.correspondence_context();
        let correspondings: Vec<_> = points
            .into_par_iter()
            .map(|point| {
                let corresponding = context.corresponding_point(point.borrow());
                (point, corresponding)
            })
            .collect();
        Some(correspondings)
    }

    /// Finds, for each input point, the nearest point on the physical board.
    ///
    /// Serial build. The per-point body is shared with the parallel build — see
    /// [`CorrespondenceContext::corresponding_point`].
    #[cfg(not(feature = "parallel"))]
    pub fn find_correspondences<InputPoint, DataIter>(
        &self,
        points: DataIter,
    ) -> Option<Vec<(InputPoint, na::Point3<f64>)>>
    where
        DataIter: IntoIterator<Item = InputPoint>,
        InputPoint: Borrow<na::Point3<f64>>,
    {
        let context = self.correspondence_context();
        let correspondings: Vec<_> = points
            .into_iter()
            .map(|point| {
                let corresponding = context.corresponding_point(point.borrow());
                (point, corresponding)
            })
            .collect();
        Some(correspondings)
    }

    /// Hoists everything about the board that does not vary per point.
    fn correspondence_context(&self) -> CorrespondenceContext<'_> {
        CorrespondenceContext {
            model: self,
            origin: self.pose.transform_point(&na::Point3::origin()),
            board_x_axis: self.board_x_axis(),
            board_y_axis: self.board_y_axis(),
            half_diagonal: self.half_diagonal().as_meters(),
            hole_radius: self.board_shape.hole_radius.as_meters(),
            left_circle_center: self.left_circle_center(),
            right_circle_center: self.right_circle_center(),
            top_circle_center: self.top_circle_center(),
        }
    }
}

/// Projects `(x, y)` onto the closed L¹ ball of radius `radius`, i.e. onto the filled
/// diamond `|x| + |y| ≤ radius`.
///
/// The plate is that diamond in the board's local coordinates, because the frame's axes
/// run along its diagonals. Two properties the old edge-aligned frame had are gone and
/// are worth naming, because code written against them silently produces wrong answers
/// here: membership no longer factorises per coordinate, and the nearest point is no
/// longer found by clamping each coordinate independently.
///
/// The projection folds the point into the first quadrant, drops a perpendicular onto
/// the line `a + b = radius`, and snaps to a vertex when that foot falls outside the
/// segment. At most one of `pa`/`pb` can go negative, since they sum to `radius > 0`.
///
/// `copysign` rather than `signum`, because `signum(0.0)` is `1.0` and would push a
/// point sitting exactly on an axis off to one side.
fn project_onto_diamond(x: f64, y: f64, radius: f64) -> (f64, f64) {
    let (a, b) = (x.abs(), y.abs());
    if a + b <= radius {
        return (x, y);
    }

    let t = (a + b - radius) / 2.0;
    let (mut pa, mut pb) = (a - t, b - t);
    if pa < 0.0 {
        pa = 0.0;
        pb = radius;
    } else if pb < 0.0 {
        pa = radius;
        pb = 0.0;
    }

    (pa.copysign(x), pb.copysign(y))
}

/// Board geometry hoisted out of the per-point correspondence loop.
///
/// Exists so the parallel and serial `find_correspondences` wrappers can share one
/// per-point body instead of two textually identical copies. Only the wrappers'
/// iterator and trait bounds differ.
struct CorrespondenceContext<'a> {
    model: &'a BoardModel,
    origin: na::Point3<f64>,
    board_x_axis: na::UnitVector3<f64>,
    board_y_axis: na::UnitVector3<f64>,
    half_diagonal: f64,
    hole_radius: f64,
    left_circle_center: na::Point3<f64>,
    right_circle_center: na::Point3<f64>,
    top_circle_center: na::Point3<f64>,
}

impl CorrespondenceContext<'_> {
    /// The nearest point on the physical board to `point`.
    ///
    /// The board is the plate minus its three holes, so the search is: drop onto the
    /// board plane; if that lands outside the plate, project onto the plate's boundary;
    /// otherwise, if it landed inside a hole, push it out to that hole's rim.
    fn corresponding_point(&self, point: &na::Point3<f64>) -> na::Point3<f64> {
        let Self {
            model,
            origin,
            board_x_axis,
            board_y_axis,
            half_diagonal,
            hole_radius,
            left_circle_center,
            right_circle_center,
            top_circle_center,
        } = self;

        // Drop the point onto the board plane, in the board's own in-plane coordinates.
        let offset = point - origin;
        let x = offset.dot(board_x_axis);
        let y = offset.dot(board_y_axis);

        // The plate is the diamond |x| + |y| <= R. Outside it, the answer is on the
        // boundary and no hole can be involved.
        if x.abs() + y.abs() > *half_diagonal {
            let (border_x, border_y) = project_onto_diamond(x, y, *half_diagonal);
            return model.board_plane_point_meters(border_x, border_y);
        }

        let plane_projection_point = model.board_plane_point_meters(x, y);

        // Inside the plate: check the three holes and, if the point fell in one, push it
        // radially out to that hole's rim.
        let find_border_point_on_circle = |circle_center: &na::Point3<f64>| {
            let vec_circle_center_to_proj = plane_projection_point - circle_center;
            let dist_circle_center_to_proj = vec_circle_center_to_proj.norm();
            let is_inside_circle = dist_circle_center_to_proj < *hole_radius;

            let circle_border_point = if dist_circle_center_to_proj < 1e-10 {
                // Degenerate case: the point sits exactly at the hole centre, so there is
                // no radial direction. Any rim point is equally near; pick one.
                circle_center + board_x_axis.scale(*hole_radius)
            } else {
                let radial_unit = na::Unit::new_normalize(vec_circle_center_to_proj);
                circle_center + radial_unit.scale(*hole_radius)
            };

            (is_inside_circle, circle_border_point)
        };

        let (is_inside_left_circle, left_circle_border_point) =
            find_border_point_on_circle(left_circle_center);
        let (is_inside_right_circle, right_circle_border_point) =
            find_border_point_on_circle(right_circle_center);
        let (is_inside_top_circle, top_circle_border_point) =
            find_border_point_on_circle(top_circle_center);

        if is_inside_left_circle {
            left_circle_border_point
        } else if is_inside_right_circle {
            right_circle_border_point
        } else if is_inside_top_circle {
            top_circle_border_point
        } else {
            plane_projection_point
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use measurements::Length;

    /// A 1 m plate at the origin. In the canonical frame this is a diamond: its corners
    /// sit on the axes at ±R, and its edges are the four lines `|x| + |y| = R`.
    const R: f64 = std::f64::consts::FRAC_1_SQRT_2; // half-diagonal of a 1 m plate
    /// Hole centres sit `s√2` from the plate centre, along the plate's diagonals.
    const HOLE_DISTANCE: f64 = 0.2 * std::f64::consts::SQRT_2;
    const HOLE_RADIUS: f64 = 0.15;

    /// Tight: these cases are exact decimal geometry, so only f64 round-off is in play.
    const TOL: f64 = 1e-9;

    /// Helper function to create a simple board model for testing
    fn create_test_board_model() -> BoardModel {
        let board_shape = BoardShape {
            board_width: Length::from_meters(1.0),         // 1m board
            hole_radius: Length::from_meters(HOLE_RADIUS), // 15cm holes
            hole_center_shift: Length::from_meters(0.2),   // 20cm from center, along an edge
        };

        BoardModel {
            pose: na::Isometry3::identity(),
            marker_paper_size: Length::from_meters(0.3),
            marker_paper_placement: MarkerPaperPlacement::flush_with_bottom_corner(
                board_shape.board_width,
                Length::from_meters(0.3),
            ),
            board_shape,
        }
    }

    /// Runs one point through the correspondence search and returns where it landed.
    fn correspondence_of(board: &BoardModel, point: na::Point3<f64>) -> na::Point3<f64> {
        let result = board
            .find_correspondences(vec![point])
            .expect("find_correspondences returns a result");
        assert_eq!(result.len(), 1, "one correspondence per input point");
        result[0].1
    }

    fn assert_lands_on(actual: na::Point3<f64>, expected: na::Point3<f64>, what: &str) {
        let err = (actual - expected).norm();
        assert!(
            err < TOL,
            "{what}: got {actual:?}, expected {expected:?} (error {err:e} m)"
        );
    }

    #[test]
    fn a_point_on_the_plate_is_its_own_correspondence() {
        let board = create_test_board_model();
        // The plate centre: inside the diamond, and outside all three holes.
        let point = na::Point3::new(0.0, 0.0, 0.0);
        assert_lands_on(
            correspondence_of(&board, point),
            point,
            "plate centre maps to itself",
        );
    }

    #[test]
    fn a_point_above_the_plate_drops_straight_down_onto_it() {
        let board = create_test_board_model();
        assert_lands_on(
            correspondence_of(&board, na::Point3::new(0.0, 0.0, 0.5)),
            na::Point3::new(0.0, 0.0, 0.0),
            "point 0.5 m above the plate centre",
        );
    }

    #[test]
    fn a_point_below_the_plate_drops_straight_up_onto_it() {
        let board = create_test_board_model();
        assert_lands_on(
            correspondence_of(&board, na::Point3::new(0.0, 0.0, -0.5)),
            na::Point3::new(0.0, 0.0, 0.0),
            "point 0.5 m below the plate centre",
        );
    }

    /// The plate's edges run at 45° to the frame's axes, so projecting onto one is
    /// **not** a per-coordinate clamp. This is the case a clamp gets wrong: it would
    /// leave the point at `(R, R)`, which is off the plate entirely.
    #[test]
    fn a_point_beyond_an_edge_projects_perpendicularly_onto_that_edge() {
        let board = create_test_board_model();
        let point = na::Point3::new(R, R, 0.0);
        assert_lands_on(
            correspondence_of(&board, point),
            na::Point3::new(R / 2.0, R / 2.0, 0.0),
            "point beyond the top-left edge",
        );
    }

    /// Every corner must be reachable, and a point beyond one must snap to it exactly
    /// rather than to some nearby edge point. This is the branch where a quadrant-folding
    /// bug shows up.
    #[test]
    fn a_point_beyond_a_corner_snaps_to_that_corner() {
        let board = create_test_board_model();
        for (point, expected, name) in [
            (na::Point3::new(1.5, 0.0, 0.0), board.left_corner(), "left"),
            (
                na::Point3::new(-1.5, 0.0, 0.0),
                board.right_corner(),
                "right",
            ),
            (na::Point3::new(0.0, 1.5, 0.0), board.top_corner(), "top"),
            (
                na::Point3::new(0.0, -1.5, 0.0),
                board.bottom_corner(),
                "bottom",
            ),
        ] {
            assert_lands_on(
                correspondence_of(&board, point),
                expected,
                &format!("point beyond the {name} corner"),
            );
        }
    }

    /// The three holes are the only feature that can resolve the square's four-fold
    /// symmetry, so each one must actually be modelled. A point inside one is pushed
    /// radially out to its rim.
    #[test]
    fn a_point_inside_a_hole_is_pushed_out_to_that_holes_rim() {
        let board = create_test_board_model();
        for (center, name) in [
            (board.left_circle_center(), "left"),
            (board.right_circle_center(), "right"),
            (board.top_circle_center(), "top"),
        ] {
            let correspondence = correspondence_of(&board, center);
            let distance = (correspondence - center).norm();
            assert!(
                (distance - HOLE_RADIUS).abs() < TOL,
                "{name} hole centre should map to the rim at {HOLE_RADIUS} m, got {distance}"
            );
            assert!(
                correspondence.z.abs() < TOL,
                "{name} hole rim point should stay in the board plane"
            );
        }
    }

    #[test]
    fn a_point_inside_a_hole_leaves_along_its_own_radius() {
        let board = create_test_board_model();
        let center = board.left_circle_center();
        // 0.05 m from the hole centre, so still well inside its 0.15 m radius.
        let point = center + na::Vector3::new(0.05, 0.0, 0.0);
        let direction = (point - center).normalize();
        assert_lands_on(
            correspondence_of(&board, point),
            center + direction.scale(HOLE_RADIUS),
            "point inside the left hole",
        );
    }

    /// The holes sit on the plate's *diagonals*, `s√2` from the centre — not `s`, which
    /// is measured along an edge. Getting this wrong puts every hole in the wrong place
    /// while leaving the plate outline correct.
    #[test]
    fn hole_centres_sit_on_the_diagonals_at_the_scaled_shift() {
        let board = create_test_board_model();
        assert_lands_on(
            board.left_circle_center(),
            na::Point3::new(HOLE_DISTANCE, 0.0, 0.0),
            "left hole centre",
        );
        assert_lands_on(
            board.right_circle_center(),
            na::Point3::new(-HOLE_DISTANCE, 0.0, 0.0),
            "right hole centre",
        );
        assert_lands_on(
            board.top_circle_center(),
            na::Point3::new(0.0, HOLE_DISTANCE, 0.0),
            "top hole centre",
        );
    }

    #[test]
    fn a_batch_of_points_is_answered_pointwise() {
        let board = create_test_board_model();
        let points = vec![
            na::Point3::new(0.0, 0.0, 0.0), // on the plate, no hole
            na::Point3::new(1.5, 0.0, 0.0), // beyond the left corner
            board.left_circle_center(),     // inside the left hole
            na::Point3::new(0.0, 0.0, 0.5), // above the plate
            na::Point3::new(R, R, 0.0),     // beyond the top-left edge
        ];

        let correspondences = board
            .find_correspondences(points.clone())
            .expect("find_correspondences returns a result");
        assert_eq!(correspondences.len(), points.len());

        assert_lands_on(
            correspondences[0].1,
            na::Point3::new(0.0, 0.0, 0.0),
            "batched plate centre",
        );
        assert_lands_on(correspondences[1].1, board.left_corner(), "batched corner");
        let hole_distance = (correspondences[2].1 - board.left_circle_center()).norm();
        assert!(
            (hole_distance - HOLE_RADIUS).abs() < TOL,
            "batched hole point should land on the rim, got {hole_distance}"
        );
        assert_lands_on(
            correspondences[3].1,
            na::Point3::new(0.0, 0.0, 0.0),
            "batched point above the plate",
        );
        assert_lands_on(
            correspondences[4].1,
            na::Point3::new(R / 2.0, R / 2.0, 0.0),
            "batched point beyond an edge",
        );
    }

    /// Everything above uses an identity pose, where a wrong axis choice can cancel out.
    /// This runs the same question against a board that is rotated and translated.
    #[test]
    fn correspondences_follow_the_board_when_it_is_posed() {
        let board_shape = BoardShape {
            board_width: Length::from_meters(1.0),
            hole_radius: Length::from_meters(HOLE_RADIUS),
            hole_center_shift: Length::from_meters(0.2),
        };

        let rotation =
            na::UnitQuaternion::from_euler_angles(0.3, -0.7, std::f64::consts::FRAC_PI_4);
        let pose = na::Isometry3::from_parts(na::Translation3::new(2.0, -1.0, 0.5), rotation);

        let board = BoardModel {
            pose,
            marker_paper_size: Length::from_meters(0.3),
            marker_paper_placement: MarkerPaperPlacement::flush_with_bottom_corner(
                board_shape.board_width,
                Length::from_meters(0.3),
            ),
            board_shape,
        };

        // A point off the plate's face projects back onto the plate centre.
        let board_center = board.board_center();
        let above_center = board_center + board.board_z_axis().scale(0.5);
        assert_lands_on(
            correspondence_of(&board, above_center),
            board_center,
            "point above a posed board's centre",
        );

        // A point far beyond the left corner snaps to that corner, wherever it now is.
        let beyond_left = board_center + board.plate_left_diagonal().scale(1.5);
        assert_lands_on(
            correspondence_of(&board, beyond_left),
            board.left_corner(),
            "point beyond a posed board's left corner",
        );
    }

    /// The origin of the board frame is the plate centre, so the model's pose translation
    /// and its centre accessor must be the same point. Anything else means a consumer
    /// reading `pose.translation` and a consumer calling `board_center()` disagree about
    /// where the board is.
    #[test]
    fn the_pose_translation_is_the_plate_centre() {
        let board = create_test_board_model();
        assert_lands_on(
            board.board_center(),
            board.pose.transform_point(&na::Point3::origin()),
            "identity pose",
        );

        let posed = BoardModel {
            pose: na::Isometry3::from_parts(
                na::Translation3::new(1.0, 2.0, 3.0),
                na::UnitQuaternion::from_euler_angles(0.2, 0.4, 0.6),
            ),
            ..create_test_board_model()
        };
        assert_lands_on(
            posed.board_center(),
            posed.pose.transform_point(&na::Point3::origin()),
            "posed board",
        );
    }
}
