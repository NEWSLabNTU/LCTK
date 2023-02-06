#[cfg(feature = "kiss3d")]
mod with_kiss3d;

use approx::abs_diff_eq;
use aruco_config::multi_aruco::MultiArucoPattern;
use measurements::Length;
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::borrow::Borrow;

const EPS_F64: f64 = 1e-4;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoardShape {
    /// The entire board rectangle size.
    #[serde(with = "serde_types::serde_length")]
    pub board_width: Length,
    /// The hole radius.
    #[serde(with = "serde_types::serde_length")]
    pub hole_radius: Length,
    /// The displacement of the hole center from the center of rectangle board.
    #[serde(with = "serde_types::serde_length")]
    pub hole_center_shift: Length,
}

/// The model of a square board with three holes on it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardModel {
    #[serde(with = "serde_types::serde_euler_isometry3")]
    pub pose: na::Isometry3<f64>,
    /// The marker size on the board.
    pub marker_paper_size: Length,
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

    pub fn board_plane_point(&self, x: Length, y: Length) -> na::Point3<f64> {
        self.pose.transform_point(&na::Point3::origin())
            + self.board_x_axis().scale(x.as_meters())
            + self.board_y_axis().scale(y.as_meters())
    }

    pub fn board_center(&self) -> na::Point3<f64> {
        self.board_plane_point(
            self.board_shape.board_width / 2.0,
            self.board_shape.board_width / 2.0,
        )
    }

    pub fn top_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(self.board_shape.board_width, self.board_shape.board_width)
    }

    pub fn bottom_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(Length::from_meters(0.0), Length::from_meters(0.0))
    }

    pub fn left_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(self.board_shape.board_width, Length::from_meters(0.0))
    }

    pub fn right_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(Length::from_meters(0.0), self.board_shape.board_width)
    }

    pub fn left_circle_center(&self) -> na::Point3<f64> {
        self.board_plane_point(
            self.board_shape.board_width / 2.0 + self.board_shape.hole_center_shift,
            self.board_shape.board_width / 2.0 - self.board_shape.hole_center_shift,
        )
    }

    pub fn right_circle_center(&self) -> na::Point3<f64> {
        self.board_plane_point(
            self.board_shape.board_width / 2.0 - self.board_shape.hole_center_shift,
            self.board_shape.board_width / 2.0 + self.board_shape.hole_center_shift,
        )
    }

    pub fn top_circle_center(&self) -> na::Point3<f64> {
        self.board_plane_point(
            self.board_shape.board_width / 2.0 + self.board_shape.hole_center_shift,
            self.board_shape.board_width / 2.0 + self.board_shape.hole_center_shift,
        )
    }

    pub fn marker_bottom_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(Length::from_meters(0.0), Length::from_meters(0.0))
    }

    pub fn marker_top_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(self.marker_paper_size, self.marker_paper_size)
    }

    pub fn marker_left_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(self.marker_paper_size, Length::from_meters(0.0))
    }

    pub fn marker_right_corner(&self) -> na::Point3<f64> {
        self.board_plane_point(Length::from_meters(0.0), self.marker_paper_size)
    }

    pub fn marker_center(&self) -> na::Point3<f64> {
        self.board_plane_point(self.marker_paper_size / 2.0, self.marker_paper_size / 2.0)
    }

    /// Computes the 3D positions of marker corner points
    ///
    /// The returned vector has format `[bottom_corners, left_corners, right_corners, top_corners]`,
    /// where each `*_corners` is a vector of points in order `[right, top, left, bottom]`.
    pub fn multi_marker_corners(&self, pattern: &MultiArucoPattern) -> Vec<Vec<na::Point3<f64>>> {
        let MultiArucoPattern {
            board_border_size,
            marker_square_size_ratio,
            ..
        } = *pattern;

        let square_size = (self.marker_paper_size - 2.0 * board_border_size) / 2.0;
        let marker_size = square_size * marker_square_size_ratio.raw();
        let marker_border = (square_size - marker_size) / 2.0;

        let make_corners = |[base_x, base_y]: [_; 2]| {
            let bottom = self.board_plane_point(base_x, base_y);
            let left = self.board_plane_point(base_x + marker_size, base_y);
            let right = self.board_plane_point(base_x, base_y + marker_size);
            let top = self.board_plane_point(base_x + marker_size, base_y + marker_size);
            vec![right, top, left, bottom]
        };

        let origin_x = board_border_size + marker_border;
        let origin_y = board_border_size + marker_border;

        let bottom_corners = make_corners([origin_x, origin_y]);
        let left_corners = make_corners([origin_x + square_size, origin_y]);
        let right_corners = make_corners([origin_x, origin_y + square_size]);
        let top_corners = make_corners([origin_x + square_size, origin_y + square_size]);

        vec![bottom_corners, left_corners, right_corners, top_corners]
    }

    pub fn marker_pose(&self) -> na::Isometry3<f64> {
        let translation = na::Translation3::from(self.marker_center() - na::Point3::origin());
        na::Isometry3::from_parts(translation, self.pose.rotation)
    }

    pub fn find_correspondences<InputPoint, DataIter>(
        &self,
        points: DataIter,
    ) -> Option<Vec<(InputPoint, na::Point3<f64>)>>
    where
        DataIter: IntoIterator<Item = InputPoint>,
        InputPoint: Borrow<na::Point3<f64>>,
    {
        let half_board_diagonal = self.board_shape.board_width / 2f64.sqrt();
        let board_x_axis = self.board_x_axis();
        let board_y_axis = self.board_y_axis();
        let board_z_axis = self.board_z_axis();
        let top_corner = self.top_corner();
        let bottom_corner = self.bottom_corner();
        let left_corner = self.left_corner();
        let right_corner = self.right_corner();
        let left_circle_center = self.left_circle_center();
        let right_circle_center = self.right_circle_center();
        let top_circle_center = self.top_circle_center();

        debug_assert!(abs_diff_eq!(
            (board_x_axis.cross(&board_y_axis) - *board_z_axis).norm(),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (top_corner - left_corner).dot(&(top_corner - right_corner)),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (bottom_corner - left_corner).dot(&(bottom_corner - right_corner)),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (left_corner - top_corner).dot(&(left_corner - bottom_corner)),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (right_corner - top_corner).dot(&(right_corner - bottom_corner)),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (top_corner - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (bottom_corner - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (left_corner - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (right_corner - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (left_circle_center - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (right_circle_center - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (top_circle_center - self.board_center()).dot(&board_z_axis),
            0.0,
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (top_corner - left_corner).norm(),
            self.board_shape.board_width.as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (left_corner - bottom_corner).norm(),
            self.board_shape.board_width.as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (bottom_corner - right_corner).norm(),
            self.board_shape.board_width.as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (right_corner - top_corner).norm(),
            self.board_shape.board_width.as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (left_circle_center - self.board_center()).norm(),
            (self.board_shape.hole_center_shift * 2f64.sqrt()).as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (right_circle_center - self.board_center()).norm(),
            (self.board_shape.hole_center_shift * 2f64.sqrt()).as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (top_circle_center - self.board_center()).norm(),
            (self.board_shape.hole_center_shift * 2f64.sqrt()).as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (left_circle_center - left_corner).norm(),
            (half_board_diagonal - self.board_shape.hole_center_shift * 2f64.sqrt()).as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (right_circle_center - right_corner).norm(),
            (half_board_diagonal - self.board_shape.hole_center_shift * 2f64.sqrt()).as_meters(),
            epsilon = EPS_F64
        ));
        debug_assert!(abs_diff_eq!(
            (top_circle_center - top_corner).norm(),
            (half_board_diagonal - self.board_shape.hole_center_shift * 2f64.sqrt()).as_meters(),
            epsilon = EPS_F64
        ));

        let correspondings: Vec<_> = points
            .into_iter()
            .map(|point_generic| {
                let point = point_generic.borrow();

                // fidn projection point on board plane (regardless of boundary)
                let vec_point_to_origin = self.bottom_corner() - point;
                let vec_point_to_proj = board_z_axis.scale(vec_point_to_origin.dot(&board_z_axis));
                let plane_projection_point: na::Point3<_> = point + vec_point_to_proj;

                // check if projection point is outside the board
                let vec_origin_to_proj = plane_projection_point - self.bottom_corner();
                debug_assert!(abs_diff_eq!(
                    vec_origin_to_proj.dot(&board_z_axis),
                    0.0,
                    epsilon = EPS_F64
                ));

                let plane_position = {
                    let x = Length::from_meters(vec_origin_to_proj.dot(&board_x_axis));
                    let y = Length::from_meters(vec_origin_to_proj.dot(&board_y_axis));
                    (x, y)
                };

                let border_position = {
                    let (x, y) = plane_position;
                    let border_x = Length::from_meters(
                        x.as_meters()
                            .clamp(0.0, self.board_shape.board_width.as_meters()),
                    );
                    let border_y = Length::from_meters(
                        y.as_meters()
                            .clamp(0.0, self.board_shape.board_width.as_meters()),
                    );
                    (border_x, border_y)
                };

                let corresponding_point = if plane_position != border_position {
                    let (x, y) = border_position;
                    self.board_plane_point(x, y)
                } else {
                    // check if projection point is inside one of the circles
                    // and find nearest point on circles
                    let find_border_point_on_circle = |circle_center: &na::Point3<f64>| {
                        let vec_circle_center_to_proj = plane_projection_point - circle_center;
                        let dist_circle_center_to_proj = vec_circle_center_to_proj.norm();
                        let is_inside_circle =
                            dist_circle_center_to_proj < self.board_shape.hole_radius.as_meters();
                        let circle_border_point = {
                            let radical_unit = na::Unit::new_normalize(vec_circle_center_to_proj);
                            circle_center
                                + radical_unit.scale(self.board_shape.hole_radius.as_meters())
                        };

                        debug_assert!(abs_diff_eq!(
                            vec_circle_center_to_proj.dot(&board_z_axis),
                            0.0,
                            epsilon = EPS_F64
                        ));
                        debug_assert!(abs_diff_eq!(
                            (circle_border_point - self.bottom_corner()).dot(&board_z_axis),
                            0.0,
                            epsilon = EPS_F64
                        ));

                        (is_inside_circle, circle_border_point)
                    };

                    let (is_inside_left_circle, left_circle_border_point) =
                        find_border_point_on_circle(&left_circle_center);
                    let (is_inside_right_circle, right_circle_border_point) =
                        find_border_point_on_circle(&right_circle_center);
                    let (is_inside_top_circle, top_circle_border_point) =
                        find_border_point_on_circle(&top_circle_center);

                    if is_inside_left_circle {
                        left_circle_border_point
                    } else if is_inside_right_circle {
                        right_circle_border_point
                    } else if is_inside_top_circle {
                        top_circle_border_point
                    } else {
                        plane_projection_point
                    }
                };

                (point_generic, corresponding_point)
            })
            .collect();

        Some(correspondings)
    }
}
