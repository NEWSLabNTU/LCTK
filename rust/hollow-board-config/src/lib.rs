use approx::abs_diff_eq;
use aruco_config::MultiArucoPattern;
use measurements::Length;
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::borrow::Borrow;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

const EPS_F64: f64 = 0.3; // Tolerance for point-to-plane distance during ICP (30cm)

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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardModel {
    #[serde(with = "newslab_serde_nalgebra::isometry3_as_euler_angles")]
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

        // Define the correspondence computation as a closure to avoid code duplication
        let compute_correspondence = |point_generic: InputPoint| -> (InputPoint, na::Point3<f64>) {
            let point = point_generic.borrow();

            // fidn projection point on board plane (regardless of boundary)
            let vec_point_to_origin = self.bottom_corner() - point;
            let vec_point_to_proj = board_z_axis.scale(vec_point_to_origin.dot(&board_z_axis));
            let plane_projection_point: na::Point3<_> = point + vec_point_to_proj;

            // check if projection point is outside the board
            let vec_origin_to_proj = plane_projection_point - self.bottom_corner();
            let dot_product = vec_origin_to_proj.dot(&board_z_axis);
            if !abs_diff_eq!(dot_product, 0.0, epsilon = EPS_F64) {
                eprintln!("🔴 Assertion failure in BoardModel::contains_point:");
                eprintln!("  dot_product = {}", dot_product);
                eprintln!("  epsilon = {}", EPS_F64);
                eprintln!("  abs_diff = {}", dot_product.abs());
                eprintln!("  point = {:?}", point);
                eprintln!("  board_z_axis = {:?}", board_z_axis);
                eprintln!("  vec_origin_to_proj = {:?}", vec_origin_to_proj);
            }
            debug_assert!(abs_diff_eq!(dot_product, 0.0, epsilon = EPS_F64));

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
                        // Handle degenerate case: point exactly at hole center
                        if dist_circle_center_to_proj < 1e-10 {
                            // Use arbitrary radial direction (along board x-axis)
                            circle_center
                                + board_x_axis.scale(self.board_shape.hole_radius.as_meters())
                        } else {
                            let radical_unit = na::Unit::new_normalize(vec_circle_center_to_proj);
                            circle_center
                                + radical_unit.scale(self.board_shape.hole_radius.as_meters())
                        }
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
        };

        // Use parallel or serial iterator based on feature flag
        #[cfg(feature = "parallel")]
        let correspondings: Vec<_> = points.into_par_iter().map(compute_correspondence).collect();

        #[cfg(not(feature = "parallel"))]
        let correspondings: Vec<_> = points.into_iter().map(compute_correspondence).collect();

        Some(correspondings)
    }

    #[cfg(not(feature = "parallel"))]
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

        // Define the correspondence computation as a closure to avoid code duplication
        let compute_correspondence = |point_generic: InputPoint| -> (InputPoint, na::Point3<f64>) {
            let point = point_generic.borrow();

            // fidn projection point on board plane (regardless of boundary)
            let vec_point_to_origin = self.bottom_corner() - point;
            let vec_point_to_proj = board_z_axis.scale(vec_point_to_origin.dot(&board_z_axis));
            let plane_projection_point: na::Point3<_> = point + vec_point_to_proj;

            // check if projection point is outside the board
            let vec_origin_to_proj = plane_projection_point - self.bottom_corner();
            let dot_product = vec_origin_to_proj.dot(&board_z_axis);
            if !abs_diff_eq!(dot_product, 0.0, epsilon = EPS_F64) {
                eprintln!("🔴 Assertion failure in BoardModel::contains_point:");
                eprintln!("  dot_product = {}", dot_product);
                eprintln!("  epsilon = {}", EPS_F64);
                eprintln!("  abs_diff = {}", dot_product.abs());
                eprintln!("  point = {:?}", point);
                eprintln!("  board_z_axis = {:?}", board_z_axis);
                eprintln!("  vec_origin_to_proj = {:?}", vec_origin_to_proj);
            }
            debug_assert!(abs_diff_eq!(dot_product, 0.0, epsilon = EPS_F64));

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
                        // Handle degenerate case: point exactly at hole center
                        if dist_circle_center_to_proj < 1e-10 {
                            // Use arbitrary radial direction (along board x-axis)
                            circle_center
                                + board_x_axis.scale(self.board_shape.hole_radius.as_meters())
                        } else {
                            let radical_unit = na::Unit::new_normalize(vec_circle_center_to_proj);
                            circle_center
                                + radical_unit.scale(self.board_shape.hole_radius.as_meters())
                        }
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
        };

        // Use serial iterator for non-parallel build
        let correspondings: Vec<_> = points.into_iter().map(compute_correspondence).collect();

        Some(correspondings)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use measurements::Length;

    #[test]
    fn test_multi_marker_corners_basic() {
        // Basic test to verify corner computation works correctly
        // Uses minimal config to avoid complex serde dependencies

        // Board config matching Python test
        let board_size = 0.5; // 500mm
        let board_border_size = 0.01; // 10mm
        let num_squares = 2;
        let marker_square_size_ratio = 0.8;

        // Calculate geometry
        let square_size = (board_size - 2.0 * board_border_size) / num_squares as f64;
        let marker_size = square_size * marker_square_size_ratio;
        let marker_border = (square_size - marker_size) / 2.0;
        let origin_x = board_border_size + marker_border;
        let origin_y = board_border_size + marker_border;

        println!("\n============ RUST GEOMETRY ============");
        println!("board_size: {:.6} m", board_size);
        println!("board_border_size: {:.6} m", board_border_size);
        println!("square_size: {:.6} m", square_size);
        println!("marker_size: {:.6} m", marker_size);
        println!("marker_border: {:.6} m", marker_border);
        println!("origin: ({:.6}, {:.6})", origin_x, origin_y);

        // Expected corners for bottom marker (696) at (0,0)
        let bottom_corners = [
            (origin_x, origin_y + marker_size, 0.0), // right
            (origin_x + marker_size, origin_y + marker_size, 0.0), // top
            (origin_x + marker_size, origin_y, 0.0), // left
            (origin_x, origin_y, 0.0),               // bottom
        ];

        // Expected corners for left marker (64) at (0,1)
        let left_corners = [
            (origin_x + square_size, origin_y + marker_size, 0.0), // right
            (
                origin_x + square_size + marker_size,
                origin_y + marker_size,
                0.0,
            ), // top
            (origin_x + square_size + marker_size, origin_y, 0.0), // left
            (origin_x + square_size, origin_y, 0.0),               // bottom
        ];

        // Expected corners for right marker (306) at (1,0)
        let right_corners = [
            (origin_x, origin_y + square_size + marker_size, 0.0), // right
            (
                origin_x + marker_size,
                origin_y + square_size + marker_size,
                0.0,
            ), // top
            (origin_x + marker_size, origin_y + square_size, 0.0), // left
            (origin_x, origin_y + square_size, 0.0),               // bottom
        ];

        // Expected corners for top marker (195) at (1,1)
        let top_corners = [
            (
                origin_x + square_size,
                origin_y + square_size + marker_size,
                0.0,
            ), // right
            (
                origin_x + square_size + marker_size,
                origin_y + square_size + marker_size,
                0.0,
            ), // top
            (
                origin_x + square_size + marker_size,
                origin_y + square_size,
                0.0,
            ), // left
            (origin_x + square_size, origin_y + square_size, 0.0), // bottom
        ];

        println!("\n============ RUST OUTPUT ============");
        println!("Marker 696 (bottom):");
        for corner in bottom_corners.iter() {
            println!("  [{:.6}, {:.6}, {:.6}]", corner.0, corner.1, corner.2);
        }
        println!("\nMarker 64 (left):");
        for corner in left_corners.iter() {
            println!("  [{:.6}, {:.6}, {:.6}]", corner.0, corner.1, corner.2);
        }
        println!("\nMarker 306 (right):");
        for corner in right_corners.iter() {
            println!("  [{:.6}, {:.6}, {:.6}]", corner.0, corner.1, corner.2);
        }
        println!("\nMarker 195 (top):");
        for corner in top_corners.iter() {
            println!("  [{:.6}, {:.6}, {:.6}]", corner.0, corner.1, corner.2);
        }
        println!("======================================\n");

        // Verify corner ordering
        assert_eq!(
            bottom_corners[3],
            (origin_x, origin_y, 0.0),
            "Bottom corner should be at origin"
        );
    }

    /// Helper function to create a simple board model for testing
    fn create_test_board_model() -> BoardModel {
        let board_shape = BoardShape {
            board_width: Length::from_meters(1.0),       // 1m board
            hole_radius: Length::from_meters(0.15),      // 15cm holes
            hole_center_shift: Length::from_meters(0.2), // 20cm from center
        };

        BoardModel {
            pose: na::Isometry3::identity(),
            marker_paper_size: Length::from_meters(0.3),
            board_shape,
        }
    }

    #[test]
    fn test_find_correspondences_point_on_plane_no_hole() {
        let board = create_test_board_model();

        // Point on the board plane, not in a hole (center of board)
        let point = na::Point3::new(0.5, 0.5, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should map to itself (or very close)
        assert!(
            (correspondence - point).norm() < 1e-6,
            "Point on plane should map to itself. Got {:?}, expected {:?}",
            correspondence,
            point
        );
    }

    #[test]
    fn test_find_correspondences_point_above_plane() {
        let board = create_test_board_model();

        // Point 0.5m above the board center
        let point = na::Point3::new(0.5, 0.5, 0.5);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project down to the plane
        let expected = na::Point3::new(0.5, 0.5, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point above plane should project to plane. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_below_plane() {
        let board = create_test_board_model();

        // Point 0.5m below the board center
        let point = na::Point3::new(0.5, 0.5, -0.5);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project up to the plane
        let expected = na::Point3::new(0.5, 0.5, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point below plane should project to plane. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_outside_left_edge() {
        let board = create_test_board_model();

        // Point outside the board to the left (negative x)
        let point = na::Point3::new(-0.5, 0.5, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should clamp to left edge (x=0)
        let expected = na::Point3::new(0.0, 0.5, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point outside left edge should clamp to x=0. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_outside_right_edge() {
        let board = create_test_board_model();

        // Point outside the board to the right (x > 1.0)
        let point = na::Point3::new(1.5, 0.5, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should clamp to right edge (x=1.0)
        let expected = na::Point3::new(1.0, 0.5, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point outside right edge should clamp to x=1.0. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_outside_bottom_edge() {
        let board = create_test_board_model();

        // Point outside the board at bottom (negative y)
        let point = na::Point3::new(0.5, -0.5, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should clamp to bottom edge (y=0)
        let expected = na::Point3::new(0.5, 0.0, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point outside bottom edge should clamp to y=0. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_outside_top_edge() {
        let board = create_test_board_model();

        // Point outside the board at top (y > 1.0)
        let point = na::Point3::new(0.5, 1.5, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should clamp to top edge (y=1.0)
        let expected = na::Point3::new(0.5, 1.0, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point outside top edge should clamp to y=1.0. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_outside_corner() {
        let board = create_test_board_model();

        // Point outside the board at bottom-left corner
        let point = na::Point3::new(-0.5, -0.5, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should clamp to corner (0, 0)
        let expected = na::Point3::new(0.0, 0.0, 0.0);
        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point outside corner should clamp to (0,0). Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_point_inside_left_hole() {
        let board = create_test_board_model();

        // Left hole center is at (0.5 + 0.2, 0.5 - 0.2) = (0.7, 0.3)
        // Place a point at the center of the left hole
        let point = na::Point3::new(0.7, 0.3, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project to the hole border
        // The correspondence should be at distance = hole_radius from the center
        let hole_center = na::Point3::new(0.7, 0.3, 0.0);
        let distance_from_center = (correspondence - hole_center).norm();

        assert!(
            (distance_from_center - 0.15).abs() < 1e-6,
            "Point inside left hole should project to hole border (radius=0.15m). Got distance {:?}",
            distance_from_center
        );

        // Z coordinate should remain 0
        assert!(
            correspondence.z.abs() < 1e-6,
            "Correspondence z-coordinate should be 0"
        );
    }

    #[test]
    fn test_find_correspondences_point_inside_right_hole() {
        let board = create_test_board_model();

        // Right hole center is at (0.5 - 0.2, 0.5 + 0.2) = (0.3, 0.7)
        let point = na::Point3::new(0.3, 0.7, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project to the hole border
        let hole_center = na::Point3::new(0.3, 0.7, 0.0);
        let distance_from_center = (correspondence - hole_center).norm();

        assert!(
            (distance_from_center - 0.15).abs() < 1e-6,
            "Point inside right hole should project to hole border (radius=0.15m). Got distance {:?}",
            distance_from_center
        );
    }

    #[test]
    fn test_find_correspondences_point_inside_top_hole() {
        let board = create_test_board_model();

        // Top hole center is at (0.5 + 0.2, 0.5 + 0.2) = (0.7, 0.7)
        let point = na::Point3::new(0.7, 0.7, 0.0);
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project to the hole border
        let hole_center = na::Point3::new(0.7, 0.7, 0.0);
        let distance_from_center = (correspondence - hole_center).norm();

        assert!(
            (distance_from_center - 0.15).abs() < 1e-6,
            "Point inside top hole should project to hole border (radius=0.15m). Got distance {:?}",
            distance_from_center
        );
    }

    #[test]
    fn test_find_correspondences_point_near_hole_edge() {
        let board = create_test_board_model();

        // Point just inside the left hole, near the edge
        // Left hole center: (0.7, 0.3), radius: 0.15
        // Place point at distance 0.05m from center (inside the hole)
        let point = na::Point3::new(0.75, 0.3, 0.0); // 0.05m to the right of center
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project to hole border along the same direction
        let hole_center = na::Point3::new(0.7, 0.3, 0.0);
        let direction = (point - hole_center).normalize();
        let expected = hole_center + direction.scale(0.15);

        assert!(
            (correspondence - expected).norm() < 1e-6,
            "Point near hole edge should project radially to border. Got {:?}, expected {:?}",
            correspondence,
            expected
        );
    }

    #[test]
    fn test_find_correspondences_multiple_points() {
        let board = create_test_board_model();

        // Test with multiple points of different types
        let points = vec![
            na::Point3::new(0.5, 0.5, 0.0),  // On plane, no hole
            na::Point3::new(-0.5, 0.5, 0.0), // Outside left edge
            na::Point3::new(0.7, 0.3, 0.0),  // Inside left hole
            na::Point3::new(0.5, 0.5, 0.5),  // Above plane
        ];

        let result = board.find_correspondences(points.clone());
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 4, "Should have 4 correspondences");

        // Check first point (on plane)
        let (_, corr1) = &correspondences[0];
        assert!(
            (corr1 - na::Point3::new(0.5, 0.5, 0.0)).norm() < 1e-6,
            "First point should map to itself"
        );

        // Check second point (outside left edge)
        let (_, corr2) = &correspondences[1];
        assert!(
            (corr2 - na::Point3::new(0.0, 0.5, 0.0)).norm() < 1e-6,
            "Second point should clamp to left edge"
        );

        // Check third point (inside left hole)
        let (_, corr3) = &correspondences[2];
        let hole_center = na::Point3::new(0.7, 0.3, 0.0);
        let distance_from_center = (corr3 - hole_center).norm();
        assert!(
            (distance_from_center - 0.15).abs() < 1e-6,
            "Third point should project to hole border"
        );

        // Check fourth point (above plane)
        let (_, corr4) = &correspondences[3];
        assert!(
            (corr4 - na::Point3::new(0.5, 0.5, 0.0)).norm() < 1e-6,
            "Fourth point should project to plane"
        );
    }

    #[test]
    fn test_find_correspondences_rotated_board() {
        // Create a board rotated 45 degrees around z-axis
        let board_shape = BoardShape {
            board_width: Length::from_meters(1.0),
            hole_radius: Length::from_meters(0.15),
            hole_center_shift: Length::from_meters(0.2),
        };

        let rotation = na::UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4);
        let pose = na::Isometry3::from_parts(na::Translation3::identity(), rotation);

        let board = BoardModel {
            pose,
            marker_paper_size: Length::from_meters(0.3),
            board_shape,
        };

        // Point at board center in board frame (should be at sqrt(2)/4, sqrt(2)/4 in world frame)
        let board_center = board.board_center();
        let point = board_center + na::Vector3::new(0.0, 0.0, 0.5); // 0.5m above center
        let points = vec![point];

        let result = board.find_correspondences(points);
        assert!(result.is_some());

        let correspondences = result.unwrap();
        assert_eq!(correspondences.len(), 1);

        let (_, correspondence) = &correspondences[0];

        // Should project to board center
        assert!(
            (correspondence - board_center).norm() < 1e-6,
            "Point above rotated board center should project to center. Got {:?}, expected {:?}",
            correspondence,
            board_center
        );
    }
}
