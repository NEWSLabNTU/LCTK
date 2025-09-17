use super::BoardModel;
use kiss3d::{nalgebra as na30, window::Window};
use nalgebra as na32;

impl BoardModel {
    pub fn render_kiss3d(&self, window: &mut Window) {
        let convert = |p: na32::Point3<f64>| -> na30::Point3<f32> {
            let p: [f64; 3] = p.into();
            let p = na30::Point3::from(p);
            na30::convert(p)
        };

        let border_color = na30::Point3::new(1.0, 0.7, 0.0);

        // draw board
        {
            let top_corner = convert(self.top_corner());
            let bottom_corner = convert(self.bottom_corner());
            let left_corner = convert(self.left_corner());
            let right_corner = convert(self.right_corner());
            window.draw_line(&top_corner, &left_corner, &border_color);
            window.draw_line(&left_corner, &bottom_corner, &border_color);
            window.draw_line(&bottom_corner, &right_corner, &border_color);
            window.draw_line(&right_corner, &top_corner, &border_color);
        }

        // draw aruco marker borders
        {
            let top_corner = convert(self.marker_top_corner());
            let bottom_corner = convert(self.marker_bottom_corner());
            let left_corner = convert(self.marker_left_corner());
            let right_corner = convert(self.marker_right_corner());
            window.draw_line(&top_corner, &left_corner, &border_color);
            window.draw_line(&left_corner, &bottom_corner, &border_color);
            window.draw_line(&bottom_corner, &right_corner, &border_color);
            window.draw_line(&right_corner, &top_corner, &border_color);
            window.draw_line(&top_corner, &bottom_corner, &border_color);
            window.draw_line(&left_corner, &right_corner, &border_color);
        }

        // draw axises of board
        {
            let begin = convert(self.marker_center());
            let end = convert(self.marker_center() + self.board_x_axis().scale(1.0));
            window.draw_line(&begin, &end, &na30::Point3::new(1.0, 0.3, 0.3));
        }

        {
            let begin = convert(self.marker_center());
            let end = convert(self.marker_center() + self.board_y_axis().scale(1.0));
            window.draw_line(&begin, &end, &na30::Point3::new(0.3, 1.0, 0.3));
        }

        {
            let begin = convert(self.marker_center());
            let end = convert(self.marker_center() + self.board_z_axis().scale(1.0));
            window.draw_line(&begin, &end, &na30::Point3::new(0.3, 0.3, 1.0));
        }
    }
}
