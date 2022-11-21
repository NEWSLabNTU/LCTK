use super::BoardModel;
use kiss3d::window::Window;
use nalgebra as na;

impl BoardModel {
    pub fn render_kiss3d(&self, window: &mut Window) {
        let border_color = na::Point3::new(1.0, 0.7, 0.0);

        // draw board
        {
            let top_corner = na::convert(self.top_corner());
            let bottom_corner = na::convert(self.bottom_corner());
            let left_corner = na::convert(self.left_corner());
            let right_corner = na::convert(self.right_corner());
            window.draw_line(&top_corner, &left_corner, &border_color);
            window.draw_line(&left_corner, &bottom_corner, &border_color);
            window.draw_line(&bottom_corner, &right_corner, &border_color);
            window.draw_line(&right_corner, &top_corner, &border_color);
        }

        // draw aruco marker borders
        {
            let top_corner = na::convert(self.marker_top_corner());
            let bottom_corner = na::convert(self.marker_bottom_corner());
            let left_corner = na::convert(self.marker_left_corner());
            let right_corner = na::convert(self.marker_right_corner());
            window.draw_line(&top_corner, &left_corner, &border_color);
            window.draw_line(&left_corner, &bottom_corner, &border_color);
            window.draw_line(&bottom_corner, &right_corner, &border_color);
            window.draw_line(&right_corner, &top_corner, &border_color);
            window.draw_line(&top_corner, &bottom_corner, &border_color);
            window.draw_line(&left_corner, &right_corner, &border_color);
        }

        // draw axises of board
        {
            let begin = na::convert(self.marker_center());
            let end = na::convert(self.marker_center() + self.board_x_axis().scale(1.0));
            window.draw_line(&begin, &end, &na::Point3::new(1.0, 0.3, 0.3));
        }

        {
            let begin = na::convert(self.marker_center());
            let end = na::convert(self.marker_center() + self.board_y_axis().scale(1.0));
            window.draw_line(&begin, &end, &na::Point3::new(0.3, 1.0, 0.3));
        }

        {
            let begin = na::convert(self.marker_center());
            let end = na::convert(self.marker_center() + self.board_z_axis().scale(1.0));
            window.draw_line(&begin, &end, &na::Point3::new(0.3, 0.3, 1.0));
        }
    }
}
