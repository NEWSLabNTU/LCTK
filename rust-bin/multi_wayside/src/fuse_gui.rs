use crate::{common::*, utils::p32_to_p30};
use hollow_board_detector::Detection;
use kiss3d::{
    camera::{ArcBall, Camera},
    event::{Action, WindowEvent},
    nalgebra as na30,
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    text::Font,
    window::{State, Window},
};
use nalgebra as na;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub enum WindowName {
    DetectionWindow1,
    DetectionWindow2,
    FusingWindow,
}

pub struct Gui {
    pub state: WindowName,
    pub window1: WindowState,
    pub window2: WindowState,
    pub using_same_face_of_marker: bool,
    pub camera: ArcBall,
}

impl Gui {
    fn step_fusing_window(&mut self, window: &mut Window) {
        use kiss3d::event::Key as K;
        use Action as A;
        use WindowEvent as E;

        let pose1 = self.window1.board_pose.board_model.pose;
        let pose2 = self.window2.board_pose.board_model.pose;
        let pose = if self.using_same_face_of_marker {
            pose2 * pose1.inverse()
        } else {
            let inner_pose = na::UnitQuaternion::from_axis_angle(
                &na::Unit::new_normalize(na::Vector3::new(1.0, 1.0, 0.0)),
                PI,
            );
            pose2 * inner_pose * pose1.inverse()
        };
        for pt in &self.window1.points {
            let pt: na::Point3<f32> = na::convert(pose * pt);
            let pt = p32_to_p30(pt);
            window.draw_point(&pt, &na30::Point3::new(1.0, 0.0, 0.0));
        }
        for pt in &self.window2.points {
            let pt: na::Point3<f32> = na::convert_ref(pt);
            let pt = p32_to_p30(pt);
            window.draw_point(&pt, &na30::Point3::new(0.0, 1.0, 0.0));
        }

        for mut event in window.events().iter() {
            match event.value {
                E::Key(K::Return, A::Press, _) => {
                    event.inhibited = true;
                    window.close();
                    return;
                }
                E::Key(K::Escape, _, _) => {
                    event.inhibited = true;
                }
                E::Key(K::Q, A::Press, _) => {
                    process::exit(0);
                }
                E::Key(K::Tab, A::Press, _) => {
                    self.state = WindowName::DetectionWindow1;
                }
                _ => {}
            }
        }
    }
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        // put usage in screen
        window.draw_text(
            "Instructions:",
            &na30::Point2::origin(),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Return key: save result",
            &na30::Point2::new(0.0, 50.0),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Tab: check another window",
            &na30::Point2::new(0.0, 100.0),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Ctrl-{W,A,S,D}: slightly modify board's orientations",
            &na30::Point2::new(0.0, 150.0),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Arrow keys: slightly modify board's position",
            &na30::Point2::new(0.0, 200.0),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Ctrl-{F,R}: flip and rotate board",
            &na30::Point2::new(0.0, 250.0),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Ctrl-Q: terminate process",
            &na30::Point2::new(0.0, 300.0),
            50.0,
            &Font::default(),
            &na30::Point3::new(1.0, 1.0, 1.0),
        );

        match self.state {
            WindowName::FusingWindow => {
                self.step_fusing_window(window);
            }
            WindowName::DetectionWindow1 => {
                self.window1
                    .step(window, &mut self.state, WindowName::DetectionWindow2);
            }
            WindowName::DetectionWindow2 => {
                self.window2
                    .step(window, &mut self.state, WindowName::FusingWindow);
            }
        }
    }

    #[allow(clippy::type_complexity)]
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (Some(&mut self.camera), None, None)
    }
}

pub struct WindowState {
    pub points: Vec<na::Point3<f64>>,
    pub filtered_points: Vec<na::Point3<f64>>,
    pub board_pose: Detection,
}

impl WindowState {
    pub fn step(&mut self, window: &mut Window, state: &mut WindowName, next_state: WindowName) {
        use kiss3d::event::Key as K;
        use Action as A;
        use WindowEvent as E;

        // draw entire point cloud
        for pt in &self.points {
            let pt_f32: na::Point3<f32> = na::convert(*pt);
            let pt_f32 = p32_to_p30(pt_f32);
            window.draw_point(&pt_f32, &na30::Point3::new(0.0, 0.7, 0.0));
        }

        // draw filtered point cloud
        for pt in &self.filtered_points {
            let pt_f32: na::Point3<f32> = na::convert(*pt);
            let pt_f32 = p32_to_p30(pt_f32);
            window.draw_point(&pt_f32, &na30::Point3::new(1.0, 1.0, 1.0));
        }

        // draw board pose
        self.board_pose.board_model.render_kiss3d(window);

        // handle events
        for mut event in window.events().iter() {
            let orig_board = &mut self.board_pose.board_model;

            match event.value {
                E::Key(K::Q, A::Press, _) => {
                    process::exit(0);
                }
                E::Key(K::Return, A::Press, _) => {
                    event.inhibited = true;
                    window.close();
                    return;
                }
                E::Key(K::Escape, _, _) => {
                    event.inhibited = true;
                }
                E::Key(K::Tab, A::Press, _) => {
                    *state = next_state;
                }
                E::Key(K::W, A::Press, _) => {
                    let rotation = na::UnitQuaternion::from_axis_angle(
                        &na::Unit::new_normalize(
                            orig_board.pose * na::Vector3::new(1.0, -1.0, 0.0),
                        ),
                        -0.2_f64.to_radians(),
                    );
                    (*orig_board).pose.rotation = rotation * orig_board.pose.rotation;
                }
                E::Key(K::S, A::Press, _) => {
                    let rotation = na::UnitQuaternion::from_axis_angle(
                        &na::Unit::new_normalize(
                            orig_board.pose * na::Vector3::new(1.0, -1.0, 0.0),
                        ),
                        0.2_f64.to_radians(),
                    );
                    (*orig_board).pose.rotation = rotation * orig_board.pose.rotation;
                }
                E::Key(K::A, A::Press, _) => {
                    let rotation = na::UnitQuaternion::from_axis_angle(
                        &na::Unit::new_normalize(orig_board.pose * na::Vector3::new(0.0, 0.0, 1.0)),
                        0.2_f64.to_radians(),
                    );
                    (*orig_board).pose.rotation = rotation * orig_board.pose.rotation;
                }
                E::Key(K::D, A::Press, _) => {
                    let rotation = na::UnitQuaternion::from_axis_angle(
                        &na::Unit::new_normalize(orig_board.pose * na::Vector3::new(0.0, 0.0, 1.0)),
                        -0.2_f64.to_radians(),
                    );
                    (*orig_board).pose.rotation = rotation * orig_board.pose.rotation;
                }
                E::Key(K::Up, A::Press, _) => {
                    let translation = na::Translation3::new(0.0, 0.05, 0.0);
                    (*orig_board).pose = translation * orig_board.pose;
                }
                E::Key(K::Down, A::Press, _) => {
                    let translation = na::Translation3::new(0.0, -0.05, 0.0);
                    (*orig_board).pose = translation * orig_board.pose;
                }
                E::Key(K::Right, A::Press, _) => {
                    let translation = na::Translation3::new(0.05, 0.0, 0.0);
                    (*orig_board).pose = translation * orig_board.pose;
                }
                E::Key(K::Left, A::Press, _) => {
                    let translation = na::Translation3::new(-0.05, 0.0, 0.0);
                    (*orig_board).pose = translation * orig_board.pose;
                }
                E::Key(K::PageUp, A::Press, _) => {
                    let translation = na::Translation3::new(0.0, 0.0, 0.05);
                    (*orig_board).pose = translation * orig_board.pose;
                }
                E::Key(K::PageDown, A::Press, _) => {
                    let translation = na::Translation3::new(0.0, 0.0, -0.05);
                    (*orig_board).pose = translation * orig_board.pose;
                }
                E::Key(K::R, A::Press, _) => {
                    let rotation = na::UnitQuaternion::from_axis_angle(
                        &na::Unit::new_normalize(orig_board.pose * na::Vector3::new(0.0, 0.0, 1.0)),
                        PI / 2.0,
                    );
                    orig_board.pose.translation.x = orig_board.left_corner().x;
                    orig_board.pose.translation.y = orig_board.left_corner().y;
                    orig_board.pose.translation.z = orig_board.left_corner().z;
                    (*orig_board).pose.rotation = rotation * orig_board.pose.rotation;
                }
                E::Key(K::F, A::Press, _) => {
                    let rotation = na::UnitQuaternion::from_axis_angle(
                        &na::Unit::new_normalize(orig_board.pose * na::Vector3::new(1.0, 1.0, 0.0)),
                        PI,
                    );
                    (*orig_board).pose.rotation = rotation * orig_board.pose.rotation;
                }

                _ => {}
            }
        }
    }
}
