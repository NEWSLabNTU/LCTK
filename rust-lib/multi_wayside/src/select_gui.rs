use crate::{common::*, config};
use hollow_board_detector::BoardDetection;
use kiss3d::{
    camera::{ArcBall, Camera},
    event::{Action, Key, WindowEvent},
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    text::Font,
    window::{State, Window},
};
use na::Point3;

pub struct Gui {
    pub pcap_config: config::PcapConfig,
    pub original_points: Vec<Point3<f64>>,
    pub points_in_point3_format: Vec<Point3<f64>>,
    pub points_in_lidar_point_format: Vec<protos::LidarPoint>,
    pub det: Option<BoardDetection>,
    pub press_key: Option<Key>,
    pub camera: ArcBall,
}

impl Gui {
    pub fn new(pcap_config: config::PcapConfig) -> Self {
        Self {
            pcap_config,
            original_points: vec![],
            points_in_lidar_point_format: vec![],
            points_in_point3_format: vec![],
            det: None,
            press_key: None,
            camera: {
                let mut camera = ArcBall::new(
                    na::Point3::new(0.0, -80.0, 32.0),
                    na::Point3::new(0.0, 0.0, 0.0),
                );
                camera.set_up_axis(na::Vector3::new(0.0, 0.0, 1.0));
                camera
            },
        }
    }
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        // draw texts
        window.draw_text(
            "Instructions:",
            &na::Point2::origin(),
            50.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Return key: save result",
            &na::Point2::new(0.0, 50.0),
            50.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Esc: skip to next result",
            &na::Point2::new(0.0, 100.0),
            50.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Ctrl-Q: terminate process",
            &na::Point2::new(0.0, 150.0),
            50.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
        window.draw_text(
            "Ctrl-U: Update config content",
            &na::Point2::new(0.0, 200.0),
            50.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );

        // draw points
        for point in &self.original_points {
            let point: Point3<f32> = na::convert_ref(point);
            window.draw_point(&point, &Point3::new(0.0, 0.7, 0.0));
        }
        for point in &self.points_in_point3_format {
            let point: Point3<f32> = na::convert_ref(point);
            window.draw_point(&point, &Point3::new(1.0, 1.0, 1.0));
        }

        // draw the origin and axis
        window.set_point_size(2.0);
        window.set_line_width(2.0);
        window.draw_point(&Point3::origin(), &Point3::new(0.0, 0.0, 1.0));
        window.draw_line(
            &Point3::origin(),
            &Point3::new(1.0, 0.0, 0.0),
            &Point3::new(1.0, 0.0, 0.0),
        );
        window.draw_line(
            &Point3::origin(),
            &Point3::new(0.0, 1.0, 0.0),
            &Point3::new(1.0, 1.0, 0.0),
        );
        window.draw_line(
            &Point3::origin(),
            &Point3::new(0.0, 0.0, 1.0),
            &Point3::new(0.0, 0.0, 1.0),
        );

        // draw filter
        self.pcap_config.filter.render_kiss3d(window);

        // draw detection
        if let Some(ref val) = &self.det {
            val.board_model.render_kiss3d(window);
        }

        // handle events
        for mut event in window.events().iter() {
            use Action as A;
            use Key as K;
            use WindowEvent as E;

            match event.value {
                E::Key(K::Return, A::Release, _) => {
                    event.inhibited = true;
                    window.close();
                    self.press_key = Some(K::Return);
                    return;
                }
                E::Key(K::Return, A::Press, _) => {
                    event.inhibited = true;
                }

                E::Key(K::Q, A::Press, _) => {
                    window.close();
                    self.press_key = Some(K::Q);
                    return;
                }
                E::Key(K::Escape, A::Press, _) => {
                    event.inhibited = true;
                }
                E::Key(K::Escape, A::Release, _) => {
                    event.inhibited = true;
                    window.close();
                    self.press_key = Some(K::Escape);
                    return;
                }
                E::Key(K::U, A::Release, _) => {
                    event.inhibited = true;
                    self.press_key = Some(K::U);
                }
                _ => {}
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