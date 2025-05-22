use hollow_board_detector::Detection;
use kiss3d::{
    camera::{ArcBall, Camera},
    light::Light,
    nalgebra as na30,
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    window::{State, Window},
};
use nalgebra as na;
use std::{
    sync::mpsc::{sync_channel, Receiver, SyncSender},
    thread::{spawn, JoinHandle},
    time::Duration,
};

use crate::bbox::{BBox, BBox30};

pub fn start(bbox: BBox) -> GuiHandle {
    let (tx, rx) = sync_channel(4);

    let thread_handle = spawn(move || {
        let mut window = Window::new(env!("CARGO_PKG_NAME"));
        window.set_light(Light::StickToCamera);
        let mut camera = ArcBall::new(
            na30::Point3::new(0.0, -80.0, 32.0),
            na30::Point3::new(0.0, 0.0, 0.0),
        );
        camera.set_up_axis(na30::Vector3::new(0.0, 0.0, 1.0));
        let gui = Gui {
            rx,
            points: vec![],
            detection: None,
            bbox: bbox.into(),
            camera,
        };
        window.render_loop(gui);
    });

    GuiHandle {
        tx,
        thread_handle: Some(thread_handle),
    }
}

pub struct GuiHandle {
    tx: SyncSender<Message>,
    thread_handle: Option<JoinHandle<()>>,
}

impl GuiHandle {
    pub fn update(
        self,
        points: Vec<na::Point3<f32>>,
        detection: Option<Detection>,
    ) -> Option<Self> {
        let points: Vec<na30::Point3<f32>> = points
            .into_iter()
            .map(|p| {
                let p: [f32; 3] = p.into();
                p.into()
            })
            .collect();
        let result = self.tx.send(Message::Data(Data { points, detection }));
        result.is_ok().then(|| self)
    }
}

impl Drop for GuiHandle {
    fn drop(&mut self) {
        let _ = self.tx.send(Message::End);
        let _ = self.thread_handle.take().unwrap().join();
    }
}

enum Message {
    Data(Data),
    End,
}

struct Data {
    pub points: Vec<na30::Point3<f32>>,
    pub detection: Option<Detection>,
}

struct Gui {
    rx: Receiver<Message>,
    points: Vec<na30::Point3<f32>>,
    detection: Option<Detection>,
    bbox: BBox30,
    camera: ArcBall,
}

impl Gui {
    fn update(&mut self, msg: Data) {
        let Data { points, detection } = msg;
        self.points = points;
        self.detection = detection;
    }

    fn render(&self, window: &mut Window) {
        let Self {
            points, detection, ..
        } = self;

        {
            let in_color = na30::Point3::new(0.0, 1.0, 0.0);
            let out_color = na30::Point3::new(0.5, 0.5, 0.5);

            points.iter().for_each(|position| {
                let point_f64: na30::Point3<f64> = na30::convert_ref(position);
                let color = if self.bbox.contains_point(&point_f64) {
                    &in_color
                } else {
                    &out_color
                };
                window.draw_point(position, &color);
            });
        }

        if let Some(detection) = detection {
            detection.board_model.render_kiss3d(window);
        }
    }
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        let result = self.rx.recv_timeout(Duration::from_millis(10));
        match result {
            Ok(Message::Data(data)) => self.update(data),
            Ok(Message::End) => {
                window.close();
                return;
            }
            Err(_) => {}
        }

        self.render(window);
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
