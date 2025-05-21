use kiss3d::{
    camera::{ArcBall, Camera},
    event::{Action, Key, Modifiers, WindowEvent},
    light::Light,
    nalgebra as na30,
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    window::{State, Window},
};
use kiss3d_utils::WindowPlotExt;
use nalgebra as na;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering::*},
        mpsc::{sync_channel, Receiver, SyncSender},
        Arc,
    },
    thread::{spawn, JoinHandle},
};

use crate::bbox::{BBox, BBox30};

pub fn start(points: Vec<na::Point3<f32>>, bbox: BBox) -> GuiHandle {
    let (tx, rx) = sync_channel(1);

    let notify_close = Arc::new(AtomicBool::new(false));
    let thread_handle = {
        let notify_close = notify_close.clone();

        spawn(move || {
            let mut window = Window::new(env!("CARGO_PKG_NAME"));
            window.set_light(Light::StickToCamera);
            let mut camera = ArcBall::new(
                na30::Point3::new(0.0, -80.0, 32.0),
                na30::Point3::new(0.0, 0.0, 0.0),
            );
            camera.set_up_axis(na30::Vector3::new(0.0, 0.0, 1.0));

            let points: Vec<na30::Point3<f32>> = points
                .into_iter()
                .map(|p| {
                    let p: [f32; 3] = p.into();
                    p.into()
                })
                .collect();

            let gui = Gui {
                tx: Some(tx),
                points,
                notify_close,
                bbox: bbox.into(),
                camera,
            };
            window.render_loop(gui);
        })
    };

    GuiHandle {
        rx,
        thread_handle: Some(thread_handle),
        notify_close,
    }
}

pub struct GuiHandle {
    notify_close: Arc<AtomicBool>,
    rx: Receiver<BBox>,
    thread_handle: Option<JoinHandle<()>>,
}

impl GuiHandle {
    pub fn wait(self) -> Option<BBox> {
        self.rx.recv().ok()
    }
}

impl Drop for GuiHandle {
    fn drop(&mut self) {
        self.notify_close.store(true, SeqCst);
        let _ = self.thread_handle.take().unwrap().join();
    }
}

struct Gui {
    notify_close: Arc<AtomicBool>,
    tx: Option<SyncSender<BBox>>,
    points: Vec<na30::Point3<f32>>,
    bbox: BBox30,
    camera: ArcBall,
}

impl Gui {
    fn update(&mut self, window: &mut Window) -> bool {
        for evt in window.events().iter() {
            use WindowEvent as E;

            match evt.value {
                E::Key(key, action, mods) => {
                    let event = self.process_key_event(key, action, mods);

                    match event {
                        Some(GuiEvent::Finish) => {
                            let _ = self.tx.take().unwrap().send(self.bbox.clone().into());
                            return false;
                        }
                        Some(GuiEvent::Cancel) => {
                            let _ = self.tx.take().unwrap();
                            return false;
                        }
                        None => {}
                    }
                }
                _ => {}
            }
        }

        true
    }

    // Process pending keyboard events.
    fn process_key_event(&mut self, key: Key, action: Action, mods: Modifiers) -> Option<GuiEvent> {
        use Action as A;
        use Key as K;
        use Modifiers as M;

        const LMOVE: f64 = 1.0;
        const SMOVE: f64 = 0.1;

        const LSIZE: f64 = 1.0;
        const SSIZE: f64 = 0.1;

        let lrot: f64 = 10f64.to_radians();
        let srot: f64 = 1f64.to_radians();

        let lroll = na30::UnitQuaternion::from_euler_angles(lrot, 0.0, 0.0);
        let sroll = na30::UnitQuaternion::from_euler_angles(srot, 0.0, 0.0);

        let lpitch = na30::UnitQuaternion::from_euler_angles(0.0, lrot, 0.0);
        let spitch = na30::UnitQuaternion::from_euler_angles(0.0, srot, 0.0);

        let lyaw = na30::UnitQuaternion::from_euler_angles(0.0, 0.0, lrot);
        let syaw = na30::UnitQuaternion::from_euler_angles(0.0, 0.0, srot);

        let control = !(mods & M::Control).is_empty();
        let shift = !(mods & M::Shift).is_empty();
        let super_ = !(mods & M::Super).is_empty();
        let alt = !(mods & M::Alt).is_empty();

        let mut bbox = self.bbox.clone();

        match (key, action, control, shift, alt, super_) {
            // Esc
            (K::Escape, A::Release, false, false, false, false) => {
                return Some(GuiEvent::Cancel);
            }

            // Enter
            (K::Return, A::Release, false, false, false, false) => {
                return Some(GuiEvent::Finish);
            }

            /* translation */
            // W
            (K::W, A::Release, false, false, false, false) => {
                bbox.pose.translation.y += LMOVE;
            }

            // Shift-W
            (K::W, A::Release, false, true, false, false) => {
                bbox.pose.translation.y += SMOVE;
            }

            // A
            (K::A, A::Release, false, false, false, false) => {
                bbox.pose.translation.x -= LMOVE;
            }

            // Shift-A
            (K::A, A::Release, false, true, false, false) => {
                bbox.pose.translation.x -= SMOVE;
            }

            // S
            (K::S, A::Release, false, false, false, false) => {
                bbox.pose.translation.y -= LMOVE;
            }

            // Shift-S
            (K::S, A::Release, false, true, false, false) => {
                bbox.pose.translation.y -= SMOVE;
            }

            // D
            (K::D, A::Release, false, false, false, false) => {
                bbox.pose.translation.x += LMOVE;
            }

            // Shift-D
            (K::D, A::Release, false, true, false, false) => {
                bbox.pose.translation.x += SMOVE;
            }

            // Q
            (K::Q, A::Release, false, false, false, false) => {
                bbox.pose.translation.z -= LMOVE;
            }

            // Shift-Q
            (K::Q, A::Release, false, true, false, false) => {
                bbox.pose.translation.z -= SMOVE;
            }

            // E
            (K::E, A::Release, false, false, false, false) => {
                bbox.pose.translation.z += LMOVE;
            }

            // Shift-E
            (K::E, A::Release, false, true, false, false) => {
                bbox.pose.translation.z += SMOVE;
            }

            /* rotation */
            // Ctrl-W
            (K::W, A::Release, true, false, false, false) => {
                bbox.pose.rotation = &lpitch * bbox.pose.rotation;
            }

            // Ctrl-Shift-W
            (K::W, A::Release, true, true, false, false) => {
                bbox.pose.rotation = &spitch * bbox.pose.rotation;
            }

            // Ctrl-A
            (K::A, A::Release, true, false, false, false) => {
                bbox.pose.rotation = lroll.inverse() * bbox.pose.rotation;
            }

            // Ctrl-Shift-A
            (K::A, A::Release, true, true, false, false) => {
                bbox.pose.rotation = sroll.inverse() * bbox.pose.rotation;
            }

            // Ctrl-S
            (K::S, A::Release, true, false, false, false) => {
                bbox.pose.rotation = lpitch.inverse() * bbox.pose.rotation;
            }

            // Ctrl-Shift-S
            (K::S, A::Release, true, true, false, false) => {
                bbox.pose.rotation = spitch.inverse() * bbox.pose.rotation;
            }

            // Ctrl-D
            (K::D, A::Release, true, false, false, false) => {
                bbox.pose.rotation = &lroll * bbox.pose.rotation;
            }

            // Ctrl-Shift-D
            (K::D, A::Release, true, true, false, false) => {
                bbox.pose.rotation = &sroll * bbox.pose.rotation;
            }

            // Ctrl-Q
            (K::Q, A::Release, true, false, false, false) => {
                bbox.pose.rotation = lyaw.inverse() * bbox.pose.rotation;
            }

            // Ctrl-Shift-Q
            (K::Q, A::Release, true, true, false, false) => {
                bbox.pose.rotation = syaw.inverse() * bbox.pose.rotation;
            }

            // Ctrl-E
            (K::E, A::Release, true, false, false, false) => {
                bbox.pose.rotation = &lyaw * bbox.pose.rotation;
            }

            // Ctrl-Shift-E
            (K::E, A::Release, true, true, false, false) => {
                bbox.pose.rotation = &syaw * bbox.pose.rotation;
            }

            /* size */
            // Alt-W
            (K::W, A::Release, false, false, true, false) => {
                bbox.size_xyz[1] += LSIZE;
            }

            // Alt-Shift-W
            (K::W, A::Release, false, true, true, false) => {
                bbox.size_xyz[1] += SSIZE;
            }

            // Alt-A
            (K::A, A::Release, false, false, true, false) => {
                bbox.size_xyz[0] -= LSIZE;
            }

            // Alt-Shift-A
            (K::A, A::Release, false, true, true, false) => {
                bbox.size_xyz[0] -= SSIZE;
            }

            // Alt-S
            (K::S, A::Release, false, false, true, false) => {
                bbox.size_xyz[1] -= LSIZE;
            }

            // Alt-Shift-S
            (K::S, A::Release, false, true, true, false) => {
                bbox.size_xyz[1] -= SSIZE;
            }

            // Alt-D
            (K::D, A::Release, false, false, true, false) => {
                bbox.size_xyz[0] += LSIZE;
            }

            // Alt-Shift-D
            (K::D, A::Release, false, true, true, false) => {
                bbox.size_xyz[0] += SSIZE;
            }

            // Alt-Q
            (K::Q, A::Release, false, false, true, false) => {
                bbox.size_xyz[2] -= LSIZE;
            }

            // Alt-Shift-Q
            (K::Q, A::Release, false, true, true, false) => {
                bbox.size_xyz[2] -= SSIZE;
            }

            // Alt-E
            (K::E, A::Release, false, false, true, false) => {
                bbox.size_xyz[2] += LSIZE;
            }

            // Alt-Shift-E
            (K::E, A::Release, false, true, true, false) => {
                bbox.size_xyz[2] += SSIZE;
            }

            _ => {}
        }

        self.bbox = bbox;
        None
    }

    fn render(&self, window: &mut Window) {
        let Self { points, bbox, .. } = self;

        // Draw axis
        window.draw_axes(na30::Isometry3::identity(), 1.0);

        // draw points
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

        // draw bbox
        {
            let color = na30::Point3::new(1.0, 1.0, 0.0);
            let pose: na30::Isometry3<f32> = na30::convert_ref(&self.bbox.pose);
            window.draw_box(bbox.size_xyz, pose, color);
        }
    }
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        if self.notify_close.load(SeqCst) {
            window.close();
            return;
        }

        let ok = self.update(window);
        if !ok {
            window.close();
            return;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum GuiEvent {
    Finish,
    Cancel,
}
