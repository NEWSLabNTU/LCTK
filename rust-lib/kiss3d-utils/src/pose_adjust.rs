use crate::common::*;
use kiss3d::{
    event::{Action, WindowEvent},
    window::Window,
};

/// Interactive pose adjustment on kiss3d [Window].
#[derive(Debug)]
pub struct PoseAdjust {
    pose: Isometry3<f64>,
}

impl PoseAdjust {
    /// Creates a new [PoseAdjust] instance.
    pub fn new(orig_pose: Isometry3<f64>) -> Self {
        Self { pose: orig_pose }
    }

    /// Updates the pose according to window events.
    pub fn update(&mut self, window: Window) -> &Isometry3<f64> {
        use kiss3d::event::Key;

        for event in window.events().iter() {
            use Action as A;
            use Key as K;
            use WindowEvent::Key as KE;

            match event.value {
                KE(K::W, A::Press, _) => {
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(self.pose * Vector3::new(1.0, -1.0, 0.0)),
                        -0.2_f64.to_radians(),
                    );
                    self.pose.rotation = rotation * self.pose.rotation;
                }
                KE(K::S, A::Press, _) => {
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(self.pose * Vector3::new(1.0, -1.0, 0.0)),
                        0.2_f64.to_radians(),
                    );
                    self.pose.rotation = rotation * self.pose.rotation;
                }
                KE(K::A, A::Press, _) => {
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(self.pose * Vector3::new(0.0, 0.0, 1.0)),
                        0.2_f64.to_radians(),
                    );
                    self.pose.rotation = rotation * self.pose.rotation;
                }
                KE(K::D, A::Press, _) => {
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(self.pose * Vector3::new(0.0, 0.0, 1.0)),
                        -0.2_f64.to_radians(),
                    );
                    self.pose.rotation = rotation * self.pose.rotation;
                }
                KE(K::Up, A::Press, _) => {
                    let translation = Translation3::new(0.0, 0.05, 0.0);
                    self.pose = translation * self.pose;
                }
                KE(K::Down, A::Press, _) => {
                    let translation = Translation3::new(0.0, -0.05, 0.0);
                    self.pose = translation * self.pose;
                }
                KE(K::Right, A::Press, _) => {
                    let translation = Translation3::new(0.05, 0.0, 0.0);
                    self.pose = translation * self.pose;
                }
                KE(K::Left, A::Press, _) => {
                    let translation = Translation3::new(-0.05, 0.0, 0.0);
                    self.pose = translation * self.pose;
                }
                KE(K::PageUp, A::Press, _) => {
                    let translation = Translation3::new(0.0, 0.0, 0.05);
                    self.pose = translation * self.pose;
                }
                KE(K::PageDown, A::Press, _) => {
                    let translation = Translation3::new(0.0, 0.0, -0.05);
                    self.pose = translation * self.pose;
                }
                KE(K::R, A::Press, _) => {
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(self.pose * Vector3::new(0.0, 0.0, 1.0)),
                        f64::consts::PI / 2.0,
                    );
                    self.pose.translation.x = (self.pose * Vector3::new(1.0, 0.0, 0.0)).x;
                    self.pose.translation.y = (self.pose * Vector3::new(1.0, 0.0, 0.0)).y;
                    self.pose.translation.z = (self.pose * Vector3::new(1.0, 0.0, 0.0)).z;
                    self.pose.rotation = rotation * self.pose.rotation;
                }
                KE(K::F, A::Press, _) => {
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(self.pose * Vector3::new(1.0, 1.0, 0.0)),
                        f64::consts::PI,
                    );
                    self.pose.rotation = rotation * self.pose.rotation;
                }
                _ => {}
            }
        }

        &self.pose
    }

    /// Returns the current pose.
    pub fn pose(&self) -> &Isometry3<f64> {
        &self.pose
    }
}
