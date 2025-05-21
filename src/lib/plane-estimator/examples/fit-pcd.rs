use anyhow::{anyhow, Result};
use clap::Parser;
use itertools::Itertools;
use kiss3d::{
    light::Light,
    nalgebra as na30,
    window::{State, Window},
};
use nalgebra as na;
use pcd_rs::DynReader;
use plane_estimator::PlaneEstimator;
use sample_consensus::Estimator;
use std::path::PathBuf;

fn main() -> Result<()> {
    let Opts { input_file } = Opts::parse();

    let points: Vec<_> = DynReader::open(input_file)?
        .map(|point| {
            let [x, y, z]: [f32; 3] = point?.to_xyz().unwrap();
            let point = na::Point3::new(x, y, z);
            anyhow::Ok(point)
        })
        .try_collect()?;

    let estimator = PlaneEstimator::new();
    let plane_model = estimator
        .estimate(points.iter())
        .ok_or_else(|| anyhow!("no plane detected"))?;

    let mut window = Window::new("Kiss3d: wasm example");
    window.set_light(Light::StickToCamera);

    let pose = plane_model.pose();
    let pose = {
        let pose: na::Isometry3<f32> = na::convert(pose);
        let na::Isometry3 {
            rotation,
            translation,
        } = pose;
        let na::coordinates::XYZ { x, y, z } = *translation;
        let na::coordinates::IJKW { i, j, k, w } = **rotation;
        let translation = na30::Translation3::new(x, y, z);
        let rotation = na30::UnitQuaternion::from_quaternion(na30::Quaternion::new(w, i, j, k));
        na30::Isometry3 {
            rotation,
            translation,
        }
    };

    let points: Vec<_> = points
        .into_iter()
        .map(|p| {
            let p: [f32; 3] = p.into();
            na30::Point3::from(p)
        })
        .collect();

    let state = Gui { points, pose };
    window.render_loop(state);

    Ok(())
}

/// Find a plane from a .pcd file.
#[derive(Debug, Clone, Parser)]
struct Opts {
    /// Input .pcd file.
    pub input_file: PathBuf,
}

struct Gui {
    points: Vec<na30::Point3<f32>>,
    pose: na30::Isometry3<f32>,
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        // Draw axis at origin
        {
            let origin = na30::Point3::new(0.0, 0.0, 0.0);
            let x_pt = na30::Point3::new(0.1, 0.0, 0.0);
            let y_pt = na30::Point3::new(0.0, 0.1, 0.0);
            let z_pt = na30::Point3::new(0.0, 0.0, 0.1);

            window.draw_line(&origin, &x_pt, &na30::Point3::new(0.5, 0.0, 0.0));
            window.draw_line(&origin, &y_pt, &na30::Point3::new(0.0, 0.5, 0.0));
            window.draw_line(&origin, &z_pt, &na30::Point3::new(0.0, 0.0, 0.5));
        }

        // Draw axis at the plane origin
        {
            let origin = self.pose * na30::Point3::new(0.0, 0.0, 0.0);
            let x_pt = self.pose * na30::Point3::new(1.0, 0.0, 0.0);
            let y_pt = self.pose * na30::Point3::new(0.0, 1.0, 0.0);
            let z_pt = self.pose * na30::Point3::new(0.0, 0.0, 0.1);

            window.draw_line(&origin, &x_pt, &na30::Point3::new(1.0, 0.0, 0.0));
            window.draw_line(&origin, &y_pt, &na30::Point3::new(0.0, 1.0, 0.0));
            window.draw_line(&origin, &z_pt, &na30::Point3::new(0.0, 0.0, 1.0));
        }

        // Draw points
        self.points.iter().for_each(|&point| {
            window.draw_point(&point, &na30::Point3::new(0.5, 0.5, 0.5));
        });
    }
}
