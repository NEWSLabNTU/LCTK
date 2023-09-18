use anyhow::{anyhow, Result};
use clap::Parser;
use itertools::Itertools;
use kiss3d::{
    light::Light,
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
    let pose = na::convert(pose);

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
    points: Vec<na::Point3<f32>>,
    pose: na::Isometry3<f32>,
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        // Draw axis at origin
        {
            let origin = na::Point3::new(0.0, 0.0, 0.0);
            let x_pt = na::Point3::new(0.1, 0.0, 0.0);
            let y_pt = na::Point3::new(0.0, 0.1, 0.0);
            let z_pt = na::Point3::new(0.0, 0.0, 0.1);

            window.draw_line(&origin, &x_pt, &na::Point3::new(0.5, 0.0, 0.0));
            window.draw_line(&origin, &y_pt, &na::Point3::new(0.0, 0.5, 0.0));
            window.draw_line(&origin, &z_pt, &na::Point3::new(0.0, 0.0, 0.5));
        }

        // Draw axis at the plane origin
        {
            let origin = self.pose * na::Point3::new(0.0, 0.0, 0.0);
            let x_pt = self.pose * na::Point3::new(1.0, 0.0, 0.0);
            let y_pt = self.pose * na::Point3::new(0.0, 1.0, 0.0);
            let z_pt = self.pose * na::Point3::new(0.0, 0.0, 0.1);

            window.draw_line(&origin, &x_pt, &na::Point3::new(1.0, 0.0, 0.0));
            window.draw_line(&origin, &y_pt, &na::Point3::new(0.0, 1.0, 0.0));
            window.draw_line(&origin, &z_pt, &na::Point3::new(0.0, 0.0, 1.0));
        }

        // Draw points
        self.points.iter().for_each(|point| {
            window.draw_point(point, &na::Point3::new(0.5, 0.5, 0.5));
        });
    }
}
