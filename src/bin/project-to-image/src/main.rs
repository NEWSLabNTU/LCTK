use anyhow::{ensure, Result};
use clap::Parser;
use cv_convert::{prelude::*, OpenCvPose};
use itertools::izip;
use nalgebra as na;
use opencv::{
    calib3d,
    core::{no_array, Point2d, Point2i, Point3d, Scalar, Vector},
    highgui, imgcodecs,
    imgcodecs::IMREAD_COLOR,
    imgproc,
    imgproc::LINE_8,
    prelude::*,
};
use palette::{Hsv, IntoColor, RgbHue, Srgb};
use pcd_rs::DynReader;
use serde_types::MrptCalibration;
use std::{fs, path::PathBuf};

#[derive(Debug, Parser)]
struct Opts {
    /// Do not show image.
    #[clap(long)]
    pub no_gui: bool,

    /// The file storing intrinsic camera parameters.
    #[clap(long)]
    pub intrinsics_file: PathBuf,

    /// The file storing extrinsic camera parameters.
    #[clap(long)]
    pub extrinsics_file: PathBuf,

    #[clap(long)]
    pub pcd_file: PathBuf,

    #[clap(long)]
    pub image_file: String,

    #[clap(long)]
    pub output_file: Option<String>,

    #[clap(long)]
    pub keep_back_points: bool,

    #[clap(long, default_value = "10.0")]
    pub max_distance: f64,

    #[clap(long, default_value = "1.0")]
    pub min_distance: f64,

    #[clap(long, default_value = "0.0")]
    pub camera_depth_thresh: f64,
}

fn main() -> Result<()> {
    let Opts {
        no_gui,
        intrinsics_file,
        extrinsics_file,
        pcd_file,
        image_file,
        output_file,
        keep_back_points,
        max_distance,
        min_distance,
        camera_depth_thresh,
    } = Opts::parse();
    ensure!(min_distance < max_distance && min_distance >= 0.0 && max_distance >= 0.0);

    let mrpt_calib: MrptCalibration = {
        let yaml_text = fs::read_to_string(&intrinsics_file)?;
        serde_yaml::from_str(&yaml_text)?
    };
    let camera_matrix = mrpt_calib.camera_matrix.to_opencv();
    let dist_coefs = mrpt_calib.distortion_coefficients.to_opencv();
    let pose: na::Isometry3<f64> = {
        let json5_text = fs::read_to_string(&extrinsics_file)?;
        json5::from_str(&json5_text)?
    };

    let input_points: Vec<na::Point3<f64>> = {
        let reader = DynReader::open(pcd_file)?;
        reader
            .map(|point| {
                let [x, y, z]: [f32; 3] = point.unwrap().to_xyz().unwrap();
                na::Point3::new(x as f64, y as f64, z as f64)
            })
            .collect()
    };

    let mut image = imgcodecs::imread(&image_file, IMREAD_COLOR)?;
    let image_h = image.rows();
    let image_w = image.cols();

    // let mut image = {
    //     let mut out = Mat::default();
    //     calib3d::undistort(&image, &mut out, &camera_matrix, &dist_coefs, &no_array())?;
    //     out
    // };

    let distance_range = min_distance..=max_distance;
    let width_range = 0.0..=(image_w as f64);
    let height_range = 0.0..=(image_h as f64);

    let (pcd_points, image_points) = {
        let (pcd_points, opencv_points): (Vec<_>, Vector<Point3d>) = input_points
            .iter()
            .filter(|&pcd_point| {
                let distance = na::distance(&na::Point3::origin(), pcd_point);
                distance_range.contains(&distance)
            })
            .filter(|&pcd_point| {
                keep_back_points || {
                    let camera_point = pose * pcd_point;
                    // distance_range.contains(&camera_point.z)
                    camera_point.z >= camera_depth_thresh
                }
            })
            .map(|point| {
                let cv_point: Point3d = point.to_cv();
                (point, cv_point)
            })
            .unzip();

        let mut image_points = Vector::<Point2d>::new();

        if opencv_points.is_empty() {
            (pcd_points, image_points)
        } else {
            let OpenCvPose::<Mat> { rvec, tvec } = pose.try_to_cv()?;

            // lidar_points must be non-empty, otherwise it panics

            calib3d::project_points(
                &opencv_points,
                &rvec,
                &tvec,
                &camera_matrix,
                &dist_coefs,
                &mut image_points,
                &mut no_array(),
                0.0,
            )?;

            (pcd_points, image_points)
        }
    };

    izip!(pcd_points, image_points)
        .filter(|(_pt3, pt2)| width_range.contains(&pt2.x) && height_range.contains(&pt2.y))
        .for_each(|(pt3, pt2)| {
            let distance = na::distance(&na::Point3::origin(), &pt3);
            let color = {
                let ratio = (distance - min_distance) / (max_distance - min_distance);
                if (0.0..=1.0).contains(&ratio) {
                    let hue = RgbHue::from_degrees(ratio as f32 * 270.0);
                    let hsv = Hsv::new(hue, 0.8, 1.0);
                    let srgb: Srgb = hsv.into_color();
                    let (r, g, b) = srgb.into_components();
                    Scalar::new(
                        (b * 255.0) as f64,
                        (g * 255.0) as f64,
                        (r * 255.0) as f64,
                        0.0,
                    )
                } else {
                    Scalar::new(100.0, 100.0, 100.0, 0.0)
                }
            };

            let position: Point2i = pt2.to().unwrap();
            imgproc::circle(&mut image, position, 1, color, 1, LINE_8, 0).unwrap();
        });

    if let Some(path) = output_file {
        imgcodecs::imwrite(&path, &image, &Vector::new())?;
    }

    if !no_gui {
        highgui::imshow(env!("CARGO_PKG_NAME"), &image)?;
        highgui::wait_key(0)?;
    }

    Ok(())
}
