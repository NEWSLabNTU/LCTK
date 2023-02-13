#![cfg(all(feature = "with-nalgebra", feature = "with-opencv"))]

use anyhow::{ensure, Result};
use cv_convert::prelude::*;
use itertools::izip;
use nalgebra as na;
use opencv::{calib3d, core as core_cv};
use rand::prelude::*;
use serde_types::DevicePath;
use wayside_params::infra_v1;

const INFRA_V1_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/infra_v1_example",);

#[test]
fn coord_transform_map_test() -> Result<()> {
    let params = infra_v1::InfraV1::load(INFRA_V1_DIR)?;
    let _map = params.to_coord_transform_map();

    Ok(())
}

#[test]
fn combine_nalgebra_opencv_camera_model() -> Result<()> {
    const NUM_POINTS: usize = 1000;

    let mut rng = rand::thread_rng();
    let params = infra_v1::InfraV1::load(INFRA_V1_DIR)?;

    let lidar_key = DevicePath::new("wayside_1", "lidar1");
    let camera_key = DevicePath::new("wayside_1", "camera1");

    let camera_intrinsic = &params.camera_intrinsic[&camera_key].intrinsic;
    let coord_transform = &params.coordinate_transform[&(lidar_key, camera_key)].transform;

    let camera_matrix: core_cv::Mat = (&camera_intrinsic.camera_matrix).into();
    let distortion_coefs: core_cv::Mat = (&camera_intrinsic.distortion_coefs).into();

    let world_points: Vec<[f64; 3]> = (0..NUM_POINTS).map(|_| rng.gen()).collect();

    let pixels_1 = {
        let cv_convert::OpenCvPose::<core_cv::Mat> { rvec, tvec } = coord_transform.into();

        let world_points: core_cv::Vector<core_cv::Point3d> = world_points
            .iter()
            .map(|point| {
                let [x, y, z] = *point;
                core_cv::Point3d::new(x, y, z)
            })
            .collect();
        let mut pixels = core_cv::Vector::<core_cv::Point2d>::new();

        calib3d::project_points(
            &world_points,
            &rvec,
            &tvec,
            &camera_matrix,
            &distortion_coefs,
            &mut pixels,
            &mut core_cv::no_array(), // jacobian
            0.0,                      // aspect_ratio
        )?;

        pixels
    };

    let pixels_2 = {
        let isometry: na::Isometry3<f64> = coord_transform.into();
        let cv_convert::OpenCvPose::<core_cv::Mat> { rvec, tvec } =
            na::Isometry3::<f64>::identity().try_into_cv()?;

        let camera_points: core_cv::Vector<core_cv::Point3d> = world_points
            .iter()
            .map(|point| {
                let [x, y, z] = *point;
                let camera_point = isometry * na::Point3::new(x, y, z);
                camera_point.into_cv()
            })
            .collect();
        let mut pixels = core_cv::Vector::<core_cv::Point2d>::new();

        calib3d::project_points(
            &camera_points,
            &rvec,
            &tvec,
            &camera_matrix,
            &distortion_coefs,
            &mut pixels,
            &mut core_cv::no_array(), // jacobian
            0.0,                      // aspect_ratio
        )?;

        pixels
    };

    izip!(world_points, pixels_1, pixels_2)
        .enumerate()
        .try_for_each(|(index, (world_point, lhs, rhs))| {
            let ok = relative_eq(lhs.x, rhs.x) && relative_eq(lhs.y, rhs.y);
            ensure!(
                ok,
                "output image point mismatch
index = {}
world_point = ({}, {}, {})
image_point_1 = ({}, {})
image_point_2 = ({}, {})
",
                index,
                world_point[0],
                world_point[1],
                world_point[2],
                lhs.x,
                lhs.y,
                rhs.x,
                rhs.y,
            );
            Ok(())
        })?;

    Ok(())
}

//     #[test]
//     fn project_points() -> Result<()> {
//         const NUM_POINTS: usize = 1000;

//         let mut rng = rand::thread_rng();
//         let params = infra_v1::InfraV1::load(format!(
//             "{}/../config/params/two-vlp-32c",
//             env!("CARGO_MANIFEST_DIR")
//         ))?;

//         let lidar_key = DevicePath {
//             host: "wayside_1".into(),
//             device: "lidar1".into(),
//         };
//         let camera_key = DevicePath {
//             host: "wayside_1".into(),
//             device: "camera1".into(),
//         };

//         let camera_intrinsic = &params.camera_intrinsic[&camera_key].intrinsic;
//         let coord_transform = &params.coordinate_transform[&(lidar_key, camera_key)].transform;

//         let world_points: Vec<[f64; 3]> = (0..NUM_POINTS).map(|_| rng.gen()).collect();

//         let instant = Instant::now();
//         let na_pixels = {
//             let extrinsics = cam_geom::ExtrinsicParameters::from_pose(&coord_transform.into());
//             let intrinsics: opencv_ros_camera::RosOpenCvIntrinsics<f64> = camera_intrinsic.into();
//             let camera = cam_geom::Camera::new(intrinsics, extrinsics);

//             let world_points = {
//                 let rows: Vec<_> = world_points
//                     .iter()
//                     .map(|point| {
//                         let [x, y, z] = *point;
//                         na::RowVector3::new(x, y, z)
//                     })
//                     .collect();
//                 cam_geom::Points::new(na::MatrixXx3::from_rows(&rows))
//             };

//             let pixels = camera.world_to_pixel(&world_points);
//             pixels
//         };
//         eprintln!("nalgebra time {:?}", instant.elapsed());

//         let instant = Instant::now();
//         let cv_pixels = {
//             let cv_convert::OpenCvPose::<core_cv::Mat> { rvec, tvec } = coord_transform.into();
//             let camera_matrix: core_cv::Mat = (&camera_intrinsic.camera_matrix).into();
//             let distortion_coefs: core_cv::Mat = (&camera_intrinsic.distortion_coefs).into();

//             let world_points: core_cv::Vector<core_cv::Point3d> = world_points
//                 .iter()
//                 .map(|point| {
//                     let [x, y, z] = *point;
//                     core_cv::Point3d::new(x, y, z)
//                 })
//                 .collect();
//             let mut pixels = core_cv::Vector::<core_cv::Point2d>::new();

//             calib3d::project_points(
//                 &world_points,
//                 &rvec,
//                 &tvec,
//                 &camera_matrix,
//                 &distortion_coefs,
//                 &mut pixels,
//                 &mut core_cv::no_array(), // jacobian
//                 0.0,                      // aspect_ratio
//             )?;

//             pixels
//         };
//         eprintln!("opencv time {:?}", instant.elapsed());

//         na_pixels
//             .data
//             .row_iter()
//             .zip_eq(cv_pixels.iter())
//             .enumerate()
//             .try_for_each(|(index, (na_pixel, cv_pixel))| {
//                 let ok =
//                     relative_eq(na_pixel[0], cv_pixel.x) && relative_eq(na_pixel[1], cv_pixel.y);

//                 ensure!(
//                     ok,
//                     "output pixel mismatch
// - index = {}
// - world point = ({}, {}, {})
// - na_pixel = ({}, {})
// - cv_pixel = ({}, {})",
//                     index,
//                     world_points[index][0],
//                     world_points[index][1],
//                     world_points[index][2],
//                     na_pixel[0],
//                     na_pixel[1],
//                     cv_pixel.x,
//                     cv_pixel.y,
//                 );

//                 Ok(())
//             })?;

//         Ok(())
//     }

fn relative_eq(lhs: f64, rhs: f64) -> bool {
    const EPSILON: f64 = 1e-10;
    let diff = (lhs - rhs).abs();
    diff <= lhs.abs() * EPSILON && diff <= rhs.abs() * EPSILON
}
