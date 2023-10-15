use crate::{
    common::*,
    config,
    config::Config,
    fuse_gui, select_gui,
    utils::{isometry3_30_to_32, isometry3_32_to_30, p30_to_p32_vec},
};
use chrono::offset::Local;
use generic_point_filter::Pt64;
use hollow_board_detector::Detection;
use kiss3d::{
    camera::ArcBall,
    event::Key,
    light::Light,
    nalgebra as na,
    window::{State, Window},
};
use nalgebra as na32;
use protos::LidarPoint;
use rand::rngs::OsRng;
use serde_loader::Json5Path;
use std::f64::consts::{FRAC_PI_2, PI};
use velodyne_lidar::types::{
    measurements::Measurement,
    point::{Point, PointD, PointS},
};
use wayside_params::infra_v1;

pub struct ResultStruct {
    detection: Detection,
    original_points: Vec<protos::LidarPoint>,
    filtered_points: Vec<protos::LidarPoint>,
}

pub enum PcapNumber {
    First,
    Second,
}

pub fn detector(config_path: PathBuf) -> Result<()> {
    let config: Config = Json5Path::open_and_take(config_path.clone())?;

    eprintln!("Start detection on pcap file 1 ...");
    let res_1 = pcap_to_pose(config_path.clone(), PcapNumber::First);
    eprintln!("Pcap file1's pose detection finished.");

    eprintln!();
    eprintln!("Start detection on pcap file 2 ...");
    let res_2 = pcap_to_pose(config_path.clone(), PcapNumber::Second);
    eprintln!("Pcap file2's pose detection finished.");

    let (pose_1, pose_2) = fuse_points(
        res_1.unwrap(),
        res_2.unwrap(),
        config.using_same_face_of_marker,
    )
    .with_context(|| "fusing failed.")?;

    let exec_time = Local::now();
    let dir_name = exec_time.to_rfc3339();
    let path = config.output_dir.join(dir_name);
    fs::create_dir_all(&path)?;
    logging(pose_1, pose_2, config.using_same_face_of_marker, &path)
        .with_context(|| "Logging failed.")?;

    Ok(())
}

fn pcap_to_pose(config_path: PathBuf, pcap_number: PcapNumber) -> Result<ResultStruct> {
    let config: Config = Json5Path::open_and_take(config_path.clone())?;

    let pcap_config = match pcap_number {
        PcapNumber::First => config.pcap1_config,
        PcapNumber::Second => config.pcap2_config,
    };

    let pcap_file: Cow<'_, Path> = if pcap_config.file_path.is_dir() {
        let latest_pcap_file = fs::read_dir(&pcap_config.file_path)?
            .filter_map(|entry| {
                let entry = entry.ok()?;
                let file_name = entry.file_name();
                let file_name = file_name.to_str()?;
                let time = DateTime::parse_from_rfc3339(file_name).ok()?;
                let path = entry.path();
                Some((path, time))
            })
            .max_by_key(|(_, time)| *time)
            .map(|(path, _)| path)
            .ok_or_else(|| anyhow!("This directory has no pcap directory."))?;

        eprintln!("Selected the file: {}", latest_pcap_file.display());
        latest_pcap_file.into()
    } else {
        pcap_config.file_path.get().into()
    };

    let lidar_config = {
        use config::SensorType as S;
        use velodyne_lidar::Config as C;

        match pcap_config.sensor {
            S::PuckHires => C::new_puck_hires_strongest(),
            S::Vlp32c => C::new_vlp_32c_strongest(),
        }
    };

    let board_detector = hollow_board_detector::Detector::new(
        (*config.board_detector).clone(),
        (*config.aruco_pattern).clone(),
    );

    let mut gui = select_gui::Gui::new(pcap_config.clone());
    let mut filtered_points = vec![];
    let mut frame_selected = pcap_config.frame_selected.unwrap_or(0);

    while gui.press_key.is_none() || gui.press_key.unwrap() == Key::Escape {
        frame_selected = pcap_config
            .frame_selected
            .unwrap_or_else(|| frame_selected + OsRng.gen_range(0..20));

        let mut frame_iter =
            velodyne_lidar::iter::frame_xyz_iter_from_file(lidar_config.clone(), &pcap_file)?;
        let frame = frame_iter
            .nth(frame_selected)
            .ok_or_else(|| anyhow!("frame at index {} does not exist", frame_selected))?;

        let (original_points, points_in_lidar_point_format) = frame?
            .into_indexed_point_iter()
            .map(|((row, col), point)| transform_point(row, col, point))
            .map(|point| {
                let position = na::Point3::new(point.x, point.y, point.z);
                (position, point)
            })
            .unzip_n_vec();

        filtered_points = preprocess_points(&points_in_lidar_point_format, &pcap_config.filter);

        // transform Vec<protos::LidarPoint> into Vec<na::Point3>
        let points_in_point3_format: Vec<_> = filtered_points
            .iter()
            .map(|point| na::Point3::new(point.x, point.y, point.z))
            .collect();

        let det = board_detector.detect(&p30_to_p32_vec(&points_in_point3_format))?;

        let mut window = Window::new_with_size(
            &format!("detection_result_with_frame{}", frame_selected),
            3000,
            1000,
        );
        let mut camera = ArcBall::new(
            na::Point3::new(0.0, -80.0, 32.0),
            na::Point3::new(0.0, 0.0, 0.0),
        );

        camera.set_up_axis(na::Vector3::new(0.0, 0.0, 1.0));
        window.set_light(Light::StickToCamera);

        gui.original_points = original_points;
        gui.points_in_lidar_point_format = points_in_lidar_point_format.clone();
        gui.points_in_point3_format = points_in_point3_format;
        gui.det = det;

        while window.render_with_camera(&mut camera) {
            if let Some(Key::U) = gui.press_key {
                let config: Config = Json5Path::open_and_take(config_path.clone())?;
                let pcap_config = match pcap_number {
                    PcapNumber::First => config.pcap1_config,
                    PcapNumber::Second => config.pcap2_config,
                };
                filtered_points =
                    preprocess_points(&points_in_lidar_point_format, &pcap_config.filter);
                // transform Vec<protos::LidarPoint> into Vec<na::Point3>
                let points_in_point3_format: Vec<_> = filtered_points
                    .iter()
                    .map(|point| na::Point3::new(point.x, point.y, point.z))
                    .collect();
                let det = board_detector.detect(&p30_to_p32_vec(&points_in_point3_format))?;
                gui.det = det;
                gui.pcap_config = pcap_config;
                gui.points_in_point3_format = points_in_point3_format;
                gui.press_key = None;
            }
            gui.step(&mut window);
        }
    }
    eprintln!(
        "You chose the frame{}'s detection result as your pose.",
        frame_selected
    );

    let board_detection = gui
        .det
        .ok_or_else(|| anyhow!("unable to detect board in point cloud"))?;

    let result = ResultStruct {
        detection: board_detection,
        original_points: gui.points_in_lidar_point_format,
        filtered_points,
    };

    Ok(result)
}

fn preprocess_points(
    points: &[protos::LidarPoint],
    filter: &generic_point_filter::Filter,
) -> Vec<protos::LidarPoint> {
    let points: Vec<_> = points
        .iter()
        .cloned()
        .filter(|point| {
            let LidarPoint {
                x, y, z, intensity, ..
            } = *point;

            let point = Pt64 {
                xyz: [x, y, z],
                intensity,
            };
            filter.contains(&point)
        })
        .collect();
    filter.step();
    points
}

pub fn fuse_points(
    det1: ResultStruct,
    det2: ResultStruct,
    using_same_face_of_marker: bool,
) -> Result<(na::Isometry3<f64>, na::Isometry3<f64>)> {
    let points1: Vec<_> = det1
        .original_points
        .iter()
        .map(na32::Point3::from)
        // .map(p32_to_p30)
        .collect();
    let points2: Vec<_> = det2
        .original_points
        .iter()
        .map(na32::Point3::from)
        // .map(p32_to_p30)
        .collect();
    let filtered_points1: Vec<_> = det1
        .filtered_points
        .iter()
        .map(na32::Point3::from)
        // .map(p32_to_p30)
        .collect();
    let filtered_points2: Vec<_> = det2
        .filtered_points
        .iter()
        .map(na32::Point3::from)
        // .map(p32_to_p30)
        .collect();

    let mut gui = fuse_gui::Gui {
        state: fuse_gui::WindowName::FusingWindow,
        window1: fuse_gui::WindowState {
            points: points1,
            filtered_points: filtered_points1,
            board_pose: det1.detection,
        },
        window2: fuse_gui::WindowState {
            points: points2,
            filtered_points: filtered_points2,
            board_pose: det2.detection,
        },
        camera: {
            let mut camera = ArcBall::new(
                na::Point3::new(0.0, -80.0, 32.0),
                na::Point3::new(0.0, 0.0, 0.0),
            );
            camera.set_up_axis(na::Vector3::new(0.0, 0.0, 1.0));
            camera
        },
        using_same_face_of_marker,
    };

    let mut window = Window::new_with_size("3D Points Fusion Example", 3000, 1000);
    window.set_light(Light::StickToCamera);

    while window.render_with_state(&mut gui) {}

    Ok((
        isometry3_32_to_30(gui.window1.board_pose.board_model.pose),
        isometry3_32_to_30(gui.window2.board_pose.board_model.pose),
    ))
}

fn logging(
    pose1: na::Isometry3<f64>,
    pose2: na::Isometry3<f64>,
    using_same_face_of_marker: bool,
    path: impl AsRef<Path>,
) -> Result<()> {
    let path = path.as_ref();
    let bug_transform = na::Isometry3::from_parts(
        na::Translation3::identity(),
        na::UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
    );

    let lidar1_to_lidar2 = if using_same_face_of_marker {
        bug_transform.inverse() * pose2 * pose1.inverse() * bug_transform
    } else {
        let inner_pose = na::Isometry3::from_parts(
            na::Translation3::identity(),
            na::UnitQuaternion::from_axis_angle(
                &na::Unit::new_normalize(na::Vector3::new(1.0, 1.0, 0.0)),
                PI,
            ),
        );
        bug_transform.inverse() * pose2 * inner_pose * pose1.inverse() * bug_transform
    };

    let lidar2_to_lidar1 = lidar1_to_lidar2.inverse();

    let from_device = DevicePath::new("from", "lidar1");
    let to_device = DevicePath::new("to", "lidar1");

    let lidar1_to_lidar2: infra_v1::ParameterConfig = infra_v1::CoordinateTransform {
        from: from_device.clone().into(),
        to: to_device.clone().into(),
        transform: common_types::serde_types::Isometry3D::from(&isometry3_30_to_32(
            lidar1_to_lidar2,
        )),
    }
    .into();
    let lidar2_to_lidar1: infra_v1::ParameterConfig = infra_v1::CoordinateTransform {
        from: to_device.into(),
        to: from_device.into(),
        transform: isometry3_30_to_32(lidar2_to_lidar1).into(),
    }
    .into();

    fs::write(
        path.join("lidar1_to_lidar2_pose.json"),
        &serde_json::to_string_pretty(&lidar1_to_lidar2)?,
    )?;
    fs::write(
        path.join("lidar2_to_lidar1_pose.json"),
        &serde_json::to_string_pretty(&lidar2_to_lidar1)?,
    )?;
    Ok(())
}

fn transform_point(row: usize, col: usize, point: Point) -> protos::LidarPoint {
    match point {
        Point::Single(point) => {
            let PointS {
                laser_id,
                time,
                measurement:
                    Measurement {
                        intensity,
                        xyz: [x, y, z],
                        distance,
                        ..
                    },
                azimuth,
                ..
            } = point;
            protos::LidarPoint {
                x: x.as_meters(),
                y: y.as_meters(),
                z: z.as_meters(),
                timestamp_ns: time.as_nanos() as u64,
                intensity: Some(intensity as f64),
                laser_id: Some(laser_id as u32),
                distance: Some(distance.as_meters()),
                original_azimuth_angle: Some(azimuth.as_radians()),
                corrected_azimuth_angle: Some(azimuth.as_radians()),
                row_idx: Some(row as u64),
                col_idx: Some(col as u64),
            }
        }
        Point::Dual(point) => {
            // takes strongesst laser return
            let PointD {
                laser_id,
                time,
                azimuth,
                ..
            } = point;
            let Measurement {
                intensity,
                xyz: [x, y, z],
                distance,
                ..
            } = *point.measurement_strongest();

            protos::LidarPoint {
                x: x.as_meters(),
                y: y.as_meters(),
                z: z.as_meters(),
                timestamp_ns: time.as_nanos() as u64,
                intensity: Some(intensity as f64),
                laser_id: Some(laser_id as u32),
                distance: Some(distance.as_meters()),
                original_azimuth_angle: Some(azimuth.as_radians()),
                corrected_azimuth_angle: Some(azimuth.as_radians()),
                row_idx: Some(row as u64),
                col_idx: Some(col as u64),
            }
        }
    }
}
