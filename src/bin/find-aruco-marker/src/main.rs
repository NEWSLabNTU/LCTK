use anyhow::ensure;
use aruco_config::MultiArucoPattern;
use clap::Parser;
use opencv::{
    aruco, calib3d,
    core::{no_array, Point2i, Scalar},
    highgui,
    imgproc::{self, HersheyFonts, LINE_8},
    prelude::*,
    videoio::{VideoCapture, CAP_ANY},
};
use serde_loader::Json5Path;
use serde_types::{CameraIntrinsics, MrptCalibration};
use std::{fs, path::PathBuf};

const ARUCO_PATTERN_CONFIG: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/config/aruco_pattern.json5");

#[derive(Parser)]
struct Opts {
    /// The file storing intrinsic camera parameters.
    pub intrinsics_file: PathBuf,
    /// The directory containing input PCD files.
    pub input_file: String,
    /// The directory to store output detections.
    pub output_dir: PathBuf,
    #[clap(long)]
    pub gui: bool,
}

fn main() -> Result<(), anyhow::Error> {
    let opts: Opts = Opts::parse();

    let mrpt_calib: MrptCalibration = {
        let yaml_text = fs::read_to_string(&opts.intrinsics_file)?;
        serde_yaml::from_str(&yaml_text)?
    };
    let camera_intrinsic: CameraIntrinsics = mrpt_calib.intrinsic_params()?;
    let aruco_pattern: MultiArucoPattern = Json5Path::open_and_take(ARUCO_PATTERN_CONFIG)?;
    let detector = aruco_detector::multi_aruco::Builder {
        pattern: aruco_pattern,
        camera_intrinsic: camera_intrinsic.clone(),
    }
    .build()?;

    let mut capture = VideoCapture::from_file(&opts.input_file, CAP_ANY)?;

    ensure!(!opts.output_dir.exists(), "output directory already exists");
    fs::create_dir_all(&opts.output_dir)?;

    for idx in 0.. {
        let mut image = Mat::default();
        capture.read(&mut image)?;
        let detection = detector.detect_markers(&image)?;

        if opts.gui {
            let orig_image = image;
            let mut image = Mat::default();
            let camera_matrix: Mat = (&camera_intrinsic.camera_matrix).into();
            let dist_coeffs: Mat = (&camera_intrinsic.distortion_coefs).into();

            calib3d::undistort(
                &orig_image,
                &mut image,
                &camera_matrix,
                &dist_coeffs,
                &mut no_array(),
            )?;

            let draw_text = |image: &mut Mat, text: &str, (x, y), (b, g, r)| {
                imgproc::put_text(
                    image,
                    text,
                    Point2i { x, y },
                    HersheyFonts::FONT_HERSHEY_SIMPLEX as i32,
                    1.0,
                    Scalar::new(b, g, r, 0.0),
                    2,
                    LINE_8,
                    false,
                )
            };

            // draw detected marker
            if let Some(detection) = &detection {
                // draw text
                draw_text(
                    &mut image,
                    &format!("Found ArUco IDs: {:?}", detection.id()),
                    (10, 50),
                    (0.0, 255.0, 0.0),
                )?;

                // draw detected markers
                aruco::draw_detected_markers(
                    &mut image,
                    detection.corners(),
                    detection.id(),
                    Scalar::new(0.0, 255.0, 0.0, 0.0), // color
                )?;
            } else {
                draw_text(&mut image, "No ArUco detected", (10, 50), (255.0, 0.0, 0.0))?;
            }

            highgui::imshow(env!("CARGO_PKG_NAME"), &image)?;
            highgui::wait_key(1)?;
        }

        if let Some(detection) = detection {
            let file_name = format!("{}.json5", idx);
            let output_path = opts.output_dir.join(file_name);
            let markers: Vec<_> = detection.markers().collect();

            let json5_text = serde_json::to_string_pretty(&markers)?;
            fs::write(&output_path, &json5_text)?;
        }
    }

    Ok(())
}
