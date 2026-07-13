//! Measure corner-localisation accuracy across refinement methods and apparent marker sizes.
//!
//! This is the evidence behind the H-08 default (`SUBPIX`, `win_size = 5`). Run it with:
//!
//! ```text
//! cargo run -p aruco-detector --example refinement-sweep
//! ```
//!
//! Ground truth is a board rendered at high resolution and downscaled by an exact factor, so the
//! true corner positions are known analytically: a corner at `(x, y)` in the reference render sits
//! at `(x / s, y / s)` after downscaling by `s`. Corners therefore land off the pixel grid, which
//! is precisely the regime sub-pixel refinement exists to recover.

use anyhow::Result;
use aruco_config::{
    AdaptiveThreshParams, ArucoDetectorParams, ArucoDictionary, CornerRefinementMethod,
    CornerRefinementParams, MultiArucoPattern,
};
use aruco_detector::multi_aruco::{Builder, Detector};
use measurements::Length;
use noisy_float::prelude::*;
use opencv::{
    core::{self, Mat, Scalar, Size, BORDER_CONSTANT},
    imgproc,
    prelude::*,
};
use sensor_msgs::msg::CameraInfo;

const REFERENCE_DPI: f64 = 200.0;
const PAD_PX: i32 = 60;

fn pattern() -> MultiArucoPattern {
    MultiArucoPattern {
        marker_ids: vec![696, 64, 306, 195],
        dictionary: ArucoDictionary::DICT_5X5_1000,
        board_size: Length::from_millimeters(500.0),
        board_border_size: Length::from_millimeters(10.0),
        marker_square_size_ratio: r64(0.8),
        num_squares_per_side: 2,
        border_bits: 1,
    }
}

fn reference_board() -> Result<Mat> {
    let board = pattern().to_opencv_mat(REFERENCE_DPI)?;
    let mut padded = Mat::default();
    core::copy_make_border(
        &board,
        &mut padded,
        PAD_PX,
        PAD_PX,
        PAD_PX,
        PAD_PX,
        BORDER_CONSTANT,
        Scalar::new(255.0, 255.0, 255.0, 0.0),
    )?;
    Ok(padded)
}

fn detector(image: &Mat, params: ArucoDetectorParams) -> Result<Detector> {
    let (w, h) = (image.cols() as f64, image.rows() as f64);
    Builder {
        pattern: pattern(),
        camera_info: CameraInfo {
            k: [900.0, 0.0, w / 2.0, 0.0, 900.0, h / 2.0, 0.0, 0.0, 1.0],
            d: vec![0.0; 5],
            width: image.cols() as u32,
            height: image.rows() as u32,
            distortion_model: "plumb_bob".to_string(),
            ..Default::default()
        },
        detector_params: params,
    }
    .build()
}

fn params(method: CornerRefinementMethod, win_size: i32) -> ArucoDetectorParams {
    ArucoDetectorParams {
        corner_refinement: CornerRefinementParams {
            method,
            win_size,
            ..CornerRefinementParams::default()
        },
        adaptive_thresh: AdaptiveThreshParams::default(),
    }
}

fn corners(detector: &Detector, image: &Mat) -> Result<Option<Vec<(i32, f64, f64)>>> {
    let Some(detection) = detector.detect_markers(image)? else {
        return Ok(None);
    };
    Ok(Some(
        detection
            .markers()
            .flat_map(|m| {
                let id = m.id;
                m.corners
                    .into_iter()
                    .map(move |c| (id, c.x as f64, c.y as f64))
            })
            .collect(),
    ))
}

fn rmse(truth: &[(i32, f64, f64)], got: &[(i32, f64, f64)]) -> f64 {
    let sum_sq: f64 = truth
        .iter()
        .zip(got)
        .map(|((_, tx, ty), (_, gx, gy))| (tx - gx).powi(2) + (ty - gy).powi(2))
        .sum();
    (sum_sq / truth.len() as f64).sqrt()
}

fn main() -> Result<()> {
    let reference = reference_board()?;

    // Ground truth, measured once on the high-resolution render with the best available refinement.
    let truth_hi = corners(
        &detector(&reference, params(CornerRefinementMethod::SUBPIX, 5))?,
        &reference,
    )?
    .expect("reference board must detect");

    println!(
        "\nReference render: {}x{} px, markers ~{} px\n",
        reference.cols(),
        reference.rows(),
        (pattern().marker_size().as_inches() * REFERENCE_DPI).round() as i32
    );
    println!(
        "{:>10} {:>12} {:>10} {:>10} {:>10}",
        "scale", "marker px", "NONE", "SUBPIX", "CONTOUR"
    );
    println!(
        "{:->10} {:->12} {:->10} {:->10} {:->10}",
        "", "", "", "", ""
    );

    // Downscale by exact factors. The true corner positions scale analytically.
    for scale in [5.0_f64, 7.0, 10.0, 14.0, 20.0, 28.0] {
        let mut small = Mat::default();
        imgproc::resize(
            &reference,
            &mut small,
            Size::new(
                (reference.cols() as f64 / scale).round() as i32,
                (reference.rows() as f64 / scale).round() as i32,
            ),
            0.0,
            0.0,
            imgproc::INTER_AREA,
        )?;

        let truth: Vec<(i32, f64, f64)> = truth_hi
            .iter()
            .map(|(id, x, y)| (*id, x / scale, y / scale))
            .collect();

        let marker_px = ((pattern().marker_size().as_inches() * REFERENCE_DPI) / scale).round();

        let cell = |method| -> String {
            match detector(&small, params(method, 5)).and_then(|d| corners(&d, &small)) {
                Ok(Some(got)) => format!("{:.3}", rmse(&truth, &got)),
                Ok(None) => "no detect".to_string(),
                Err(_) => "err".to_string(),
            }
        };

        println!(
            "{:>10} {:>12} {:>10} {:>10} {:>10}",
            format!("1/{scale:.0}"),
            format!("{marker_px:.0}"),
            cell(CornerRefinementMethod::NONE),
            cell(CornerRefinementMethod::SUBPIX),
            cell(CornerRefinementMethod::CONTOUR),
        );
    }

    // win_size sensitivity at the far end of the working range.
    println!("\nSUBPIX win_size sensitivity (RMSE px):\n");
    println!(
        "{:>10} {:>12} {:>8} {:>8} {:>8}",
        "scale", "marker px", "w=3", "w=5", "w=7"
    );
    println!("{:->10} {:->12} {:->8} {:->8} {:->8}", "", "", "", "", "");

    for scale in [5.0_f64, 10.0, 20.0, 28.0] {
        let mut small = Mat::default();
        imgproc::resize(
            &reference,
            &mut small,
            Size::new(
                (reference.cols() as f64 / scale).round() as i32,
                (reference.rows() as f64 / scale).round() as i32,
            ),
            0.0,
            0.0,
            imgproc::INTER_AREA,
        )?;

        let truth: Vec<(i32, f64, f64)> = truth_hi
            .iter()
            .map(|(id, x, y)| (*id, x / scale, y / scale))
            .collect();
        let marker_px = ((pattern().marker_size().as_inches() * REFERENCE_DPI) / scale).round();

        let cell = |win| -> String {
            match detector(&small, params(CornerRefinementMethod::SUBPIX, win))
                .and_then(|d| corners(&d, &small))
            {
                Ok(Some(got)) => format!("{:.3}", rmse(&truth, &got)),
                Ok(None) => "no det".to_string(),
                Err(_) => "err".to_string(),
            }
        };

        println!(
            "{:>10} {:>12} {:>8} {:>8} {:>8}",
            format!("1/{scale:.0}"),
            format!("{marker_px:.0}"),
            cell(3),
            cell(5),
            cell(7),
        );
    }
    println!();

    Ok(())
}
