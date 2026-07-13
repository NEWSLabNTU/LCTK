//! Regression tests for C-03 (image undistorted twice before ArUco detection).
//!
//! The contract is: `Detector::rectify` is the *only* place the detection path removes lens
//! distortion, and `Detector::detect_markers` consumes an already-rectified image without
//! warping it further. These tests pin that contract down, because the failure it prevents is
//! silent — a doubly-rectified image still detects markers, it just reports them in the wrong
//! place, and every downstream extrinsic inherits the bias.

use anyhow::Result;
use aruco_config::{ArucoDictionary, MultiArucoPattern};
use aruco_detector::multi_aruco::{Builder, Detector};
use measurements::Length;
use noisy_float::prelude::*;
use opencv::{
    core::{self, Mat, Scalar, BORDER_CONSTANT},
    prelude::*,
};
use sensor_msgs::msg::CameraInfo;

const BOARD_DPI: f64 = 40.0;
const PAD_PX: i32 = 60;

/// The pattern shipped in `ros/lctk_launch/config/aruco/aruco_pattern.json5`.
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

/// Render the pattern to an image, with a white quiet zone around the board.
fn board_image() -> Result<Mat> {
    let board = pattern().to_opencv_mat(BOARD_DPI)?;

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

fn camera_info(image: &Mat, distortion: Vec<f64>) -> CameraInfo {
    let width = image.cols() as f64;
    let height = image.rows() as f64;

    CameraInfo {
        k: [
            900.0,
            0.0,
            width / 2.0,
            0.0,
            900.0,
            height / 2.0,
            0.0,
            0.0,
            1.0,
        ],
        d: distortion,
        width: image.cols() as u32,
        height: image.rows() as u32,
        distortion_model: "plumb_bob".to_string(),
        ..Default::default()
    }
}

fn detector(image: &Mat, distortion: Vec<f64>) -> Result<Detector> {
    Builder {
        pattern: pattern(),
        camera_info: camera_info(image, distortion),
    }
    .build()
}

/// Flatten a detection into `(id, x, y)` per corner, in the detector's configured marker order.
fn corners_of(detector: &Detector, image: &Mat) -> Result<Vec<(i32, f32, f32)>> {
    let detection = detector
        .detect_markers(image)?
        .expect("the rendered board must be detected");

    Ok(detection
        .markers()
        .flat_map(|marker| {
            let id = marker.id;
            marker
                .corners
                .into_iter()
                .map(move |corner| (id, corner.x, corner.y))
        })
        .collect())
}

/// C-03: `detect_markers` must not warp the image.
///
/// Two detectors that differ *only* in their distortion coefficients must report identical
/// corners for the same input image. Before the fix, `detect_markers` undistorted internally,
/// so the heavily-distorted camera model moved every corner — which is exactly what happened in
/// production, where the ROS node had already rectified the frame.
#[test]
fn detect_markers_does_not_rectify() -> Result<()> {
    let image = board_image()?;

    let no_distortion = corners_of(&detector(&image, vec![0.0; 5])?, &image)?;
    let heavy_distortion =
        corners_of(&detector(&image, vec![-0.25, 0.08, 0.0, 0.0, 0.0])?, &image)?;

    assert_eq!(no_distortion.len(), 16, "4 markers x 4 corners");
    assert_eq!(
        no_distortion, heavy_distortion,
        "detect_markers must consume the image as given; the distortion coefficients \
         belong to rectify(), not to detection"
    );

    Ok(())
}

/// Rectifying with zero distortion is a no-op, so it must not disturb the corners either.
#[test]
fn rectify_with_zero_distortion_preserves_corners() -> Result<()> {
    let image = board_image()?;
    let detector = detector(&image, vec![0.0; 5])?;

    let direct = corners_of(&detector, &image)?;
    let rectified = corners_of(&detector, &detector.rectify(&image)?)?;

    assert_eq!(direct, rectified);

    Ok(())
}

/// Rectifying a distorted image must move the corners — i.e. `rectify` really is doing the work
/// that `detect_markers` no longer does. Without this, `detect_markers_does_not_rectify` could
/// pass simply because the distortion model was inert.
#[test]
fn rectify_with_real_distortion_moves_corners() -> Result<()> {
    let image = board_image()?;
    let detector = detector(&image, vec![-0.25, 0.08, 0.0, 0.0, 0.0])?;

    let direct = corners_of(&detector, &image)?;
    let rectified = corners_of(&detector, &detector.rectify(&image)?)?;

    let max_shift = direct
        .iter()
        .zip(&rectified)
        .map(|((_, ax, ay), (_, bx, by))| ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt())
        .fold(0.0f32, f32::max);

    assert!(
        max_shift > 1.0,
        "rectify() should visibly move corners under this distortion model, \
         moved at most {max_shift} px"
    );

    Ok(())
}
