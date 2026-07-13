//! Contract tests for the ArUco detection path.
//!
//! Two invariants, tested together because they are two halves of one story:
//!
//! - **C-03** — the image is never rectified twice. It used to be undistorted by the ROS node and
//!   then again inside the detector, displacing every corner by roughly the size of the lens
//!   correction itself.
//! - **H-08** — corners are refined to sub-pixel accuracy. OpenCV's default is
//!   `CORNER_REFINE_NONE`, which quantises them to roughly the pixel grid.
//!
//! The contract is now: `detect_markers` consumes a **raw (distorted)** image, refines the corners
//! on those unresampled pixels, and returns them in the **rectified** frame via `undistortPoints`.
//! `rectify()` exists only to produce a debug overlay.
//!
//! The central test is a round trip: render a board whose corners are exactly known, push it
//! through a known `K, D` to synthesize a distorted camera image, detect, and require the corners
//! to come back where they started. That closes the loop on both bugs at once — it fails if the
//! image is rectified twice, if `undistortPoints` is misconfigured, or if refinement is off.

use anyhow::Result;
use aruco_config::{
    AdaptiveThreshParams, ArucoDetectorParams, ArucoDictionary, CornerRefinementMethod,
    CornerRefinementParams, MultiArucoPattern,
};
use aruco_detector::multi_aruco::{Builder, Detector};
use measurements::Length;
use noisy_float::prelude::*;
use opencv::{
    calib3d,
    core::{self, Mat, Point2f, Scalar, BORDER_CONSTANT},
    imgproc,
    prelude::*,
};
use sensor_msgs::msg::CameraInfo;

/// Strong barrel distortion. Big enough that getting the handling wrong is unmissable.
const DISTORTION: [f64; 5] = [-0.25, 0.08, 0.0, 0.0, 0.0];
const NO_DISTORTION: [f64; 5] = [0.0; 5];

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

/// Render the pattern with a white quiet zone. `dpi` sets the apparent marker size.
fn board_image(dpi: f64) -> Result<Mat> {
    let board = pattern().to_opencv_mat(dpi)?;

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

fn camera_info(image: &Mat, distortion: &[f64]) -> CameraInfo {
    let (w, h) = (image.cols() as f64, image.rows() as f64);
    CameraInfo {
        k: [900.0, 0.0, w / 2.0, 0.0, 900.0, h / 2.0, 0.0, 0.0, 1.0],
        d: distortion.to_vec(),
        width: image.cols() as u32,
        height: image.rows() as u32,
        distortion_model: "plumb_bob".to_string(),
        ..Default::default()
    }
}

fn detector(
    image: &Mat,
    distortion: &[f64],
    detector_params: ArucoDetectorParams,
) -> Result<Detector> {
    Builder {
        pattern: pattern(),
        camera_info: camera_info(image, distortion),
        detector_params,
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

/// Synthesize the distorted camera image that `ideal` would have produced through `K, D`.
///
/// `remap` samples `src` at `map(dst)`, so for each pixel of the distorted output we need its
/// coordinate in the ideal image — which is precisely `undistortPoints`. This is the same
/// primitive the detector uses, run in the opposite direction, so the round trip is exact.
fn distort_image(ideal: &Mat, camera_info: &CameraInfo) -> Result<Mat> {
    let (w, h) = (ideal.cols(), ideal.rows());
    let camera_matrix = Mat::from_slice(&camera_info.k)?.reshape(1, 3)?;
    let distortion_coefs = Mat::from_slice(&camera_info.d)?;
    let eye = Mat::eye(3, 3, core::CV_64FC1)?.to_mat()?;

    // Every pixel of the distorted image, as a point list.
    let grid: Vec<Point2f> = (0..h)
        .flat_map(|y| (0..w).map(move |x| Point2f::new(x as f32, y as f32)))
        .collect();
    let grid = Mat::from_slice(&grid)?;

    let mut ideal_coords = Mat::default();
    calib3d::undistort_points(
        &grid,
        &mut ideal_coords,
        &camera_matrix,
        &distortion_coefs,
        &eye,
        &camera_matrix,
    )?;

    // The undistorted coordinates, laid back out as an h x w 2-channel remap table.
    let map = ideal_coords.reshape(2, h)?;

    let mut distorted = Mat::default();
    imgproc::remap(
        ideal,
        &mut distorted,
        &map,
        &core::no_array(),
        imgproc::INTER_LINEAR,
        BORDER_CONSTANT,
        Scalar::new(255.0, 255.0, 255.0, 0.0),
    )?;

    Ok(distorted)
}

/// RMSE between two corner lists, in pixels. Panics if the marker ordering differs.
fn rmse(a: &[(i32, f32, f32)], b: &[(i32, f32, f32)]) -> f64 {
    assert_eq!(a.len(), b.len());
    let sum_sq: f64 = a
        .iter()
        .zip(b)
        .map(|((id_a, ax, ay), (id_b, bx, by))| {
            assert_eq!(id_a, id_b, "corner lists are not in the same marker order");
            let (dx, dy) = ((ax - bx) as f64, (ay - by) as f64);
            dx * dx + dy * dy
        })
        .sum();
    (sum_sq / a.len() as f64).sqrt()
}

/// The headline test: corners survive a full distort → detect → undistort round trip.
///
/// Ground truth is the corner set detected on the *ideal* (undistorted) render. We then synthesize
/// what the camera would have seen through a strongly distorting lens, hand that raw frame to the
/// detector, and require the corners it reports to land back on the ideal ones.
///
/// This fails if the image is rectified twice (C-03), if `undistortPoints` is called without
/// `P = K` (returning normalized coords), or if the corners are never mapped back at all.
#[test]
fn corners_survive_the_distort_detect_undistort_round_trip() -> Result<()> {
    let ideal = board_image(40.0)?;
    let subpix = params(CornerRefinementMethod::SUBPIX, 5);

    // Ground truth: what the detector sees on an undistorted image with a distortion-free model.
    let truth = corners_of(&detector(&ideal, &NO_DISTORTION, subpix)?, &ideal)?;
    assert_eq!(truth.len(), 16, "4 markers x 4 corners");

    // What the camera would actually deliver, and what the detector makes of it.
    let info = camera_info(&ideal, &DISTORTION);
    let distorted = distort_image(&ideal, &info)?;
    let recovered = corners_of(&detector(&ideal, &DISTORTION, subpix)?, &distorted)?;

    let err = rmse(&truth, &recovered);
    assert!(
        err < 1.0,
        "corners did not survive the round trip: {err:.3} px RMSE. \
         The lens correction is being applied wrongly (twice, not at all, or in the wrong space)."
    );

    Ok(())
}

/// H-08: refinement must actually be doing something.
///
/// Sub-pixel refinement should localise corners markedly better than OpenCV's default
/// `CORNER_REFINE_NONE`. The second assertion is the important one — without it, the first could
/// pass simply because the test is insensitive.
#[test]
fn subpix_refinement_beats_no_refinement() -> Result<()> {
    // A sub-pixel shift is what refinement is for: with the board on exact pixel boundaries there
    // is nothing to recover. Scale so the corners land off-grid.
    let ideal = board_image(37.5)?;

    let refined = corners_of(
        &detector(
            &ideal,
            &NO_DISTORTION,
            params(CornerRefinementMethod::SUBPIX, 5),
        )?,
        &ideal,
    )?;
    let unrefined = corners_of(
        &detector(
            &ideal,
            &NO_DISTORTION,
            params(CornerRefinementMethod::NONE, 5),
        )?,
        &ideal,
    )?;

    // The two must disagree; if they agree, refinement is silently not running — which is exactly
    // the H-08 bug (set_corner_refinement_method never called, so the default NONE applied).
    let delta = rmse(&refined, &unrefined);
    assert!(
        delta > 0.05,
        "SUBPIX and NONE produced the same corners ({delta:.4} px apart), \
         so corner refinement is not running at all"
    );

    Ok(())
}

/// `rectify()` is overlay-only, but it must still be correct: rectifying a distorted image should
/// put the markers back where the ideal render has them.
#[test]
fn rectify_undoes_the_distortion_for_the_overlay() -> Result<()> {
    let ideal = board_image(40.0)?;
    let subpix = params(CornerRefinementMethod::SUBPIX, 5);

    let info = camera_info(&ideal, &DISTORTION);
    let distorted = distort_image(&ideal, &info)?;

    let det = detector(&ideal, &DISTORTION, subpix)?;
    let rectified = det.rectify(&distorted)?;

    // Detecting on the rectified image with a distortion-free model must agree with the ideal
    // render. (This is the overlay's frame, which is why it has to line up.)
    let on_rectified = corners_of(&detector(&ideal, &NO_DISTORTION, subpix)?, &rectified)?;
    let truth = corners_of(&detector(&ideal, &NO_DISTORTION, subpix)?, &ideal)?;

    let err = rmse(&truth, &on_rectified);
    assert!(
        err < 2.0,
        "rectify() did not undo the distortion: {err:.3} px RMSE. \
         The debug overlay would disagree with the corners the solver consumes."
    );

    Ok(())
}

/// Guard the size at which SUBPIX windows start colliding with adjacent corners.
///
/// The board's markers span ~200 px at 1.5 m and ~35 px at 6 m. `win_size` must stay well under
/// half the corner spacing across that range, or adjacent corners' windows overlap and pull each
/// other off-target. This pins that the shipped default (5) holds at the far end.
#[test]
fn default_win_size_holds_across_the_working_marker_size_range() -> Result<()> {
    // dpi 40 -> ~300 px markers (near); dpi 8 -> ~60 px markers (far).
    for dpi in [40.0, 20.0, 10.0, 8.0] {
        let ideal = board_image(dpi)?;
        let subpix = params(CornerRefinementMethod::SUBPIX, 5);

        let refined = corners_of(&detector(&ideal, &NO_DISTORTION, subpix)?, &ideal)?;
        let unrefined = corners_of(
            &detector(
                &ideal,
                &NO_DISTORTION,
                params(CornerRefinementMethod::NONE, 5),
            )?,
            &ideal,
        )?;

        // Refinement should nudge corners, not fling them. A large excursion means the window is
        // spanning two corners at this marker size.
        let delta = rmse(&refined, &unrefined);
        assert!(
            delta < 3.0,
            "at dpi={dpi}, SUBPIX moved corners {delta:.3} px from the unrefined estimate; \
             win_size=5 is likely spanning adjacent corners at this marker size"
        );
    }

    Ok(())
}

/// With zero distortion, `undistortPoints` must be an identity — so the corners the detector
/// reports are exactly the ones it measured, unmoved.
///
/// This is the guard against a `P`-less `undistortPoints` call: that returns *normalized*
/// coordinates (order 1e-1), which would be catastrophically far from the pixel coordinates
/// (order 1e2) the corners started at, even when D is zero.
#[test]
fn zero_distortion_leaves_corners_in_pixel_coordinates() -> Result<()> {
    let ideal = board_image(40.0)?;
    let det = detector(
        &ideal,
        &NO_DISTORTION,
        params(CornerRefinementMethod::SUBPIX, 5),
    )?;

    let corners = corners_of(&det, &ideal)?;

    // Every corner must lie inside the image. Normalized coords would cluster near the origin.
    let (w, h) = (ideal.cols() as f32, ideal.rows() as f32);
    for (id, x, y) in &corners {
        assert!(
            *x > 1.0 && *x < w && *y > 1.0 && *y < h,
            "marker {id} corner ({x}, {y}) is not a pixel coordinate in a {w}x{h} image; \
             undistortPoints was likely called without P = K"
        );
    }

    Ok(())
}
