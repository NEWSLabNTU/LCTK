use anyhow::Result;
use aruco_config::{ArucoDictionary, MultiArucoPattern};
use aruco_detector::multi_aruco::Builder;
use clap::Parser;
use measurements::Length;
use noisy_float::prelude::*;
use opencv::imgcodecs::{imread, IMREAD_GRAYSCALE};
use serde_types::CameraIntrinsics;

/// Detect the multi-aruco pattern board in an image.
#[derive(Parser)]
struct Args {
    /// The path to the input image file.
    pub input_file: String,
}

fn main() -> Result<()> {
    // Parse command-line arguments
    let args = Args::parse();

    // Configure and build the detector
    let pattern = MultiArucoPattern {
        marker_ids: vec![149, 391, 385, 482],
        dictionary: ArucoDictionary::DICT_5X5_1000,
        board_size: Length::from_millimeters(500.0),
        board_border_size: Length::from_millimeters(10.0),
        marker_square_size_ratio: r64(0.8),
        num_squares_per_side: 2,
        border_bits: 1,
    };
    let detector = Builder {
        pattern,
        camera_intrinsic: CameraIntrinsics::default(),
    }
    .build()?;

    // Perform pattern detection.
    let image = imread(&args.input_file, IMREAD_GRAYSCALE)?;
    let detection = detector.detect_markers(&image)?;

    // Print the detection results.
    if let Some(detection) = detection {
        for marker in detection.markers() {
            print!("id={}, corners=", marker.id);

            let corners: Vec<_> = marker.corners[1..]
                .into_iter()
                .map(|corner| format!("({}, {})", corner.x, corner.y))
                .collect();
            let corners = corners.join(", ");
            println!("[{corners}]");
        }
    } else {
        println!("No ArUco pattern found!");
    }

    Ok(())
}
