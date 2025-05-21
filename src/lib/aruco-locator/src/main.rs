use anyhow::Result;
use aruco_locator::{ArucoDetector, ArucoDetectorConfig};
use clap::Parser;
use opencv::{imgcodecs, prelude::*};
use std::{fs, path::PathBuf};

const ARUCO_PATTERN_CONFIG: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/config/aruco_pattern.json5");

#[derive(Parser)]
#[command(name = "aruco_locator")]
#[command(about = "Detect ArUco markers in a single image")]
struct Opts {
    /// The file storing intrinsic camera parameters.
    pub intrinsics_file: PathBuf,
    /// The input image file path.
    pub input_image: PathBuf,
    /// The output file to store detection results (JSON format).
    #[arg(short, long)]
    pub output_file: Option<PathBuf>,
    /// Show GUI with detected markers.
    #[arg(long)]
    pub gui: bool,
}

fn main() -> Result<()> {
    let opts: Opts = Opts::parse();

    // Load detector configuration
    let config = ArucoDetectorConfig::from_files(
        &opts.intrinsics_file,
        &PathBuf::from(ARUCO_PATTERN_CONFIG),
    )?;

    // Create detector
    let detector = ArucoDetector::new(config)?;

    // Load input image
    let image = imgcodecs::imread(opts.input_image.to_str().unwrap(), imgcodecs::IMREAD_COLOR)?;

    if image.empty() {
        anyhow::bail!("Failed to load image from {:?}", opts.input_image);
    }

    // Detect ArUco markers
    let detection_result = detector.detect_markers(&image)?;

    // Display results
    if detection_result.markers_found {
        println!(
            "Found ArUco markers with IDs: {:?}",
            detection_result.marker_ids
        );

        // Save detection results if output file specified
        if let Some(output_file) = &opts.output_file {
            let json_text = serde_json::to_string_pretty(&detection_result)?;
            fs::write(output_file, &json_text)?;
            println!("Detection results saved to: {:?}", output_file);
        }
    } else {
        println!("No ArUco markers detected in the image.");
    }

    // Show GUI if requested
    if opts.gui {
        let visualization = detector.create_visualization(&image, &detection_result)?;
        detector.show_visualization(&visualization, "ArUco Detection")?;
    }

    Ok(())
}
