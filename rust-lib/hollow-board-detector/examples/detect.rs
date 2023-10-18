use anyhow::Result;
use aruco_config::MultiArucoPattern;
use clap::Parser;
use hollow_board_detector::{Config, Detector};
use nalgebra as na;
use pcd_rs::DynReader;
use std::path::PathBuf;

/// Detect the hollow-board from a point cloud.
#[derive(Parser)]
struct Args {
    /// The path to the input .pcd point cloud file.
    pub input_file: PathBuf,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Configure and build the detector
    let aruco_pattern: MultiArucoPattern = json5::from_str(include_str!("aruco_pattern.json5"))?;
    let config: Config = json5::from_str(include_str!("board_detector.json5"))?;
    let detector = Detector::new(config, aruco_pattern);

    // Load the point cloud file.
    let reader = DynReader::open(&args.input_file)?;
    let points: Vec<na::Point3<f64>> = reader
        .map(|point| {
            let [x, y, z]: [f32; 3] = point.unwrap().to_xyz().unwrap();
            na::Point3::new(x as f64, y as f64, z as f64)
        })
        .collect();

    // Perform detection
    let detection = detector.detect(&points)?;

    // Show the detection results
    if let Some(detection) = detection {
        println!("{:?}", detection.board_model);
    } else {
        println!("No board detected");
    }

    Ok(())
}
