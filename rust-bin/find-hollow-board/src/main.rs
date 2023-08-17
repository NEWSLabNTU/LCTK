mod bbox;
mod bbox_gui;
mod preview_gui;

use crate::bbox::BBox;
use anyhow::ensure;
use aruco_config::multi_aruco::MultiArucoPattern;
use clap::Parser;
use hollow_board_detector::{
    Config as BoardDetectorConfig, Detection as BoardDetection, Detector as BoardDetector,
};
use nalgebra as na;
use once_cell::sync::Lazy;
use pcd_rs::DynReader;
use serde::Deserialize;
use std::{
    borrow::{Cow, Cow::*},
    fs,
    path::{Path, PathBuf},
};

static DEFAULT_BOARD_DETECTOR_CONFIG: Lazy<BoardDetectorConfig> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/board_detector.json5"
    ));
    json5::from_str(&text).unwrap()
});

static DEFAULT_ARUCO_PATTERN_CONFIG: Lazy<MultiArucoPattern> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/aruco_pattern.json5"
    ));
    json5::from_str(&text).unwrap()
});

#[derive(Parser)]
struct Opts {
    /// Board detector configuration file.
    #[clap(long)]
    pub board_detector_file: Option<PathBuf>,

    /// ArUco pattern configuration file.
    #[clap(long)]
    pub aruco_pattern_file: Option<PathBuf>,

    /// The directory containing input PCD files.
    pub input_dir: PathBuf,

    /// The directory to store output detections.
    pub output_dir: PathBuf,

    /// The input file to obtain bbox parameters.
    #[clap(long)]
    pub load_bbox: Option<PathBuf>,

    /// The output file to store bbox parameters.
    #[clap(long)]
    pub save_bbox: Option<PathBuf>,

    /// Preview detection in GUI interface.
    #[clap(long)]
    pub preview: bool,

    /// Skip BBox tweaking.
    #[clap(long)]
    pub skip_bbox_tweak: bool,
}

fn main() -> Result<(), anyhow::Error> {
    let opts: Opts = Opts::parse();

    let aruco_pattern: Cow<'_, MultiArucoPattern> = match &opts.aruco_pattern_file {
        Some(path) => Owned(load_json5(path)?),
        None => Borrowed(&*DEFAULT_ARUCO_PATTERN_CONFIG),
    };
    let board_detector_config: Cow<'_, BoardDetectorConfig> = match &opts.board_detector_file {
        Some(path) => Owned(load_json5(path)?),
        None => Borrowed(&*DEFAULT_BOARD_DETECTOR_CONFIG),
    };
    let detector = BoardDetector::new(
        board_detector_config.into_owned(),
        aruco_pattern.into_owned(),
    );

    // Enumerate input PCD files
    ensure!(!opts.output_dir.exists(), "output directory already exists");
    fs::create_dir_all(&opts.output_dir)?;

    let entries = glob::glob(&format!("{}/*.pcd", opts.input_dir.display()))?;
    let paths: Vec<_> = entries
        .filter_map(|result| -> Option<PathBuf> {
            match result {
                Ok(path) => Some(path),
                Err(err) => {
                    eprintln!("{}", err);
                    None
                }
            }
        })
        .collect();

    if paths.is_empty() {
        eprintln!("No input PCD files.");
        return Ok(());
    }

    // Open a window to choose a bbox
    let bbox: BBox = if let Some(bbox_file) = &opts.load_bbox {
        let json5_text = fs::read_to_string(bbox_file)?;
        json5::from_str(&json5_text)?
    } else {
        BBox::default()
    };

    let bbox = if !opts.skip_bbox_tweak {
        let reader = DynReader::open(&paths[0])?;
        let points: Vec<na::Point3<f32>> = reader
            .map(|point| {
                let [x, y, z]: [f32; 3] = point.unwrap().to_xyz().unwrap();
                na::Point3::new(x, y, z)
            })
            .collect();

        let handle = bbox_gui::start(points, bbox);
        let bbox = handle.wait();

        match bbox {
            Some(bbox) => bbox,
            None => {
                eprintln!("Give up finding a bbox");
                return Ok(());
            }
        }
    } else {
        bbox
    };

    if let Some(bbox_file) = &opts.save_bbox {
        eprintln!("Saving bbox parameters to '{}'", bbox_file.display());
        let json5_text = serde_json::to_string_pretty(&bbox)?;
        fs::write(bbox_file, &json5_text)?;
    }

    // Process pcd files
    let mut preview_gui = opts.preview.then(|| preview_gui::start(bbox.clone()));

    for path in paths {
        let reader = DynReader::open(&path)?;
        let orig_points: Vec<na::Point3<f64>> = reader
            .map(|point| {
                let [x, y, z]: [f32; 3] = point.unwrap().to_xyz().unwrap();
                na::Point3::new(x as f64, y as f64, z as f64)
            })
            .collect();
        let active_points: Vec<_> = orig_points
            .iter()
            .filter(|pt| bbox.contains_point(pt))
            .cloned()
            .collect();

        let detection: Option<BoardDetection> = detector.detect(&active_points)?;

        if let Some(detection) = &detection {
            let file_stem = path.file_stem().unwrap();
            let file_name = format!("{}.json5", file_stem.to_str().unwrap());
            let output_path = opts.output_dir.join(&file_name);
            let json5_text = serde_json::to_string_pretty(&detection.board_model)?;
            fs::write(output_path, json5_text)?;
        }

        if let Some(gui_) = preview_gui.take() {
            let points: Vec<na::Point3<f32>> =
                orig_points.iter().map(|pt| na::convert_ref(pt)).collect();
            let detection = detection;
            let gui_ = gui_.update(points, detection);

            match gui_ {
                Some(gui_) => {
                    preview_gui = Some(gui_);
                }
                None => break,
            }
        }
    }

    Ok(())
}

fn load_json5<T>(path: impl AsRef<Path>) -> Result<T, anyhow::Error>
where
    T: for<'de> Deserialize<'de>,
{
    let text = fs::read_to_string(path)?;
    let value: T = json5::from_str(&text)?;
    Ok(value)
}
