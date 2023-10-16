use anyhow::{ensure, Context};
use aruco_config::MultiArucoPattern;
use aruco_detector::multi_aruco::ImageMarker;
use clap::Parser;
use cv_convert::prelude::*;
use hollow_board_config::BoardModel;
use itertools::{izip, Itertools};
use once_cell::sync::Lazy;
use opencv::core::{Point2d, Point2f, Point3d};
use pnp_solver::{PnpMethod, PnpSolver};
use serde_types::{CameraIntrinsics, DistortionCoefs, Isometry3D, MrptCalibration};
use std::{
    borrow::{Cow, Cow::*},
    fs,
    path::PathBuf,
};

static DEFAULT_ARUCO_PATTERN: Lazy<MultiArucoPattern> = Lazy::new(|| {
    let text = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/config/aruco_pattern.json5"
    ));
    json5::from_str(text).unwrap()
});

#[derive(Debug, Parser)]
struct Opts {
    /// The file storing intrinsic camera parameters.
    #[clap(long)]
    pub intrinsics_file: PathBuf,

    /// The output file where extrinsic parameters are written to.
    #[clap(long)]
    pub output_file: PathBuf,

    #[clap(long, default_value = "SQPNP")]
    pub method: PnpMethod,

    /// Comma-sperated list of 3D board detection files.
    #[clap(long, value_delimiter(','))]
    pub boards: Vec<PathBuf>,

    /// Comma-sperated list of 2D ArUco marker detection files.
    #[clap(long, value_delimiter(','))]
    pub arucos: Vec<PathBuf>,

    /// The file that describes the ArUco marker pattern.
    #[clap(long)]
    pub aruco_pattern_file: Option<PathBuf>,

    /// Print the default ArUco marker pattern configuration file.
    #[clap(long)]
    pub print_default_aruco_pattern: bool,
}

fn main() -> Result<(), anyhow::Error> {
    let Opts {
        aruco_pattern_file,
        intrinsics_file,
        output_file,
        boards,
        arucos,
        method,
        print_default_aruco_pattern,
    } = Opts::parse();

    if print_default_aruco_pattern {
        let text = serde_json::to_string_pretty(&*DEFAULT_ARUCO_PATTERN).unwrap();
        println!("{}", text);
        return Ok(());
    }

    ensure!(
        boards.len() == arucos.len(),
        "the number of board files ({}) does not match the number of aruco files ({})",
        boards.len(),
        arucos.len()
    );
    let aruco_pattern: Cow<'_, MultiArucoPattern> = match &aruco_pattern_file {
        Some(path) => {
            let text = fs::read_to_string(path)
                .with_context(|| format!("unable to open file '{}'", path.display()))?;
            let pattern = json5::from_str(&text)?;
            Owned(pattern)
        }
        None => Borrowed(&*DEFAULT_ARUCO_PATTERN),
    };
    let mrpt_calib: MrptCalibration = {
        let yaml_text = fs::read_to_string(&intrinsics_file)
            .with_context(|| format!("unable to open file '{}'", intrinsics_file.display()))?;
        serde_yaml::from_str(&yaml_text)?
    };
    let camera_intrinsics = CameraIntrinsics {
        distortion_coefs: DistortionCoefs::zeros(),
        ..mrpt_calib.intrinsic_params()?
    };
    // let camera_intrinsics = mrpt_calib.intrinsic_params()?;
    let pnp_solver = PnpSolver::new(&camera_intrinsics, method);

    let detection_pairs: Vec<(BoardModel, Vec<ImageMarker>)> = izip!(boards, arucos)
        .map(|(board_file, aruco_file)| {
            let board: BoardModel = {
                let json5_text = fs::read_to_string(&board_file)
                    .with_context(|| format!("unable to open file '{}'", board_file.display()))?;
                json5::from_str(&json5_text)?
            };
            let markers: Vec<ImageMarker> = {
                let json5_text = fs::read_to_string(&aruco_file)
                    .with_context(|| format!("unable to open file '{}'", aruco_file.display()))?;
                json5::from_str(&json5_text)?
            };
            anyhow::Ok((board, markers))
        })
        .try_collect()?;

    let point_pairs: Vec<(Point3d, Point2d)> = detection_pairs
        .into_iter()
        .map(|(board, markers)| {
            let object_points: Vec<Point3d> = board
                .multi_marker_corners(&aruco_pattern)
                .into_iter()
                .flatten()
                .map(Point3d::from_cv)
                .collect();
            let image_points: Vec<Point2d> = markers
                .into_iter()
                .flat_map(|marker| {
                    let corners: Vec<_> = marker
                        .corners
                        .iter()
                        .map(|corner| {
                            let point2f: Point2f = corner.into_cv();
                            let point2d: Point2d = point2f.to().unwrap();
                            point2d
                        })
                        .collect();
                    corners
                })
                .collect();

            ensure!(
                object_points.len() == image_points.len(),
                "the number of object points does not match the number of image points"
            );
            let point_pairs = izip!(object_points, image_points);
            anyhow::Ok(point_pairs)
        })
        .flatten_ok()
        .try_collect()?;

    let transform = pnp_solver.solve(point_pairs);

    if let Some(transform) = transform {
        let transform = Isometry3D::from(transform);
        let json5_text = serde_json::to_string_pretty(&transform)?;
        fs::write(&output_file, &json5_text)
            .with_context(|| format!("unable to write to file '{}'", output_file.display()))?;
    }

    Ok(())
}
