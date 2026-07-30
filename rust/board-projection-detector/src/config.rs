use serde::Deserialize;
use std::{path::Path, str::FromStr};

#[derive(Debug, Clone, Deserialize)]
pub struct BoardConfig {
    #[serde(default = "d_side_m")]
    pub side_m: f64,
    #[serde(default = "d_side_tol")]
    pub side_tol: f64,
    #[serde(default = "d_cell_m")]
    pub cell_m: f64,
    #[serde(default = "d_vertical_gap_deg")]
    pub vertical_gap_deg: f64,
    #[serde(default = "d_cluster_min_points")]
    pub cluster_min_points: usize,
    #[serde(default = "d_up_axis")]
    pub up_axis: [f64; 3],
    #[serde(default = "d_flatness")]
    pub flatness_rms_max: f64,
    #[serde(default = "d_stance_floor")]
    pub stance_floor: f64,
    #[serde(default = "d_square_res")]
    pub square_icp_residual_max: f64,
    #[serde(default)]
    pub isolation: bool,
    #[serde(default = "d_iso_density")]
    pub isolation_max_density: f64,
}

fn d_side_m() -> f64 {
    1.0
}
fn d_side_tol() -> f64 {
    0.20
}
fn d_cell_m() -> f64 {
    0.02
}
fn d_vertical_gap_deg() -> f64 {
    3.0
}
fn d_cluster_min_points() -> usize {
    30
}
fn d_up_axis() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}
fn d_flatness() -> f64 {
    0.035
} // BoardConfig default; production overrides to 0.045
fn d_stance_floor() -> f64 {
    0.0
}
fn d_square_res() -> f64 {
    0.45
}
fn d_iso_density() -> f64 {
    0.3
}

pub fn production_config(side_m: f64, up_axis: [f64; 3], cluster_min_points: usize) -> BoardConfig {
    BoardConfig {
        side_m,
        up_axis,
        cluster_min_points,
        side_tol: 0.20,
        cell_m: 0.02,
        vertical_gap_deg: 3.0,
        flatness_rms_max: 0.045, // presets.py
        stance_floor: 0.9,       // presets.py
        square_icp_residual_max: 0.45,
        isolation: true, // presets.py
        isolation_max_density: 0.3,
    }
}

pub fn load_board_config_json5(path: &Path) -> anyhow::Result<BoardConfig> {
    let text = std::fs::read_to_string(path)?;
    Ok(json5::from_str(&text)?)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForegroundMethod {
    PlaneStrip,
    BackgroundSubtraction,
}

impl FromStr for ForegroundMethod {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "plane_strip" => Ok(Self::PlaneStrip),
            "background_subtraction" => Ok(Self::BackgroundSubtraction),
            other => anyhow::bail!("unknown foreground method: {other}"),
        }
    }
}
