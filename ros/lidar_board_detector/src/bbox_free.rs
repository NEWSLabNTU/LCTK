//! Crop-box-free detection config + Method-E warmup state machine.
//!
//! Pure logic (no rclrs) so it is unit-testable. `main.rs` parses the node's
//! board config into these types and threads them through the processing
//! thread; the ROS wiring stays thin.

use anyhow::{bail, Result};
use board_projection_detector::config::{BoardConfig, ForegroundMethod};
use serde::Deserialize;
use std::str::FromStr;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionMode {
    Bbox,
    BboxFree,
}

impl DetectionMode {
    pub fn parse(s: &str) -> Result<Self> {
        match s {
            "bbox" => Ok(Self::Bbox),
            "bbox_free" => Ok(Self::BboxFree),
            other => bail!("unknown detection_mode: {other} (expected \"bbox\" or \"bbox_free\")"),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct DetectionConfig {
    #[serde(default = "default_mode")]
    pub detection_mode: String,
    #[serde(default)]
    pub bbox_free: Option<BboxFreeRaw>,
}

fn default_mode() -> String {
    "bbox".to_string()
}

#[derive(Debug, Clone, Deserialize)]
pub struct BboxFreeRaw {
    pub foreground_method: String,
    pub voxel: f64,
    pub board: BoardConfig,
    pub background: BackgroundParams,
}

impl BboxFreeRaw {
    pub fn method(&self) -> Result<ForegroundMethod> {
        ForegroundMethod::from_str(&self.foreground_method)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct BackgroundParams {
    pub dilation_radius: i64,
    pub warmup_frames: usize,
}

pub fn parse_detection_config(json5_text: &str) -> Result<DetectionConfig> {
    Ok(json5::from_str(json5_text)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use board_projection_detector::config::ForegroundMethod;

    const SHIPPED: &str = include_str!(
        "../../lctk_launch/config/board/board_detector.json5"
    );

    #[test]
    fn shipped_config_defaults_to_bbox() {
        let cfg = parse_detection_config(SHIPPED).unwrap();
        assert_eq!(cfg.detection_mode, "bbox");
        assert_eq!(DetectionMode::parse(&cfg.detection_mode).unwrap(), DetectionMode::Bbox);
    }

    #[test]
    fn shipped_bbox_free_is_production_operating_point() {
        let cfg = parse_detection_config(SHIPPED).unwrap();
        let bf = cfg.bbox_free.expect("bbox_free block present");
        assert_eq!(bf.method().unwrap(), ForegroundMethod::BackgroundSubtraction);
        assert_eq!(bf.voxel, 0.05);
        // Production operating point — NOT the BoardConfig serde defaults.
        assert_eq!(bf.board.flatness_rms_max, 0.045);
        assert_eq!(bf.board.stance_floor, 0.9);
        assert!(bf.board.isolation);
        assert_eq!(bf.board.cluster_min_points, 30);
        assert_eq!(bf.background.warmup_frames, 20);
        assert_eq!(bf.background.dilation_radius, 1);
    }

    #[test]
    fn detection_mode_parse_rejects_unknown() {
        assert!(DetectionMode::parse("nope").is_err());
        assert_eq!(DetectionMode::parse("bbox_free").unwrap(), DetectionMode::BboxFree);
    }

    #[test]
    fn method_rejects_unknown() {
        let raw = BboxFreeRaw {
            foreground_method: "bogus".into(),
            voxel: 0.05,
            board: board_projection_detector::config::production_config(1.0, [0.0, 0.0, 1.0], 30),
            background: BackgroundParams { dilation_radius: 1, warmup_frames: 20 },
        };
        assert!(raw.method().is_err());
    }
}
