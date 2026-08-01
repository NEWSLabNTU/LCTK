//! Crop-box-free detection config + Method-E warmup state machine.
//!
//! Pure logic (no rclrs) so it is unit-testable. `main.rs` parses the node's
//! board config into these types and threads them through the processing
//! thread; the ROS wiring stays thin.

use anyhow::{bail, Result};
use board_projection_detector::background::BackgroundModel;
use board_projection_detector::config::{BoardConfig, ForegroundMethod};
use board_projection_detector::detector::RejectReason;
use nalgebra::Point3;
use serde::Deserialize;
use std::str::FromStr;

/// Human-readable one-liner for a detector reject reason.
pub fn describe_reject(reason: &RejectReason) -> &'static str {
    match reason {
        RejectReason::NoClusters => "no candidate clusters survived foreground extraction",
        RejectReason::Flatness => "best candidate exceeded flatness_rms_max (not planar enough)",
        RejectReason::Extent => "best candidate failed the board-size extent gate",
        RejectReason::SizeGate => "best candidate failed the coarse square size gate",
        RejectReason::SquareResidual => "square fit residual exceeded square_icp_residual_max",
        RejectReason::Stance => "best candidate failed the 3D diamond-stance gate",
        RejectReason::Isolation => {
            "best candidate failed the isolation-density gate (embedded clutter)"
        }
    }
}

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

/// Fixed single live session: one background source.
const MIN_SOURCES: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarmupOutcome {
    Warming { seen: usize, needed: usize },
    Ready,
}

enum Phase {
    Warming { model: BackgroundModel, seen: usize },
    Ready { model: BackgroundModel },
}

pub struct BackgroundState {
    phase: Phase,
    voxel: f64,
    dilation_radius: i64,
    warmup_frames: usize,
}

impl BackgroundState {
    pub fn new(voxel: f64, params: &BackgroundParams) -> Self {
        Self {
            phase: Phase::Warming {
                model: BackgroundModel::new(voxel, params.dilation_radius, MIN_SOURCES),
                seen: 0,
            },
            voxel,
            dilation_radius: params.dilation_radius,
            warmup_frames: params.warmup_frames,
        }
    }

    pub fn observe_frame(&mut self, points: &[Point3<f64>]) -> WarmupOutcome {
        match &mut self.phase {
            Phase::Ready { .. } => WarmupOutcome::Ready,
            Phase::Warming { model, seen } => {
                model.observe(points, "live");
                *seen += 1;
                if *seen >= self.warmup_frames {
                    // Move the model out of Warming into Ready, finalized.
                    let Phase::Warming { mut model, .. } =
                        std::mem::replace(&mut self.phase, Phase::Ready {
                            model: BackgroundModel::new(self.voxel, self.dilation_radius, MIN_SOURCES),
                        })
                    else {
                        unreachable!("just matched Warming");
                    };
                    model.finalize();
                    self.phase = Phase::Ready { model };
                    WarmupOutcome::Ready
                } else {
                    WarmupOutcome::Warming { seen: *seen, needed: self.warmup_frames }
                }
            }
        }
    }

    pub fn model(&self) -> Option<&BackgroundModel> {
        match &self.phase {
            Phase::Ready { model } => Some(model),
            Phase::Warming { .. } => None,
        }
    }

    pub fn reset(&mut self) {
        self.phase = Phase::Warming {
            model: BackgroundModel::new(self.voxel, self.dilation_radius, MIN_SOURCES),
            seen: 0,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use board_projection_detector::config::ForegroundMethod;
    use nalgebra::Point3;

    fn cloud(n: usize, offset: f64) -> Vec<Point3<f64>> {
        (0..n).map(|i| Point3::new(offset + i as f64 * 0.001, 0.0, 0.0)).collect()
    }

    #[test]
    fn warmup_observes_then_becomes_ready() {
        let params = BackgroundParams { dilation_radius: 1, warmup_frames: 3 };
        let mut state = BackgroundState::new(0.05, &params);
        assert!(state.model().is_none());

        // Frames 1 and 2: still warming, no model yet.
        for i in 1..=2 {
            match state.observe_frame(&cloud(50, 999.0)) {
                WarmupOutcome::Warming { seen, needed } => {
                    assert_eq!(seen, i);
                    assert_eq!(needed, 3);
                }
                WarmupOutcome::Ready => panic!("ready too early"),
            }
            assert!(state.model().is_none());
        }

        // Frame 3: reaches the count, finalizes, becomes Ready.
        assert!(matches!(state.observe_frame(&cloud(50, 999.0)), WarmupOutcome::Ready));
        assert!(state.model().is_some());

        // Subsequent frames stay Ready and do NOT observe.
        assert!(matches!(state.observe_frame(&cloud(50, 0.0)), WarmupOutcome::Ready));
    }

    #[test]
    fn reset_reenters_warming() {
        let params = BackgroundParams { dilation_radius: 1, warmup_frames: 1 };
        let mut state = BackgroundState::new(0.05, &params);
        assert!(matches!(state.observe_frame(&cloud(50, 999.0)), WarmupOutcome::Ready));
        assert!(state.model().is_some());

        state.reset();
        assert!(state.model().is_none());
        assert!(matches!(state.observe_frame(&cloud(50, 999.0)), WarmupOutcome::Ready));
    }

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
