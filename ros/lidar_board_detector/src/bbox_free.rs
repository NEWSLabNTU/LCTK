//! Crop-box-free detection config + Method-E warmup state machine.
//!
//! Pure logic (no rclrs) so it is unit-testable. `main.rs` parses the node's
//! board config into these types and threads them through the processing
//! thread; the ROS wiring stays thin.

use anyhow::Result;
use board_projection_detector::background::BackgroundModel;
use board_projection_detector::config::{BoardConfig, ForegroundMethod};
use board_projection_detector::detector::RejectReason;
use nalgebra::Point3;
use serde::Deserialize;

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

/// Unit / comparison hint for a reject reason's `measured` vs `threshold`
/// numbers (see `RejectDetail`). Used to annotate the reject log so the operator
/// knows how narrowly a gate failed and in what units the config knob is set.
pub fn reject_unit(reason: &RejectReason) -> &'static str {
    match reason {
        // measured > threshold fails
        RejectReason::SquareResidual => "coverage residual, unitless; fail when measured >= max",
        RejectReason::Isolation => "points per metre of quad perimeter; fail when measured > max",
        RejectReason::Flatness => "RMS metres; fail when measured > max",
        // measured <= threshold fails
        RejectReason::Stance => {
            "normalized diagonal·up 0-1 (~0.71 flat, ~1.0 corner-standing); fail when measured <= floor"
        }
        RejectReason::Extent | RejectReason::SizeGate => "metres; board-size gate",
        RejectReason::NoClusters => "candidate count",
    }
}

/// Deserializes directly from the config's `detection_mode` string, so an
/// invalid value fails at parse time with serde naming the valid variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum DetectionMode {
    #[default]
    Bbox,
    BboxFree,
}

impl DetectionMode {
    /// The config spelling, for logging.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Bbox => "bbox",
            Self::BboxFree => "bbox_free",
        }
    }
}

/// The whole board_detector.json5 deserialized flat. The crop-box-free
/// detector's parameters are top-level here (no nested `bbox_free` object) and
/// the board sub-config is `#[serde(flatten)]`ed in, so its keys sit at the top
/// level too. Legacy (bbox-mode) keys in the same file are read by a separate
/// deserializer and ignored here. Every field defaults, so a bbox-mode file
/// with none of these keys still parses.
#[derive(Debug, Clone, Deserialize)]
pub struct DetectionConfig {
    #[serde(default)]
    pub detection_mode: DetectionMode,
    #[serde(default = "default_foreground_method")]
    pub foreground_method: ForegroundMethod,
    #[serde(default = "default_bbf_voxel")]
    pub bbf_voxel: f64,
    #[serde(default = "default_dilation_radius")]
    pub bg_dilation_radius: i64,
    #[serde(default = "default_warmup_frames")]
    pub bg_warmup_frames: usize,
    #[serde(flatten)]
    pub board: BoardConfig,
}

fn default_foreground_method() -> ForegroundMethod {
    ForegroundMethod::BackgroundSubtraction
}
fn default_bbf_voxel() -> f64 {
    0.05
}
fn default_dilation_radius() -> i64 {
    1
}
fn default_warmup_frames() -> usize {
    20
}

impl DetectionConfig {
    /// Collect the crop-box-free detector parameters into one bundle.
    pub fn into_bbox_free(self) -> BboxFreeRaw {
        BboxFreeRaw {
            method: self.foreground_method,
            voxel: self.bbf_voxel,
            board: self.board,
            background: BackgroundParams {
                dilation_radius: self.bg_dilation_radius,
                warmup_frames: self.bg_warmup_frames,
            },
        }
    }
}

/// Crop-box-free detector parameters, assembled from the flat `DetectionConfig`.
#[derive(Debug, Clone)]
pub struct BboxFreeRaw {
    /// Validated at parse time by serde — no post-parse re-validation needed.
    pub method: ForegroundMethod,
    pub voxel: f64,
    pub board: BoardConfig,
    pub background: BackgroundParams,
}

#[derive(Debug, Clone)]
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
                // `BackgroundModel::observe` early-returns on an empty cloud, contributing
                // nothing to the model — don't let an empty/invalid frame advance the warmup
                // counter either, or warmup can "complete" on frames that taught it nothing.
                if points.is_empty() {
                    return WarmupOutcome::Warming { seen: *seen, needed: self.warmup_frames };
                }
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
    fn empty_frame_during_warming_does_not_advance_seen() {
        let params = BackgroundParams { dilation_radius: 1, warmup_frames: 3 };
        let mut state = BackgroundState::new(0.05, &params);

        // One real frame: seen advances to 1.
        match state.observe_frame(&cloud(50, 999.0)) {
            WarmupOutcome::Warming { seen, needed } => {
                assert_eq!(seen, 1);
                assert_eq!(needed, 3);
            }
            WarmupOutcome::Ready => panic!("ready too early"),
        }

        // Empty frame: must NOT advance seen, and must stay Warming.
        match state.observe_frame(&[]) {
            WarmupOutcome::Warming { seen, needed } => {
                assert_eq!(seen, 1, "empty cloud must not advance warmup progress");
                assert_eq!(needed, 3);
            }
            WarmupOutcome::Ready => panic!("empty frame must not finalize warmup"),
        }
        assert!(state.model().is_none());

        // Another empty frame for good measure — still stuck at seen=1.
        match state.observe_frame(&[]) {
            WarmupOutcome::Warming { seen, .. } => assert_eq!(seen, 1),
            WarmupOutcome::Ready => panic!("empty frame must not finalize warmup"),
        }
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
        assert_eq!(cfg.detection_mode, DetectionMode::Bbox);
    }

    #[test]
    fn shipped_bbox_free_is_production_operating_point() {
        let cfg = parse_detection_config(SHIPPED).unwrap();
        let bf = cfg.into_bbox_free();
        assert_eq!(bf.method, ForegroundMethod::BackgroundSubtraction);
        assert_eq!(bf.voxel, 0.05);
        // Production operating point — NOT the BoardConfig serde defaults.
        assert_eq!(bf.board.flatness_rms_max, 0.045);
        assert_eq!(bf.board.stance_floor, 0.9);
        assert!(bf.board.isolation);
        assert_eq!(bf.board.cluster_min_points, 30);
        assert_eq!(bf.background.warmup_frames, 20);
        assert_eq!(bf.background.dilation_radius, 1);
    }

    // Both enums are validated by serde at parse time, so a typo fails when the
    // config is read rather than surfacing later as silent non-detection.
    #[test]
    fn detection_mode_rejects_unknown() {
        assert!(parse_detection_config(r#"{ "detection_mode": "nope" }"#).is_err());
        let cfg = parse_detection_config(r#"{ "detection_mode": "bbox_free" }"#).unwrap();
        assert_eq!(cfg.detection_mode, DetectionMode::BboxFree);
    }

    #[test]
    fn foreground_method_rejects_unknown() {
        assert!(parse_detection_config(r#"{ "foreground_method": "bogus" }"#).is_err());
        let cfg = parse_detection_config(r#"{ "foreground_method": "plane_strip" }"#).unwrap();
        assert_eq!(cfg.into_bbox_free().method, ForegroundMethod::PlaneStrip);
    }

    #[test]
    fn omitted_keys_take_documented_defaults() {
        let cfg = parse_detection_config("{}").unwrap();
        assert_eq!(cfg.detection_mode, DetectionMode::Bbox);
        let bf = cfg.into_bbox_free();
        assert_eq!(bf.method, ForegroundMethod::BackgroundSubtraction);
        assert_eq!(bf.voxel, 0.05);
        assert_eq!(bf.background.warmup_frames, 20);
    }
}
