use serde::Deserialize;
use std::{ops::Deref, str::FromStr};

#[derive(Debug, Clone, Deserialize)]
pub struct DetectorTuning {
    #[serde(default = "d_side_tol")]
    pub side_tol: f64,
    #[serde(default = "d_cell_m")]
    pub cell_m: f64,
    #[serde(default = "d_vertical_gap_deg")]
    pub vertical_gap_deg: f64,
    #[serde(default = "d_cluster_min_points")]
    pub cluster_min_points: usize,
    /// DBSCAN neighbour radius (metres) for foreground clustering. Larger values
    /// connect sparser points — raise it for a board seen at long range, where
    /// point spacing exceeds the default and the board otherwise fragments.
    #[serde(default = "d_cluster_eps")]
    pub cluster_eps: f64,
    #[serde(default = "d_up_axis")]
    pub up_axis: [f64; 3],
    #[serde(default = "d_flatness")]
    pub flatness_rms_max: f64,
    #[serde(default = "d_stance_floor")]
    pub stance_floor: f64,
    #[serde(default = "d_square_res")]
    pub square_icp_residual_max: f64,
    /// Optional acceptance gate on the square fit's GEOMETRIC residual alone, replacing
    /// `square_icp_residual_max` when set.
    ///
    /// The default residual sums a geometric term (points falling outside the modelled
    /// square) and a perimeter-coverage term. Coverage is load-bearing for the theta
    /// search but is unreachable as a gate when the sensor cannot sample the perimeter:
    /// a 600 mm plate at 7-8 m is crossed by about four VLP-32C rings, so roughly half
    /// its perimeter bins can never hold a point and the best achievable residual sits
    /// above any sane threshold (H-17).
    ///
    /// Set this for a small target at range, where the geometric term still separates a
    /// real plate (~0.01) from a bad fit, and let flatness/extent/isolation do the
    /// discriminating that coverage cannot. Leave unset to keep the historical
    /// behaviour exactly -- which is what every existing preset and parity fixture
    /// depends on.
    #[serde(default)]
    pub square_geometric_residual_max: Option<f64>,
    #[serde(default)]
    pub isolation: bool,
    #[serde(default = "d_iso_density")]
    pub isolation_max_density: f64,

    // ---- formerly-hardcoded tuning knobs (defaults preserve prior behaviour) ----
    #[serde(default = "d_strip_plane_dist")]
    pub strip_plane_dist: f64,
    #[serde(default = "d_strip_plane_min_frac")]
    pub strip_plane_min_frac: f64,
    #[serde(default = "d_merge_seed_min_points")]
    pub merge_seed_min_points: usize,
    #[serde(default = "d_merge_offset_tol")]
    pub merge_offset_tol: f64,
    #[serde(default = "d_merge_dist_factor")]
    pub merge_dist_factor: f64,
    #[serde(default = "d_patch_min_points")]
    pub patch_min_points: usize,
    #[serde(default = "d_patch_extent_lo_frac")]
    pub patch_extent_lo_frac: f64,
    #[serde(default = "d_patch_extent_hi_diag_frac")]
    pub patch_extent_hi_diag_frac: f64,
    #[serde(default = "d_iso_coplanar_tol")]
    pub isolation_coplanar_tol: f64,
    #[serde(default = "d_iso_band_lo")]
    pub isolation_band_lo: f64,
    #[serde(default = "d_iso_band_hi")]
    pub isolation_band_hi: f64,
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
fn d_cluster_eps() -> f64 {
    0.15 // matches the former hardcoded candidates::CLUSTER_EPS
}
fn d_up_axis() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}
fn d_flatness() -> f64 {
    0.035
} // serde default; production overrides to 0.045
fn d_stance_floor() -> f64 {
    0.0
}
fn d_square_res() -> f64 {
    0.45
}
fn d_iso_density() -> f64 {
    0.3
}
fn d_strip_plane_dist() -> f64 {
    0.05
}
fn d_strip_plane_min_frac() -> f64 {
    0.08
}
fn d_merge_seed_min_points() -> usize {
    40
}
fn d_merge_offset_tol() -> f64 {
    0.02
}
fn d_merge_dist_factor() -> f64 {
    1.6
}
fn d_patch_min_points() -> usize {
    60
}
fn d_patch_extent_lo_frac() -> f64 {
    0.5
}
fn d_patch_extent_hi_diag_frac() -> f64 {
    1.8
}
fn d_iso_coplanar_tol() -> f64 {
    0.03
}
fn d_iso_band_lo() -> f64 {
    0.05
}
fn d_iso_band_hi() -> f64 {
    0.30
}

/// The exterior-band geometry the isolation gate measures over: how far off the
/// board plane a point may sit and still count as coplanar, and the inner/outer
/// radii of the ring outside the quad that the density is counted in.
///
/// These three always travel together; `DetectorTuning` stores them as flat keys
/// (the config file is flat by design) and hands them over as one value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IsolationBand {
    pub coplanar_tol: f64,
    pub lo: f64,
    pub hi: f64,
}

impl DetectorTuning {
    /// The isolation gate's exterior-band geometry.
    pub fn isolation_band(&self) -> IsolationBand {
        IsolationBand {
            coplanar_tol: self.isolation_coplanar_tol,
            lo: self.isolation_band_lo,
            hi: self.isolation_band_hi,
        }
    }
}

/// Validated physical side of the selected calibration target, in metres.
///
/// This deliberately carries only the fact board clustering needs. The target
/// definition remains owned by `calibration-target`; this crate must not infer
/// target-frame axes, cutouts, or fiducial layout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetSide(f64);

impl TargetSide {
    pub fn metres(side_m: f64) -> anyhow::Result<Self> {
        if !side_m.is_finite() || side_m <= 0.0 {
            anyhow::bail!("target side must be finite and positive, got {side_m}");
        }
        Ok(Self(side_m))
    }

    pub fn as_metres(self) -> f64 {
        self.0
    }
}

impl TryFrom<f64> for TargetSide {
    type Error = anyhow::Error;

    fn try_from(side_m: f64) -> Result<Self, Self::Error> {
        Self::metres(side_m)
    }
}

/// Neutral view shared by candidate generation and square fitting. It couples
/// the selected target's one required geometric fact with detector tuning.
#[derive(Debug, Clone, Copy)]
pub struct TargetDetectionParams<'a> {
    target_side: TargetSide,
    tuning: &'a DetectorTuning,
}

impl<'a> TargetDetectionParams<'a> {
    pub fn new(target_side: TargetSide, tuning: &'a DetectorTuning) -> Self {
        Self {
            target_side,
            tuning,
        }
    }

    pub fn target_side(self) -> TargetSide {
        self.target_side
    }

    pub fn tuning(self) -> &'a DetectorTuning {
        self.tuning
    }
}

impl Deref for TargetDetectionParams<'_> {
    type Target = DetectorTuning;

    fn deref(&self) -> &Self::Target {
        self.tuning
    }
}

pub fn production_tuning(up_axis: [f64; 3], cluster_min_points: usize) -> DetectorTuning {
    DetectorTuning {
        cluster_eps: d_cluster_eps(),
        side_tol: d_side_tol(),
        cell_m: d_cell_m(),
        vertical_gap_deg: d_vertical_gap_deg(),
        flatness_rms_max: 0.045, // production override of serde default
        stance_floor: 0.9,       // production override of serde default
        square_icp_residual_max: d_square_res(),
        square_geometric_residual_max: None,
        isolation: true, // production override of serde default
        isolation_max_density: d_iso_density(),
        strip_plane_dist: d_strip_plane_dist(),
        strip_plane_min_frac: d_strip_plane_min_frac(),
        merge_seed_min_points: d_merge_seed_min_points(),
        merge_offset_tol: d_merge_offset_tol(),
        merge_dist_factor: d_merge_dist_factor(),
        patch_min_points: d_patch_min_points(),
        patch_extent_lo_frac: d_patch_extent_lo_frac(),
        patch_extent_hi_diag_frac: d_patch_extent_hi_diag_frac(),
        isolation_coplanar_tol: d_iso_coplanar_tol(),
        isolation_band_lo: d_iso_band_lo(),
        isolation_band_hi: d_iso_band_hi(),
        up_axis,
        cluster_min_points,
    }
}

/// Deserializes directly from the config's `foreground_method` string, so an
/// invalid value fails at parse time with serde naming the valid variants —
/// no separate validation pass, no `String` carried around post-parse.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
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

#[cfg(test)]
mod flatten_tests {
    use super::*;
    use serde::Deserialize;

    // Mirrors the node's DetectionConfig: flat detector params + a
    // `#[serde(flatten)]` DetectorTuning, deserialized from a file that also
    // carries unrelated keys. Guards the config-flattening refactor.
    //
    // `side_m` appears below as one of those unrelated keys on purpose. It used
    // to be a real field of the `BoardConfig` adapter W5-E2 removed; physical
    // geometry now reaches detection only as a `TargetSide` argument, so a file
    // still carrying the key must be read as tuning-with-a-stray-key rather
    // than silently reintroducing a 1 m board assumption.
    #[derive(Debug, Deserialize)]
    struct DetectionConfigLike {
        #[serde(default)]
        detection_mode: String,
        #[serde(default)]
        bbf_voxel: f64,
        #[serde(flatten)]
        board: DetectorTuning,
    }

    #[test]
    fn flatten_reads_tuning_and_ignores_legacy_geometry_keys() {
        let s = r#"{
            "detection_mode": "bbox_free",
            "bbf_voxel": 0.05,
            "side_m": 1.0,
            "up_axis": [1.0, 0.0, 0.0],
            "cluster_eps": 0.30,
            "cluster_min_points": 20,
            "merge_seed_min_points": 40,
            "patch_min_points": 60,
            "isolation_band_hi": 0.30,
            "board_width": "1000mm",
            "icp_min_inlier_points": 300
        }"#;
        let dc: DetectionConfigLike = json5::from_str(s).unwrap();
        assert_eq!(dc.detection_mode, "bbox_free");
        assert_eq!(dc.bbf_voxel, 0.05);
        assert_eq!(dc.board.cluster_eps, 0.30);
        assert_eq!(dc.board.merge_seed_min_points, 40);
        assert_eq!(dc.board.patch_min_points, 60);
        assert_eq!(dc.board.up_axis, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn omitted_new_knobs_fall_back_to_prior_hardcoded_defaults() {
        let dc: DetectionConfigLike = json5::from_str(r#"{ "side_m": 1.0 }"#).unwrap();
        assert_eq!(dc.board.merge_seed_min_points, 40);
        assert_eq!(dc.board.strip_plane_dist, 0.05);
        assert_eq!(dc.board.isolation_coplanar_tol, 0.03);
        assert_eq!(dc.board.patch_extent_hi_diag_frac, 1.8);
    }
}
