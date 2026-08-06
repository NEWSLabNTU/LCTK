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
    #[serde(default)]
    pub isolation: bool,
    #[serde(default = "d_iso_density")]
    pub isolation_max_density: f64,

    // ---- formerly-hardcoded tuning knobs (defaults preserve prior behaviour) ----
    /// plane-strip: RANSAC inlier distance (m) for big-plane removal.
    #[serde(default = "d_strip_plane_dist")]
    pub strip_plane_dist: f64,
    /// plane-strip: min inlier fraction to treat a plane as "big" and strip it.
    #[serde(default = "d_strip_plane_min_frac")]
    pub strip_plane_min_frac: f64,
    /// coplanar-merge: min cluster size to seed a merge group.
    #[serde(default = "d_merge_seed_min_points")]
    pub merge_seed_min_points: usize,
    /// coplanar-merge: max mean point-to-plane offset (m) to absorb a cluster.
    #[serde(default = "d_merge_offset_tol")]
    pub merge_offset_tol: f64,
    /// coplanar-merge: max centroid gap to merge = factor * board diagonal.
    #[serde(default = "d_merge_dist_factor")]
    pub merge_dist_factor: f64,
    /// board-patch gate: min points for a patch to be a candidate.
    #[serde(default = "d_patch_min_points")]
    pub patch_min_points: usize,
    /// board-patch gate: accept extent >= this fraction of side_m.
    #[serde(default = "d_patch_extent_lo_frac")]
    pub patch_extent_lo_frac: f64,
    /// board-patch gate: accept extent <= this fraction of the board diagonal.
    #[serde(default = "d_patch_extent_hi_diag_frac")]
    pub patch_extent_hi_diag_frac: f64,
    /// isolation: max |point-to-plane| (m) to count a point coplanar.
    #[serde(default = "d_iso_coplanar_tol")]
    pub isolation_coplanar_tol: f64,
    /// isolation: inner edge of the exterior density band (m).
    #[serde(default = "d_iso_band_lo")]
    pub isolation_band_lo: f64,
    /// isolation: outer edge of the exterior density band (m).
    #[serde(default = "d_iso_band_hi")]
    pub isolation_band_hi: f64,
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
fn d_cluster_eps() -> f64 {
    0.15 // matches the former hardcoded candidates::CLUSTER_EPS
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

pub fn production_config(side_m: f64, up_axis: [f64; 3], cluster_min_points: usize) -> BoardConfig {
    BoardConfig {
        side_m,
        up_axis,
        cluster_min_points,
        cluster_eps: 0.15,
        side_tol: 0.20,
        cell_m: 0.02,
        vertical_gap_deg: 3.0,
        flatness_rms_max: 0.045, // presets.py
        stance_floor: 0.9,       // presets.py
        square_icp_residual_max: 0.45,
        isolation: true, // presets.py
        isolation_max_density: 0.3,
        strip_plane_dist: 0.05,
        strip_plane_min_frac: 0.08,
        merge_seed_min_points: 40,
        merge_offset_tol: 0.02,
        merge_dist_factor: 1.6,
        patch_min_points: 60,
        patch_extent_lo_frac: 0.5,
        patch_extent_hi_diag_frac: 1.8,
        isolation_coplanar_tol: 0.03,
        isolation_band_lo: 0.05,
        isolation_band_hi: 0.30,
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

#[cfg(test)]
mod flatten_tests {
    use super::*;
    use serde::Deserialize;

    // Mirrors the node's DetectionConfig: flat detector params + a
    // `#[serde(flatten)]` BoardConfig, deserialized from a file that also
    // carries unrelated legacy keys. Guards the config-flattening refactor.
    #[derive(Debug, Deserialize)]
    struct DetectionConfigLike {
        #[serde(default)]
        detection_mode: String,
        #[serde(default)]
        bbf_voxel: f64,
        #[serde(flatten)]
        board: BoardConfig,
    }

    #[test]
    fn flatten_reads_board_and_ignores_legacy_keys() {
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
