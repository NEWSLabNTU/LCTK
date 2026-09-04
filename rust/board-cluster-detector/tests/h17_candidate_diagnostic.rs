//! H-17 diagnostic: where does a real solid-board frame lose its candidate?
//!
//! Runs the real pipeline stages on real exported frames, outside ROS, so the numbers
//! are deterministic (the ROS path drops frames nondeterministically in realtime mode
//! and receives nothing in offline mode -- M-30).
//!
//! Ignored by default: it needs `tmp/h17diag/` from `tmp/export_diag.py`. Run with
//!   cargo test -p board-cluster-detector --test h17_candidate_diagnostic -- --ignored --nocapture

use board_cluster_detector::{
    background::BackgroundModel,
    candidates::{foreground_and_candidates, plausible_board_patch},
    config::{DetectorTuning, ForegroundMethod, TargetDetectionParams, TargetSide},
    dbscan::cluster_anisotropic,
    geometry::{finite_only, fit_plane, plane_rms, project_to_plane, voxel_downsample},
};
use nalgebra::Point3;
use std::{collections::HashMap, fs, path::Path};

fn load_f32(path: &Path) -> Vec<Point3<f64>> {
    fs::read(path)
        .unwrap()
        .chunks_exact(12)
        .map(|c| {
            let f = |i: usize| f32::from_le_bytes(c[i * 4..i * 4 + 4].try_into().unwrap()) as f64;
            Point3::new(f(0), f(1), f(2))
        })
        .collect()
}

#[test]
#[ignore]
fn where_do_solid_board_candidates_die() {
    let dir = Path::new("../../tmp/h17diag");
    let tuning: DetectorTuning = json5::from_str(
        &fs::read_to_string("../../ros/lctk_launch/config/board/solid_600/velodyne.json5").unwrap(),
    )
    .expect("the shipped solid_600 preset must deserialize");
    let side = TargetSide::metres(0.6).unwrap();
    let params = TargetDetectionParams::new(side, &tuning);

    // Build the background exactly as the node does during warmup: each frame is its
    // own source, and a voxel becomes background once min_sources frames have seen it.
    for source in ["bgframe", "selfbg", "spreadbg", "nodebg"] {
        run_with_background(dir, &params, &tuning, source);
    }
}

/// Build a background from files starting with `prefix` and report where candidates die.
///
/// `bgframe` = the board-free `newtype_background` bag.
/// `selfbg`  = `newtype_1`'s own opening frames, which is what the shipped session uses.
fn run_with_background(
    dir: &Path,
    params: &TargetDetectionParams<'_>,
    tuning: &DetectorTuning,
    prefix: &str,
) {
    let mut bg = BackgroundModel::new(0.05, 1, 3);
    let mut bg_frames: Vec<_> = fs::read_dir(dir)
        .unwrap()
        .filter_map(|e| {
            let p = e.unwrap().path();
            let name = p.file_name()?.to_str()?.to_string();
            (name.starts_with(prefix) && p.extension()? == "f32").then_some(p)
        })
        .collect();
    bg_frames.sort();
    assert!(!bg_frames.is_empty(), "run tmp/export_diag.py first");
    for (i, f) in bg_frames.iter().enumerate() {
        let dn = voxel_downsample(&finite_only(&load_f32(f)), 0.05);
        bg.observe(&dn, &format!("frame{i}"));
    }
    bg.finalize();
    println!(
        "\n=== background from {prefix}*: {} frames, {} voxels ===",
        bg_frames.len(),
        bg.keys().len()
    );

    let mut frames: Vec<_> = fs::read_dir(dir)
        .unwrap()
        .filter_map(|e| {
            let p = e.unwrap().path();
            let name = p.file_name()?.to_str()?.to_string();
            (name.starts_with("frame") && p.extension()? == "f32").then_some(p)
        })
        .collect();
    frames.sort();
    assert!(!frames.is_empty(), "run tmp/export_diag.py first");

    println!(
        "\n{:>6} {:>7} {:>7} {:>8} {:>8} {:>9}  {}",
        "frame", "raw", "fg", "clusters", "cands", "biggest", "why the biggest cluster failed"
    );

    for f in &frames {
        let raw = load_f32(f);
        let dn = voxel_downsample(&finite_only(&raw), 0.05);
        let (fg, cands) = foreground_and_candidates(
            &dn,
            params,
            ForegroundMethod::BackgroundSubtraction,
            Some(&bg),
        );

        let labels = cluster_anisotropic(
            &fg,
            tuning.cluster_eps,
            tuning.cluster_min_points,
            tuning.vertical_gap_deg,
        );
        let mut by_label: HashMap<i64, Vec<Point3<f64>>> = HashMap::new();
        for (p, &l) in fg.iter().zip(labels.iter()) {
            if l >= 0 {
                by_label.entry(l).or_default().push(*p);
            }
        }
        let biggest = by_label.values().max_by_key(|v| v.len());

        let why = match biggest {
            None => "no cluster formed".to_string(),
            Some(c) => {
                if c.len() < tuning.patch_min_points {
                    format!(
                        "points {} < patch_min_points {}",
                        c.len(),
                        tuning.patch_min_points
                    )
                } else {
                    let plane = fit_plane(c);
                    let rms = plane_rms(c, &plane);
                    if rms > tuning.flatness_rms_max {
                        format!("flatness {rms:.4} > {:.4}", tuning.flatness_rms_max)
                    } else {
                        let coords = project_to_plane(c, &plane);
                        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
                        for a in 0..2 {
                            let vals: Vec<f64> = coords.iter().map(|p| p[a]).collect();
                            lo = lo.min(vals.iter().cloned().fold(f64::MAX, f64::min));
                            hi = hi.max(vals.iter().cloned().fold(f64::MIN, f64::max));
                        }
                        let ext = hi - lo;
                        let diag = 0.6 * 2f64.sqrt();
                        let (glo, ghi) = (
                            tuning.patch_extent_lo_frac * 0.6,
                            tuning.patch_extent_hi_diag_frac * diag,
                        );
                        if ext < glo || ext > ghi {
                            format!("extent {ext:.3} outside [{glo:.3}, {ghi:.3}]")
                        } else if plausible_board_patch(c, params).is_some() {
                            "PASSES the patch gate".to_string()
                        } else {
                            "rejected by the patch gate for another reason".to_string()
                        }
                    }
                }
            }
        };

        println!(
            "{:>6} {:>7} {:>7} {:>8} {:>8} {:>9}  {}",
            f.file_stem().unwrap().to_str().unwrap(),
            raw.len(),
            fg.len(),
            by_label.len(),
            cands.len(),
            biggest.map(|c| c.len()).unwrap_or(0),
            why
        );
    }
}
