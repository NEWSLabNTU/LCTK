//! Candidate generators: full scene -> plausible board plane patches.
//!
//! Port of `experiments/board-detection-2d/src/boarddet/candidates/__init__.py`,
//! `cluster_after_ground.py` and `background_diff.py`.

use crate::{
    background::BackgroundModel,
    config::{ForegroundMethod, TargetDetectionParams},
    dbscan::{anisotropic_scaled, cluster_anisotropic, dbscan},
    geometry::{extent_2d, fit_plane, plane_rms, project_to_plane, PlaneModel},
};
use nalgebra::{Point3, Vector3};
use rand::{rngs::StdRng, Rng, SeedableRng};
use std::collections::{BTreeMap, HashMap, HashSet};

/// A plausible board-plane patch: its member points and fitted plane.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub points: Vec<Point3<f64>>,
    pub plane: PlaneModel,
}

fn centroid(points: &[Point3<f64>]) -> Point3<f64> {
    let n = points.len() as f64;
    let sum = points
        .iter()
        .fold(Vector3::zeros(), |acc, p| acc + p.coords);
    Point3::from(sum / n)
}

/// Gate a 3D patch: enough points, flat, board-sized. `None` if implausible.
///
/// Ports `candidates/__init__.py:plausible_board_patch` (default threshold
/// path only -- no `rejects` side channel; that's a diagnostics feature not
/// in this port's scope).
pub fn plausible_board_patch(
    points: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
) -> Option<Candidate> {
    if points.len() < board.patch_min_points {
        return None;
    }
    let threshold = board.flatness_rms_max;
    let plane = fit_plane(points);
    let rms = plane_rms(points, &plane);
    if rms > threshold {
        return None;
    }
    let ext = extent_2d(&project_to_plane(points, &plane));
    let side_m = board.target_side().as_metres();
    let diag = side_m * 2.0_f64.sqrt();
    let (lo, hi) = (
        board.patch_extent_lo_frac * side_m,
        board.patch_extent_hi_diag_frac * diag,
    );
    if !(lo <= ext && ext <= hi) {
        return None;
    }
    Some(Candidate {
        points: points.to_vec(),
        plane,
    })
}

/// Plain seeded RANSAC plane fit, matching open3d's
/// `segment_plane(distance_threshold=dist, ransac_n=3, num_iterations=300)`:
/// 300 iterations of "sample 3 distinct points, fit a plane, count inliers
/// within `dist`", keeping the largest inlier set. Deterministic (seed 0),
/// so `remove_big_planes` is reproducible. Returns inlier indices into `pts`.
///
/// Replaces the earlier `arrsac::Arrsac` port, whose preemptive SPRT scored
/// hypotheses on only a small point block before pruning and so discarded the
/// dominant ground/wall plane on full scenes (~50 s/frame, 13/20 parity --
/// see task-9 report). Exhaustive per-hypothesis inlier counting here is both
/// faithful to open3d and far faster.
fn ransac_plane_inliers(pts: &[Point3<f64>], dist: f64) -> Vec<usize> {
    if pts.len() < 3 {
        return vec![];
    }
    let mut rng = StdRng::seed_from_u64(0);
    let mut best: Vec<usize> = vec![];
    for _ in 0..300 {
        // pick 3 distinct random indices
        let a = rng.gen_range(0..pts.len());
        let mut b = rng.gen_range(0..pts.len());
        while b == a {
            b = rng.gen_range(0..pts.len());
        }
        let mut c = rng.gen_range(0..pts.len());
        while c == a || c == b {
            c = rng.gen_range(0..pts.len());
        }
        let (pa, pb, pc) = (pts[a], pts[b], pts[c]);
        let n = (pb.coords - pa.coords).cross(&(pc.coords - pa.coords));
        let norm = n.norm();
        if norm < 1e-9 {
            continue; // collinear sample
        }
        let n = n / norm;
        let d = -n.dot(&pa.coords);
        let inliers: Vec<usize> = pts
            .iter()
            .enumerate()
            .filter(|(_, p)| (n.dot(&p.coords) + d).abs() <= dist)
            .map(|(i, _)| i)
            .collect();
        if inliers.len() > best.len() {
            best = inliers;
        }
    }
    best
}

/// Iteratively strip planes whose inlier patch is far larger than a board.
///
/// Ports `_remove_big_planes`. open3d's RANSAC (`segment_plane`) is
/// reimplemented by `ransac_plane_inliers` (plain seeded 300-iteration
/// RANSAC), seeded deterministically per RANSAC call.
pub fn remove_big_planes(
    points: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
    dist: f64,
    min_frac: f64,
    vertical_gap_deg: f64,
) -> Vec<Point3<f64>> {
    let diag = board.target_side().as_metres() * 2.0_f64.sqrt();
    let mut remaining: Vec<Point3<f64>> = points.to_vec();

    for _ in 0..6 {
        if remaining.len() < 100 {
            break;
        }
        let inlier_idx = ransac_plane_inliers(&remaining, dist);
        if inlier_idx.is_empty() {
            break;
        }
        let cutoff = 100.max((min_frac * remaining.len() as f64) as usize);
        if inlier_idx.len() < cutoff {
            break;
        }
        let inliers: Vec<Point3<f64>> = inlier_idx.iter().map(|&i| remaining[i]).collect();

        // Judge big-vs-board-scale on the largest connected component of the
        // inliers (looser eps=0.20/min_points=10 bridges VLP-32C ring gaps),
        // not the raw inlier set -- see cluster_after_ground.py's comment.
        let scaled_in = anisotropic_scaled(&inliers, 0.20, vertical_gap_deg);
        let labels = dbscan(&scaled_in, 0.20, 10);

        let mut counts: BTreeMap<i64, usize> = BTreeMap::new();
        for &l in &labels {
            if l >= 0 {
                *counts.entry(l).or_default() += 1;
            }
        }
        if counts.is_empty() {
            // No component has >= 10 points at eps=0.20: an unmeasurable,
            // fragmented patch is not the board -- strip and keep looking.
            let mask: HashSet<usize> = inlier_idx.into_iter().collect();
            remaining = remaining
                .into_iter()
                .enumerate()
                .filter(|(i, _)| !mask.contains(i))
                .map(|(_, p)| p)
                .collect();
            continue;
        }
        // np.bincount(valid).argmax(): first label (ascending) achieving the
        // max count -- BTreeMap iterates ascending, so a strict `>` keeps
        // the first-seen max on ties.
        let mut best_label = counts.keys().next().copied().unwrap();
        let mut best_count = 0usize;
        for (&lbl, &c) in &counts {
            if c > best_count {
                best_count = c;
                best_label = lbl;
            }
        }
        let biggest: Vec<Point3<f64>> = inliers
            .iter()
            .zip(labels.iter())
            .filter(|(_, &l)| l == best_label)
            .map(|(&p, _)| p)
            .collect();
        let plane = fit_plane(&biggest);
        let ext = extent_2d(&project_to_plane(&biggest, &plane));
        if ext <= 2.0 * diag {
            // Largest remaining coherent plane patch is board-scale: stop.
            break;
        }
        let mask: HashSet<usize> = inlier_idx.into_iter().collect();
        remaining = remaining
            .into_iter()
            .enumerate()
            .filter(|(i, _)| !mask.contains(i))
            .map(|(_, p)| p)
            .collect();
    }
    remaining
}

/// Points surviving generator B's big-plane strip -- the "foreground" its
/// clustering step sees. Shares the generator's own strip params; ports
/// `big_plane_residual`.
pub fn big_plane_residual(
    points: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
    vertical_gap_deg: f64,
) -> Vec<Point3<f64>> {
    remove_big_planes(
        points,
        board,
        board.strip_plane_dist,
        board.strip_plane_min_frac,
        vertical_gap_deg,
    )
}

/// Greedily grow each sizeable DBSCAN cluster with nearby coplanar ones.
///
/// Ports `_merge_coplanar_clusters`. Keeps the "grow outward from a
/// reliable-plane seed (>= `seed_min_points`) by point-to-plane distance"
/// semantics exactly -- see the Python docstring for why a pairwise
/// normal-similarity test does not work (a lone ring stripe's own SVD
/// normal is too unstable to compare against another stripe's).
fn merge_coplanar_clusters(
    points: &[Point3<f64>],
    labels: &[i64],
    board: &TargetDetectionParams<'_>,
) -> Vec<Vec<Point3<f64>>> {
    let seed_min_points = board.merge_seed_min_points;
    let offset_tol = board.merge_offset_tol;
    let merge_dist_factor = board.merge_dist_factor;

    let diag = board.target_side().as_metres() * 2.0_f64.sqrt();

    let mut label_ids: Vec<i64> = labels.iter().copied().filter(|&l| l >= 0).collect();
    label_ids.sort_unstable();
    label_ids.dedup();

    let clusters: HashMap<i64, Vec<Point3<f64>>> = label_ids
        .iter()
        .map(|&lbl| {
            let pts: Vec<Point3<f64>> = points
                .iter()
                .zip(labels.iter())
                .filter(|(_, &l)| l == lbl)
                .map(|(&p, _)| p)
                .collect();
            (lbl, pts)
        })
        .collect();

    // Stable sort by descending size; label_ids starts ascending, so ties
    // preserve ascending label order -- matches Python's
    // `sorted(clusters, key=lambda lbl: -len(clusters[lbl]))` over a dict
    // built from `np.unique` (ascending) keys.
    let mut order = label_ids.clone();
    order.sort_by_key(|lbl| std::cmp::Reverse(clusters[lbl].len()));

    let mut used: HashSet<i64> = HashSet::new();
    let mut groups: Vec<Vec<Point3<f64>>> = vec![];
    for &seed in &order {
        if used.contains(&seed) {
            continue;
        }
        used.insert(seed);
        let mut group_pts = clusters[&seed].clone();
        if group_pts.len() >= seed_min_points {
            let mut plane = fit_plane(&group_pts);
            let mut center = centroid(&group_pts);
            let mut grew = true;
            while grew {
                grew = false;
                for &lbl in &order {
                    if used.contains(&lbl) {
                        continue;
                    }
                    let pts = &clusters[&lbl];
                    let pts_center = centroid(pts);
                    if (pts_center.coords - center.coords).norm() > merge_dist_factor * diag {
                        continue;
                    }
                    let mean_offset: f64 = pts
                        .iter()
                        .map(|p| (p.coords - plane.center.coords).dot(&plane.normal).abs())
                        .sum::<f64>()
                        / pts.len() as f64;
                    if mean_offset > offset_tol {
                        continue;
                    }
                    used.insert(lbl);
                    group_pts.extend_from_slice(pts);
                    plane = fit_plane(&group_pts); // refit: more points, more stable normal
                    center = centroid(&group_pts);
                    grew = true;
                }
            }
        }
        groups.push(group_pts);
    }
    groups
}

/// Shared B/E tail: anisotropic DBSCAN -> coplanar-stripe merge -> gate.
///
/// Ports `_cluster_and_gate`.
pub fn cluster_and_gate(
    fg: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
    cluster_eps: f64,
    cluster_min_points: usize,
    vertical_gap_deg: f64,
) -> Vec<Candidate> {
    if fg.len() < cluster_min_points {
        return vec![];
    }
    let labels = cluster_anisotropic(fg, cluster_eps, cluster_min_points, vertical_gap_deg);
    let mut out = vec![];
    for group_pts in merge_coplanar_clusters(fg, &labels, board) {
        if let Some(cand) = plausible_board_patch(&group_pts, board) {
            out.push(cand);
        }
    }
    out
}

/// Approach B: remove large planes, Euclidean-cluster the rest, gate
/// clusters. Ports `generate_cluster_after_ground` (renamed in this port --
/// see task-5 brief).
pub fn generate_plane_strip(
    points: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
) -> Vec<Candidate> {
    let rest = remove_big_planes(
        points,
        board,
        board.strip_plane_dist,
        board.strip_plane_min_frac,
        board.vertical_gap_deg,
    );
    cluster_and_gate(
        &rest,
        board,
        board.cluster_eps,
        board.cluster_min_points,
        board.vertical_gap_deg,
    )
}

/// Approach E (Method E): background/motion subtraction. Ports
/// `generate_background_diff`. Deliberately no `remove_big_planes` stage:
/// ground and walls are background by construction and are already gone
/// before clustering runs.
pub fn generate_background_diff(
    dn: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
    background: &BackgroundModel,
) -> Vec<Candidate> {
    let fg = background.foreground_points(dn);
    cluster_and_gate(
        &fg,
        board,
        board.cluster_eps,
        board.cluster_min_points,
        board.vertical_gap_deg,
    )
}

/// Compute the RAW per-method foreground and the gated candidates in one pass.
///
/// `foreground` is the exact point set handed to `cluster_and_gate` BEFORE any
/// clustering / coplanar merge / board-patch gating — Method E: the
/// background-subtracted points; Method B: the non-big-plane remainder. Exposed
/// so a caller can visualize the true foreground (not just surviving clusters).
/// Returns `(foreground, candidates)`. Background-subtraction with no model
/// yields empty for both.
pub fn foreground_and_candidates(
    dn: &[Point3<f64>],
    board: &TargetDetectionParams<'_>,
    method: ForegroundMethod,
    background: Option<&BackgroundModel>,
) -> (Vec<Point3<f64>>, Vec<Candidate>) {
    let fg = match method {
        ForegroundMethod::BackgroundSubtraction => match background {
            Some(bg) => bg.foreground_points(dn),
            None => return (Vec::new(), Vec::new()),
        },
        ForegroundMethod::PlaneStrip => remove_big_planes(
            dn,
            board,
            board.strip_plane_dist,
            board.strip_plane_min_frac,
            board.vertical_gap_deg,
        ),
    };
    let cands = cluster_and_gate(
        &fg,
        board,
        board.cluster_eps,
        board.cluster_min_points,
        board.vertical_gap_deg,
    );
    (fg, cands)
}
