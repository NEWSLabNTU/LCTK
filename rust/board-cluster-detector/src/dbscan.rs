//! Anisotropic-scaled, grid-accelerated Euclidean DBSCAN.
//!
//! Ports `_anisotropic_scaled` from
//! `experiments/board-detection-2d/src/boarddet/candidates/cluster_after_ground.py:19-47`
//! plus a grid-accelerated Euclidean DBSCAN matching open3d `cluster_dbscan`
//! semantics: a point is core if it has `>= min_points` neighbours within
//! `eps` INCLUDING itself; clusters are connected components of core points
//! plus their border points.

use nalgebra::Point3;
use std::collections::HashMap;

/// Return a z-compressed COPY of `points` for clustering only.
///
/// A VLP-32C's vertical ring gap at horizontal range r is ~r*tan(gap_deg),
/// which grows multi-cm at just a few metres -- well past a fixed isotropic
/// DBSCAN eps -- while adjacent points *within* a ring stay tight. Scaling
/// each point's z by eps_h / eps_v(r) (eps_v the range-scaled vertical
/// tolerance) turns that into an elliptical neighbourhood: horizontal
/// tolerance stays eps_h, vertical tolerance widens with range, so a plain
/// isotropic DBSCAN call on the scaled cloud reconnects ring-gap-fragmented
/// surfaces. eps_v is clamped to >= eps_h so nearby/dense clouds are
/// unaffected (r small -> eps_v == eps_h -> z unscaled).
///
/// Labels produced by clustering the returned array index back into the
/// original (unscaled) `points` by position -- callers must check cluster
/// labels against `points`, never against this scaled copy.
pub fn anisotropic_scaled(
    points: &[Point3<f64>],
    eps_h: f64,
    vertical_gap_deg: f64,
) -> Vec<Point3<f64>> {
    if vertical_gap_deg <= 0.0 {
        return points.to_vec();
    }
    let t = vertical_gap_deg.to_radians().tan();
    points
        .iter()
        .map(|p| {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            let eps_v = eps_h.max(2.0 * r * t);
            Point3::new(p.x, p.y, p.z * (eps_h / eps_v))
        })
        .collect()
}

/// Grid-accelerated Euclidean DBSCAN matching open3d `cluster_dbscan`
/// semantics. Returns labels, `-1` = noise. Deterministic in point iteration
/// order (0..N), so results are reproducible run-to-run.
pub fn dbscan(points: &[Point3<f64>], eps: f64, min_points: usize) -> Vec<i64> {
    let n = points.len();
    let mut labels = vec![-1_i64; n]; // -1 = unvisited/noise
    let mut visited = vec![false; n];
    let eps2 = eps * eps;
    let key = |p: &Point3<f64>| {
        (
            (p.x / eps).floor() as i64,
            (p.y / eps).floor() as i64,
            (p.z / eps).floor() as i64,
        )
    };
    let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (i, p) in points.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i);
    }
    let region = |i: usize| -> Vec<usize> {
        let (cx, cy, cz) = key(&points[i]);
        let mut out = vec![];
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(c) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &j in c {
                            if (points[i].coords - points[j].coords).norm_squared() <= eps2 {
                                out.push(j);
                            }
                        }
                    }
                }
            }
        }
        out
    };
    let mut cluster = 0_i64;
    for i in 0..n {
        if visited[i] {
            continue;
        }
        visited[i] = true;
        let neigh = region(i);
        if neigh.len() < min_points {
            continue; // stays noise (-1)
        }
        labels[i] = cluster;
        let mut queue = neigh;
        let mut qi = 0;
        while qi < queue.len() {
            let j = queue[qi];
            qi += 1;
            if !visited[j] {
                visited[j] = true;
                let jn = region(j);
                if jn.len() >= min_points {
                    queue.extend(jn);
                }
            }
            if labels[j] < 0 {
                labels[j] = cluster;
            }
        }
        cluster += 1;
    }
    labels
}

/// Scale (copy), cluster the scaled copy, and return labels indexed back to
/// the ORIGINAL `points` by position.
pub fn cluster_anisotropic(
    points: &[Point3<f64>],
    eps: f64,
    min_points: usize,
    vertical_gap_deg: f64,
) -> Vec<i64> {
    let scaled = anisotropic_scaled(points, eps, vertical_gap_deg);
    dbscan(&scaled, eps, min_points)
}
