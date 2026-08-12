//! Plane fitting and 2D plane-coordinate projection (the chosen projection).
//!
//! Port of `experiments/board-detection-2d/src/boarddet/geometry.py`.

use nalgebra::{DMatrix, Point3, Vector3};
use std::collections::BTreeMap;

/// A fitted plane: center + orthonormal basis (`u`, `v` in-plane, `normal` out-of-plane).
#[derive(Debug, Clone, Copy)]
pub struct PlaneModel {
    pub center: Point3<f64>,
    pub normal: Vector3<f64>,
    pub u: Vector3<f64>,
    pub v: Vector3<f64>,
}

/// Drop points with any non-finite (NaN/inf) coordinate.
///
/// Raw LiDAR PointCloud2 encodes invalid returns as NaN; `fit_plane`'s SVD
/// would otherwise propagate a NaN normal and poison every downstream pose.
pub fn finite_only(points: &[Point3<f64>]) -> Vec<Point3<f64>> {
    points
        .iter()
        .filter(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
        .copied()
        .collect()
}

/// Fit a plane to `points` via SVD of the centered coordinates.
///
/// Matches numpy's `np.linalg.svd(q)`, where `vt` rows are the right
/// singular vectors: `u = vt[0]`, `v = vt[1]`, `normal = vt[2]` (smallest
/// singular vector = normal; the two largest span the plane). SVD sign is
/// arbitrary — downstream (`pose::board_pose`) fixes the normal sign toward
/// the sensor; do not try to pin signs here.
pub fn fit_plane(points: &[Point3<f64>]) -> PlaneModel {
    let n = points.len() as f64;
    let center = points.iter().fold(Vector3::zeros(), |a, p| a + p.coords) / n;
    // q: rows = centered points (N x 3). SVD -> V^T rows are the 3 principal axes.
    let q = DMatrix::from_row_iterator(
        points.len(),
        3,
        points.iter().flat_map(|p| {
            let d = p.coords - center;
            [d.x, d.y, d.z]
        }),
    );
    let svd = q.svd(true, true);
    let vt = svd.v_t.expect("v_t"); // 3x3, rows = right singular vectors (matches numpy vt)
    let row = |i: usize| Vector3::new(vt[(i, 0)], vt[(i, 1)], vt[(i, 2)]);
    PlaneModel {
        center: Point3::from(center),
        normal: row(2),
        u: row(0),
        v: row(1),
    }
}

/// Project 3D points into the plane's 2D (u, v) coordinate frame.
pub fn project_to_plane(points: &[Point3<f64>], plane: &PlaneModel) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|p| {
            let q = p.coords - plane.center.coords;
            [q.dot(&plane.u), q.dot(&plane.v)]
        })
        .collect()
}

/// Inverse of [`project_to_plane`]: map 2D (u, v) plane coordinates back to 3D.
pub fn unproject(coords: &[[f64; 2]], plane: &PlaneModel) -> Vec<Point3<f64>> {
    coords
        .iter()
        .map(|c| Point3::from(plane.center.coords + c[0] * plane.u + c[1] * plane.v))
        .collect()
}

/// RMS of the perpendicular (normal-direction) distance of `points` from `plane`.
pub fn plane_rms(points: &[Point3<f64>], plane: &PlaneModel) -> f64 {
    let s: f64 = points
        .iter()
        .map(|p| {
            let d = (p.coords - plane.center.coords).dot(&plane.normal);
            d * d
        })
        .sum();
    (s / points.len() as f64).sqrt()
}

/// Largest span (max of the two axis extents) of 2D plane coordinates.
pub fn extent_2d(coords: &[[f64; 2]]) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for c in coords {
        for k in 0..2 {
            lo[k] = lo[k].min(c[k]);
            hi[k] = hi[k].max(c[k]);
        }
    }
    (hi[0] - lo[0]).max(hi[1] - lo[1])
}

/// Voxel-grid downsample: group points by `floor(coord / voxel)` per axis,
/// emit each occupied voxel's centroid (mean of its member points).
///
/// Matches open3d's `voxel_down_sample`, which the Python `downsample`
/// wraps — this is a centroid per voxel, not grid-snapping.
///
/// Uses a `BTreeMap` (not `HashMap`) so the emitted point ORDER is
/// deterministic across runs, keyed by voxel index ascending. This matters
/// downstream: `candidates::remove_big_planes` feeds this output straight
/// into `ransac_plane_inliers`, whose seeded RNG draws samples by index into
/// the input sequence -- a `HashMap`'s randomized-per-process iteration order
/// would silently make RANSAC's result (and therefore candidate sets)
/// non-reproducible run to run despite the seed, defeating the whole point
/// of seeding it.
pub fn voxel_downsample(points: &[Point3<f64>], voxel: f64) -> Vec<Point3<f64>> {
    let mut voxels: BTreeMap<(i64, i64, i64), (Vector3<f64>, usize)> = BTreeMap::new();
    for p in points {
        let key = (
            (p.x / voxel).floor() as i64,
            (p.y / voxel).floor() as i64,
            (p.z / voxel).floor() as i64,
        );
        let entry = voxels.entry(key).or_insert((Vector3::zeros(), 0));
        entry.0 += p.coords;
        entry.1 += 1;
    }
    voxels
        .values()
        .map(|(sum, count)| Point3::from(sum / (*count as f64)))
        .collect()
}
