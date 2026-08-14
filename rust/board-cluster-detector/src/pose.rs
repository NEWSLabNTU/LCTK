//! Board pose from a scored quad + its plane, plus the stance/isolation gates.
//!
//! Port of `experiments/board-detection-2d/src/boarddet/pose.py` (`board_pose`),
//! `detector.py:_stance`, and `isolation.py:isolation_density`.
//!
//! **Deliberate divergence from that Python source:** [`board_pose`] returns its
//! rotation in the REP-103 / Autoware body convention (**X forward, Y left,
//! Z up**), whereas `pose.py` returns columns `[up-direction, y, normal]`. The
//! three axes are numerically identical in both; only their column order and
//! the sign of the normal column differ. Everything else — the normal-sign
//! flip, the up-most-corner search, and the physical corner winding — is a
//! faithful port. See [`board_pose`] for the exact mapping.

use nalgebra::{Matrix3, Point3, Vector3};

use crate::{
    config::IsolationBand,
    geometry::{project_to_plane, unproject, PlaneModel},
};

/// A solved board pose: center + orthonormal, right-handed rotation in the
/// REP-103 / Autoware body convention (**X forward, Y left, Z up**) + the 3D
/// corners it was built from.
#[derive(Debug, Clone)]
pub struct BoardDetection {
    pub center: Point3<f64>,
    /// Columns: board **X** (forward), board **Y** (left), board **Z** (up).
    ///
    /// The board faces the sensor, so the plane normal oriented *toward* the
    /// sensor (call it `n`) is the board's **−X**: `+X` points out the back of
    /// the board, away from the sensor at the origin. `+Z` points at the
    /// board's up-most corner (the diamond "top"), projected into the board
    /// plane. `+Y = Z × X` (equivalently `n × Z`) completes the right-handed
    /// frame; `det(rotation) = +1`.
    pub rotation: Matrix3<f64>,
    /// The four corners wound counter-clockwise **about the sensor-facing
    /// normal `n = −X`** — i.e. CCW as seen from the sensor, which is
    /// *clockwise* about this frame's `+X`. See [`board_pose`] for why that
    /// sense (and not "CCW about +X") is the one produced.
    pub corners_3d: [Point3<f64>; 4],
    pub score: f64,
}

/// Port of `pose.py:board_pose`, faithful in every number it computes but
/// **diverging in the frame convention it reports**: `pose.py` stacks its
/// columns as `[up-direction, y, n]` (n = normal toward the sensor); this port
/// relabels the very same axes into REP-103 / Autoware order
/// `[X = −n, Y = y, Z = up-direction]`. That is a pure column permutation plus
/// one sign flip — `Y` is bit-for-bit the same vector in both, and no corner
/// moves. Keep this note when touching the file: the provenance is still
/// `pose.py`, only the labelling is ours.
///
/// - Unprojects `corners_2d` via `plane`, takes their mean as `center`.
/// - Orients the plane normal `n` toward the sensor at the origin (flips if it
///   points away). Because the board *faces* the sensor, the board's forward
///   axis is `x_axis = -n`, pointing out the back of the board.
/// - Board Z axis: `center -> up-most corner` (max `corners_3d . up`),
///   projected in-plane by removing the normal component.
/// - `y_axis = n x z_axis`, which is exactly `z_axis x x_axis` — so
///   `[x_axis | y_axis | z_axis]` is right-handed (`det = +1`, since
///   `y_axis x z_axis = -n = x_axis`).
/// - Corners are re-wound by an ascending sort on `atan2(rel.y_axis,
///   rel.z_axis)`. That angle increases when rotating `z_axis -> y_axis`,
///   which is a rotation about `z_axis x y_axis = n`, i.e. **CCW about the
///   sensor-facing normal `n = -x_axis`** — counter-clockwise as seen from the
///   sensor, and therefore *clockwise* about the frame's own `+X`.
///
///   This physical sense is preserved verbatim from the pre-REP-103
///   implementation (and from `pose.py`) on purpose. Re-defining the winding as
///   "CCW about `+X`" would reverse the corner sequence for no geometric gain
///   and would silently invalidate every consumer that has already pinned the
///   order — the golden-vector corner fixtures, and any ArUco correspondence
///   that indexes `corners_3d`. [`stance_3d`] survives either winding (it only
///   needs `[2]-[0]` and `[3]-[1]` to be the diagonals), so it is not the
///   deciding constraint; the pinned ordering is.
pub fn board_pose(
    plane: &PlaneModel,
    corners_2d: &[[f64; 2]; 4],
    score: f64,
    up: [f64; 3],
) -> BoardDetection {
    let up = Vector3::new(up[0], up[1], up[2]).normalize();

    let corners_3d_vec = unproject(corners_2d, plane);
    let center = Point3::from(
        corners_3d_vec
            .iter()
            .fold(Vector3::zeros(), |a, p| a + p.coords)
            / corners_3d_vec.len() as f64,
    );

    // Orient the normal toward the sensor at the origin.
    let mut n = plane.normal.normalize();
    if n.dot(&center.coords) > 0.0 {
        n = -n;
    }

    // Board Z axis (up): center -> up-most corner, projected in-plane.
    // Pick the up-most corner; strict > keeps the FIRST max on ties (matches numpy argmax).
    let (top_i, _) = corners_3d_vec.iter().enumerate().fold(
        (0usize, f64::NEG_INFINITY),
        |(max_i, max_v), (i, c)| {
            let v = c.coords.dot(&up);
            if v > max_v {
                (i, v)
            } else {
                (max_i, max_v)
            }
        },
    );
    let top = corners_3d_vec[top_i];
    let mut z_axis = top - center;
    z_axis -= n * z_axis.dot(&n);
    z_axis = z_axis.normalize();
    // The board faces the sensor, so its forward (+X) axis is the normal
    // pointing AWAY from the sensor.
    let x_axis = -n;
    // Same vector as the Python port's `y = n x x`; also equals z_axis x x_axis,
    // so [x_axis | y_axis | z_axis] is right-handed.
    let y_axis = n.cross(&z_axis);

    // Canonical winding: CCW about n (== the frame's -X, i.e. CCW as seen from
    // the sensor) in the (z_axis, y_axis) basis. Physically identical to the
    // pre-REP-103 ordering — see the doc comment on why it is NOT re-based on
    // the new +X.
    let mut corners_3d: Vec<Point3<f64>> = corners_3d_vec;
    corners_3d.sort_by(|a, b| {
        let rel_a = a - center;
        let rel_b = b - center;
        let ang_a = rel_a.dot(&y_axis).atan2(rel_a.dot(&z_axis));
        let ang_b = rel_b.dot(&y_axis).atan2(rel_b.dot(&z_axis));
        ang_a.total_cmp(&ang_b)
    });

    let rotation = Matrix3::from_columns(&[x_axis, y_axis, z_axis]);
    let corners_3d: [Point3<f64>; 4] = corners_3d.try_into().expect("4 corners");

    BoardDetection {
        center,
        rotation,
        corners_3d,
        score,
    }
}

/// Port of `detector.py:_stance`.
///
/// `corners_3d` must be wound consistently around the board plane (as produced
/// by [`board_pose`]: CCW about the sensor-facing normal), so that
/// `corners_3d[2] - corners_3d[0]` and `corners_3d[3] - corners_3d[1]` are
/// the two diagonals. Only "opposite corners are two apart" matters here, so
/// this is insensitive to the winding's handedness. Returns the max of
/// `|diagonal . up|` over both
/// diagonals (normalized): ~1 for a board standing on a corner, ~0.71 for an
/// axis-aligned flat panel.
pub fn stance_3d(corners_3d: &[Point3<f64>; 4], up: [f64; 3]) -> f64 {
    let up = Vector3::new(up[0], up[1], up[2]).normalize();
    let d1 = (corners_3d[2] - corners_3d[0]).normalize();
    let d2 = (corners_3d[3] - corners_3d[1]).normalize();
    d1.dot(&up).abs().max(d2.dot(&up).abs())
}

/// Point-to-segment distance in 2D: `pts` vs segment `a -> b`.
fn segment_dists(pts: &[[f64; 2]], a: [f64; 2], b: [f64; 2]) -> Vec<f64> {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let denom = ab[0] * ab[0] + ab[1] * ab[1];
    if denom < 1e-12 {
        return pts
            .iter()
            .map(|p| ((p[0] - a[0]).powi(2) + (p[1] - a[1]).powi(2)).sqrt())
            .collect();
    }
    pts.iter()
        .map(|p| {
            let ap = [p[0] - a[0], p[1] - a[1]];
            let t = ((ap[0] * ab[0] + ap[1] * ab[1]) / denom).clamp(0.0, 1.0);
            let proj = [a[0] + t * ab[0], a[1] + t * ab[1]];
            ((p[0] - proj[0]).powi(2) + (p[1] - proj[1]).powi(2)).sqrt()
        })
        .collect()
}

/// Even-odd ray-cast point-in-polygon against a (small) polygon given as
/// consecutive-boundary-order 2D vertices.
fn point_in_polygon(polygon: &[[f64; 2]], pt: [f64; 2]) -> bool {
    let n = polygon.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (polygon[i][0], polygon[i][1]);
        let (xj, yj) = (polygon[j][0], polygon[j][1]);
        let intersects =
            ((yi > pt[1]) != (yj > pt[1])) && (pt[0] < (xj - xi) * (pt[1] - yi) / (yj - yi) + xi);
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Port of `isolation.py:isolation_density`.
///
/// Exterior-band coplanar density, in points per metre of quad perimeter.
/// `dn` is the frame's full pre-strip cloud; `plane`/`corners_2d` are the
/// winning candidate's fitted plane and its (4,2) quad corners in that
/// plane's (u, v) basis. Uses the ORIGINAL `plane.normal` for the coplanar
/// residual test (do not re-flip toward the sensor here).
pub fn isolation_density(
    dn: &[Point3<f64>],
    plane: &PlaneModel,
    corners_2d: &[[f64; 2]; 4],
    band: &IsolationBand,
) -> f64 {
    let coords2d = project_to_plane(dn, plane);
    let coplanar: Vec<bool> = dn
        .iter()
        .map(|p| ((p.coords - plane.center.coords).dot(&plane.normal)).abs() < band.coplanar_tol)
        .collect();

    let edges: [([f64; 2], [f64; 2]); 4] =
        std::array::from_fn(|i| (corners_2d[i], corners_2d[(i + 1) % 4]));

    let mut min_dist = vec![f64::INFINITY; coords2d.len()];
    for (a, b) in edges {
        let d = segment_dists(&coords2d, a, b);
        for (m, v) in min_dist.iter_mut().zip(d.iter()) {
            *m = m.min(*v);
        }
    }

    let count = coords2d
        .iter()
        .zip(min_dist.iter())
        .zip(coplanar.iter())
        .filter(|((c, &md), &cop)| {
            let inside = point_in_polygon(corners_2d, **c);
            let in_band = !inside && (band.lo..=band.hi).contains(&md);
            in_band && cop
        })
        .count();

    let perimeter: f64 = edges
        .iter()
        .map(|(a, b)| ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt())
        .sum();

    if perimeter > 1e-9 {
        count as f64 / perimeter
    } else {
        0.0
    }
}
