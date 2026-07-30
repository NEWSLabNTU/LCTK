//! Board pose from a scored quad + its plane, plus the stance/isolation gates.
//!
//! Port of `experiments/board-detection-2d/src/boarddet/pose.py` (`board_pose`),
//! `detector.py:_stance`, and `isolation.py:isolation_density`.

use nalgebra::{Matrix3, Point3, Vector3};

use crate::geometry::{project_to_plane, unproject, PlaneModel};

/// A solved board pose: center + orthonormal rotation (columns = board x,
/// board y, plane normal) + the CCW-wound 3D corners it was built from.
#[derive(Debug, Clone)]
pub struct BoardDetection {
    pub center: Point3<f64>,
    /// Columns: board x, board y, normal.
    pub rotation: Matrix3<f64>,
    pub corners_3d: [Point3<f64>; 4],
    pub score: f64,
}

/// Port of `pose.py:board_pose`.
///
/// - Unprojects `corners_2d` via `plane`, takes their mean as `center`.
/// - Orients the plane normal toward the sensor at the origin (flips if it
///   points away).
/// - Board X axis: `center -> up-most corner` (max `corners_3d . up`),
///   projected in-plane by removing the normal component.
/// - `y = n x x` (right-handed).
/// - Corners are re-wound CCW about `n` in the `(x, y)` basis via
///   `atan2(rel.y, rel.x)` ascending sort.
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

    // Board X axis: center -> up-most corner, projected in-plane.
    let top = corners_3d_vec
        .iter()
        .copied()
        .max_by(|a, b| (a.coords.dot(&up)).total_cmp(&b.coords.dot(&up)))
        .expect("4 corners");
    let mut x = top - center;
    x -= n * x.dot(&n);
    x = x.normalize();
    let y = n.cross(&x);

    // Canonical CCW winding about n in the (x, y) basis.
    let mut corners_3d: Vec<Point3<f64>> = corners_3d_vec;
    corners_3d.sort_by(|a, b| {
        let rel_a = a - center;
        let rel_b = b - center;
        let ang_a = rel_a.dot(&y).atan2(rel_a.dot(&x));
        let ang_b = rel_b.dot(&y).atan2(rel_b.dot(&x));
        ang_a.total_cmp(&ang_b)
    });

    let rotation = Matrix3::from_columns(&[x, y, n]);
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
/// `corners_3d` must be CCW-ordered (as produced by [`board_pose`]), so
/// `corners_3d[2] - corners_3d[0]` and `corners_3d[3] - corners_3d[1]` are
/// the two diagonals. Returns the max of `|diagonal . up|` over both
/// diagonals (normalized): ~1 for a board standing on a corner, ~0.71 for an
/// axis-aligned flat panel.
pub fn stance_3d(corners_3d: &[Point3<f64>; 4], up: [f64; 3]) -> f64 {
    let up = Vector3::new(up[0], up[1], up[2]).normalize();
    let d1 = (corners_3d[2] - corners_3d[0]).normalize();
    let d2 = (corners_3d[3] - corners_3d[1]).normalize();
    d1.dot(&up).abs().max(d2.dot(&up).abs())
}

const COPLANAR_TOL: f64 = 0.03;
const BAND_LO: f64 = 0.05;
const BAND_HI: f64 = 0.30;

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
) -> f64 {
    let coords2d = project_to_plane(dn, plane);
    let coplanar: Vec<bool> = dn
        .iter()
        .map(|p| ((p.coords - plane.center.coords).dot(&plane.normal)).abs() < COPLANAR_TOL)
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
            let band = !inside && (BAND_LO..=BAND_HI).contains(&md);
            band && cop
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
