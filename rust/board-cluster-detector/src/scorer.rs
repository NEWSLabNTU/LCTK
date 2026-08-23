//! Reduced port of `boarddet/scorer.py`.
//!
//! Under the production `square_icp=True` path, `score_candidate`'s
//! raster/morphology/fillPoly/findContours/fill-ratio machinery does not
//! affect the accept/reject decision — only its `minAreaRect` quad center
//! feeds `fit_fixed_square` as a seed (falling back to the coordinate
//! centroid). This module ports exactly that: `min_area_rect` (convex hull +
//! rotating calipers) and `seed_center` (the size-gated seed selection from
//! `detector.py::_quad_center` / the `square_icp` seed logic).

use crate::config::TargetDetectionParams;

/// Result of a minimum-area bounding rectangle computation, matching the
/// geometry (not necessarily the corner ordering/labeling) of
/// `cv2.minAreaRect` + `cv2.boxPoints`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MinAreaRect {
    pub center: [f64; 2],
    pub size: [f64; 2],
    pub corners: [[f64; 2]; 4],
}

/// Andrew's monotone-chain convex hull. Returns hull points in
/// counter-clockwise order, deduplicated. Returns fewer than 3 points if the
/// input is degenerate (all points coincident or collinear collapses to a
/// segment).
fn convex_hull(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut pts = points.to_vec();
    pts.sort_by(|a, b| {
        a[0].partial_cmp(&b[0])
            .unwrap()
            .then(a[1].partial_cmp(&b[1]).unwrap())
    });
    pts.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-12 && (a[1] - b[1]).abs() < 1e-12);

    if pts.len() < 3 {
        return pts;
    }

    fn cross(o: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    }

    let mut lower: Vec<[f64; 2]> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }

    let mut upper: Vec<[f64; 2]> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Minimum-area bounding rectangle via convex hull + rotating calipers.
///
/// Returns `None` for degenerate inputs (fewer than 3 unique points, or a
/// collinear point set with zero hull area).
pub fn min_area_rect(coords: &[[f64; 2]]) -> Option<MinAreaRect> {
    let hull = convex_hull(coords);
    if hull.len() < 3 {
        return None;
    }

    type RectCandidate = ([f64; 2], [f64; 2], [[f64; 2]; 4]);

    let n = hull.len();
    let mut best_area = f64::INFINITY;
    let mut best: Option<RectCandidate> = None;

    for i in 0..n {
        let p0 = hull[i];
        let p1 = hull[(i + 1) % n];
        let edge = [p1[0] - p0[0], p1[1] - p0[1]];
        let len = (edge[0] * edge[0] + edge[1] * edge[1]).sqrt();
        if len < 1e-12 {
            continue;
        }
        // Unit axes: u along the edge, v perpendicular.
        let ux = edge[0] / len;
        let uy = edge[1] / len;
        let vx = -uy;
        let vy = ux;

        let mut min_u = f64::INFINITY;
        let mut max_u = f64::NEG_INFINITY;
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        for &p in &hull {
            let du = p[0] * ux + p[1] * uy;
            let dv = p[0] * vx + p[1] * vy;
            min_u = min_u.min(du);
            max_u = max_u.max(du);
            min_v = min_v.min(dv);
            max_v = max_v.max(dv);
        }

        let w = max_u - min_u;
        let h = max_v - min_v;
        let area = w * h;

        if area < best_area {
            best_area = area;
            let cu = (min_u + max_u) / 2.0;
            let cv = (min_v + max_v) / 2.0;
            // Map center back to original (x, y) frame: center = cu*u + cv*v.
            let center = [cu * ux + cv * vx, cu * uy + cv * vy];

            // Corners in (u, v) space, mapped back to (x, y).
            let corner_uv = [
                (min_u, min_v),
                (max_u, min_v),
                (max_u, max_v),
                (min_u, max_v),
            ];
            let mut corners = [[0.0; 2]; 4];
            for (k, (cu_k, cv_k)) in corner_uv.iter().enumerate() {
                corners[k] = [cu_k * ux + cv_k * vx, cu_k * uy + cv_k * vy];
            }

            best = Some((center, [w, h], corners));
        }
    }

    best.map(|(center, size, corners)| MinAreaRect {
        center,
        size,
        corners,
    })
}

/// Ports the `square_icp` seed-selection logic (`detector.py::_quad_center`
/// plus the size gate from `scorer.py`'s `anisotropic` branch): compute
/// `min_area_rect`, and if it exists and passes the size gate, return its
/// center; otherwise fall back to the centroid of `coords`.
pub fn seed_center(coords: &[[f64; 2]], board: &TargetDetectionParams<'_>) -> [f64; 2] {
    let centroid = || {
        let n = coords.len() as f64;
        let sx: f64 = coords.iter().map(|p| p[0]).sum();
        let sy: f64 = coords.iter().map(|p| p[1]).sum();
        [sx / n, sy / n]
    };

    if coords.is_empty() {
        return [0.0, 0.0];
    }

    let rect = match min_area_rect(coords) {
        Some(r) => r,
        None => return centroid(),
    };

    let min_size = rect.size[0].min(rect.size[1]);
    if min_size < 3.0 * board.cell_m {
        return centroid();
    }

    let side_m = board.target_side().as_metres();
    let lo = side_m * (1.0 - 2.0 * board.side_tol);
    let hi = side_m * (1.0 + 2.0 * board.side_tol);

    let mean_side: f64 = {
        let c = rect.corners;
        let mut total = 0.0;
        for i in 0..4 {
            let a = c[i];
            let b = c[(i + 1) % 4];
            total += ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        }
        total / 4.0
    };

    if mean_side > lo && mean_side < hi {
        rect.center
    } else {
        centroid()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convex_hull_of_triangle_is_itself() {
        let pts = vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
        let hull = convex_hull(&pts);
        assert_eq!(hull.len(), 3);
    }

    #[test]
    fn min_area_rect_of_collinear_points_is_none() {
        let pts = vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
        assert!(min_area_rect(&pts).is_none());
    }
}
