//! Fixed-size square model fit: refine (or rescue) a candidate's pose by
//! fitting a KNOWN-side square's 3 DOF (center, theta) directly to its 2D
//! points, instead of trusting the 2D quad's (raw-point minAreaRect) angle.
//!
//! Port of `experiments/board-detection-2d/src/boarddet/square_fit.py`.
//!
//! Deliberately NOT filled-square ICP: an ICP-style fit that only pulls its
//! model's occupied interior onto matched points has zero residual gradient
//! whenever the model already encloses every point. The coverage residual
//! below charges for BOTH (a) points that fall outside the fixed-size square
//! and (b) perimeter band it doesn't reach, so an over-large or mis-rotated
//! enclosing box is still penalized and the search has a gradient to follow.

const MIN_POINTS: usize = 20;
// Trim this much off each end when reading the points' own extent per axis.
const EXTENT_PCTL: f64 = 2.0;
const N_BINS_PER_SIDE: usize = 10;
const BAND_FRAC: f64 = 0.06;
const COARSE_STEPS: usize = 37;
const COARSE_REFINE_WINDOW_DEG: f64 = 4.0;
const RESOLUTION_DEG: f64 = 0.25;
const LOCALIZE_RADIUS_FACTOR: f64 = 1.5;

/// Result of a fixed-side square fit.
#[derive(Debug, Clone, Copy)]
pub struct SquareFit {
    /// Plane coords, same frame as the input `coords`.
    pub center: [f64; 2],
    /// Radians; determined mod 90 deg (4-fold symmetry).
    pub theta: f64,
    /// Coverage residual (geometric + perimeter coverage), lower is better; ~0 is
    /// exact. This is the score the theta search minimises.
    pub residual: f64,
    /// The geometric half of `residual` alone: mean distance of points outside the
    /// square, over side. Unlike `residual` this stays meaningful when the sensor is
    /// too sparse to reach the perimeter, so it is what an acceptance gate should use
    /// for a small target at range (H-17).
    pub geometric_residual: f64,
    /// Cyclic order (not necessarily CCW).
    pub corners_2d: [[f64; 2]; 4],
}

/// `np.percentile(sorted_vals, p)` with numpy's default linear interpolation.
/// `sorted_vals` must already be sorted ascending and non-empty.
fn percentile(sorted_vals: &[f64], p: f64) -> f64 {
    let n = sorted_vals.len();
    if n == 1 {
        return sorted_vals[0];
    }
    let idx = p / 100.0 * (n as f64 - 1.0);
    let lo = idx.floor() as usize;
    let hi = idx.ceil() as usize;
    if lo == hi {
        return sorted_vals[lo];
    }
    let frac = idx - lo as f64;
    sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo])
}

/// `rel` is points in the square's own axis-aligned frame, already centered
/// on the candidate center (so the square occupies `[-side/2, side/2]^2`).
/// Lower is better; 0 is an exact, fully-covered fit.
/// The two halves of the coverage residual, reported separately.
///
/// `geometric` is the mean distance of points falling OUTSIDE the fixed-size square,
/// normalised by side. It measures how well the square models the points actually
/// observed, and is near zero for a real plate however sparsely it is sampled.
///
/// `coverage` is the fraction of perimeter bins holding no nearby point. It is what
/// gives the theta search a gradient -- an over-large or mis-rotated enclosing box is
/// penalised by it -- but it is NOT a measure of fit quality when the sensor cannot
/// reach the perimeter. A 600 mm plate at 7-8 m is crossed by about four VLP-32C rings,
/// so roughly half its perimeter bins are unfillable by construction (H-17).
///
/// `total` is `geometric + coverage`, the historical single number: it still selects
/// theta, so search behaviour is unchanged.
#[derive(Debug, Clone, Copy)]
pub struct ResidualTerms {
    pub geometric: f64,
    pub coverage: f64,
    pub total: f64,
}

fn coverage_residual(rel: &[[f64; 2]], side: f64) -> ResidualTerms {
    let half = side / 2.0;

    // Term 1: mean per-point distance outside the square.
    let n = rel.len() as f64;
    let mean_outside: f64 = rel
        .iter()
        .map(|p| {
            let dx = (p[0].abs() - half).max(0.0);
            let dy = (p[1].abs() - half).max(0.0);
            dx.hypot(dy)
        })
        .sum::<f64>()
        / n;

    // Term 2: fraction of the perimeter's binned bands with no nearby point.
    let corners_sq = [[half, half], [-half, half], [-half, -half], [half, -half]];
    let band = BAND_FRAC * side;
    let mut misses = 0usize;
    for i in 0..4 {
        let a = corners_sq[i];
        let b = corners_sq[(i + 1) % 4];
        let ab = [b[0] - a[0], b[1] - a[1]];
        let length = ab[0].hypot(ab[1]);
        let ab_hat = [ab[0] / length, ab[1] / length];

        let mut covered = [false; N_BINS_PER_SIDE];
        for p in rel {
            let r = [p[0] - a[0], p[1] - a[1]];
            let t = (r[0] * ab_hat[0] + r[1] * ab_hat[1]) / length;
            let perp = (r[0] * ab_hat[1] - r[1] * ab_hat[0]).abs();
            let near = perp < band && t > -0.02 && t < 1.02;
            if near {
                let bin_idx = ((t * N_BINS_PER_SIDE as f64) as i64)
                    .clamp(0, N_BINS_PER_SIDE as i64 - 1) as usize;
                covered[bin_idx] = true;
            }
        }
        misses += covered.iter().filter(|c| !**c).count();
    }
    let coverage_penalty = misses as f64 / (4 * N_BINS_PER_SIDE) as f64;

    let geometric = mean_outside / side;
    ResidualTerms {
        geometric,
        coverage: coverage_penalty,
        total: geometric + coverage_penalty,
    }
}

/// Closed-form center (robust per-axis extent midpoint) + coverage residual
/// for a fixed theta. Returns (residual, center_world, corners_world).
///
/// Matches numpy `p_sq = coords_2d @ rot` (row-vector @ matrix) with
/// `rot = [[c, -s], [s, c]]`, which works out to `p_sq = rot^T @ coords`,
/// i.e. `p_sq_x = x*c + y*s`, `p_sq_y = -x*s + y*c` (unrotate by theta).
/// The inverse transforms (`center_world = center_sq @ rot.T`,
/// `corners_world = (corners_sq + center_sq) @ rot.T`) work out to the
/// standard forward rotation `rot @ v`: `out_x = c*x - s*y`, `out_y = s*x + c*y`.
fn fit_at_theta(
    coords: &[[f64; 2]],
    side: f64,
    theta: f64,
) -> (ResidualTerms, [f64; 2], [[f64; 2]; 4]) {
    let (c, s) = (theta.cos(), theta.sin());

    let mut xs: Vec<f64> = Vec::with_capacity(coords.len());
    let mut ys: Vec<f64> = Vec::with_capacity(coords.len());
    let p_sq: Vec<[f64; 2]> = coords
        .iter()
        .map(|p| {
            let x = p[0] * c + p[1] * s;
            let y = -p[0] * s + p[1] * c;
            xs.push(x);
            ys.push(y);
            [x, y]
        })
        .collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let lo = [percentile(&xs, EXTENT_PCTL), percentile(&ys, EXTENT_PCTL)];
    let hi = [
        percentile(&xs, 100.0 - EXTENT_PCTL),
        percentile(&ys, 100.0 - EXTENT_PCTL),
    ];
    let center_sq = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];

    let rel: Vec<[f64; 2]> = p_sq
        .iter()
        .map(|p| [p[0] - center_sq[0], p[1] - center_sq[1]])
        .collect();

    let terms = coverage_residual(&rel, side);

    let half = side / 2.0;
    let corners_sq = [[half, half], [-half, half], [-half, -half], [half, -half]];

    // Forward rotation (standard rot @ v): out_x = c*x - s*y; out_y = s*x + c*y
    let rotate_fwd = |v: [f64; 2]| [c * v[0] - s * v[1], s * v[0] + c * v[1]];
    let center_world = rotate_fwd(center_sq);
    let corners_world = [0, 1, 2, 3].map(|i| {
        rotate_fwd([
            corners_sq[i][0] + center_sq[0],
            corners_sq[i][1] + center_sq[1],
        ])
    });

    (terms, center_world, corners_world)
}

fn best_over(coords: &[[f64; 2]], side: f64, thetas: &[f64]) -> SquareFit {
    // Theta is still selected by the FULL residual: the coverage term is what penalises
    // a mis-rotated or over-large enclosing square, so removing it from the search would
    // leave rotation nearly unconstrained. Only the reported geometric half is new.
    let mut best_residual = f64::INFINITY;
    let mut best: Option<(f64, f64, [f64; 2], [[f64; 2]; 4])> = None;
    for &theta in thetas {
        let (terms, center, corners) = fit_at_theta(coords, side, theta);
        if terms.total < best_residual {
            best_residual = terms.total;
            best = Some((theta, terms.geometric, center, corners));
        }
    }
    let (theta, geometric, center, corners) = best.expect("thetas must be non-empty");
    SquareFit {
        center,
        theta,
        residual: best_residual,
        geometric_residual: geometric,
        corners_2d: corners,
    }
}

/// Linspace matching numpy's `np.linspace(start, stop, n, endpoint)`.
fn linspace(start: f64, stop: f64, n: usize, endpoint: bool) -> Vec<f64> {
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![start];
    }
    let divisor = if endpoint { n - 1 } else { n };
    let step = (stop - start) / divisor as f64;
    (0..n).map(|i| start + i as f64 * step).collect()
}

/// Fit a fixed-side-`side` square's center + theta to `coords`.
///
/// `init_theta = None`: no orientation prior at all — coarse full sweep over
/// `[0, 90 deg)` (4-fold symmetry), then a finer windowed refine around the
/// coarse winner.
///
/// `init_theta = Some`: search is a bounded window (+/- `theta_window_deg`)
/// around it instead of the full range.
///
/// `init_center = Some`: points farther than `1.5 * side` away are dropped
/// before fitting (only if at least `MIN_POINTS` remain), so unrelated
/// clutter can't bias the robust-extent center estimate.
///
/// Returns `None` if too few points remain to fit.
pub fn fit_fixed_square(
    coords: &[[f64; 2]],
    side: f64,
    init_center: Option<[f64; 2]>,
    init_theta: Option<f64>,
) -> Option<SquareFit> {
    fit_fixed_square_windowed(coords, side, init_center, init_theta, 20.0)
}

/// As [`fit_fixed_square`], with an explicit `theta_window_deg` for the
/// `init_theta = Some` case (defaults to 20 deg via [`fit_fixed_square`]).
pub fn fit_fixed_square_windowed(
    coords: &[[f64; 2]],
    side: f64,
    init_center: Option<[f64; 2]>,
    init_theta: Option<f64>,
    theta_window_deg: f64,
) -> Option<SquareFit> {
    if coords.len() < MIN_POINTS {
        return None;
    }

    let mut work: Vec<[f64; 2]> = coords.to_vec();
    if let Some(center) = init_center {
        let radius = LOCALIZE_RADIUS_FACTOR * side;
        let near: Vec<[f64; 2]> = coords
            .iter()
            .copied()
            .filter(|p| (p[0] - center[0]).hypot(p[1] - center[1]) < radius)
            .collect();
        if near.len() >= MIN_POINTS {
            work = near;
        }
    }

    if work.len() < MIN_POINTS {
        return None;
    }

    let (lo, hi) = if let Some(theta0) = init_theta {
        (
            theta0 - theta_window_deg.to_radians(),
            theta0 + theta_window_deg.to_radians(),
        )
    } else {
        let coarse_thetas = linspace(0.0, std::f64::consts::FRAC_PI_2, COARSE_STEPS, false);
        let coarse_fit = best_over(&work, side, &coarse_thetas);
        (
            coarse_fit.theta - COARSE_REFINE_WINDOW_DEG.to_radians(),
            coarse_fit.theta + COARSE_REFINE_WINDOW_DEG.to_radians(),
        )
    };

    let n_steps = (((hi - lo) / RESOLUTION_DEG.to_radians()).round() as i64 + 1).max(9) as usize;
    let fine_thetas = linspace(lo, hi, n_steps, true);
    Some(best_over(&work, side, &fine_thetas))
}
