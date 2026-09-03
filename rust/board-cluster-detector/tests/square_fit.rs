use board_cluster_detector::square_fit::fit_fixed_square;

#[test]
fn fits_clean_unit_square_zero_residual() {
    // dense border of a 1 m square at ~15°
    let th = 15f64.to_radians();
    let (c, s) = (th.cos(), th.sin());
    let mut pts = vec![];
    let n = 60;
    for i in 0..n {
        let t = i as f64 / n as f64;
        for e in [[t, 0.0], [t, 1.0], [0.0, t], [1.0, t]] {
            let (x, y) = (e[0] - 0.5, e[1] - 0.5);
            pts.push([x * c - y * s, x * s + y * c]);
        }
    }
    let fit = fit_fixed_square(&pts, 1.0, None, None).unwrap();
    assert!(fit.residual < 0.05, "residual {}", fit.residual);
    // theta determined mod 90°
    let deg = fit.theta.to_degrees().rem_euclid(90.0);
    assert!(
        (deg - 15.0).abs() < 3.0 || (deg - 75.0).abs() < 3.0,
        "theta {deg}"
    );
}

#[test]
fn too_few_points_returns_none() {
    assert!(fit_fixed_square(&[[0.0, 0.0]; 5], 1.0, None, None).is_none());
}

#[test]
fn recovers_center_of_clean_square() {
    // Same construction as above, but offset so we can check recovered center.
    let th = 15f64.to_radians();
    let (c, s) = (th.cos(), th.sin());
    let true_center = [3.0, -2.0];
    let mut pts = vec![];
    let n = 60;
    for i in 0..n {
        let t = i as f64 / n as f64;
        for e in [[t, 0.0], [t, 1.0], [0.0, t], [1.0, t]] {
            let (x, y) = (e[0] - 0.5, e[1] - 0.5);
            pts.push([
                x * c - y * s + true_center[0],
                x * s + y * c + true_center[1],
            ]);
        }
    }
    let fit = fit_fixed_square(&pts, 1.0, None, None).unwrap();
    let dx = fit.center[0] - true_center[0];
    let dy = fit.center[1] - true_center[1];
    assert!(
        dx.hypot(dy) < 0.02,
        "center {:?} too far from true {:?}",
        fit.center,
        true_center
    );
}

// --- H-17: the geometric half of the residual, reported separately -----------------
//
// The full residual sums a geometric term (points outside the modelled square) and a
// perimeter-coverage term. Coverage is what gives the theta search a gradient, but it is
// unreachable as an acceptance gate when the sensor cannot sample the perimeter: a
// 600 mm plate at 7-8 m is crossed by ~4 VLP-32C rings, so about half its perimeter bins
// can never hold a point. These tests pin the split so a gate can use the geometric half.

/// Points on a ring-sampled square: dense within a row, rows far apart. This is the
/// shape that defeats the coverage term, and the shape a spinning LiDAR actually
/// produces on a small plate at range.
fn ring_sampled_square(side: f64, row_gap: f64, in_row: f64) -> Vec<[f64; 2]> {
    let half = side / 2.0;
    let mut pts = Vec::new();
    let mut y = -half;
    while y <= half + 1e-9 {
        let mut x = -half;
        while x <= half + 1e-9 {
            pts.push([x, y]);
            x += in_row;
        }
        y += row_gap;
    }
    pts
}

#[test]
fn geometric_residual_stays_small_when_coverage_cannot_be_satisfied() {
    // ~4 rows across a 0.6 m plate, i.e. the real VLP-32C sampling at 7-8 m.
    let pts = ring_sampled_square(0.6, 0.15, 0.028);
    let fit = fit_fixed_square(&pts, 0.6, None, None).expect("a square this dense must fit");

    // The square models the observed points well...
    assert!(
        fit.geometric_residual < 0.02,
        "geometric residual {} should be near zero for a real plate",
        fit.geometric_residual
    );
    // ...while the full residual is dominated by perimeter bins no ring can reach.
    assert!(
        fit.residual > fit.geometric_residual,
        "full residual {} must exceed its geometric half {}",
        fit.residual,
        fit.geometric_residual
    );
}

#[test]
fn densely_sampled_square_satisfies_both_terms() {
    // With sampling fine enough to reach the perimeter, the two agree closely: the
    // geometric term is not merely a looser gate, it is the same measure minus a
    // penalty that only sparse sampling incurs.
    let pts = ring_sampled_square(0.6, 0.01, 0.01);
    let fit = fit_fixed_square(&pts, 0.6, None, None).expect("dense square must fit");
    assert!(fit.geometric_residual < 0.01);
    assert!(
        fit.residual - fit.geometric_residual < 0.05,
        "dense sampling should leave almost no coverage penalty, got {}",
        fit.residual - fit.geometric_residual
    );
}

#[test]
fn geometric_residual_still_rejects_points_outside_the_square() {
    // The geometric term must remain discriminating: a cloud far larger than the model
    // square scores badly on it, so gating on it is not gating on nothing.
    let pts = ring_sampled_square(1.2, 0.05, 0.02);
    let fit = fit_fixed_square(&pts, 0.6, None, None).expect("fit should return");
    assert!(
        fit.geometric_residual > 0.05,
        "a 1.2 m cloud against a 0.6 m model should score badly, got {}",
        fit.geometric_residual
    );
}
