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
