mod common;
use board_projection_detector::background::BackgroundModel;
use nalgebra::Point3;

#[test]
fn foreground_keeps_new_geometry_drops_static() {
    let mut bg = BackgroundModel::new(0.06, 1, 1);
    // static wall at x=2
    let wall: Vec<_> = (0..50)
        .map(|i| Point3::new(2.0, i as f64 * 0.02, 0.0))
        .collect();
    bg.observe(&wall, "live");
    bg.finalize();
    // query: wall + a new blob at x=5
    let mut q = wall.clone();
    q.push(Point3::new(5.0, 0.0, 0.0));
    let fg = bg.foreground_points(&q);
    assert!(
        fg.iter().all(|p| p.x > 4.0),
        "static wall not suppressed: {fg:?}"
    );
    assert_eq!(fg.len(), 1);
}

#[test]
fn foreground_parity_against_python() {
    for f in common::load_all()
        .into_iter()
        .filter(|f| f.generator_is_bg())
    {
        common::assert_foreground_parity(&f);
    }
}
