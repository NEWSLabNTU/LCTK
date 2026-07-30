mod common;

#[test]
fn fixtures_load_and_are_nonempty() {
    let fx = common::load_all();
    assert!(!fx.is_empty(), "no fixtures found — run export_golden.py");
    assert!(fx.iter().any(|f| f.golden.detected));
    assert!(fx.iter().any(|f| !f.golden.detected));
    for f in &fx {
        assert!(!f.input.is_empty(), "empty input: {}", f.name);
    }
}
