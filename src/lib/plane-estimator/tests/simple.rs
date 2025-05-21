use plane_estimator::PlaneEstimator;
use sample_consensus::Estimator;

#[test]
fn simple() {
    let points = vec![
        [1.0, 0.0, 0.0],
        [1.0, 1.2, 2.4],
        [1.0, -0.9, 1.7],
        [1.0, 0.5, -1.8],
        [1.0, 4.4, 6.1],
    ];

    let estimator = PlaneEstimator::new();
    let plane = estimator.estimate(points.iter());
    let plane = plane.expect("not able to fit a plane");
    eprintln!("A plane is found with pose {}", plane.pose());
}
