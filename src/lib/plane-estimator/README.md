# Plane Estimator

The library implements an estimator that fits a plane against a point
cloud.

## Example

```rust
use plane_estimator::PlaneEstimator;
use sample_consensus::Estimator;
use nalgebra as na;

let points: Vec<na::Point3<f32>> = vec![
    [0.0, 0.0, 0.0].into(),
    [1.0, 0.0, 0.0].into(),
    [0.0, 1.0, 0.0].into(),
    [0.0, 0.0, 1.0].into(),
];

let estimator = PlaneEstimator::new();
let plane_model = estimator
    .estimate(points.iter())
    .ok_or_else(|| anyhow!("no plane detected"))?;
```
