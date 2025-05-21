# 3D Hollow Board Detector

This library implements a detector that locates the *hollow-board*
from a point cloud. The board description is defined in the
[hollow-board-config](../hollow-board-config/README.md) library.

## Example

```rust
// Configure and build the detector
let aruco_pattern: MultiArucoPattern = json5::from_str(include_str!("aruco_pattern.json5"))?;
let config: Config = json5::from_str(include_str!("board_detector.json5"))?;
let detector = Detector::new(config, aruco_pattern);

// Load the point cloud file.
let reader = DynReader::open(&args.input_file)?;
let points: Vec<na::Point3<f64>> = reader
    .map(|point| {
        let [x, y, z]: [f32; 3] = point.unwrap().to_xyz().unwrap();
        na::Point3::new(x as f64, y as f64, z as f64)
    })
    .collect();

// Perform detection
let detection = detector.detect(&points)?;

// Show the detection results
if let Some(detection) = detection {
    println!("{:?}", detection.board_model);
} else {
    println!("No board detected");
}
```
