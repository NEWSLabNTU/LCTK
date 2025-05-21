# ArUco Board Detector

This Rust library provides an ArUco board detector, which pattern is
described by the [aruco-config](../aruco-config/README.md) library.

## Example

```rust
// Configure and build the detector
let pattern = MultiArucoPattern {
    marker_ids: vec![149, 391, 385, 482],
    dictionary: ArucoDictionary::DICT_5X5_1000,
    board_size: Length::from_millimeters(500.0),
    board_border_size: Length::from_millimeters(10.0),
    marker_square_size_ratio: r64(0.8),
    num_squares_per_side: 2,
    border_bits: 1,
};
let detector = Builder {
    pattern,
    camera_intrinsic: CameraIntrinsics::default(),
}
.build()?;

// Perform pattern detection.
let image = imread(&args.input_file, IMREAD_GRAYSCALE)?;
let detection = detector.detect_markers(&image)?;

// Print the detection results.
if let Some(detection) = detection {
    for marker in detection.markers() {
        print!("id={}, corners=", marker.id);

        let corners: Vec<_> = marker.corners[1..]
            .into_iter()
            .map(|corner| format!("({}, {})", corner.x, corner.y))
            .collect();
        let corners = corners.join(", ");
        println!("[{corners}]");
    }
} else {
    println!("No ArUco pattern found!");
}
```
