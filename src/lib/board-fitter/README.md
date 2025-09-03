# board-fitter

A Rust library for detecting diamond-oriented square calibration boards in point cloud data.

## Overview

This library provides robust detection of calibration boards with circular hole patterns in 3D point cloud data. It specializes in diamond-oriented (45° rotated) square boards and uses advanced algorithms for accurate pose estimation, making it ideal for LiDAR calibration applications.

## Requirements

- Rust 1.56 or later
- Optional: CUDA for ICP acceleration

## Quick Start

```rust
use board_fitter::{BoardDetector, DetectionConfig, PointCloud};
use board_fitter_config::{BoardConfig, Point2D, SquareBoard};
use measurements::Length;
use nalgebra::Point3;

// Create board configuration
let mut board = SquareBoard::new(Length::from_meters(1.0));
board.add_hole(
    Length::from_meters(0.1),
    Point2D {
        x: Length::from_meters(0.0),
        y: Length::from_meters(0.5),
    },
    Some("top_hole".to_string()),
);

let config = BoardConfig {
    board,
    detection: None,
    metadata: None,
};

// Create detector
let mut detector = BoardDetector::new(DetectionConfig::new_with_default(config));

// Process point cloud
let points: Vec<Point3<f32>> = load_point_cloud();
let cloud = PointCloud::from_points(points);
let detections = detector.detect(&cloud)?;

// Check results
for detection in detections {
    println!("Board found at: {:?}", detection.pose);
}
```

## Features

- **Diamond-oriented board detection**: Optimized for 45° rotated square boards
- **Asymmetric hole patterns**: Supports orientation determination using different hole sizes
- **Multi-board tracking**: Kalman filter-based motion prediction and tracking
- **Adaptive ROI management**: Efficient processing with region-of-interest focusing
- **Robust algorithms**: RANSAC plane detection, PCA-based square fitting, circle fitting
- **ICP refinement**: Multi-stage ICP refinement for sub-centimeter accuracy
- **Real-time performance**: Optimized for real-time processing with configurable timeouts

## Optional Features

- `download`: Enable automatic dataset downloading for examples
- `cuda`: Enable CUDA acceleration for ICP refinement

## License

MIT License