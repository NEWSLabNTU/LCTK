# aruco-locator

A Rust library for detecting and locating ArUco markers in images.

## Overview

This library provides functionality to detect ArUco markers in images using OpenCV's ArUco module. It supports camera calibration parameters and multi-marker patterns for accurate pose estimation and localization.

## Requirements

- Rust 1.56 or later
- OpenCV 4.6.0 with ArUco module

## Quick Start

```rust
use aruco_locator::{ArucoDetector, ArucoDetectorConfig};
use std::path::Path;

// Load configuration
let config = ArucoDetectorConfig::from_files(
    Path::new("camera_intrinsics.yaml"),
    Path::new("aruco_pattern.json5")
)?;

// Create detector
let detector = ArucoDetector::new(config);

// Detect markers in image
let image = opencv::imgcodecs::imread("image.jpg", opencv::imgcodecs::IMREAD_COLOR)?;
let result = detector.detect(&image)?;

// Check results
if result.markers_found {
    println!("Found {} markers", result.marker_ids.len());
    for id in &result.marker_ids {
        println!("Marker ID: {}", id);
    }
}
```

## Features

- ArUco marker detection with configurable dictionaries
- Support for camera intrinsics (MRPT calibration format)
- Multi-marker pattern support
- Pose estimation for detected markers
- Visualization utilities for drawing detected markers

## Configuration

### Camera Intrinsics (YAML)
```yaml
camera_params:
  dist: [k1, k2, p1, p2, k3]
  intrinsic: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
image_height: 1080
image_width: 1920
```

### ArUco Pattern (JSON5)
```json5
{
  "dictionary": "DICT_4X4_50",
  "markers": [
    { "id": 0, "size": 0.05 },
    { "id": 1, "size": 0.05 }
  ]
}
```

## License

MIT License