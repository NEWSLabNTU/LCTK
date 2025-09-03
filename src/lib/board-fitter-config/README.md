# board-fitter-config

Configuration types for board-fitter calibration board detection library.

## Overview

This library provides serializable configuration types for defining calibration board geometry and detection parameters. It supports square boards with circular holes, commonly used in LiDAR and camera calibration workflows.

## Requirements

- Rust 1.56 or later

## Quick Start

```rust
use board_fitter_config::{BoardConfig, Point2D, SquareBoard};
use measurements::Length;

// Create a square board with 60cm side length
let mut board = SquareBoard::new(Length::from_meters(0.6));

// Add circular holes for orientation detection
board.add_hole(
    Length::from_meters(0.02),  // 2cm radius
    Point2D {
        x: Length::from_meters(0.0),
        y: Length::from_meters(0.25),
    },
    Some("top_hole".to_string()),
);

// Create board configuration
let config = BoardConfig {
    board,
    detection: None,
    metadata: None,
};

// Serialize to JSON
let json = serde_json::to_string_pretty(&config)?;
```

## Types

### SquareBoard
Defines a square calibration board with:
- Side length
- Circular holes for orientation detection
- Optional thickness for 3D modeling

### CircleHole
Represents a circular hole with:
- Radius
- Position relative to board center
- Optional identifier

### Point2D
2D point with physical units using the `measurements` crate.

### DetectionConfig
Algorithm configuration for board detection with:
- Method name
- Algorithm-specific parameters

## Board Orientations

The library supports two board orientations:
- **Diamond**: 45° rotated square (corners at cardinal directions)
- **Aligned**: Standard square orientation

## License

MIT License