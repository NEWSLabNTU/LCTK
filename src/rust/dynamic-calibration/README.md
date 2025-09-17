# dynamic-calibration

A Rust library for intelligent calibration parameter adjustment based on real-time quality metrics.

## Overview

This library provides dynamic adjustment of calibration parameters during runtime based on detection confidence, scene characteristics, and calibration quality metrics. It helps maintain optimal calibration performance across varying environmental conditions and sensor configurations.

## Requirements

- Rust 1.56 or later

## Quick Start

```rust
use dynamic_calibration::{DynamicCalibrationController, AdjustmentStrategy};
use calibration_quality::{CalibrationMetrics, QualityScore};

// Create controller with balanced strategy
let mut controller = DynamicCalibrationController::new();

// Or use specific strategy
let mut controller = DynamicCalibrationController::with_strategy(
    AdjustmentStrategy::Aggressive
);

// Update parameters based on calibration results
let metrics = CalibrationMetrics { /* ... */ };
let quality = QualityScore { /* ... */ };
let updated_params = controller.update(&metrics, &quality)?;

// Check if parameters have stabilized
if controller.is_stable(5) {
    println!("Calibration parameters stable");
}
```

## Features

### Dynamic Parameter Adjustment
- Detection threshold
- Minimum inliers
- RANSAC iterations
- ICP convergence criteria
- Point cloud downsampling
- ROI size multiplier
- Noise filtering threshold
- Feature matching threshold

### Adjustment Strategies
- **Conservative**: Minimal changes, prioritizes stability
- **Balanced**: Default strategy, balances performance and stability
- **Aggressive**: Rapid adaptation to changing conditions

### Scene Analysis
- Complexity estimation
- Noise level detection
- Feature density analysis
- Stability tracking

### Presets
```rust
use dynamic_calibration::presets;

// High accuracy for critical applications
let params = presets::high_accuracy();

// Fast processing for real-time requirements
let params = presets::fast_processing();

// Noisy environment for outdoor/industrial settings
let params = presets::noisy_environment();

// Sparse data for limited feature scenarios
let params = presets::sparse_data();
```

## Components

- **DynamicCalibrationController**: Main controller for parameter management
- **ParameterAdjuster**: Implements adjustment strategies
- **ConfidenceAnalyzer**: Analyzes detection confidence metrics
- **SceneAnalyzer**: Evaluates scene characteristics

## License

MIT License