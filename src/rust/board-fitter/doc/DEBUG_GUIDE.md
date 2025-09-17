# Board-Fitter Debug Guide

This guide explains how to use the BoardDetector with debug callbacks to get intermediate outputs during the detection process.

## Overview

The board-fitter library provides a comprehensive debug system that allows you to:
- Get timing information for each processing stage
- Access intermediate data (planes, holes, etc.)
- Collect performance metrics
- Track algorithm convergence statistics

## Quick Start

```rust
use board_fitter::{
    debug::{DataCallback, DebugConfigBuilder, DebugContext, DebugData},
    BoardDetectorBuilder,
};
use std::sync::Arc;

// 1. Create a debug callback
struct MyCallback;

impl DataCallback for MyCallback {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        // Handle intermediate data
        println!("Stage {}: {:?}", stage, data);
    }

    fn on_point_cloud(&self, stage: &str, cloud: &PointCloud) {
        println!("Stage {}: {} points", stage, cloud.points.len());
    }
}

// 2. Configure debug settings
let debug_config = DebugConfigBuilder::new()
    .with_timing()                    // Enable timing measurements
    .capture_stages([                 // Select stages to capture
        "plane_detection",
        "diamond_fitting",
        "hole_detection",
    ])
    .build();

// 3. Create debug context with callbacks
let mut debug_context = DebugContext::new(debug_config);
debug_context.data_callback = Some(Arc::new(MyCallback));

// 4. Create detector with debug enabled
let detector = BoardDetectorBuilder::new(board_config)
    .with_debug(debug_context)
    .build()?;

// 5. Run detection - callbacks will be invoked
let result = detector.detect(&point_cloud)?;
```

## Available Stages

The following stages can be captured:

| Stage | Description | Data Type |
|-------|-------------|-----------|
| `preprocessing` | ROI extraction and filtering | `PointCloud` |
| `plane_detection` | RANSAC plane detection | `PlaneData` |
| `diamond_fitting` | Diamond square fitting | `PointCloud`, `Generic` |
| `hole_detection` | Hole/circle detection | `CircleData` |
| `validation` | Board validation | `DetectionResult` |
| `board_tracking` | Temporal tracking | `DetectionResult` |

## Callback Types

### 1. TimingCallback
Tracks timing information for each stage:

```rust
impl TimingCallback for MyTimer {
    fn on_stage_start(&self, stage: &str, timestamp: Instant) {
        println!("Stage {} started", stage);
    }

    fn on_stage_end(&self, stage: &str, duration: Duration, memory_usage: Option<usize>) {
        println!("Stage {} took {:?}", stage, duration);
    }
}
```

### 2. DataCallback
Receives intermediate processing data:

```rust
impl DataCallback for MyDataHandler {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        match data {
            DebugData::PlaneData { planes, inlier_counts, .. } => {
                for (i, plane) in planes.iter().enumerate() {
                    println!("Plane {}: {} inliers", i, inlier_counts[i]);
                }
            }
            DebugData::CircleData { holes, fitting_residuals, .. } => {
                for (i, hole) in holes.iter().enumerate() {
                    println!("Hole {}: residual = {:.6}", i, fitting_residuals[i]);
                }
            }
            _ => {}
        }
    }
}
```

### 3. MetricsCallback
Collects performance metrics:

```rust
impl MetricsCallback for MyMetrics {
    fn on_metrics(&self, stage: &str, metrics: &StageMetrics) {
        println!("{}: {} → {} points in {:?}",
            stage,
            metrics.input_points,
            metrics.output_points,
            metrics.processing_time
        );
    }

    fn on_algorithm_stats(&self, stage: &str, stats: &AlgorithmStats) {
        println!("{}: {} iterations, converged: {}",
            stage,
            stats.iterations,
            stats.converged
        );
    }
}
```

## Debug Data Types

### PlaneData
Contains detected planes and quality metrics:
```rust
DebugData::PlaneData {
    planes: Vec<DetectedPlane>,        // Detected plane parameters
    inlier_counts: Vec<usize>,         // Points per plane
    quality_scores: Vec<f64>,          // Plane quality scores
    metadata: HashMap<String, String>, // Additional info
}
```

### CircleData
Contains detected holes/circles:
```rust
DebugData::CircleData {
    holes: Vec<DetectedHole>,         // Detected hole parameters
    fitting_residuals: Vec<f64>,      // Circle fitting errors
    iteration_counts: Vec<usize>,     // Iterations for convergence
    metadata: HashMap<String, String>,
}
```

### DetectionResult
Final or intermediate detection results:
```rust
DebugData::DetectionResult {
    detections: Vec<BoardDetection>,  // Board detections
    confidence_scores: Vec<f64>,      // Confidence values
    metadata: HashMap<String, String>,
}
```

## Advanced Usage

### Collecting All Data

```rust
use std::sync::{Arc, Mutex};
use std::collections::HashMap;

struct DataCollector {
    data: Arc<Mutex<HashMap<String, Vec<DebugData>>>>,
}

impl DataCallback for DataCollector {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        self.data
            .lock()
            .unwrap()
            .entry(stage.to_string())
            .or_insert_with(Vec::new)
            .push(data.clone());
    }
}
```

### Saving Intermediate Point Clouds

```rust
impl DataCallback for CloudSaver {
    fn on_point_cloud(&self, stage: &str, cloud: &PointCloud) {
        let filename = format!("debug_{}.pcd", stage);
        // Save cloud to file
        save_point_cloud(&filename, cloud).unwrap();
    }
}
```

### Performance Profiling

```rust
let debug_config = DebugConfigBuilder::new()
    .with_timing()
    .with_memory_tracking()
    .capture_stages(["plane_detection", "hole_detection"])
    .build();
```

## Example: Complete Debug Pipeline

See the full example in `examples/debug_detection.rs` which demonstrates:
- Setting up all three callback types
- Capturing data from all stages
- Printing comprehensive debug information
- Analyzing performance metrics

## Tips

1. **Performance Impact**: Debug callbacks have minimal overhead when disabled, but can impact performance when capturing large point clouds.

2. **Memory Usage**: Use `max_point_clouds()` to limit memory usage when capturing point cloud data.

3. **Selective Capture**: Only capture stages you need to minimize overhead:
   ```rust
   .capture_stage("hole_detection")  // Only capture hole detection
   ```

4. **Thread Safety**: All callbacks must be `Send + Sync` as they may be called from multiple threads.

5. **Error Handling**: Callbacks should not panic - handle errors gracefully to avoid disrupting detection.