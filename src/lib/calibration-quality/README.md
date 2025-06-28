# Calibration Quality Assessment Library

This library provides comprehensive tools for automatic calibration quality assessment, validation, and convergence monitoring in real-time calibration workflows.

## Features

### Quality Metrics
- **Reprojection Error**: Mean squared error between transformed and target points
- **Consistency Score**: How well the transform preserves geometric relationships
- **Inlier Ratio**: Percentage of correspondences within error threshold
- **Geometric Errors**: Translation and rotation error statistics
- **Statistical Metrics**: Standard deviation, median, percentiles, and outlier detection

### Validation
- **Physical Constraints**: Ensures transforms are physically plausible
- **Temporal Consistency**: Checks for sudden changes between calibrations
- **Adaptive Thresholds**: Adjusts validation criteria based on scene complexity
- **Comprehensive Reporting**: Detailed validation messages and confidence scores

### Convergence Monitoring
- **Real-time Tracking**: Monitors calibration convergence during optimization
- **Convergence Prediction**: Estimates iterations remaining to convergence
- **Adaptive Monitoring**: Adjusts convergence criteria based on performance
- **History Analysis**: Tracks quality trends over time

## Usage

### Basic Quality Assessment

```rust
use calibration_quality::{QualityAssessor, ValidationConfig};
use nalgebra::{Isometry3, Point3};

// Create quality assessor with default config
let config = ValidationConfig::default();
let mut assessor = QualityAssessor::new(config);

// Assess calibration quality
let transform = Isometry3::identity(); // Your calibration transform
let detection_pairs = vec![
    (Point3::new(0.0, 0.0, 1.0), Point3::new(0.01, 0.01, 1.01)),
    (Point3::new(1.0, 0.0, 1.0), Point3::new(1.01, 0.01, 1.01)),
];
let detection_confidence = 0.95;

let quality = assessor.assess(&transform, &detection_pairs, detection_confidence)?;

println!("Calibration quality: {}", quality.summary());
println!("Overall score: {:.2}%", quality.overall_score * 100.0);
```

### Custom Validation Configuration

```rust
use calibration_quality::ValidationConfig;

let mut config = ValidationConfig {
    max_translation: 5.0,           // Maximum 5 meters
    max_rotation: 1.57,             // Maximum 90 degrees
    min_inlier_ratio: 0.7,          // Require 70% inliers
    max_reprojection_error: 0.05,   // Maximum 5cm error
    min_consistency_score: 0.8,     // Require 80% consistency
    check_physical_constraints: true,
    check_temporal_consistency: true,
};
```

### Convergence Monitoring

```rust
use calibration_quality::{ConvergenceMonitor, ConvergenceConfig};

// Create convergence monitor
let config = ConvergenceConfig {
    translation_threshold: 0.001,    // 1mm convergence threshold
    rotation_threshold: 0.001,       // ~0.06 degree threshold
    min_iterations: 5,
    max_iterations: 100,
    convergence_window: 5,
    quality_improvement_threshold: 0.01,
};

let mut monitor = ConvergenceMonitor::with_config(config);

// Update during calibration iterations
for (transform, metrics) in calibration_iterations {
    monitor.update(&transform, &metrics);
    
    let status = monitor.status();
    if status.is_converged {
        println!("Calibration converged after {} iterations", status.iterations);
        break;
    }
    
    if let Some(remaining) = status.estimated_iterations_remaining {
        println!("Estimated iterations remaining: {}", remaining);
    }
}
```

### Adaptive Quality Assessment

```rust
use calibration_quality::AdaptiveValidator;

// Create adaptive validator that adjusts thresholds based on scene
let base_config = ValidationConfig::default();
let mut validator = AdaptiveValidator::new(base_config);

// Validation config is automatically adjusted based on metrics
let adapted_config = validator.adapt_config(&metrics);
```

## Quality Metrics Explained

### Overall Score (0.0 to 1.0)
Weighted combination of:
- **Accuracy** (30%): Based on reprojection error
- **Precision** (30%): Based on geometric error
- **Robustness** (20%): Based on inlier ratio
- **Consistency** (20%): Based on geometric preservation

### Validation Checks
- **Translation Magnitude**: Ensures translation is within reasonable bounds
- **Rotation Magnitude**: Ensures rotation is physically plausible
- **Inlier Ratio**: Sufficient percentage of good correspondences
- **Reprojection Error**: Transformation accuracy within threshold
- **Consistency Score**: Geometric relationships preserved
- **Physical Constraints**: No impossible transforms (e.g., negative determinant)
- **Temporal Consistency**: Smooth changes between calibrations

### Convergence Criteria
- **Translation Change**: Movement between iterations below threshold
- **Rotation Change**: Angular change between iterations below threshold
- **Quality Stability**: Quality score plateaus within tolerance
- **Minimum Iterations**: Ensures sufficient optimization attempts

## Integration with ROS 2

This library is designed to integrate seamlessly with ROS 2 calibration nodes:

```rust
// In your ROS 2 calibration node
use calibration_quality::{QualityAssessor, ValidationConfig};

impl CalibrationNode {
    fn handle_calibration_result(&mut self, result: CalibrationResult) {
        // Assess quality
        let quality = self.quality_assessor.assess(
            &result.transform,
            &result.correspondences,
            result.confidence,
        )?;
        
        // Publish quality metrics
        let quality_msg = QualityMessage {
            overall_score: quality.overall_score,
            is_valid: quality.validation.is_valid,
            is_converged: quality.convergence.is_converged,
            messages: quality.validation.messages,
        };
        
        self.quality_publisher.publish(quality_msg)?;
        
        // Only accept high-quality calibrations
        if quality.meets_requirements(0.8) {
            self.accept_calibration(result);
        }
    }
}
```

## Best Practices

1. **Start with Default Configuration**: The default validation config works well for most scenarios
2. **Monitor Trends**: Use quality trends to detect degradation over time
3. **Adaptive Thresholds**: Enable adaptive validation for varying scene conditions
4. **Set Appropriate Timeouts**: Use max_iterations to prevent infinite optimization
5. **Log Quality Metrics**: Record quality assessments for debugging and analysis

## Performance Considerations

- Quality assessment is lightweight and suitable for real-time use
- Convergence monitoring maintains bounded history (configurable size)
- Statistical computations use efficient algorithms
- Thread-safe for concurrent use in multi-threaded applications