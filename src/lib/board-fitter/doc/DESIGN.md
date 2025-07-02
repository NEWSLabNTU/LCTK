# Board Fitter Design Document

## Executive Summary

The board-fitter library provides high-precision detection of calibration boards in LiDAR point clouds. The design emphasizes accuracy through multi-stage refinement, robustness through redundant detection methods, and performance through selective optimization.

## Design Philosophy

### Core Principles

1. **Accuracy First**: Sub-centimeter precision is the primary goal
2. **Graceful Degradation**: System remains functional with reduced accuracy when ideal conditions aren't met
3. **Zero-Cost Abstractions**: Debug and optional features have no runtime cost when disabled
4. **Modular Pipeline**: Each stage can be tested, optimized, and replaced independently
5. **Real-Time Capable**: Design supports streaming processing for live applications

### Design Trade-offs

| Decision | Benefit | Cost |
|----------|---------|------|
| Diamond orientation constraint | Robust detection, fewer false positives | Cannot detect axis-aligned boards |
| Multi-stage ICP refinement | High accuracy | Increased computational cost |
| Dual detection methods (intensity + geometric) | Handles varied LiDAR characteristics | Code complexity |
| Kalman filter tracking | Smooth temporal consistency | Memory overhead for state tracking |

## Algorithmic Design

### Detection Strategy

The detection pipeline employs a coarse-to-fine approach:

1. **Coarse Detection**: Fast algorithms identify candidate regions
2. **Refinement**: ICP algorithms improve initial estimates
3. **Validation**: Geometric constraints verify results

### Key Algorithms

#### 1. RANSAC Plane Detection

- **Purpose**: Identify planar surfaces that could contain boards
- **Design**: Adaptive RANSAC with early termination
- **Parameters**: 
  - Min inliers: 100 points
  - Distance threshold: 2cm
  - Max iterations: 1000

#### 2. Diamond Square Fitting

- **Purpose**: Find 45° rotated squares in planar point sets
- **Design**: Convex hull → PCA → rotation constraint
- **Innovation**: Exploits diamond orientation for robust detection

#### 3. Hybrid Hole Detection

- **Intensity Method**: Fast detection using intensity gradients
- **Geometric Method**: Robust detection using point density
- **Fusion**: Combine results with confidence weighting

#### 4. ICP Refinement Pipeline

Multi-stage refinement for maximum accuracy:

```
Initial Detection
      ↓
Square Pose Refinement (PlaneICP)
      ↓
Hole Pattern Alignment (Point-to-Point ICP)
      ↓
Board Pose Refinement (GICP with covariance)
      ↓
Temporal Alignment (VGICP for tracking)
```

### Pattern Matching Design

Board validation uses multiple geometric constraints:

1. **Hole Count**: Expected number of holes detected
2. **Hole Spacing**: Regular grid pattern validation
3. **Hole Size**: Consistent radius across detections
4. **Board Dimensions**: Match expected physical size

## Configuration Design

### Hierarchical Configuration

```rust
BoardConfig
├── DetectionConfig
│   ├── PlaneDetectionConfig
│   ├── DiamondSquareConfig
│   └── HoleDetectionConfig
├── IcpConfig
│   ├── SquareRefinementConfig
│   ├── HoleAlignmentConfig
│   ├── BoardRefinementConfig
│   └── TemporalAlignmentConfig
└── TrackingConfig
    ├── KalmanFilterConfig
    └── AssociationConfig
```

### Configuration Philosophy

- **Sensible Defaults**: Works out-of-box for common cases
- **Progressive Disclosure**: Advanced options available when needed
- **Runtime Adjustable**: Parameters can be modified without recompilation
- **Validated**: Configuration errors caught at construction time

## Debug System Design

### Zero-Overhead Architecture

```rust
pub trait DebugCallback: Send + Sync {
    fn on_stage_start(&mut self, stage: &str);
    fn on_stage_end(&mut self, stage: &str, metrics: StageMetrics);
    fn on_stage_data(&mut self, stage: &str, data: &DebugData);
}
```

- Callbacks compiled out in release builds
- No heap allocations when disabled
- Structured data for analysis tools

### Debug Data Categories

1. **Timing Metrics**: Per-stage processing time
2. **Algorithm State**: Intermediate results
3. **Quality Metrics**: Confidence scores, error estimates
4. **Visualization Data**: Point clouds, detected features

## Error Handling Design

### Error Categories

1. **Configuration Errors**: Invalid parameters, caught early
2. **Data Errors**: Insufficient points, no planes found
3. **Algorithm Errors**: Convergence failures, numerical issues
4. **System Errors**: CUDA unavailable, memory allocation

### Error Recovery Strategy

```rust
// Example: ICP fallback
match icp_cuda_refinement(data) {
    Ok(refined) => refined,
    Err(CudaError) => {
        warn!("CUDA unavailable, falling back to CPU");
        icp_cpu_refinement(data)?
    }
    Err(e) => return Err(e),
}
```

## Performance Design

> **Note**: For detailed profiling and optimization strategies, see:
> - [DESIGN_PROFILING_OPTIMIZATION.md](DESIGN_PROFILING_OPTIMIZATION.md) - Comprehensive profiling infrastructure and optimization design
> - [OPTIMIZATION_GUIDE.md](OPTIMIZATION_GUIDE.md) - Practical guide for achieving <100ms detection time

### Optimization Levels

1. **Level 0**: Full accuracy, all refinements enabled
2. **Level 1**: Skip temporal refinement (single-frame mode)
3. **Level 2**: Skip hole pattern refinement
4. **Level 3**: Skip square pose refinement
4. **Level 4**: Basic detection only (no ICP)

### Performance Features

- **Adaptive Downsampling**: Automatic voxel filtering for large clouds
- **Early Termination**: Stop when confidence exceeds threshold
- **Caching**: Reuse ICP structures across frames
- **SIMD**: Vectorized operations via nalgebra

### CUDA Acceleration

Optional CUDA support for:
- Board pose refinement (GICP)
- Large point cloud processing
- Multi-board parallel detection

## API Design

### Builder Pattern

```rust
let detector = BoardDetectorBuilder::new(config)
    .with_debug_callback(callback)
    .with_cuda(true)
    .min_confidence(0.8)
    .timeout_ms(100)
    .build()?;
```

### Streaming Interface

```rust
// Process point cloud stream
for point_cloud in stream {
    let result = detector.detect(&point_cloud)?;
    tracker.update(result.detections);
}
```

### Batch Interface

```rust
// Process multiple clouds in parallel
let results = detector.detect_batch(&clouds)?;
```

## Testing Design

### Test Categories

1. **Unit Tests**: Algorithm correctness
2. **Integration Tests**: Pipeline behavior
3. **Performance Tests**: Speed benchmarks
4. **Robustness Tests**: Noise, occlusion handling
5. **Property Tests**: Invariant validation

### Test Data Strategy

- Synthetic data with ground truth
- Real sensor data with annotations
- Adversarial cases for robustness
- Performance regression suite

## Future Design Considerations

### Planned Extensions

1. **Multi-Modal Fusion**: Combined LiDAR + camera detection
2. **Active Learning**: Adaptive parameter tuning
3. **Distributed Processing**: Multi-sensor coordination
4. **Online Calibration**: Continuous refinement during operation

### Design Flexibility

The modular architecture supports:
- Alternative board patterns (checkerboard, ArUco)
- Different sensor types (stereo, RGB-D)
- Custom refinement algorithms
- Application-specific optimizations