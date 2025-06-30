# Board Fitter Architecture

## Overview

The `board-fitter` library detects diamond-oriented square calibration boards with circular holes in LiDAR point cloud data. The library implements a sophisticated multi-stage detection pipeline with iterative refinement using ICP (Iterative Closest Point) algorithms.

## System Architecture

### Component Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        Board Fitter Library                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Detection     │  │   Refinement    │  │    Tracking     │ │
│  │    Pipeline     │  │    (ICP)        │  │   & Temporal    │ │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘ │
│           │                    │                     │           │
│  ┌────────┴────────────────────┴─────────────────────┴────────┐ │
│  │                    Core Data Structures                     │ │
│  │  • PointCloud  • BoardDetection  • DetectionConfig         │ │
│  └─────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                  External Dependencies                       │ │
│  │  • nalgebra (linear algebra)  • fast-gicp (ICP backend)    │ │
│  │  • opencv (circle detection)  • plane-estimator (RANSAC)   │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### Module Hierarchy

```
src/
├── lib.rs                    # Public API interface
├── detection.rs              # Main detection pipeline orchestration
├── types.rs                  # Core data structures and traits
├── plane.rs                  # RANSAC-based plane detection
├── diamond.rs                # Diamond square fitting algorithms
├── hole.rs                   # Hole detection (intensity + geometric)
├── roi.rs                    # Region of Interest management
├── tracking.rs               # Temporal tracking with Kalman filter
├── debug.rs                  # Zero-overhead debug instrumentation
└── refinement/               # ICP refinement modules
    ├── mod.rs                # ICP integration and configuration
    ├── config.rs             # ICP configuration builder
    ├── board_pose_refinement.rs    # Final board pose refinement
    ├── square_pose_refinement.rs   # Post-PCA square refinement
    ├── hole_pattern_alignment.rs   # Hole pattern matching refinement
    └── temporal_alignment.rs       # Frame-to-frame tracking refinement
```

## Detection Pipeline Architecture

### Stage Flow

```
Input: PointCloud<f64>
         │
         ▼
┌─────────────────────┐
│   ROI Management    │ ← Voxel filtering, preprocessing
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Plane Detection   │ ← RANSAC with multi-plane support
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Diamond Square Fit  │ ← Convex hull + PCA analysis
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Square ICP Refine   │ ← Optional: PlaneICP refinement
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Hole Detection    │ ← Intensity + geometric methods
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Hole ICP Alignment  │ ← Optional: Pattern matching refinement
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Coordinate Transform│ ← 2D plane → 3D board coordinates
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Board ICP Refinement│ ← Final high-precision alignment
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Pattern Matching   │ ← Geometric validation
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Board Tracking    │ ← Temporal consistency
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Temporal ICP Align  │ ← Optional: Frame-to-frame refinement
└──────────┬──────────┘
           │
           ▼
Output: DetectionResult
```

### Stage Dependencies

Each stage can operate independently but is designed to feed into the next:

1. **ROI Management** → Reduces point cloud to relevant region
2. **Plane Detection** → Identifies candidate planar surfaces
3. **Diamond Square Fitting** → Extracts board orientation from plane
4. **Square ICP Refinement** → Improves initial PCA-based orientation
5. **Hole Detection** → Finds circular features within square
6. **Hole ICP Alignment** → Matches detected holes to expected pattern
7. **Coordinate Transform** → Converts to board-centric coordinates
8. **Board ICP Refinement** → Final high-precision pose estimation
9. **Pattern Matching** → Validates against expected geometry
10. **Board Tracking** → Maintains temporal consistency
11. **Temporal ICP Alignment** → Smooths frame-to-frame transitions

## Data Flow Architecture

### Input Processing

```
Raw PointCloud
    │
    ├─→ Voxel Downsampling (if > 10k points)
    │
    ├─→ ROI Bounds Checking
    │
    └─→ Preprocessing (noise removal, outlier filtering)
```

### Intermediate Data Structures

1. **PlaneCandidate**
   - Points belonging to plane
   - Plane equation (normal + distance)
   - Inlier indices
   - Fitting residual

2. **DiamondSquare**
   - Corner points (4 vertices)
   - Center position
   - Rotation (45° constraint)
   - Size (width/height)
   - Local coordinate system

3. **DetectedHole**
   - Center position (3D)
   - Radius
   - Confidence score
   - Contributing points

4. **BoardDetection**
   - Pose (Isometry3<f64>)
   - Detected holes
   - Overall confidence
   - Debug metadata

### Output Format

```rust
pub struct DetectionResult {
    pub detections: Vec<BoardDetection>,
    pub processing_time: Duration,
    pub debug_data: Option<DebugData>,
}
```

## Concurrency Architecture

### Thread Safety

- All core types implement `Send + Sync`
- Detection pipeline is thread-safe for concurrent processing
- ICP refinement can utilize multiple threads via configuration

### Parallelization Points

1. **Plane Detection**: Multiple RANSAC attempts in parallel
2. **Hole Detection**: Parallel processing of intensity/geometric methods
3. **ICP Refinement**: Multi-threaded point correspondence computation
4. **Multi-Board Detection**: Independent processing of each plane

## Memory Architecture

### Allocation Strategy

1. **Pre-allocation**: Reusable buffers for common operations
2. **Lazy Initialization**: ICP structures created on-demand
3. **Reference Counting**: Shared data uses `Arc` for efficiency

### Memory Optimization

- Voxel filtering reduces point count early
- Index-based operations avoid point copying
- Debug data is zero-cost when disabled

## Error Handling Architecture

### Error Hierarchy

```
DetectionError
├── ConfigurationError
│   ├── InvalidParameter
│   └── MissingRequiredField
├── ProcessingError
│   ├── InsufficientPoints
│   ├── NoPlaneFound
│   └── PatternMismatch
└── RefinementError
    ├── IcpConvergenceFailed
    └── CudaNotAvailable
```

### Recovery Strategy

1. **Graceful Degradation**: Continue with reduced accuracy if refinement fails
2. **Fallback Options**: CPU fallback when CUDA unavailable
3. **Partial Results**: Return best available detection even if incomplete

## Extension Points

### Plugin Architecture

1. **Custom Hole Detectors**: Implement `HoleDetector` trait
2. **Alternative Plane Fitters**: Implement `PlaneFitter` trait
3. **Custom Debug Handlers**: Implement `DebugCallback` trait
4. **Additional ICP Methods**: Extend refinement module

### Configuration Extension

```rust
// Custom configuration can be added via:
detector.with_custom_config(|config| {
    config.custom_parameter = value;
});
```

## Performance Architecture

### Optimization Strategies

1. **Early Termination**: Stop processing when confidence threshold met
2. **Adaptive Sampling**: Adjust processing based on point density
3. **Caching**: Reuse computed transformations across frames
4. **SIMD Operations**: Vectorized operations via nalgebra

### Performance Targets

- **Latency**: < 100ms for typical point cloud (10k points)
- **Throughput**: > 10 Hz for continuous processing
- **Memory**: < 100MB working set
- **Accuracy**: < 1cm position error, < 1° rotation error