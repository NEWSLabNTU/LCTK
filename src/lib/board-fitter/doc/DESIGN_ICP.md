# ICP Refinement Design

## Overview

The board-fitter library implements a sophisticated multi-stage ICP (Iterative Closest Point) refinement pipeline to achieve sub-centimeter accuracy in board pose estimation. This document details the design and implementation of the ICP integration.

## ICP Architecture

### Refinement Stages

```
┌─────────────────────────────────────────────────────────┐
│                  ICP Refinement Pipeline                  │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  1. Square Pose Refinement (PlaneICP)                    │
│     └─→ Refines initial PCA-based orientation            │
│                                                           │
│  2. Hole Pattern Alignment (Point-to-Point ICP)          │
│     └─→ Matches detected holes to expected pattern       │
│                                                           │
│  3. Board Pose Refinement (GICP)                         │
│     └─→ Final high-precision alignment                   │
│                                                           │
│  4. Temporal Alignment (VGICP)                           │
│     └─→ Frame-to-frame tracking refinement               │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

### Backend Integration

The library uses `fast-gicp` (Rust bindings for `small_gicp`) as the ICP backend:

```rust
// Fast-GICP provides multiple registration algorithms
pub enum RegistrationType {
    ICP,        // Standard point-to-point
    PlaneICP,   // Point-to-plane with normals
    GICP,       // Generalized ICP with covariance
    VGICP,      // Voxelized GICP for large clouds
}
```

## Stage 1: Square Pose Refinement

### Purpose
Refine the initial PCA-based square orientation using planar constraints.

### Algorithm
```rust
// PlaneICP with DOF restriction to planar motion
let config = IcpStageConfig {
    registration_type: RegistrationType::PlaneICP,
    max_iterations: 20,
    convergence_threshold: 1e-4,
    dof_restriction: Some(DofRestrictionConfig::Planar { 
        plane_normal 
    }),
};
```

### Design Rationale
- PCA provides good initial orientation but may have ~5° error
- PlaneICP exploits known planar constraint
- DOF restriction prevents out-of-plane drift

### Implementation Details
```rust
pub fn refine_square_pose(
    points: &[Point3<f64>],
    initial_square: &DiamondSquare,
    plane_normal: &Vector3<f64>,
) -> Result<DiamondSquare> {
    // Generate ideal square points
    let target = generate_ideal_square_points(initial_square.size);
    
    // Configure PlaneICP with planar DOF
    let registration = register_advanced(
        points,
        &target,
        Some(initial_square.to_transform()),
        &config,
    )?;
    
    // Apply refined transform
    Ok(initial_square.apply_transform(registration.transformation))
}
```

## Stage 2: Hole Pattern Alignment

### Purpose
Align detected holes with the expected hole pattern for improved accuracy.

### Algorithm
```rust
// Point-to-point ICP for hole centers
let config = IcpStageConfig {
    registration_type: RegistrationType::ICP,
    max_iterations: 50,
    convergence_threshold: 1e-5,
    correspondence_rejection: Some(CorrespondenceRejection {
        max_distance: hole_radius * 2.0,
    }),
};
```

### Pattern Matching Strategy
1. Generate expected hole positions from board specification
2. Find correspondences between detected and expected holes
3. Use ICP to refine the alignment
4. Handle partial matches (missing/occluded holes)

### Robustness Features
- RANSAC-based correspondence rejection
- Adaptive distance thresholds
- Minimum correspondence requirement (≥50%)

## Stage 3: Board Pose Refinement

### Purpose
Final high-precision alignment using all available points.

### Algorithm
```rust
// GICP with full covariance estimation
let config = IcpStageConfig {
    registration_type: RegistrationType::GICP,
    max_iterations: 100,
    convergence_threshold: 1e-6,
    num_threads: 4,
    voxel_resolution: Some(0.01), // 1cm voxels
};
```

### GICP Advantages
- Models local surface structure via covariance
- More robust to noise and outliers
- Provides uncertainty estimates

### CUDA Acceleration
```rust
if config.enable_cuda && cuda_available() {
    // Use GPU-accelerated GICP
    register_gicp_cuda(source, target, config)
} else {
    // Fallback to CPU implementation
    register_gicp_cpu(source, target, config)
}
```

## Stage 4: Temporal Alignment

### Purpose
Smooth tracking between frames using previous detections.

### Algorithm
```rust
// VGICP for efficient large cloud registration
let config = IcpStageConfig {
    registration_type: RegistrationType::VGICP,
    max_iterations: 30,
    convergence_threshold: 1e-4,
    voxel_resolution: Some(0.02), // 2cm voxels
};
```

### Temporal Smoothing
```rust
pub fn apply_temporal_smoothing(
    current: &Isometry3<f64>,
    previous: &Isometry3<f64>,
    smoothing_factor: f64,
) -> Isometry3<f64> {
    // Quaternion slerp for rotation
    let smoothed_rot = previous.rotation
        .quaternion()
        .slerp(&current.rotation.quaternion(), 1.0 - smoothing_factor);
    
    // Linear interpolation for translation
    let smoothed_trans = previous.translation.vector
        .lerp(&current.translation.vector, 1.0 - smoothing_factor);
    
    Isometry3::from_parts(smoothed_trans.into(), smoothed_rot.into())
}
```

## Configuration System

### Hierarchical Configuration
```rust
pub struct IcpConfig {
    pub enable_square_refinement: bool,
    pub enable_hole_alignment: bool,
    pub enable_board_refinement: bool,
    pub enable_temporal_alignment: bool,
    
    pub square_config: IcpStageConfig,
    pub hole_config: IcpStageConfig,
    pub board_config: IcpStageConfig,
    pub temporal_config: IcpStageConfig,
    
    pub global_settings: GlobalIcpSettings,
}

pub struct GlobalIcpSettings {
    pub num_threads: usize,
    pub enable_cuda: bool,
    pub fallback_to_cpu: bool,
    pub cache_kd_trees: bool,
}
```

### Configuration Builder
```rust
let icp_config = IcpConfigBuilder::new()
    .with_cuda(true)
    .with_threads(8)
    .square_refinement(|cfg| {
        cfg.max_iterations(20)
           .convergence_threshold(1e-4)
    })
    .board_refinement(|cfg| {
        cfg.registration_type(RegistrationType::GICP)
           .max_iterations(100)
    })
    .build()?;
```

## Performance Optimization

### Multi-Threading Strategy
1. **Parallel Correspondence Search**: OpenMP acceleration
2. **Batch Processing**: Process multiple boards in parallel
3. **Shared KD-Trees**: Reuse spatial indices across stages

### Memory Optimization
1. **Voxel Filtering**: Reduce point count while preserving geometry
2. **Index-Based Operations**: Avoid copying point data
3. **Pooled Allocators**: Reuse buffers across frames

### Caching Strategy
```rust
pub struct IcpCache {
    kd_trees: HashMap<u64, KdTree>,
    voxel_maps: HashMap<u64, VoxelMap>,
    last_transform: Option<Isometry3<f64>>,
}
```

## Error Handling

### Convergence Monitoring
```rust
pub struct ConvergenceMonitor {
    iteration_count: usize,
    error_history: Vec<f64>,
    improvement_threshold: f64,
}

impl ConvergenceMonitor {
    pub fn has_converged(&self) -> bool {
        if self.error_history.len() < 2 {
            return false;
        }
        
        let improvement = (self.error_history[n-2] - self.error_history[n-1]) 
                         / self.error_history[n-2];
        
        improvement < self.improvement_threshold
    }
}
```

### Fallback Strategy
1. **Stage Failure**: Skip failed stage, continue with reduced accuracy
2. **CUDA Failure**: Automatic CPU fallback
3. **Timeout**: Return best result so far

## Quality Metrics

### Per-Stage Metrics
```rust
pub struct IcpMetrics {
    pub initial_error: f64,
    pub final_error: f64,
    pub iterations: usize,
    pub convergence_rate: f64,
    pub inlier_ratio: f64,
    pub processing_time: Duration,
}
```

### Overall Quality Assessment
```rust
pub struct RefinementQuality {
    pub position_uncertainty: f64,  // meters
    pub rotation_uncertainty: f64,  // radians
    pub fitness_score: f64,         // 0-1
    pub confidence: f64,            // 0-1
}
```

## Future Enhancements

### Planned Improvements
1. **Adaptive Stage Selection**: Skip stages based on data quality
2. **Learning-Based Initialization**: ML-predicted initial poses
3. **Multi-Resolution ICP**: Coarse-to-fine registration
4. **Robust Kernels**: M-estimators for outlier handling

### Research Directions
1. **Probabilistic ICP**: Full uncertainty propagation
2. **Semantic ICP**: Use semantic labels for correspondence
3. **Neural ICP**: Learned correspondence and weighting
4. **Active ICP**: Adaptive sampling for efficiency