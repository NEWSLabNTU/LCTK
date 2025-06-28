# Board Fitter Architecture and Design

## Overview

The `board-fitter` library is designed to detect diamond-oriented square calibration boards with circular holes in LiDAR point cloud data. The library follows a modular pipeline architecture that processes point clouds through multiple detection stages.

## Design Principles

### 1. Modular Pipeline Architecture
The detection pipeline is broken into discrete, testable modules:
- **Plane Detection**: Identifies planar surfaces using RANSAC
- **Diamond Square Fitting**: Fits diamond-oriented squares to planar regions
- **Hole Detection**: Locates circular holes within diamond squares
- **Pattern Matching**: Validates detected patterns against expected board geometry
- **Tracking**: Maintains temporal consistency across frames

### 2. Zero-Overhead Debug System
- Callback-based instrumentation that can be completely compiled out
- No performance impact in release builds
- Comprehensive debug information for development and testing

### 3. Configurable Detection Parameters
- All detection thresholds and parameters are configurable
- Separate configuration for different detection stages
- Runtime parameter adjustment for different environments

### 4. Memory-Efficient Processing
- Voxel-based downsampling for large point clouds
- Adaptive ROI (Region of Interest) management
- Stream-based processing to minimize memory footprint

## Architecture Components

### Core Detection Pipeline

```
Point Cloud Input
       ↓
[ROI Management] ← Voxel filtering, adaptive preprocessing
       ↓
[Plane Detection] ← RANSAC-based plane fitting
       ↓
[Diamond Square Fitting] ← Convex hull + PCA analysis
       ↓                      ↓
       ↓              [Square Pose ICP Refinement] ← Optional pose refinement
       ↓
[Hole Detection] ← Intensity + geometric analysis
       ↓                      ↓
       ↓              [Hole Pattern ICP Alignment] ← Optional pattern refinement
       ↓
[Coordinate Transform] ← 2D plane → 3D board coordinates
       ↓
[Board Pose ICP Refinement] ← small_gicp final alignment (CPU/CUDA)
       ↓
[Pattern Matching] ← Geometric validation
       ↓
[Board Tracking] ← Kalman filter + Hungarian algorithm
       ↓                      ↓
       ↓              [Temporal ICP Alignment] ← Optional frame-to-frame refinement
       ↓
Detection Results
```

### Module Dependencies

```
detection.rs (main pipeline)
    ├── types.rs (core data structures)
    ├── plane.rs (RANSAC plane detection)
    ├── diamond.rs (square fitting + optional ICP refinement)
    ├── hole.rs (hole detection + optional ICP alignment)
    ├── refinement.rs (small_gicp integration module)
    │   ├── board_pose_refinement (final pose refinement)
    │   ├── square_pose_refinement (PCA result refinement)
    │   ├── hole_pattern_alignment (pattern matching refinement)
    │   └── temporal_alignment (frame-to-frame refinement)
    ├── roi.rs (ROI management)
    ├── tracking.rs (Kalman filter + optional ICP tracking)
    └── debug.rs (instrumentation system)
```

## Key Design Decisions

### 1. Diamond Orientation Constraint
**Decision**: Only detect boards tilted 30-150° from horizontal
**Rationale**: 
- Diamond boards viewed horizontally appear as regular squares
- Tilted orientation provides geometric constraints for robust detection
- Reduces false positives from other square objects

### 2. Multi-Method Hole Detection
**Decision**: Combine intensity-based and geometric hole detection
**Rationale**:
- Intensity method fast but may miss holes in sparse data
- Geometric method robust but computationally expensive
- Hybrid approach maximizes detection reliability

### 3. Coordinate Transform Strategy with ICP Refinement
**Decision**: Use small_gicp_rust with optional CUDA acceleration for high-precision refinement
**Rationale**:
- Initial transform from 2D plane to 3D board coordinates provides rough alignment
- Small_gicp GICP (Generalized ICP) achieves sub-centimeter accuracy
- Dual processing modes for flexibility:
  - CPU mode: Multi-threaded (OpenMP/TBB) for broad compatibility (<20ms per board)
  - CUDA mode: GPU acceleration for high-performance systems (<10ms per board)
- Point-to-plane ICP with covariance estimation improves convergence
- Robust kernels (Huber/Cauchy) handle outliers in noisy data
- Refined transforms enable precise pattern matching validation

### 4. Kalman Filter Tracking
**Decision**: Use Kalman filter with Hungarian algorithm for multi-board tracking
**Rationale**:
- Temporal consistency improves detection reliability
- Handles partial occlusion and detection failures
- Hungarian algorithm optimally assigns detections to tracks
- Predictive capability for occlusion handling

### 5. Occupancy Grid Representation
**Decision**: Convert point clouds to 2D occupancy grids for hole detection
**Rationale**:
- Simplifies geometric analysis on planar surfaces
- Enables efficient circular pattern detection
- Handles varying point cloud densities
- Allows reuse of 2D computer vision algorithms

### 6. Comprehensive ICP Integration Strategy
**Decision**: Integrate small_gicp_rust at multiple pipeline stages for maximum accuracy
**Rationale**:
- Multiple refinement points address different sources of error
- Each stage has specific ICP configuration optimized for its purpose
- Optional stages allow performance/accuracy tradeoffs
- Flexible deployment with CPU/CUDA support at each stage

**Integration Points**:

#### 6.1 Square Pose Refinement (Post-PCA)
- **Purpose**: Refine initial PCA-based square orientation
- **Input**: PCA-fitted square points, expected square model
- **Algorithm**: Point-to-plane ICP with planar DOF restriction
- **Benefit**: Corrects PCA bias in sparse or noisy data

#### 6.2 Hole Pattern Alignment
- **Purpose**: Align detected holes with expected pattern
- **Input**: Detected hole positions, reference hole pattern
- **Algorithm**: Point-to-point ICP with rigid transformation
- **Benefit**: Robust pattern matching with partial hole visibility

#### 6.3 Board Pose Final Refinement
- **Purpose**: Final high-precision pose estimation
- **Input**: Complete board region, ideal board model
- **Algorithm**: GICP with covariance estimation
- **Benefit**: Sub-centimeter accuracy for calibration

#### 6.4 Temporal Tracking Refinement
- **Purpose**: Smooth tracking between frames
- **Input**: Current and previous board detections
- **Algorithm**: VGICP for efficient large cloud alignment
- **Benefit**: Consistent tracking, reduced jitter

**Implementation Details**:
- Modular refinement functions in `refinement.rs`
- Per-stage configuration in `IcpRefinementConfig`
- Compile-time feature flag `cuda` enables GPU acceleration
- Runtime stage enable/disable for performance tuning
- Shared point cloud preprocessing pipeline
- Configurable convergence criteria per stage

## Algorithm Choices

### Plane Detection: RANSAC
- **Pros**: Robust to outliers, handles multiple planes
- **Cons**: Iterative, parameter-sensitive
- **Alternative considered**: Direct plane fitting (rejected due to outlier sensitivity)

### Circle Fitting: Multiple Methods
1. **Least Squares**: Fast, good for clean data
2. **RANSAC**: Robust to outliers
3. **Three-Point**: Exact solution for minimal data
- **Strategy**: Try methods in order of speed vs. robustness

### Square Fitting: Convex Hull + PCA
- **Convex Hull**: Finds boundary of planar region
- **PCA**: Determines principal axes for orientation
- **Geometric Validation**: Checks square properties (side ratios, angles)

### ICP Refinement: small_gicp_rust with Optional CUDA
- **Algorithm Options**:
  - ICP: Basic point-to-point
  - Plane ICP: Point-to-plane for planar surfaces
  - GICP: Generalized ICP with covariance estimation
  - VGICP: Voxelized GICP for large point clouds
- **Features**:
  - Dual processing modes:
    - CPU: Multi-threaded (OpenMP/TBB backends)
    - CUDA: GPU-accelerated for supported hardware
  - Robust kernels (Huber, Cauchy) for outlier rejection
  - DOF restrictions for constrained alignment
  - Configurable preprocessing pipeline
  - Extended results with information matrix
- **Performance**: 
  - CPU Mode: Sub-centimeter accuracy in <20ms (4 threads)
  - CUDA Mode: Sub-centimeter accuracy in <10ms (GPU)
  - Handles 100K+ points efficiently with VGICP
  - Memory: ~112 bytes/point (CPU), GPU memory managed internally
- **Integration**: 
  - Uses board plane normal for DOF restriction
  - Applies robust kernel for noisy LiDAR data
  - Returns transformation with uncertainty estimates
  - Automatic fallback to CPU if CUDA unavailable

### Pattern Matching: Asymmetric Layout
- **Design**: 3 holes in asymmetric pattern for unique orientation
- **Validation**: Checks hole positions after ICP refinement
- **Confidence**: ICP fitness score integrated into confidence calculation
- **Accuracy**: Sub-centimeter hole position validation enabled by ICP

## Error Handling Strategy

### 1. Graceful Degradation
- Continue processing with partial data when possible
- Return confidence levels for all detections
- Allow configurable tolerance levels

### 2. Comprehensive Error Types
```rust
pub enum DetectionError {
    InsufficientData,
    PlaneDetectionFailed,
    SquareFittingFailed,
    HoleDetectionFailed,
    IcpRefinementFailed,
    PatternValidationFailed,
    CoordinateTransformFailed,
    IcpPreprocessingFailed,
    CudaInitializationFailed,
}
```

### 3. Debug Information
- Full pipeline state capture on failures
- Intermediate result visualization
- Performance timing for bottleneck identification

## Configuration Architecture

### Hierarchical Configuration
```rust
pub struct BoardFitterConfig {
    pub plane_detection: PlaneDetectionConfig,
    pub diamond_fitting: DiamondFittingConfig,
    pub hole_detection: HoleDetectionConfig,
    pub icp_refinement: IcpRefinementConfig,
    pub pattern_matching: PatternMatchingConfig,
    pub tracking: TrackingConfig,
    pub roi_management: RoiConfig,
}

pub struct IcpRefinementConfig {
    pub enable_cuda: bool,                  // Use CUDA if available
    pub cuda_device_id: Option<i32>,        // Specific GPU device
    pub fallback_to_cpu: bool,              // Auto-fallback if CUDA fails
    pub num_threads: usize,                 // For CPU mode
    
    // Stage-specific configurations
    pub square_pose_refinement: IcpStageConfig,
    pub hole_pattern_alignment: IcpStageConfig,
    pub board_pose_refinement: IcpStageConfig,
    pub temporal_alignment: IcpStageConfig,
}

pub struct IcpStageConfig {
    pub enabled: bool,                      // Enable this refinement stage
    pub registration_type: RegistrationType, // ICP, PlaneICP, GICP, VGICP
    pub max_iterations: u32,
    pub convergence_criteria: ConvergenceCriteria,
    pub downsampling_resolution: Option<f64>,
    pub num_neighbors: usize,               // For normal estimation
    pub robust_kernel: Option<RobustKernelType>,
    pub dof_restriction: Option<DofRestriction>,
    pub use_covariance_estimation: bool,
}
```

### Environment-Specific Profiles
- **Indoor**: Tight tolerances, high quality data
- **Outdoor**: Relaxed tolerances, noise robustness
- **Long Range**: Large search areas, coarse resolution
- **Close Range**: Small search areas, fine resolution
- **High Performance**: CUDA enabled for real-time processing
- **Compatibility**: CPU-only mode for broad deployment

## Performance Considerations

### Computational Complexity
- **Plane Detection**: O(n) per RANSAC iteration
- **Diamond Fitting**: O(k log k) for convex hull of k points
- **Hole Detection**: O(w×h) for occupancy grid of w×h cells
- **ICP Refinement**: O(n log n) - CPU multi-threaded or GPU parallel
- **Pattern Matching**: O(1) geometric calculations
- **Tracking**: O(n²) Hungarian algorithm for n boards

### Memory Usage
- **Point Cloud**: O(n) for n points
- **Occupancy Grid**: O(w×h) for w×h grid resolution
- **ICP State**: O(n) for point clouds + KdTree + covariances (+ GPU buffers if CUDA)
- **Tracking State**: O(m) for m tracked boards
- **Debug Information**: O(pipeline_stages) when enabled

### Optimization Strategies
1. **Early Termination**: Stop pipeline on clear failures
2. **Adaptive Resolution**: Adjust grid resolution based on distance
3. **ROI Limiting**: Process only relevant regions
4. **Parallel Processing**: Multi-threaded plane detection + parallel ICP
5. **Memory Pooling**: Reuse allocations across frames
6. **Thread Pool Management**: Persistent thread pools for CPU mode
7. **GPU Memory Management**: Persistent CUDA contexts when enabled
8. **Preprocessing Optimization**: Parallel normal/covariance estimation
9. **Voxelized Processing**: VGICP for large point clouds
10. **Hybrid Pipeline**: CPU preprocessing with GPU ICP when available
11. **Selective Refinement**: Enable ICP stages based on confidence thresholds
12. **Cascaded Processing**: Fast ICP → Precise GICP for critical detections

## Expected Benefits of Multi-Stage ICP Integration

### Accuracy Improvements
| Detection Aspect | Without ICP | With ICP | Improvement |
|------------------|-------------|----------|-------------|
| Position Error | 60+ cm | <1 cm | 60x better |
| Orientation Error | 5-10° | <0.5° | 10-20x better |
| Hole Matching Rate | 40% | 90%+ | 2.25x better |
| Pattern Recognition | 60% success | 95%+ success | 1.6x better |
| Tracking Stability | High jitter | Smooth | Significant |

### Performance Impact
| Configuration | Pipeline Time (CPU) | Pipeline Time (CUDA) |
|---------------|--------------------|--------------------|
| No ICP | ~50ms | N/A |
| Final ICP only | ~70ms | ~55ms |
| All stages (CPU) | ~120ms | N/A |
| All stages (CUDA) | ~80ms | ~65ms |
| Selective stages | ~80-100ms | ~60-70ms |

### Use Case Recommendations
1. **High-Speed Applications**: Enable only final ICP refinement
2. **High-Precision Calibration**: Enable all ICP stages
3. **Real-Time Tracking**: Enable temporal ICP only
4. **Noisy Environments**: Enable square and hole ICP stages
5. **GPU Systems**: Enable all stages with CUDA acceleration

## Testing Strategy

### Unit Testing
- Individual algorithm validation
- Edge case handling
- Parameter sensitivity analysis
- Performance regression testing

### Integration Testing
- End-to-end pipeline validation
- Multi-board scenario testing
- Noise robustness testing
- Real-world data validation

### Benchmark Testing
- Performance characterization
- Memory usage profiling
- Latency measurements
- Comparison with baseline implementations

## Future Extensions

### Planned Enhancements
1. **Machine Learning Integration**: Neural network hole detection
2. **Multi-Sensor Fusion**: Camera + LiDAR detection
3. **Real-Time Processing**: Stream processing optimization
4. **Adaptive Parameters**: Self-tuning detection thresholds
5. **Board Type Variants**: Support for different calibration board geometries
6. **Advanced ICP Variants**: Colored ICP, feature-based ICP
7. **Multi-GPU Support**: Distributed processing for multiple boards

### API Stability
- Core detection interface designed for stability
- Configuration system allows parameter evolution
- Debug system extensible for new instrumentation
- Modular architecture supports component replacement

## ICP Refinement Implementation Example

### Cargo.toml Configuration

```toml
[dependencies]
small_gicp_rust = { version = "0.1", default-features = false }

[features]
default = ["cpu"]
cpu = ["small_gicp_rust/cpu"]
cuda = ["small_gicp_rust/cuda"]
```

### Integration with Board Detection Pipeline

```rust
use small_gicp_rust::{
    PointCloud, GaussianVoxelMap, 
    register_advanced, RegistrationSettings,
    RegistrationType, RobustKernel, DofRestriction,
    PreprocessorConfig, ExtendedRegistrationResult
};
use nalgebra::{Isometry3, Point3, Vector3};

pub struct IcpRefinement {
    config: IcpRefinementConfig,
    thread_pool: Option<ThreadPool>,
    cuda_available: bool,
}

impl IcpRefinement {
    pub fn new(config: IcpRefinementConfig) -> Self {
        let cuda_available = if config.enable_cuda {
            #[cfg(feature = "cuda")]
            {
                small_gicp_rust::cuda::is_available()
            }
            #[cfg(not(feature = "cuda"))]
            {
                false
            }
        } else {
            false
        };
        
        let thread_pool = if !cuda_available {
            Some(ThreadPool::new(config.num_threads))
        } else {
            None
        };
        
        Self {
            config,
            thread_pool,
            cuda_available,
        }
    }
    
    pub fn refine_board_pose(
        &self,
        board_points: &[Point3<f64>],     // Detected board region
        template_points: &[Point3<f64>],  // Ideal board template
        initial_transform: &Isometry3<f64>,
        plane_normal: &Vector3<f64>,
    ) -> Result<RefinementResult, DetectionError> {
        // Create point clouds
        let source = PointCloud::from_points(board_points)
            .map_err(|_| DetectionError::IcpPreprocessingFailed)?;
        let target = PointCloud::from_points(template_points)
            .map_err(|_| DetectionError::IcpPreprocessingFailed)?;
        
        // Preprocess with normal estimation
        let preprocess_config = PreprocessorConfig {
            downsampling_resolution: self.config.downsampling_resolution,
            num_neighbors: self.config.num_neighbors,
            num_threads: self.config.num_threads,
        };
        
        let source_processed = source.preprocess_points(&preprocess_config)?;
        let target_processed = target.preprocess_points(&preprocess_config)?;
        
        // Configure registration with CUDA awareness
        let mut settings = RegistrationSettings {
            registration_type: self.config.registration_type,
            num_threads: if self.cuda_available { 1 } else { self.config.num_threads },
            initial_guess: Some(*initial_transform),
        };
        
        // Enable CUDA if available
        #[cfg(feature = "cuda")]
        if self.cuda_available {
            settings.use_cuda = true;
            if let Some(device_id) = self.config.cuda_device_id {
                settings.cuda_device = device_id;
            }
        }
        
        // Set up robust kernel for outlier rejection
        let robust_kernel = match self.config.robust_kernel {
            Some(RobustKernelType::Huber(threshold)) => {
                Some(RobustKernel::huber(threshold)?)
            }
            _ => None,
        };
        
        // Apply DOF restriction based on plane normal
        let dof_restriction = if self.config.use_planar_constraint {
            Some(DofRestriction::planar_with_normal(plane_normal)?)
        } else {
            None
        };
        
        // Perform registration with automatic CUDA fallback
        let result = match register_advanced(
            &target_processed.cloud,
            &source_processed.cloud,
            &target_processed.tree,
            &settings,
            robust_kernel.as_ref(),
            dof_restriction.as_ref(),
            Some(*initial_transform),
        ) {
            Ok(res) => res,
            Err(e) if self.cuda_available && self.config.fallback_to_cpu => {
                // CUDA failed, fallback to CPU
                log::warn!("CUDA ICP failed, falling back to CPU: {:?}", e);
                settings.use_cuda = false;
                settings.num_threads = self.config.num_threads;
                
                register_advanced(
                    &target_processed.cloud,
                    &source_processed.cloud,
                    &target_processed.tree,
                    &settings,
                    robust_kernel.as_ref(),
                    dof_restriction.as_ref(),
                    Some(*initial_transform),
                )?
            }
            Err(e) => return Err(DetectionError::IcpRefinementFailed),
        };
        
        // Validate result
        if !result.converged || result.error > self.config.max_error {
            return Err(DetectionError::IcpRefinementFailed);
        }
        
        Ok(RefinementResult {
            transformation: result.transformation,
            fitness: 1.0 - result.error,
            num_inliers: result.num_inliers,
            covariance: result.information_matrix,
        })
    }
}

// Usage in board detection pipeline
impl BoardDetector {
    fn detect_and_refine(&self, point_cloud: &PointCloud) -> Result<Board> {
        // ... existing detection steps ...
        
        // After hole detection, refine with ICP
        let board_region = self.extract_board_points(&plane, &square)?;
        let template = self.generate_board_template(&detected_holes)?;
        
        let refined_transform = self.icp_refiner.refine_board_pose(
            &board_region,
            &template,
            &initial_transform,
            &plane.normal,
        )?;
        
        // Apply refined transform to holes
        let refined_holes = detected_holes.iter()
            .map(|hole| refined_transform.transformation * hole.position)
            .collect();
        
        // Validate pattern with refined positions
        self.validate_pattern(&refined_holes, refined_transform.fitness)?;
        
        Ok(Board {
            transform: refined_transform.transformation,
            holes: refined_holes,
            confidence: refined_transform.fitness,
        })
    }
}
```

### Build and Deployment Considerations

#### Building with CUDA Support
```bash
# CPU-only build (default)
cargo build --release

# CUDA-enabled build
cargo build --release --features cuda

# Both CPU and CUDA (runtime selection)
cargo build --release --features "cpu,cuda"
```

#### Runtime Configuration
```yaml
# config/board_fitter.yaml
icp_refinement:
  registration_type: GICP
  enable_cuda: true              # Try to use CUDA if available
  cuda_device_id: 0              # Specific GPU device
  fallback_to_cpu: true          # Automatic fallback
  num_threads: 4                 # For CPU mode
  robust_kernel:
    type: huber
    threshold: 0.1
  convergence_criteria:
    max_iterations: 50
    rotation_epsilon: 0.001
    translation_epsilon: 0.001
```

#### Performance Comparison
| Configuration | Point Cloud Size | Processing Time | Accuracy |
|---------------|------------------|-----------------|----------|
| CPU (4 threads) | 10K points | ~15ms | <1cm |
| CPU (8 threads) | 10K points | ~10ms | <1cm |
| CUDA (RTX 3080) | 10K points | ~3ms | <1cm |
| CUDA (RTX 3080) | 100K points | ~8ms | <1cm |
| CPU (8 threads) | 100K points | ~80ms | <1cm |

## Multi-Stage ICP Refinement Implementation

### Stage 1: Square Pose Refinement
```rust
impl DiamondSquareFitter {
    pub fn fit_square_with_icp_refinement(
        &self,
        points: &[Point3<f64>],
        plane: &PlaneModel,
        icp_refiner: &IcpRefinement,
    ) -> Result<Square, DetectionError> {
        // Initial PCA-based fitting
        let initial_square = self.fit_square_pca(points, plane)?;
        
        if !icp_refiner.config.square_pose_refinement.enabled {
            return Ok(initial_square);
        }
        
        // Generate ideal square model
        let square_template = self.generate_square_template(initial_square.size);
        
        // Refine with ICP
        let refined_transform = icp_refiner.refine_square_pose(
            points,
            &square_template,
            &initial_square.pose,
            &plane.normal,
        )?;
        
        Ok(Square {
            pose: refined_transform.transformation,
            size: initial_square.size,
            confidence: refined_transform.fitness,
        })
    }
}
```

### Stage 2: Hole Pattern Alignment
```rust
impl HoleDetector {
    pub fn align_hole_pattern(
        &self,
        detected_holes: &[Hole],
        expected_pattern: &HolePattern,
        icp_refiner: &IcpRefinement,
    ) -> Result<Vec<Hole>, DetectionError> {
        if !icp_refiner.config.hole_pattern_alignment.enabled {
            return Ok(detected_holes.to_vec());
        }
        
        // Convert holes to point cloud
        let hole_points: Vec<Point3<f64>> = detected_holes.iter()
            .map(|h| h.center)
            .collect();
        
        let pattern_points = expected_pattern.hole_positions();
        
        // Use point-to-point ICP for hole alignment
        let alignment = icp_refiner.align_hole_pattern(
            &hole_points,
            &pattern_points,
            None, // No initial guess for pattern matching
        )?;
        
        // Apply transformation to holes
        let aligned_holes = detected_holes.iter()
            .map(|hole| Hole {
                center: alignment.transformation * hole.center,
                radius: hole.radius,
                confidence: hole.confidence * alignment.fitness,
            })
            .collect();
        
        Ok(aligned_holes)
    }
}
```

### Stage 3: Temporal Tracking Refinement
```rust
impl BoardTracker {
    pub fn refine_temporal_alignment(
        &self,
        current_detection: &Board,
        previous_detection: &Board,
        icp_refiner: &IcpRefinement,
    ) -> Result<Board, DetectionError> {
        if !icp_refiner.config.temporal_alignment.enabled {
            return Ok(current_detection.clone());
        }
        
        // Use VGICP for efficient large cloud alignment
        let temporal_transform = icp_refiner.align_temporal(
            &current_detection.points,
            &previous_detection.points,
            Some(&self.motion_prediction), // Kalman filter prediction
        )?;
        
        // Smooth the transformation for stable tracking
        let smoothed_transform = self.smooth_transformation(
            &temporal_transform.transformation,
            &current_detection.pose,
            self.smoothing_factor,
        );
        
        Ok(Board {
            pose: smoothed_transform,
            holes: current_detection.holes.clone(),
            confidence: current_detection.confidence * temporal_transform.fitness,
            ..current_detection.clone()
        })
    }
}
```

### Configuration Example
```yaml
icp_refinement:
  enable_cuda: true
  fallback_to_cpu: true
  num_threads: 8
  
  square_pose_refinement:
    enabled: true
    registration_type: PlaneICP
    max_iterations: 20
    convergence_criteria:
      rotation_epsilon: 0.001
      translation_epsilon: 0.001
    dof_restriction: planar_3dof
    
  hole_pattern_alignment:
    enabled: true
    registration_type: ICP
    max_iterations: 30
    robust_kernel:
      type: huber
      threshold: 0.05
      
  board_pose_refinement:
    enabled: true
    registration_type: GICP
    max_iterations: 50
    use_covariance_estimation: true
    downsampling_resolution: 0.02
    
  temporal_alignment:
    enabled: true
    registration_type: VGICP
    max_iterations: 10
    downsampling_resolution: 0.05
```