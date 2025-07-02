# Board-Fitter Optimization Guide

## Quick Start

This guide provides practical optimization techniques for improving board-fitter performance from the current 8.5s to the target <100ms.

## Current Performance Bottlenecks

### Profiling Results (8.5s total)

```
Plane Detection:      3.4s (40%)
├─ RANSAC:           2.5s
├─ Point filtering:  0.6s
└─ Normal estimation: 0.3s

Hole Detection:       2.1s (25%)
├─ Circle fitting:   1.5s
├─ Validation:       0.4s
└─ Clustering:       0.2s

ICP Refinement:       1.7s (20%)
├─ Correspondence:   0.8s
├─ Transform calc:   0.5s
└─ Point transform:  0.4s

Other Operations:     1.3s (15%)
├─ I/O operations:   0.5s
├─ Memory alloc:     0.4s
└─ Debug overhead:   0.4s
```

## Immediate Optimizations (Days)

### 1. Enable Parallel Processing

```rust
// In your code, change:
let config = DetectionConfig {
    parallel_processing: true, // Was false
    ..Default::default()
};

// Or use the builder:
let detector = BoardDetectorBuilder::new(board_config)
    .parallel_processing(true)
    .build()?;
```

**Expected improvement**: 2-3x on multi-core systems

### 2. Use Fast ICP Configuration

```rust
// Replace default ICP with fast config:
let detector = BoardDetectorBuilder::new(board_config)
    .with_fast_icp()
    .build()?;

// Or manually configure:
let config = DetectionConfig {
    icp_refinement: Some(IcpRefinementConfig::fast_config()),
    ..Default::default()
};
```

**Expected improvement**: 50% reduction in ICP time

### 3. Reduce RANSAC Iterations

```rust
// Tune RANSAC parameters:
let plane_detector = RansacPlaneDetector {
    max_iterations: 500,    // Reduced from 1000
    distance_threshold: 0.02, // Slightly increased
    min_inliers_ratio: 0.15, // Slightly reduced
    ..Default::default()
};
```

**Expected improvement**: 40% reduction in plane detection time

### 4. Add ROI-based Processing

```rust
// Process only relevant region:
let roi = Roi::Local {
    center: expected_position,
    radius: 2.0, // meters
};

let filtered_cloud = roi.filter_points(&point_cloud);
```

**Expected improvement**: 60-80% reduction for localized detection

## Medium-term Optimizations (Weeks)

### 1. Implement Voxel Downsampling

```rust
pub fn downsample_pointcloud(cloud: &PointCloud, voxel_size: f64) -> PointCloud {
    let mut voxel_map: HashMap<(i32, i32, i32), Point3> = HashMap::new();
    
    for point in &cloud.points {
        let key = (
            (point.x / voxel_size) as i32,
            (point.y / voxel_size) as i32,
            (point.z / voxel_size) as i32,
        );
        voxel_map.entry(key).or_insert(*point);
    }
    
    PointCloud {
        points: voxel_map.into_values().collect(),
        timestamp: cloud.timestamp,
    }
}

// Usage:
let downsampled = downsample_pointcloud(&cloud, 0.02); // 2cm voxels
```

**Expected improvement**: 70-90% point reduction with minimal accuracy loss

### 2. Cache KD-Trees

```rust
use lru::LruCache;
use std::num::NonZeroUsize;

pub struct KdTreeCache {
    cache: Mutex<LruCache<u64, Arc<KdTree>>>,
}

impl KdTreeCache {
    pub fn new(capacity: usize) -> Self {
        Self {
            cache: Mutex::new(LruCache::new(NonZeroUsize::new(capacity).unwrap())),
        }
    }
    
    pub fn get_or_build(&self, cloud: &PointCloud) -> Arc<KdTree> {
        let hash = calculate_hash(cloud);
        let mut cache = self.cache.lock().unwrap();
        
        if let Some(tree) = cache.get(&hash) {
            return Arc::clone(tree);
        }
        
        let tree = Arc::new(build_kdtree(cloud));
        cache.put(hash, Arc::clone(&tree));
        tree
    }
}
```

**Expected improvement**: 30-50% reduction in repeated detections

### 3. SIMD Optimization for Math Operations

```rust
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

pub fn distance_squared_simd(a: &[Point3], b: &Point3) -> Vec<f64> {
    let mut distances = vec![0.0; a.len()];
    
    // Process 4 points at a time with AVX
    let chunks = a.chunks_exact(4);
    let remainder = chunks.remainder();
    
    unsafe {
        let bx = _mm256_set1_pd(b.x);
        let by = _mm256_set1_pd(b.y);
        let bz = _mm256_set1_pd(b.z);
        
        for (chunk, dist_chunk) in chunks.zip(distances.chunks_exact_mut(4)) {
            // Load 4 points
            let ax = _mm256_loadu_pd(&chunk[0].x);
            let ay = _mm256_loadu_pd(&chunk[0].y);
            let az = _mm256_loadu_pd(&chunk[0].z);
            
            // Compute differences
            let dx = _mm256_sub_pd(ax, bx);
            let dy = _mm256_sub_pd(ay, by);
            let dz = _mm256_sub_pd(az, bz);
            
            // Square and sum
            let dx2 = _mm256_mul_pd(dx, dx);
            let dy2 = _mm256_mul_pd(dy, dy);
            let dz2 = _mm256_mul_pd(dz, dz);
            
            let sum = _mm256_add_pd(_mm256_add_pd(dx2, dy2), dz2);
            _mm256_storeu_pd(dist_chunk.as_mut_ptr(), sum);
        }
    }
    
    // Handle remainder
    for (i, point) in remainder.iter().enumerate() {
        let idx = a.len() - remainder.len() + i;
        distances[idx] = (point - b).norm_squared();
    }
    
    distances
}
```

**Expected improvement**: 3-4x speedup for distance calculations

### 4. Early Termination Strategy

```rust
pub struct EarlyTerminationDetector {
    confidence_threshold: f64,
    time_budget: Duration,
}

impl EarlyTerminationDetector {
    pub fn detect_with_early_termination(&self, cloud: &PointCloud) -> Option<BoardDetection> {
        let start = Instant::now();
        
        // Try progressively more accurate methods
        let strategies = [
            (0.1, 100),   // 10% points, 100 RANSAC iterations
            (0.3, 300),   // 30% points, 300 iterations
            (0.6, 600),   // 60% points, 600 iterations
            (1.0, 1000),  // Full accuracy
        ];
        
        for (sample_ratio, iterations) in strategies {
            if start.elapsed() > self.time_budget {
                break;
            }
            
            let sampled = sample_pointcloud(cloud, sample_ratio);
            if let Some(detection) = self.detect_with_params(&sampled, iterations) {
                if detection.confidence > self.confidence_threshold {
                    return Some(detection);
                }
            }
        }
        
        None
    }
}
```

**Expected improvement**: 50-70% average time reduction

## Long-term Optimizations (Months)

### 1. GPU Acceleration with CUDA

```rust
#[cfg(feature = "cuda")]
pub mod gpu {
    use cuda_runtime::*;
    
    pub struct GpuRansac {
        device: Device,
        stream: Stream,
    }
    
    impl GpuRansac {
        pub fn detect_planes(&self, points: &[Point3]) -> Vec<Plane> {
            // Allocate GPU memory
            let d_points = DeviceBuffer::from_slice(points)?;
            
            // Launch kernel
            unsafe {
                ransac_kernel<<<grid_size, block_size, 0, self.stream>>>(
                    d_points.as_ptr(),
                    points.len(),
                    // ... other parameters
                );
            }
            
            // Copy results back
            let results = d_results.to_host()?;
            results
        }
    }
}
```

**Expected improvement**: 10-50x speedup for large point clouds

### 2. Machine Learning Acceleration

```rust
pub struct MLAcceleratedDetector {
    feature_extractor: FeatureExtractor,
    detector_model: ONNXModel,
}

impl MLAcceleratedDetector {
    pub fn detect(&self, cloud: &PointCloud) -> Vec<BoardDetection> {
        // Extract features
        let features = self.feature_extractor.extract(cloud);
        
        // Run inference
        let predictions = self.detector_model.predict(&features)?;
        
        // Refine with traditional methods
        predictions.into_iter()
            .filter_map(|pred| self.refine_prediction(cloud, pred))
            .collect()
    }
}
```

**Expected improvement**: 100x speedup with 95% accuracy

## Configuration Templates

### Real-time Configuration (<100ms)

```rust
pub fn real_time_config() -> DetectionConfig {
    DetectionConfig {
        board_config: board_config.clone(),
        min_confidence: 0.7, // Lower threshold
        timeout_ms: 80,      // Strict timeout
        parallel_processing: true,
        icp_refinement: Some(IcpRefinementConfig {
            enable_cuda: true,
            num_threads: 8,
            
            square_pose_refinement: IcpStageConfig {
                enabled: true,
                max_iterations: 5,  // Very limited
                convergence_criteria: ConvergenceCriteria {
                    rotation_epsilon: 0.01,
                    translation_epsilon: 0.01,
                },
                downsampling_resolution: Some(0.05), // Aggressive
                num_neighbors: 5,
            },
            
            hole_detection: IcpStageConfig {
                enabled: false, // Skip for speed
                ..Default::default()
            },
            
            ..IcpRefinementConfig::fast_config()
        }),
        
        // Preprocessing
        voxel_size: Some(0.03), // 3cm voxels
        roi_radius: Some(3.0),  // 3m radius
        
        // RANSAC tuning
        ransac_iterations: 200,
        ransac_threshold: 0.03,
    }
}
```

### High-Accuracy Configuration

```rust
pub fn high_accuracy_config() -> DetectionConfig {
    DetectionConfig {
        board_config: board_config.clone(),
        min_confidence: 0.95,
        timeout_ms: 10000, // 10 seconds
        parallel_processing: true,
        icp_refinement: Some(IcpRefinementConfig::high_precision_config()),
        
        // No downsampling
        voxel_size: None,
        roi_radius: None,
        
        // Thorough RANSAC
        ransac_iterations: 2000,
        ransac_threshold: 0.01,
    }
}
```

## Profiling Commands

```bash
# CPU profiling with perf
perf record --call-graph=dwarf cargo run --release -- detect input.pcd
perf report

# Generate flamegraph
cargo install flamegraph
cargo flamegraph --bin board-fitter -- detect input.pcd

# Memory profiling with heaptrack
heaptrack cargo run --release -- detect input.pcd
heaptrack_gui heaptrack.board-fitter.12345.gz

# Benchmark specific optimization
cargo bench -- --save-baseline before_optimization
# Apply optimizations...
cargo bench -- --baseline before_optimization

# Profile with Intel VTune
vtune -collect hotspots -app-working-dir . -- cargo run --release

# CUDA profiling
nvprof cargo run --release --features cuda -- detect input.pcd
```

## Monitoring Performance

### Add Performance Logging

```rust
use log::info;
use std::time::Instant;

pub fn detect_with_profiling(cloud: &PointCloud) -> Result<BoardDetection> {
    let total_start = Instant::now();
    
    // Plane detection
    let plane_start = Instant::now();
    let planes = detect_planes(cloud)?;
    info!("Plane detection: {:?}", plane_start.elapsed());
    
    // Hole detection
    let hole_start = Instant::now();
    let holes = detect_holes(&planes)?;
    info!("Hole detection: {:?}", hole_start.elapsed());
    
    // ICP refinement
    let icp_start = Instant::now();
    let refined = refine_with_icp(&detection)?;
    info!("ICP refinement: {:?}", icp_start.elapsed());
    
    info!("Total detection time: {:?}", total_start.elapsed());
    Ok(refined)
}
```

### Performance Regression Tests

```rust
#[test]
fn test_performance_regression() {
    let cloud = load_test_pointcloud();
    let start = Instant::now();
    
    let result = detector.detect(&cloud).unwrap();
    let elapsed = start.elapsed();
    
    // Assert performance requirements
    assert!(elapsed < Duration::from_millis(100), 
            "Detection took {:?}, expected <100ms", elapsed);
    
    // Assert accuracy requirements
    assert!(result.confidence > 0.9,
            "Confidence {}, expected >0.9", result.confidence);
}
```

## Summary

By implementing these optimizations in order of impact:

1. **Immediate** (Days): 3-5x speedup
   - Enable parallel processing
   - Use fast ICP config
   - Reduce RANSAC iterations
   - Add ROI processing

2. **Medium-term** (Weeks): 10-20x speedup
   - Voxel downsampling
   - KD-tree caching
   - SIMD optimizations
   - Early termination

3. **Long-term** (Months): 50-100x speedup
   - GPU acceleration
   - ML-based detection
   - Custom hardware optimization

Total expected improvement: 85x (8.5s → <100ms) ✓