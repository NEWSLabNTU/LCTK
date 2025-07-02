# Profiling and Optimization Design

## Overview

This document outlines the profiling infrastructure and optimization strategies for the board-fitter library. The current performance baseline is 8.5 seconds per detection, which needs to be optimized to meet the target of <100ms for real-time applications.

## Table of Contents

1. [Performance Requirements](#performance-requirements)
2. [Current Performance Profile](#current-performance-profile)
3. [Profiling Infrastructure](#profiling-infrastructure)
4. [Optimization Strategies](#optimization-strategies)
5. [Implementation Roadmap](#implementation-roadmap)
6. [Benchmarking Guidelines](#benchmarking-guidelines)
7. [Performance Monitoring](#performance-monitoring)

## Performance Requirements

### Target Metrics
- **Detection Latency**: <100ms per frame (85x improvement needed)
- **Throughput**: >10 fps for real-time applications
- **Point Cloud Size**: Handle 50,000-100,000 points efficiently
- **Memory Usage**: <1GB for typical scenarios
- **CPU Usage**: Utilize multi-core effectively (4-8 cores)
- **GPU Usage**: Optional acceleration for >10x speedup

### Performance Levels

| Level | Name | Target Latency | Accuracy Trade-off | Use Case |
|-------|------|----------------|-------------------|-----------|
| 0 | Full Accuracy | <500ms | None | Offline calibration |
| 1 | Balanced | <200ms | Minimal | Online calibration |
| 2 | Fast | <100ms | 5% accuracy loss | Real-time tracking |
| 3 | Very Fast | <50ms | 10% accuracy loss | Preview mode |
| 4 | Ultra Fast | <20ms | 20% accuracy loss | Coarse detection |

## Current Performance Profile

### Bottleneck Analysis

Based on profiling data, the major bottlenecks are:

1. **Plane Detection (40% of time)**
   - RANSAC with 1000 iterations
   - Full point cloud processing
   - No spatial indexing

2. **Hole Detection (25% of time)**
   - Brute-force circle fitting
   - No early termination
   - Redundant computations

3. **ICP Refinement (20% of time)**
   - Full correspondence search
   - No KD-tree caching
   - Serial processing

4. **Point Cloud Operations (15% of time)**
   - Unnecessary copies
   - No SIMD optimization
   - Inefficient memory layout

### Memory Profile

- **Peak Memory Usage**: ~500MB for 100k points
- **Major Allocations**:
  - Point cloud storage: 40%
  - KD-tree structures: 30%
  - Intermediate results: 20%
  - Debug data: 10%

## Profiling Infrastructure

### Debug System Architecture

```rust
pub struct DebugContext {
    config: DebugConfig,
    stage_timings: HashMap<String, Duration>,
    metrics: PerformanceMetrics,
    data_callback: Option<Arc<dyn DataCallback>>,
}

pub struct PerformanceMetrics {
    pub total_points_processed: usize,
    pub stage_metrics: HashMap<String, StageMetrics>,
    pub memory_usage: Option<MemoryStats>,
    pub cache_hits: CacheStats,
}
```

### Profiling Tools Integration

1. **Built-in Profiling**
   ```rust
   // Enable detailed profiling
   let debug_config = DebugConfigBuilder::new()
       .with_timing()
       .with_metrics()
       .with_memory_tracking()
       .capture_all_stages()
       .build();
   ```

2. **External Profilers**
   - **perf**: Linux performance counters
   - **flamegraph**: Visualization of hot paths
   - **valgrind/cachegrind**: Cache analysis
   - **Intel VTune**: Detailed CPU analysis
   - **NVIDIA Nsight**: GPU profiling

3. **Continuous Profiling**
   ```rust
   #[cfg(feature = "profile")]
   pub struct ContinuousProfiler {
       metrics_sink: MetricsSink,
       sampling_rate: Duration,
   }
   ```

### Benchmark Suite

1. **Micro-benchmarks**
   - Individual algorithm components
   - Data structure operations
   - Math primitives

2. **Macro-benchmarks**
   - Full pipeline performance
   - Real-world scenarios
   - Stress tests

3. **Regression Detection**
   - Automated performance tracking
   - Alert on >5% regression
   - Historical trend analysis

## Optimization Strategies

### 1. Algorithmic Optimizations

#### Spatial Indexing
```rust
pub struct SpatialIndex {
    octree: Octree<Point3>,
    grid: UniformGrid<Point3>,
    kdtree_cache: LruCache<u64, KdTree>,
}
```

#### Early Termination
- Confidence-based early exit
- Progressive refinement
- Adaptive sampling

#### Approximate Algorithms
- Approximate nearest neighbor (ANN)
- Randomized algorithms
- Heuristic-based pruning

### 2. Data Structure Optimizations

#### Memory Layout
```rust
// SoA (Structure of Arrays) for better cache locality
pub struct PointCloudSoA {
    x: Vec<f32>,
    y: Vec<f32>, 
    z: Vec<f32>,
}
```

#### Zero-Copy Operations
- Use indices instead of copying points
- Slice-based operations
- Memory-mapped files for large datasets

### 3. Parallelization Strategies

#### Multi-threading
```rust
use rayon::prelude::*;

// Parallel plane detection
planes.par_iter()
    .map(|plane| detect_board_in_plane(plane))
    .collect()
```

#### SIMD Vectorization
```rust
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

// Vectorized distance computation
unsafe fn distance_simd(a: &[f32], b: &[f32]) -> f32 {
    // AVX2 implementation
}
```

#### GPU Acceleration
```rust
#[cfg(feature = "cuda")]
pub struct GpuAccelerator {
    context: cuda::Context,
    kernels: HashMap<String, cuda::Function>,
}
```

### 4. Caching Strategies

#### Multi-level Cache
```rust
pub struct MultiLevelCache {
    l1_cache: LruCache<u64, CacheEntry>, // Hot data
    l2_cache: DiskCache<u64, CacheEntry>, // Warm data
    prefetcher: Prefetcher,
}
```

#### Cache-friendly Algorithms
- Tiled matrix operations
- Loop blocking
- Data prefetching

### 5. I/O Optimizations

#### Async I/O
```rust
pub async fn load_point_cloud_async(path: &Path) -> Result<PointCloud> {
    // Non-blocking I/O
}
```

#### Memory-mapped Files
```rust
pub struct MmapPointCloud {
    mmap: Mmap,
    header: PointCloudHeader,
}
```

## Implementation Roadmap

### Phase 1: Quick Wins (1-2 weeks)
1. Enable parallel processing by default
2. Implement basic KD-tree caching
3. Add early termination conditions
4. Optimize memory allocations

**Expected improvement**: 2-3x speedup

### Phase 2: Core Optimizations (3-4 weeks)
1. Implement spatial indexing (octree/grid)
2. Add SIMD optimizations for math operations
3. Optimize data structures (SoA layout)
4. Implement progressive refinement

**Expected improvement**: 5-10x speedup

### Phase 3: Advanced Features (4-6 weeks)
1. GPU acceleration for ICP and RANSAC
2. Multi-level caching system
3. Adaptive algorithms based on scene complexity
4. Real-time performance mode

**Expected improvement**: 20-50x speedup

### Phase 4: Production Hardening (2-3 weeks)
1. Performance regression tests
2. Continuous profiling infrastructure
3. Auto-tuning system
4. Documentation and examples

## Benchmarking Guidelines

### Running Benchmarks

```bash
# Run all benchmarks with baseline comparison
cargo bench -- --baseline main

# Profile with flamegraph
cargo flamegraph --bench detection_benchmark

# Run with specific features
cargo bench --features "cuda simd" 

# Long-running stress test
cargo bench -- --measurement-time 60
```

### Performance Testing Matrix

| Scenario | Point Count | Noise Level | Expected Time |
|----------|-------------|-------------|---------------|
| Small | 1,000 | 0mm | <10ms |
| Medium | 10,000 | 5mm | <50ms |
| Large | 100,000 | 10mm | <200ms |
| Stress | 1,000,000 | 20mm | <1000ms |

### Benchmark Report Format

```markdown
## Performance Report - <date>

### Summary
- Commit: <hash>
- Platform: <CPU/GPU info>
- Configuration: <optimization level>

### Results
| Benchmark | Time | vs Baseline | Memory |
|-----------|------|-------------|---------|
| detection_small | 8.2ms | -15% | 12MB |
| detection_large | 145ms | -22% | 89MB |

### Analysis
<Notable changes and recommendations>
```

## Performance Monitoring

### Metrics Collection

```rust
#[derive(Serialize)]
pub struct PerformanceReport {
    timestamp: DateTime<Utc>,
    version: String,
    metrics: {
        latency_p50: f64,
        latency_p95: f64,
        latency_p99: f64,
        throughput_fps: f64,
        memory_peak_mb: f64,
        cpu_usage_percent: f64,
    },
    profile: HashMap<String, StageTiming>,
}
```

### Alerting Rules

1. **Latency Alert**: p95 > 150ms
2. **Memory Alert**: Peak > 1GB
3. **CPU Alert**: Usage > 90% sustained
4. **Regression Alert**: Any metric >10% worse

### Dashboard Integration

```yaml
# Prometheus metrics
board_fitter_detection_duration_seconds{stage="plane_detection"} 0.125
board_fitter_points_processed_total{source="lidar"} 1234567
board_fitter_cache_hit_ratio{cache="kdtree"} 0.85
```

## Best Practices

### Code Optimization

1. **Profile First**: Always measure before optimizing
2. **Algorithmic First**: Better algorithm > micro-optimizations
3. **Memory Efficiency**: Minimize allocations and copies
4. **Cache Awareness**: Optimize for L1/L2 cache hits
5. **Parallelism**: Use all available cores effectively

### Performance Testing

1. **Realistic Data**: Test with production-like datasets
2. **Statistical Rigor**: Multiple runs, confidence intervals
3. **Regression Tracking**: Monitor performance over time
4. **Platform Testing**: Test on target hardware
5. **Feature Flags**: Allow runtime performance tuning

### Documentation

1. **Performance Characteristics**: Document O(n) complexity
2. **Configuration Guide**: Explain performance trade-offs
3. **Tuning Guide**: How to optimize for specific use cases
4. **Troubleshooting**: Common performance issues

## Conclusion

The board-fitter library has a solid foundation for performance optimization with existing profiling infrastructure and benchmark suite. By following this design and implementing the proposed optimizations, we can achieve the target <100ms detection latency while maintaining accuracy for real-time applications.

The key to success is incremental optimization with continuous measurement, starting with the highest-impact changes and progressively refining the implementation based on real-world performance data.