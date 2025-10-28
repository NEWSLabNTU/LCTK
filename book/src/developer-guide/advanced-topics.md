# Advanced Topics

Deep dive into calibration quality evaluation and performance optimization.

## Calibration Quality Evaluation

### IoU-Based Evaluator

**Purpose:** Measure extrinsic calibration accuracy in real-time using Intersection over Union (IoU) metrics.

**Location:** `src/ros2/calibration_evaluator/`

**How it works:**
1. **Ground truth**: Detect ArUco board region in camera image
2. **Projection**: Project LiDAR points to image using calibration
3. **IoU**: Compare ground truth region with projected points region

```python
IoU = |Ground Truth ∩ Projected| / |Ground Truth ∪ Projected|
```

### Using the Evaluator

```bash
# Launch with calibration pipeline
ros2 launch lctk_launch lidar_camera_calibration.launch.xml

# Monitor IoU score (0.0-1.0, higher is better)
ros2 topic echo /calibration/calibration_evaluator/iou_score

# View overlay visualization
ros2 run rqt_image_view rqt_image_view \
    /calibration/calibration_evaluator/overlay_image
```

**Overlay colors:**
- **Green**: Ground truth board region
- **Red**: Projected LiDAR points region
- **Yellow** (overlap): Correct calibration

### Interpreting Metrics

| IoU | Quality | Action |
|-----|---------|--------|
| >0.9 | Excellent | Ready to use |
| 0.7-0.9 | Good | Acceptable |
| 0.5-0.7 | Fair | Consider recalibration |
| <0.5 | Poor | Recalibrate |

**Additional metrics:**
- **Coverage**: `|Intersection| / |Ground Truth|` — How much of board is covered
- **Precision**: `|Intersection| / |Projected|` — How much projection is correct

### Configuration

```yaml
# config/calibration_evaluator.yaml
sync_queue_size: 10       # Message buffer size
sync_slop: 0.1            # Time sync tolerance (100ms)
use_best_effort_qos: true # For sensor data
```

### Troubleshooting Evaluator

**"No ArUco board detected":**
- Check ArUco detections: `ros2 topic echo /aruco_detections`
- Verify board is visible to camera

**"No valid points":**
- Check point cloud: `ros2 topic echo /pointcloud`
- Verify LiDAR sensor is working

**Low IoU despite good calibration:**
- Increase `sync_slop` (temporal misalignment)
- Check for motion during capture
- Verify board geometry configuration

## Performance Optimization

### Profiling

**CPU profiling:**
```bash
# Build with debug symbols
cargo build --release --features debug_symbols

# Profile with perf
perf record -g ros2 run my_node my_node
perf report

# Find hotspots
perf top
```

**Memory profiling:**
```bash
valgrind --tool=memcheck ros2 run my_node my_node
valgrind --tool=massif ros2 run my_node my_node
```

### Common Bottlenecks

**1. ICP Iterations**

Board detection uses ICP for pose refinement. Default: 10 iterations.

```json5
// config/board/board_detector.json5
{
  "max_icp_iterations": 5,  // Reduce for speed (was 10)
}
```

**Impact:** ~2x faster detection, slightly less accurate.

**2. RANSAC Iterations**

Plane fitting uses RANSAC. Default: 2000 iterations.

```json5
{
  "plane_ransac_max_iterations": 1000,  // Reduce for speed
}
```

**Impact:** Faster but may miss board in noisy data.

**3. Point Cloud Size**

Downsample large point clouds before processing.

```rust
use pcl::VoxelGrid;

let downsampled = voxel_grid_filter(&cloud, 0.01);  // 1cm voxels
```

**Impact:** ~10x faster with minimal accuracy loss.

### Parallel Processing

**Leverage multi-core CPUs:**

```bash
# Set Cargo to use all cores
export CARGO_BUILD_JOBS=$(nproc)

# Runtime parallelism (Rayon)
export RAYON_NUM_THREADS=$(nproc)
```

**In code (using Rayon):**
```rust
use rayon::prelude::*;

// Parallel iterator
let results: Vec<_> = points.par_iter()
    .map(|p| process_point(p))
    .collect();
```

### Memory Optimization

**Reuse buffers:**
```rust
pub struct Detector {
    buffer: Vec<Point3D>,  // Reuse across calls
}

impl Detector {
    pub fn detect(&mut self, cloud: &PointCloud) -> Result<Detection> {
        self.buffer.clear();  // Reuse allocation
        // Use self.buffer for temporary data
    }
}
```

**Use references to avoid copies:**
```rust
// GOOD: Zero-copy
fn process(data: &[u8]) { }

// BAD: Copies data
fn process(data: Vec<u8>) { }
```

### GPU Acceleration (Optional)

LCTK supports CUDA for GPU-accelerated operations:

```bash
# Build with CUDA support
export CUDA_PATH=/usr/local/cuda
./setup-dev-env.sh -y  # Includes CUDA

# Enable at runtime
export CUDA_VISIBLE_DEVICES=0
```

**GPU-accelerated operations:**
- Point cloud filtering
- Image processing (OpenCV CUDA backend)

## Lock-Free Concurrency

Use `arc-swap` for configuration updates without locks:

```rust
use arc_swap::ArcSwap;

// Shared config between threads
let config = Arc::new(ArcSwap::from_pointee(initial_config));

// Service updates config (no lock!)
service_handler: {
    let config = Arc::clone(&config);
    move |new_config| {
        config.store(Arc::new(new_config));  // Atomic swap
    }
}

// Detection thread reads without blocking
detection_thread: {
    let current = config.load();  // Fast read
    detector.detect_with_config(&current);
}
```

**Benefits:**
- No mutex contention
- Service handlers respond in <10ms
- Detection thread never blocks

## Debug Mode

Enable detailed logging and visualization:

```bash
ros2 launch lctk_launch lidar_camera_calibration.launch.xml debug_mode:=true
```

**Debug topics published:**
- `/calibration/debug/all_points`: All input points
- `/calibration/debug/filtered_points`: After bounding box filter
- `/calibration/debug/plane_inliers`: RANSAC plane inliers
- `/calibration/debug/plane_marker`: Circular plane visualization
- `/calibration/debug/initial_board_marker`: PCA-based initial pose
- `/calibration/debug/icp_iterations`: ICP refinement steps
- `/calibration/debug/final_board_pose`: Successful detection

**Logging:**
```bash
export RUST_LOG=debug
export RCUTILS_LOGGING_LEVEL=DEBUG
```

## Performance Benchmarks

Typical performance on 8-core CPU, 16GB RAM:

| Operation | Time | Rate |
|-----------|------|------|
| ArUco detection (1080p) | ~30ms | 33 Hz |
| Board detection (100k points) | ~100ms | 10 Hz |
| Extrinsic solver | ~10ms | 100 Hz |
| IoU evaluator | ~50ms | 20 Hz |

**Optimization targets:**
- Detection: >10 Hz (real-time)
- Calibration: >1 Hz (acceptable)

## Advanced Configuration

### ROI Optimization

Tune bounding box for faster detection:

```json5
// Tight bounding box = faster detection
{
  "center": [3.0, 0.0, 0.5],  // Exact board location
  "size": [2.0, 2.0, 1.0]     // Minimal coverage
}
```

### Adaptive Thresholds

Adjust detection sensitivity based on environment:

```json5
{
  // Noisy environment
  "plane_ransac_inlier_threshold": 0.08,  // More tolerant

  // Clean environment
  "plane_ransac_inlier_threshold": 0.03,  // Stricter
}
```

## Further Reading

- [Calibration Evaluator Paper](https://arxiv.org/abs/xyz) (if published)
- [OpenCV PnP Documentation](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
- [Small GICP Library](https://github.com/koide3/small_gicp)
- [Arc-Swap Documentation](https://docs.rs/arc-swap/)

## Next Steps

- [Reference](./reference.md) - Configuration schemas
- [Testing](./testing.md) - Performance testing
- [Contributing](./contributing.md) - Submit optimizations
