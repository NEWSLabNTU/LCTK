# Core Libraries

Core libraries in `src/lib/` are pure Rust with no ROS dependencies. Test with `cargo test`, use in any Rust project.

## Detection Libraries

### aruco-detector
**Location:** `src/lib/aruco-detector/`

Detects ArUco markers in images using OpenCV.

```rust
use aruco_detector::{ArUcoDetector, ArUcoConfig};

let config = ArUcoConfig::from_file("aruco_pattern.json5")?;
let detector = ArUcoDetector::new(config);
let detections = detector.detect(&image)?;
```

**Key types:**
- `ArUcoConfig`: Marker IDs, positions, dictionary
- `ArUcoDetection`: Corner coordinates, marker ID

**See also:** `aruco-config` for configuration types, `aruco-generator` for creating marker images.

### hollow-board-detector
**Location:** `src/lib/hollow-board-detector/`

Detects calibration boards with circular holes in point clouds.

**Pipeline:**
1. Bounding box filter
2. RANSAC plane detection (`plane-estimator`)
3. PCA-based pose initialization
4. ICP refinement

```rust
use hollow_board_detector::{BoardDetector, BoardConfig};

let config = BoardConfig::from_file("board_pattern.json5")?;
let detector = BoardDetector::new(config);
let detection = detector.detect(&point_cloud)?;
```

**Key types:**
- `BoardConfig`: Board dimensions, hole specifications
- `BoardDetection`: 3D pose, confidence, inlier count

**See also:** `hollow-board-config` for configuration types.

## Algorithm Libraries

### plane-estimator
**Location:** `src/lib/plane-estimator/`

RANSAC-based plane fitting for point clouds.

```rust
use plane_estimator::{PlaneEstimator, RansacConfig};

let config = RansacConfig {
    max_iterations: 2000,
    inlier_threshold: 0.05, // meters
    min_inlier_ratio: 0.5,
};

let estimator = PlaneEstimator::new(config);
let plane = estimator.estimate(&points)?;
```

**Returns:**
- Plane equation (nx, ny, nz, d)
- Inlier points
- Inlier ratio

### pnp-solver
**Location:** `src/lib/pnp-solver/`

Solves Perspective-n-Point problem for camera pose estimation.

```rust
use pnp_solver::{PnPSolver, SolverMethod};

let solver = PnPSolver::new(SolverMethod::SQPNP);
let pose = solver.solve(&points_2d, &points_3d, &camera_matrix)?;
```

**Supported methods:**
- `SQPNP`: Fast, accurate (default)
- `IPPE`: Planar targets
- `ITERATIVE`: Refinement for better accuracy

**Returns:** 6-DOF pose (rotation + translation)

## Configuration Libraries

### serde-types
**Location:** `src/lib/serde-types/`

Shared types with JSON5 serialization.

```rust
use serde_types::{BoundingBox, Transform3D};

#[derive(Serialize, Deserialize)]
pub struct Config {
    bbox: BoundingBox,
    transform: Transform3D,
}

let config: Config = serde_json5::from_str(&json_string)?;
```

**Common types:**
- `BoundingBox`: 3D region of interest
- `Transform3D`: Rotation + translation
- `CameraIntrinsics`: Focal length, principal point, distortion

## Point Cloud Libraries

### small_gicp_rust
**Location:** `src/lib/small_gicp_rust/`

Rust wrapper for small_gicp C++ library (Generalized ICP).

```rust
use small_gicp_rust::{align_point_clouds, ICPConfig};

let config = ICPConfig {
    max_iterations: 100,
    convergence_threshold: 1e-6,
};

let transform = align_point_clouds(&source, &target, config)?;
```

**Use cases:**
- Fine-tuning board pose detection
- Multi-LiDAR registration
- Point cloud alignment

## Library Dependencies

```
aruco-detector → opencv-rust
hollow-board-detector → plane-estimator, small_gicp_rust
plane-estimator → nalgebra
pnp-solver → opencv-rust
small_gicp_rust → bindgen (C++ wrapper)
```

## Testing Libraries

All libraries have unit tests:

```bash
# Test specific library
cargo test --manifest-path src/lib/aruco-detector/Cargo.toml

# Test all libraries
cargo test --workspace --lib
```

## API Documentation

Generate rustdoc:

```bash
cargo doc --open --no-deps
```

Full API reference available at: `target/doc/aruco_detector/index.html`

## Adding New Libraries

**Template structure:**
```
src/lib/my-library/
├── Cargo.toml
├── src/
│   ├── lib.rs       # Public API
│   ├── config.rs    # Configuration types
│   └── algorithm.rs # Core algorithm
└── tests/
    └── integration.rs
```

**Guidelines:**
1. Keep ROS-free (use in `src/bin/` wrappers)
2. Implement `Serialize`/`Deserialize` for configs
3. Add comprehensive tests
4. Document public API with rustdoc
5. Use `thiserror` for error types

## Configuration File Patterns

Libraries use JSON5 for human-friendly config:

```json5
{
  // Comments allowed!
  "iterations": 2000,
  "threshold": 0.05,
  "enabled": true
}
```

Load with:
```rust
#[derive(Deserialize)]
struct Config {
    iterations: usize,
    threshold: f64,
    enabled: bool,
}

let config: Config = serde_json5::from_reader(file)?;
```

## Performance Tips

- **Use release builds:** `cargo build --release` (10-100x faster)
- **Profile with `perf`:** Find bottlenecks
- **Leverage SIMD:** `nalgebra` provides vectorized operations
- **Minimize allocations:** Reuse buffers in hot loops

## Common Patterns

**Builder pattern for configuration:**
```rust
let detector = BoardDetector::builder()
    .max_iterations(5000)
    .inlier_threshold(0.03)
    .build()?;
```

**Result type for errors:**
```rust
pub type Result<T> = std::result::Result<T, Error>;

pub fn detect(&self, data: &Data) -> Result<Detection> {
    // ...
}
```

**Functional struct initialization:**
```rust
let config = Config {
    field1: value1,
    field2: value2,
    ..Default::default()
};
```

## Next Steps

- [ROS 2 Nodes](./ros2-nodes.md) - How libraries are wrapped for ROS
- [Testing](./testing.md) - Testing strategies
- [Advanced Topics](./advanced-topics.md) - Performance optimization
