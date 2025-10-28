# Testing

LCTK uses standard Rust and ROS 2 testing tools for unit, integration, and performance testing.

## Quick Start

```bash
# Test all Rust code
cargo test --workspace

# Test specific library
cargo test --manifest-path src/lib/aruco-detector/Cargo.toml

# Test ROS packages
colcon test --packages-select my_node

# View test results
colcon test-result --verbose
```

## Unit Testing

### Rust Libraries

**Location:** `src/lib/<library>/tests/`

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detection() {
        let config = ArUcoConfig::default();
        let detector = ArUcoDetector::new(config);

        // Test with known input
        let result = detector.detect(&test_image);
        assert!(result.is_ok());
    }
}
```

**Run:**
```bash
cargo test --lib
```

### ROS 2 Nodes

**Location:** `src/bin/<node>/tests/`

```rust
#[test]
fn test_node_initialization() {
    let context = Context::new(std::env::args()).unwrap();
    let node = create_node(&context, "test_node").unwrap();
    assert!(node.name() == "test_node");
}
```

## Integration Testing

### End-to-End Calibration

```bash
# Launch complete pipeline with test data
ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
    pcap_file:=test_data/lidar.pcap \
    video_file:=test_data/camera.mp4

# Verify output
ros2 topic echo /calibration_transform
```

### Node Communication

Test that nodes communicate correctly:

```bash
# Start node
ros2 run my_node my_node &

# Publish test message
ros2 topic pub /input std_msgs/String "data: test"

# Verify output
ros2 topic echo /output --once
```

## Performance Testing

### Benchmarking

```bash
# Run Rust benchmarks
cargo bench

# Profile with perf
perf record -g ros2 run my_node my_node
perf report
```

### Real-time Constraints

```bash
# Check topic rates (should be >10 Hz)
ros2 topic hz /aruco_detections
ros2 topic hz /calibration_board_detections

# Measure latency
ros2 topic delay /calibration_transform
```

## Test Data

### Sample Data

**Location:** `data/sampledata/`

```bash
# Use provided test data
make launch_lidar_camera_sample_data

# Or create test fixtures
data/
├── test_images/
│   ├── aruco_markers.png
│   └── calibration_board.png
└── test_pointclouds/
    └── board_detection.pcd
```

### Synthetic Data

Generate test data programmatically:

```rust
#[test]
fn test_with_synthetic_data() {
    let test_image = generate_test_pattern();
    let detector = ArUcoDetector::new(config);
    let result = detector.detect(&test_image)?;

    assert_eq!(result.markers.len(), 4);
}
```

## Test Organization

**Pattern:**
```
src/lib/my-detector/
├── src/
│   ├── lib.rs
│   └── detector.rs
├── tests/
│   ├── unit_tests.rs       # Public API tests
│   └── integration.rs      # Full workflow tests
└── benches/
    └── performance.rs      # Benchmarks
```

## Debugging Tests

### Enable Logging

```bash
# Show test output
cargo test -- --nocapture

# Enable ROS logging in tests
export RCUTILS_LOGGING_LEVEL=DEBUG
cargo test
```

### Run Single Test

```bash
# Run specific test
cargo test test_aruco_detection

# Run with backtrace
RUST_BACKTRACE=1 cargo test
```

## Continuous Testing

### Pre-commit Checks

```bash
# Format code
cargo fmt --all

# Lint
cargo clippy --all-targets

# Run tests
cargo test --workspace

# Build
make build
```

### Automated CI

Tests run automatically on pull requests:
- Unit tests for all libraries
- Integration tests for ROS nodes
- Build verification
- Documentation generation

## Test Coverage

### Generate Coverage Report

```bash
# Install tarpaulin
cargo install cargo-tarpaulin

# Generate coverage
cargo tarpaulin --out Html --output-dir coverage/

# View report
firefox coverage/index.html
```

## Common Test Patterns

### Mock ROS Nodes

```rust
struct MockPublisher {
    messages: Arc<Mutex<Vec<Message>>>,
}

impl MockPublisher {
    fn publish(&self, msg: Message) {
        self.messages.lock().unwrap().push(msg);
    }
}
```

### Test Fixtures

```rust
fn setup_test_config() -> Config {
    Config {
        iterations: 100,
        threshold: 0.05,
        ..Default::default()
    }
}

#[test]
fn test_with_fixture() {
    let config = setup_test_config();
    // Use config in test
}
```

### Parameterized Tests

```rust
#[test]
fn test_multiple_scenarios() {
    let scenarios = vec![
        (input1, expected1),
        (input2, expected2),
    ];

    for (input, expected) in scenarios {
        assert_eq!(function(input), expected);
    }
}
```

## Testing Checklist

Before submitting code:

- [ ] All unit tests pass (`cargo test`)
- [ ] Integration tests pass (`colcon test`)
- [ ] Code formatted (`cargo fmt`)
- [ ] No clippy warnings (`cargo clippy`)
- [ ] New features have tests
- [ ] Tests cover edge cases
- [ ] Performance benchmarks run (if applicable)

## Next Steps

- [Contributing](./contributing.md) - Contribution guidelines
- [Advanced Topics](./advanced-topics.md) - Performance optimization
- [Reference](./reference.md) - Testing utilities reference
