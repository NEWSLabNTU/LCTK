# Board Fitter Development Guide

## Development Environment

### Prerequisites

1. **Rust Toolchain**
   ```bash
   # Install Rust (if not already installed)
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

   # Install nightly toolchain for formatting
   rustup toolchain install nightly
   rustup component add rustfmt --toolchain nightly
   rustup component add clippy
   ```

2. **System Dependencies**
   ```bash
   # Ubuntu/Debian
   sudo apt-get update
   sudo apt-get install -y \
       cmake \
       build-essential \
       libopencv-dev \
       libeigen3-dev \
       libboost-all-dev

   # Optional: CUDA for GPU acceleration
   # Install CUDA 11.3+ from NVIDIA website
   ```

3. **Development Tools**
   ```bash
   # Install cargo-nextest for better test output
   cargo install cargo-nextest

   # Install cargo-watch for auto-rebuild
   cargo install cargo-watch

   # Install cargo-expand for macro debugging
   cargo install cargo-expand
   ```

### Environment Setup

```bash
# Clone the repository
git clone <repository-url>
cd LCTK/src/lib/board-fitter

# Set up environment variables (from project root)
source setup/setup-env.sh

# Verify setup
cargo --version
rustc --version
```

## Building

### Standard Build

```bash
# Debug build (faster compilation, with debug symbols)
cargo build

# Release build (optimized, for performance testing)
cargo build --release

# Build with all features (including CUDA)
cargo build --release --all-features

# Build only the library (no tests/examples)
cargo build --lib
```

### Feature Flags

```bash
# Build with CUDA support
cargo build --features cuda

# Build with download capability for test data
cargo build --features download

# Build with all optional features
cargo build --all-features
```

### Build Troubleshooting

1. **OpenCV Linking Issues**
   ```bash
   # Set OpenCV path explicitly
   export OpenCV_DIR=/usr/local/share/opencv4
   export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
   ```

2. **CUDA Build Failures**
   ```bash
   # Disable CUDA if not available
   cargo build --no-default-features
   ```

3. **Clean Build**
   ```bash
   cargo clean
   rm -rf target/
   cargo build
   ```

## Testing

### Running Tests

```bash
# Run all tests with nextest (recommended)
cargo nextest run --no-fail-fast

# Run standard cargo tests
cargo test

# Run tests with output for debugging
cargo test -- --nocapture

# Run specific test
cargo nextest run test_perfect_board_detection

# Run tests matching pattern
cargo nextest run -E 'test(plane)'

# Run only unit tests
cargo nextest run -E 'kind(lib)'

# Run only integration tests
cargo nextest run -E 'kind(test)'
```

### Test Categories

1. **Unit Tests** (`src/*.rs`)
   - Test individual functions and modules
   - Fast, isolated, no external dependencies

2. **Integration Tests** (`tests/`)
   - Test end-to-end functionality
   - May require test data files

3. **Benchmarks** (`benches/`)
   - Performance measurements
   - Run with `cargo bench`

### Writing Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_example() {
        // Arrange
        let input = create_test_data();

        // Act
        let result = function_under_test(input);

        // Assert
        assert_eq!(result, expected_value);
    }

    #[test]
    #[should_panic(expected = "invalid input")]
    fn test_error_handling() {
        function_that_should_panic(invalid_input);
    }
}
```

### Test Data Management

```bash
# Test data is stored in test_data/
test_data/
├── synthetic/      # Generated test point clouds
├── real/          # Real sensor data (gitignored)
└── expected/      # Expected outputs for regression tests

# Generate synthetic test data
cargo test --test generate_test_data

# Download real test data (requires internet)
cargo test --features download test_external_data
```

## Linting

### Code Formatting

```bash
# Format all code
cargo +nightly fmt

# Check formatting without changes
cargo +nightly fmt --check

# Format specific file
rustfmt +nightly src/detection.rs
```

### Clippy Linting

```bash
# Run clippy with all warnings
cargo clippy --all-targets --all-features -- -D warnings

# Run clippy with pedantic lints
cargo clippy -- -W clippy::pedantic

# Run clippy and automatically fix issues
cargo clippy --fix --allow-dirty --allow-staged

# Common clippy fixes
cargo clippy --fix -- -A clippy::uninlined_format_args
```

### Pre-commit Workflow

```bash
# Run before every commit
make lint  # From project root

# Or manually:
cargo +nightly fmt --check && \
cargo clippy --all-targets --all-features -- -D warnings && \
cargo nextest run --no-fail-fast
```

## Debugging

### Debug Output

```rust
// Enable debug logging
RUST_LOG=board_fitter=debug cargo run

// Enable trace logging for specific module
RUST_LOG=board_fitter::detection=trace cargo run

// Use debug callback for detailed inspection
let detector = BoardDetectorBuilder::new(config)
    .with_debug_callback(ConsoleDebugHandler::new(true))
    .build()?;
```

### Common Debugging Tools

```bash
# Expand macros
cargo expand detection

# Check generated documentation
cargo doc --open

# Profile performance
cargo build --release
perf record --call-graph=dwarf target/release/example
perf report

# Memory profiling with valgrind
valgrind --leak-check=full --show-leak-kinds=all \
    target/release/example
```

### Debug Visualization

```rust
// Save intermediate results for visualization
detector.with_debug_callback(DebugFileWriter::new("debug_output/"))

// Files will be saved as:
// - debug_output/stage_plane_detection.json
// - debug_output/stage_diamond_fitting.json
// - debug_output/stage_hole_detection.json
```

## Performance Optimization

### Profiling

```bash
# CPU profiling with cargo-flamegraph
cargo install flamegraph
cargo flamegraph --bench detection_benchmark

# Criterion benchmarks
cargo bench

# View benchmark history
open target/criterion/report/index.html
```

### Optimization Workflow

1. **Baseline Measurement**
   ```bash
   cargo bench --bench detection_benchmark -- --save-baseline main
   ```

2. **Make Changes**

3. **Compare Performance**
   ```bash
   cargo bench --bench detection_benchmark -- --baseline main
   ```

### Memory Optimization

```bash
# Check binary size
cargo bloat --release

# Analyze dependencies impact
cargo tree --duplicate

# Strip debug symbols for smaller binary
strip target/release/libboard_fitter.so
```

## Continuous Integration

### Local CI Simulation

```bash
# Run full CI pipeline locally
./scripts/ci_local.sh

# Or manually:
cargo fmt --check
cargo clippy -- -D warnings
cargo test
cargo bench --no-run
cargo doc --no-deps
```

### GitHub Actions Workflow

The project uses GitHub Actions for CI:

```yaml
# .github/workflows/rust.yml
- Format check (rustfmt)
- Lint check (clippy)
- Unit tests
- Integration tests
- Benchmarks (no-run)
- Documentation build
```

## Troubleshooting

### Common Issues

1. **"cannot find -lopencv_core"**
   ```bash
   # Ubuntu/Debian
   sudo apt-get install libopencv-dev

   # Or build OpenCV from source
   ```

2. **"CUDA not found"**
   ```bash
   # Check CUDA installation
   nvcc --version

   # Build without CUDA
   cargo build --no-default-features
   ```

3. **Test failures due to floating point**
   ```rust
   // Use approx for float comparisons
   use approx::assert_relative_eq;
   assert_relative_eq!(result, expected, epsilon = 1e-6);
   ```

4. **Slow compilation times**
   ```bash
   # Use sccache for caching
   cargo install sccache
   export RUSTC_WRAPPER=sccache

   # Or use mold linker
   RUSTFLAGS="-C link-arg=-fuse-ld=mold" cargo build
   ```

## Development Workflow

### Feature Development

1. **Create feature branch**
   ```bash
   git checkout -b feature/new-detection-method
   ```

2. **Implement with TDD**
   ```bash
   # Write failing test
   # Implement feature
   # Make test pass
   cargo watch -x test
   ```

3. **Document changes**
   ```rust
   /// New detection method that improves accuracy
   ///
   /// # Example
   /// ```
   /// let result = new_method(data);
   /// ```
   pub fn new_method(data: &PointCloud) -> Result<Detection>
   ```

4. **Run full validation**
   ```bash
   make lint
   cargo test
   cargo bench
   ```

5. **Submit PR**
   - Ensure all CI checks pass
   - Update CHANGELOG.md
   - Request review

### Release Process

1. **Version bump**
   ```toml
   # Cargo.toml
   version = "0.2.0"
   ```

2. **Update documentation**
   ```bash
   cargo doc
   ```

3. **Tag release**
   ```bash
   git tag -a v0.2.0 -m "Release version 0.2.0"
   git push origin v0.2.0
   ```

## Best Practices

### Code Style

1. **Use descriptive names**
   ```rust
   // Good
   let plane_detection_threshold = 0.02;

   // Bad
   let t = 0.02;
   ```

2. **Document public APIs**
   ```rust
   /// Detects planes in the point cloud using RANSAC
   ///
   /// # Arguments
   /// * `points` - Input point cloud
   /// * `config` - Detection configuration
   ///
   /// # Returns
   /// Vector of detected plane candidates
   pub fn detect_planes(points: &PointCloud, config: &Config) -> Vec<Plane>
   ```

3. **Handle errors explicitly**
   ```rust
   // Good
   let plane = detect_plane(points)?;

   // Bad
   let plane = detect_plane(points).unwrap();
   ```

4. **Write tests first**
   - TDD helps design better APIs
   - Ensures code is testable
   - Documents expected behavior

### Performance Tips

1. **Avoid unnecessary allocations**
   ```rust
   // Good - reuse buffer
   let mut buffer = Vec::with_capacity(1000);
   for item in items {
       buffer.clear();
       process_with_buffer(item, &mut buffer);
   }
   ```

2. **Use iterators efficiently**
   ```rust
   // Good - lazy evaluation
   let sum: f64 = points.iter()
       .filter(|p| p.z > 0.0)
       .map(|p| p.x * p.x + p.y * p.y)
       .sum();
   ```

3. **Profile before optimizing**
   - Measure first
   - Optimize hotspots
   - Verify improvements