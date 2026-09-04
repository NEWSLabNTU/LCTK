# Build System

LCTK uses **colcon-cargo-ros2** to build ROS 2 packages written in Rust. This integrates seamlessly with the standard colcon build system.

## Quick Start

```bash
# Build everything
just build

# Clean and rebuild
just clean && just build

# Run tests
just test
```

## Build Commands

### Using justfile (Recommended)

```bash
just build      # Build all packages
just clean      # Remove build artifacts
just test       # Run all tests
just format     # Format code with rustfmt
just lint       # Every lint (Rust + Python)
just lint-rust  # Rust only: nightly rustfmt + clippy
just lint-py    # Python only: ruff, in seconds
```

### Using colcon Directly

```bash
source /opt/ros/humble/setup.bash
colcon build \
    --base-paths ros \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    --cargo-args --profile=test-release
```

## Project Structure

```
LCTK/
├── ros/                    # ROS 2 packages
│   ├── aruco_locator_node/
│   ├── lidar_board_detector/
│   ├── lctk_interfaces/
│   ├── lctk_launch/
│   └── ...
├── rust/                   # Pure Rust libraries
│   ├── aruco-detector/
│   ├── calibration-target/          # Target Definitions: geometry, identity
│   ├── calibration-target-detector/ # Pose estimation against a Target Definition
│   ├── board-cluster-detector/
│   └── ...
├── build/                  # Build artifacts (generated)
├── install/                # Install directory (generated)
└── justfile                # Build recipes
```

## Build Configuration

### justfile Variables

The justfile defines default configuration values:

```just
debug_mode := "true"
enable_icp_iteration_debug := "true"
enable_evaluator := "true"
enable_overlay := "true"
log_level := "info"
rviz_enabled := "false"
```

Override at runtime:

```bash
just rviz_enabled=true debug_mode=false demo
```

### Cargo Profiles

The build uses the `test-release` profile defined in `Cargo.toml`:

```toml
[profile.test-release]
inherits = "release"
debug = true
```

This provides optimized builds with debug symbols for profiling.

## Incremental Development

### Rebuild Single Package

```bash
source /opt/ros/humble/setup.bash
colcon build \
    --base-paths ros \
    --packages-select aruco_locator_node \
    --symlink-install
```

### Test Pure Rust Libraries

```bash
# Run tests for a specific library
cargo test -p calibration-target-detector

# Run all tests with nextest
cargo nextest run --config build/ros2_cargo_config.toml
```

### Quick Syntax Check

```bash
cargo check
cargo clippy --all-targets
```

## Environment Setup

### Required Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### OpenCV Configuration

The build automatically configures OpenCV:

```bash
export OPENCV_PKGCONFIG_NAME=opencv4
```

## Clean Builds

### Clean Everything

```bash
just clean
# Removes: build/, install/, log/, target/
```

### Clean Single Package

```bash
rm -rf build/<package_name> install/<package_name>
just build
```

## Common Build Issues

### Cargo Can't Find ROS Packages

**Error:** `error: failed to select a version for 'sensor_msgs'`

**Cause:** Build artifacts are stale or corrupted.

**Fix:**
```bash
just clean && just build
```

### OpenCV Binding Failures

**Error:** `fatal error: 'memory' file not found`

**Fix:**
```bash
sudo apt-get install libstdc++-12-dev libclang-dev
```

### SFCGAL Missing

**Error:** `SFCGAL/capi/sfcgal_c.h: No such file or directory`

**Fix:**
```bash
sudo apt-get install libsfcgal-dev
```

## Colcon Tips

### Flag Order Matters

Always put `--packages-select` **before** `--cmake-args`:

```bash
# CORRECT
colcon build --packages-select my_node --cmake-args -DFOO=BAR

# WRONG (--packages-select treated as CMake arg)
colcon build --cmake-args -DFOO=BAR --packages-select my_node
```

### Useful Flags

```bash
--symlink-install     # Fast rebuilds (symlink instead of copy)
--continue-on-error   # Build remaining packages on failure
--event-handlers console_direct+  # Verbose output
```

## Build Performance

### Parallel Builds

```bash
export CARGO_BUILD_JOBS=$(nproc)
```

### Caching with sccache

```bash
cargo install sccache
export RUSTC_WRAPPER=sccache
```

## Next Steps

- [Architecture](./architecture.md) - System design overview
- [Testing](./testing.md) - Testing strategies
- [Contributing](./contributing.md) - Development guidelines
