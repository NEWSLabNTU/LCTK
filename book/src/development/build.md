# Build System

LCTK uses a sophisticated three-pass build system that cleanly separates dependencies and ensures proper compilation order.

## Prerequisites

### System Dependencies
- **ROS 2 Humble** or later
- **Rust toolchain** (stable channel)
- **OpenCV 4.5.4** or 4.6.0
- **C++ development headers**: libstdc++-12-dev, libclang-dev
- **SFCGAL library**: libsfcgal-dev (for multi_wayside packages)
- **CUDA 11.3+** (optional, for GPU acceleration)

### Environment Setup
```bash
# Install dependencies using the setup script
./setup-dev-env.sh

# Or for minimal installation (no CUDA or dev tools)  
./setup-dev-env.sh -y --minimal

# For CI environments (non-interactive)
./setup-dev-env.sh -y
```

## Three-Pass Build Process

### Overview
The build system uses three distinct passes to handle the complex dependency relationships:

1. **Pass 1**: ROS 2 Rust Foundation (`make build_ros2_rust`)
2. **Pass 2**: Interface Types (`make build_interface`) 
3. **Pass 3**: LCTK Applications (`make build_packages`)

### Pass 1: ROS 2 Rust Foundation
Builds the fundamental ROS 2 Rust ecosystem:
```bash
make build_ros2_rust
```

**Components built**:
- `rclrs`: ROS 2 client library for Rust
- `ros2_rust_interfaces`: Common ROS 2 message types
- Foundation libraries required by LCTK nodes

**Location**: `src/ros2_rust_ws/`

### Pass 2: Interface Types  
Builds LCTK-specific message and service definitions:
```bash
make build_interface
```

**Components built**:
- Custom message types for calibration
- Service definitions for node control
- Shared data structures between nodes

**Dependencies**: Requires Pass 1 completion  
**Location**: `src/interface/`

### Pass 3: LCTK Applications
Builds the main LCTK nodes and applications:
```bash  
make build_packages
```

**Components built**:
- All ROS 2 calibration nodes
- Launch file packages  
- Configuration files
- Python integration modules

**Dependencies**: Requires Pass 1 and 2 completion  
**Location**: `src/bin/`

## Complete Build

### Full Project Build
```bash
# Build everything (runs all three passes)
make build

# Clean build from scratch
make clean && make build
```

### Individual Package Build
```bash  
# Build specific package after interface setup
make build_interface
source install/setup.bash
cargo build --release --manifest-path src/bin/aruco_locator_node/Cargo.toml
```

## Build Configuration

### Environment Variables
```bash
# Required for OpenCV integration
export OPENCV_PKGCONFIG_NAME=opencv4
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4

# CUDA support (optional)
export CUDA_PATH=/usr/local/cuda
export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
```

### Colcon Build Flags
The build system uses optimized colcon flags:
```makefile
COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Benefits**:
- `--symlink-install`: Fast rebuilds by symlinking instead of copying
- `CMAKE_BUILD_TYPE=Release`: Optimized performance builds
- Proper handling of mixed Rust/C++ dependencies

## Development Workflows

### Iterative Development
```bash
# Quick rebuild after code changes
source install/setup.bash
cargo build --release --manifest-path src/bin/<package>/Cargo.toml

# Or rebuild specific packages with colcon
colcon build --packages-select <package_name>
```

### Testing Builds
```bash
# Build with debug information
export CMAKE_BUILD_TYPE=Debug
make build

# Build for specific architecture  
cargo build --target x86_64-unknown-linux-gnu
```

### Clean Builds
```bash
# Clean everything
make clean

# Clean specific components  
rm -rf build install log target .cargo
make -C src/ros2_rust_ws clean
```

## Troubleshooting

### Common Build Issues

#### OpenCV Binding Failures
**Error**: "fatal error: 'memory' file not found"  
**Solution**:
```bash
sudo apt-get install libstdc++-12-dev libclang-dev
export OPENCV_PKGCONFIG_NAME=opencv4
```

#### SFCGAL Missing  
**Error**: "SFCGAL/capi/sfcgal_c.h: No such file or directory"  
**Solution**:
```bash
sudo apt-get install libsfcgal-dev
# Or exclude packages that need SFCGAL if not required
```

#### Colcon Build Aborts
**Error**: One package failure aborts all subsequent builds  
**Solution**:
```bash
# Fix failing package dependencies first, or
# Build packages individually with cargo
```

#### JSON Parsing Errors
**Error**: "JSONDecodeError: Expecting value: line 1 column 1"  
**Solution**: Fixed by modifying colcon-cargo to use `--quiet` flag

### Build Performance

#### Parallel Compilation
```bash
# Use all CPU cores
export CARGO_BUILD_JOBS=$(nproc)

# Or limit for memory-constrained systems
export CARGO_BUILD_JOBS=4
```

#### Incremental Builds
- Use `--symlink-install` for faster development iterations
- Cache Rust compilation artifacts with `sccache`
- Use `cargo check` for faster syntax validation

#### Memory Management
- Large projects may require increased memory limits
- Consider using `cargo build` with `--release` for smaller binaries
- Monitor system resources during parallel builds

## IDE Integration

### VS Code
Recommended extensions:
- rust-analyzer
- ROS extension
- CMake Tools

### CLion  
- Rust plugin for Cargo support
- ROS2 integration plugin
- CMake project support

### Build Scripts
The build system integrates with common development tools:
- Makefiles for simple command-line builds
- Cargo workspaces for Rust IDE integration  
- CMake for C++ components
- Colcon for ROS 2 packaging