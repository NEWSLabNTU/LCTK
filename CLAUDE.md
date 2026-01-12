# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Project Overview

LCTK (LiDAR and Camera Toolkit) is a set of libraries and tools for calibrating LiDAR and camera systems. Implemented in Rust with ROS 2 integration.

## Quick Start

```bash
# Set up development environment
./setup.sh

# Build the project
just build

# Launch calibration
just lidar-camera

# See all commands
just
```

## Project Structure

- **`rust/`**: Pure Rust libraries (aruco-config, aruco-detector, hollow-board-detector, etc.)
- **`ros/`**: ROS 2 nodes (aruco_locator_node, lidar_board_detector, extrinsic_solver, etc.)
- **`setup/`**: Development environment setup scripts

## Build System

- Uses `colcon-cargo-ros2` for Rust ROS 2 integration
- ROS interface bindings are auto-generated at `build/<pkg>/rosidl_cargo/`
- Uses `rclrs` v0.6.0 from crates.io (requires `ros-humble-test-msgs`)
- Launch commands use `play_launch` for foreground execution

## Key Commands

```bash
just build      # Build all packages
just clean      # Clean build artifacts
just test       # Run tests
just lint       # Run linting

just lidar-camera   # Launch calibration
just sample-data    # Launch sample data
just rviz           # Launch RViz
```

## Known Issues

1. **Old .cargo/config.toml conflicts**: If build fails with `Unable to update .../install/.../rust`:
   ```bash
   mv .cargo/config.toml .cargo/config.toml.bak
   ```

2. **Colcon-cargo conflicts**: Remove old packages before installing colcon-cargo-ros2:
   ```bash
   pip3 uninstall colcon-cargo colcon-ros-cargo
   ```

3. **ROS2 daemon issues**: Kill unresponsive daemon:
   ```bash
   pkill -9 -f ros2-daemon
   ```

## Coding Guidelines

- Use named parameters in format strings: `println!("{e}")` not `println!("{}", e)`
- Clone Arc variables in local scope before moving to closures
- Use `just build` to rebuild ROS2 packages (not `cargo build` directly)
- Always run build commands from project root directory
- Don't use Pokemon exception handling (`try: except Exception: pass`)
- Prefer functional struct initialization in Rust
- When running sudo commands, show command to user instead of executing

## ROS 2 Conventions

- Camera info topics auto-derived from image topics (image_pipeline convention)
- All nodes require explicit config file parameters (no hardcoded defaults)
- Workspace dependencies defined in root Cargo.toml
