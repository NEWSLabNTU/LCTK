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
- **`book/`**: Documentation (mdbook with mermaid diagrams)

## Build System

- Uses `colcon-cargo-ros2` for Rust ROS 2 integration
- ROS interface bindings are auto-generated at `build/<pkg>/rosidl_cargo/`
- Uses `rclrs` v0.6.0 from crates.io (requires `ros-humble-test-msgs`)
- Launch commands use `play_launch` for foreground execution
- **Always use `just build`** - never run raw `colcon build` commands. The justfile uses specific flags:
  ```bash
  colcon build \
      --base-paths ros \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      --cargo-args --profile=test-release
  ```
- To build a single package, use: `just build` with `--packages-select <pkg>` appended manually if needed, but prefer building all packages

## Key Commands

```bash
just build      # Build all packages
just clean      # Clean build artifacts
just test       # Run tests
just lint       # Run linting

just lidar-camera   # Launch calibration
just sample-data    # Launch sample data
just rviz           # Launch RViz

# Documentation (run from book/ directory)
just build          # Build docs
just serve          # Serve with live reload
just serve-public   # Serve on 0.0.0.0
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

4. **Text file busy during build**: If build fails with "Text file busy (os error 26)", kill running nodes and clean:
   ```bash
   pkill -9 -f "<node_name>"
   rm -rf build/<package> install/<package>
   just build
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

## rclrs Patterns

### Dynamic Parameters
Use `MandatoryParameter<T>` wrapped in `Arc` for runtime-configurable parameters:
```rust
let param: Arc<MandatoryParameter<f64>> = Arc::new(
    node.declare_parameter::<f64>("param_name")
        .default(1.0)
        .mandatory()?
);
// Read current value (reflects runtime changes via `ros2 param set`)
let value = param.get();
```

### High-Frequency Sensor Data with Slow Processing
The rclrs executor queues ALL messages internally, regardless of QoS KEEP_LAST settings. For slow processing (e.g., ICP taking 600ms+ with 10Hz input), use `ArcSwap` to decouple reception from processing:
```rust
use arc_swap::ArcSwap;

// Store latest message
let latest_msg: Arc<ArcSwap<Option<Arc<SensorMsg>>>> = Arc::new(ArcSwap::new(Arc::new(None)));

// Subscription callback - lightweight, just stores latest
let msg_for_callback = Arc::clone(&latest_msg);
node.create_subscription(opts, move |msg| {
    msg_for_callback.store(Arc::new(Some(Arc::new(msg))));
})?;

// Processing thread - takes latest, skips stale
let msg_for_processing = Arc::clone(&latest_msg);
std::thread::spawn(move || loop {
    let msg_opt = msg_for_processing.swap(Arc::new(None));
    if let Some(msg) = msg_opt.as_ref() {
        process(msg);  // Slow processing here
    } else {
        std::thread::sleep(Duration::from_millis(5));
    }
});
```
This ensures always processing the latest data, not stale queued messages.

## Calibration Workflow

### LiDAR-to-LiDAR Calibration

The `lidar_to_lidar_solver` Python node replaces the deprecated `multi_wayside_node` for two-LiDAR calibration. It subscribes to Detection3DArray messages from two `lidar_board_detector` nodes and computes the transform between frames. **Note: This pipeline is not yet tested.**

### Advanced Extrinsic Solver

The `advanced_extrinsic_solver` node provides multi-pose calibration with manual adjustment capabilities.

**Services** (under `~/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/`):
- `add_detection` - Add current ArUco + board detection pair to buffer
- `clear_buffer` - Clear all buffered detections
- `get_status` - Get buffer size, correspondences, solve status
- `list_buffer` - List all buffered detection pairs
- `remove_detection` - Remove detection by index
- `dump_detections` - Save detections + transform to JSON file
- `load_detections` - Load detections + transform from JSON file
- `adjust_transform` - Manual x/y/z/roll/pitch/yaw adjustment
- `reset_transform` - Reset manual adjustments (re-solve from buffer)
- `get_pose_info` - Get solved pose, current pose, and adjustment delta

**Detection File Format** (version 2):
```json
{
  "version": 2,
  "num_detections": 5,
  "detections": [...],
  "transform": {
    "rvec": [rx, ry, rz],
    "tvec": [tx, ty, tz]
  }
}
```

### Interactive Solver Controller

Rich TUI for controlling the advanced_extrinsic_solver. Run via:
```bash
ros2 run interactive_solver_controller interactive_solver_controller
```

**Key Bindings:**
```
Buffer:     Space (Add)  Backspace (Delete)  c (Clear)
File:       p (Save ~/detections.json)  o (Load)
Transform:  q/a (X)  w/s (Y)  e/d (Z)  r/f (Roll)  t/g (Pitch)  y/b (Yaw)
Step Size:  ] (Increase)  [ (Decrease)
Reset:      0 (Re-solve from buffer)
Exit:       ESC
```

**Display Panels:**
- Buffer Status: Detection count, correspondences, publishing status
- Pose Information: Three columns showing Solved (PnP), Adjustment (delta), Current (final)
- Step Size: Current translation (mm) and rotation (deg) step sizes
- Key Bindings: Quick reference for all controls
