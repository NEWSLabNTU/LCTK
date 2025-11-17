# LCTK - LiDAR Camera Toolkit

A comprehensive toolkit for calibrating LiDAR and camera systems, implemented in Rust with ROS 2 integration.

## Quick Start

```bash
# Clone the repository
git clone https://github.com/NEWSLabNTU/LCTK.git
cd LCTK

# Setup development environment
./setup-dev-env.sh         # Install system dependencies

# Build the project
just build

# Test with sample data
just sample-sensor-data start    # Launch sample data playback
just lidar-camera start          # Launch calibration pipeline
```

## Overview

LCTK provides tools and ROS 2 nodes for:
- **LiDAR to Camera Calibration**: Compute extrinsic parameters between LiDAR and camera systems
- **LiDAR to LiDAR Calibration**: Align multiple LiDAR sensors
- **ArUco Marker Detection**: Detect and localize ArUco markers in camera images
- **Calibration Board Detection**: Detect hollow calibration boards in 3D point clouds
- **Real-time Visualization**: Live visualization of calibration results and sensor data

## System Architecture

The calibration pipeline consists of sensor processing, detection, and calibration stages:

```mermaid
graph TB
    %% Input Sources
    subgraph InputSources[Input Sources]
        PCAP[PCAP File<br/>LiDAR Data]
        VIDEO[Video File<br/>Camera Data]
    end

    %% Point Cloud Processing Pipeline
    subgraph PointCloudProcessing[Point Cloud Processing]
        VD[velodyne_driver_node]
        VT[velodyne_transform_node]
        CBL[lidar_board_detector]
    end

    %% Image Processing Pipeline
    subgraph ImageProcessing[Image Processing]
        CAM[camera_driver<br/>gscam]
        AL[aruco_locator]
    end

    %% Calibration & Visualization
    subgraph CalibrationViz[Calibration & Visualization]
        SOLVER[extrinsic_solver_node]
        OVERLAY[pointcloud_image_overlay]
        VIZ[Visualization<br/>Tools]
    end

    %% Debug Topics (when debug_mode=true)
    subgraph DebugTopics[Debug Topics - debug_mode=true]
        DBG1[plane_inliers]
        DBG2[initial_board_marker]
        DBG3[icp_stats]
    end

    %% Connections
    PCAP --> VD
    VD -->|/sensing/lidar/top/velodyne_packets| VT
    VT -->|/sensing/lidar/top/pointcloud_raw| CBL
    CBL -->|/calibration/.../calibration_board_detections| SOLVER

    VIDEO --> CAM
    CAM -->|/sensing/camera/.../image_raw| AL
    CAM -->|/sensing/camera/.../camera_info| AL
    AL -->|/calibration/.../aruco_detections| SOLVER
    CAM -->|/sensing/camera/.../camera_info| SOLVER

    SOLVER -->|/calibration/.../extrinsic_transform| OVERLAY
    CAM -->|/sensing/camera/.../image_raw| OVERLAY
    VT -->|/sensing/lidar/top/pointcloud_raw| OVERLAY
    OVERLAY -->|/calibration/pointcloud_overlay| VIZ

    CBL -.->|debug_mode=true| DBG1
    CBL -.->|debug_mode=true| DBG2
    CBL -.->|debug_mode=true| DBG3

    %% Styling
    classDef input fill:#1976d2,stroke:#0d47a1,stroke-width:2px,color:#fff
    classDef pointcloud fill:#388e3c,stroke:#1b5e20,stroke-width:2px,color:#fff
    classDef image fill:#7b1fa2,stroke:#4a148c,stroke-width:2px,color:#fff
    classDef calibration fill:#f57c00,stroke:#e65100,stroke-width:2px,color:#fff
    classDef debug fill:#616161,stroke:#424242,stroke-width:1px,color:#fff

    class PCAP,VIDEO input
    class VD,VT,CBL pointcloud
    class CAM,AL image
    class SOLVER,OVERLAY,VIZ calibration
    class DBG1,DBG2,DBG3 debug
```

## Installation

### Prerequisites
- Ubuntu 22.04 LTS (or 24.04 for ROS 2 Jazzy)
- Internet connection for dependency installation

### Setup Environment

```bash
# Setup system dependencies and development environment
./setup-dev-env.sh

# For non-interactive installation:
./setup-dev-env.sh -y

# For minimal installation (skip CUDA and dev tools):
./setup-dev-env.sh -y --minimal
```

The setup process installs:
- ROS 2 Humble (on Ubuntu 22.04) or Jazzy (on Ubuntu 24.04)
- Rust toolchain (stable and nightly)
- OpenCV 4.5.4+ with development headers
- GStreamer with multimedia plugins
- Python 3.10+ with pip and development tools
- SFCGAL for geometric computations
- Build tools and system libraries

### Build Project

```bash
# Build entire project
just build

# Or use colcon directly for more control:
source install/setup.bash
colcon build --base-paths ros --symlink-install
```

## Usage

LCTK supports two data input methods:
1. **Sample Data**: Pre-recorded test data included in the repository
2. **ROS Bag Playback**: Your own sensor recordings in ROS bag format

### Using Sample Data

Test the calibration pipeline with included sample data:

```bash
# Launch sample data playback (LiDAR + camera)
just sample-sensor-data start

# Launch calibration pipeline (in another terminal)
just lidar-camera start

# Launch with debug topics disabled and custom log level
just debug_mode=false log_level=info lidar-camera start

# Launch RViz for visualization
just rviz

# Check service status
just lidar-camera status
just sample-sensor-data status

# View logs
just lidar-camera logs -f         # Follow logs
just sample-sensor-data logs -f

# Stop services when done
just lidar-camera stop
just sample-sensor-data stop
```

### Configuration Variables

You can customize the calibration pipeline behavior using configuration variables:

```bash
# Available variables (with defaults):
debug_mode=true                # Enable debug topics
enable_icp_iteration_debug=true   # Enable ICP iteration debug
enable_evaluator=true          # Enable calibration evaluator
enable_overlay=true            # Enable point cloud overlay
log_level=info                 # ROS log level (debug/info/warn/error)
rviz_enabled=true             # Launch RViz
use_best_effort_qos=true      # Use best effort QoS
use_advanced_solver=false     # Use advanced solver
camera_topic=/sensing/camera/zedxm/zed_node/left_raw/image_raw_color
pointcloud_topic=/sensing/lidar/concatenated/pointcloud

# Example usage:
just debug_mode=true rviz_enabled=true log_level=debug lidar-camera start
```

### Using Your Own Data

For custom sensor data recorded in ROS bags:

```bash
# Play your rosbag in one terminal
ros2 bag play your_data.bag

# Launch calibration with rosbag-compatible QoS in another terminal
just use_best_effort_qos=false lidar-camera start

# With custom topic remapping if your bag uses different topic names
just use_best_effort_qos=false \
     camera_topic=/your/camera/topic \
     pointcloud_topic=/your/lidar/topic \
     lidar-camera start
```

Default expected topics:
- `/sensing/lidar/concatenated/pointcloud` (sensor_msgs/PointCloud2)
- `/sensing/camera/zedxm/zed_node/left_raw/image_raw_color` (sensor_msgs/Image)
- Camera info is auto-derived following image_pipeline convention

For sample data topics:
- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)
- `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
- `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)

#### Two LiDAR Calibration

```bash
# Launch two LiDAR calibration pipeline
just two-lidar start

# Check status and logs
just two-lidar status
just two-lidar logs

# Stop when done
just two-lidar stop

# This uses multi-wayside detection to align two LiDAR sensors
```

### Visualization

```bash
# Launch RViz for real-time visualization
just rviz

# View debug topics in RViz when debug_mode=true:
# - /calibration/.../debug/plane_inliers
# - /calibration/.../debug/initial_board_marker
# - /calibration/.../debug/final_board_pose
# - /calibration/.../debug/icp_iterations
```

### Advanced Tools

```bash
# Run interactive advanced solver controller
just run-advanced-solver-controller
```

### Service Management

LCTK uses systemd user services for reliable process management:

```bash
# Check status of services
just lidar-camera status
just sample-sensor-data status
just two-lidar status

# View logs from services
just lidar-camera logs
just sample-sensor-data logs -f      # Follow logs
just two-lidar logs --since "5 min ago"

# Restart services
just lidar-camera restart
just sample-sensor-data restart

# Stop services
just lidar-camera stop
just sample-sensor-data stop
just two-lidar stop
```

## Customization

### Configuration Files

All calibration parameters and settings are stored in configuration files:

```bash
# Configuration file locations
src/ros2/lctk_launch/config/
├── board/              # Calibration board parameters
│   └── board_detector.json5    # ICP and RANSAC settings
├── aruco/              # ArUco marker patterns
│   └── aruco_5x5_*.yaml        # Marker definitions
└── camera/             # Camera calibration
    └── front_center_camera_info.yaml         # Camera intrinsic parameters
```

To customize calibration parameters:
1. Edit the relevant JSON5 or YAML configuration file
2. Rebuild if you modified any Rust code (see below)
3. Restart the calibration pipeline

### Rebuilding After Code Changes

**Important**: If you modify any Rust or C/C++ source code, you must rebuild the project:

```bash
# After modifying Rust code in src/
just build
```

Configuration file changes (JSON5/YAML) do **not** require rebuilding - just restart the nodes.

## Troubleshooting

### Debug Logging

Enable detailed debug logging to diagnose calibration issues:

```bash
# Launch with debug logging enabled
just log_level=debug debug_mode=true lidar-camera start

# View debug topics in another terminal
ros2 topic list | grep debug
ros2 topic echo /calibration/lidar_board_detector/debug/icp_stats

# Check service logs
just lidar-camera logs -f
```

Debug mode provides:
- Detailed algorithm logs (RANSAC, ICP convergence)
- Additional visualization topics for RViz
- Performance metrics and statistics

### Common Issues

1. **Build fails with missing headers**
   ```bash
   sudo apt install libstdc++-12-dev libclang-dev
   ```

2. **Video playback issues with gscam**

   If sample data playback fails, test the GStreamer pipeline:
   ```bash
   # Check the gscam configuration
   cat src/ros2/lctk_launch/launch/camera.launch.xml

   # Test the GStreamer pipeline manually
   gst-launch-1.0 filesrc location=data/sampledata/3/video.avi ! \
       decodebin ! videoconvert ! autovideosink

   # Install missing plugins if needed
   sudo apt install gstreamer1.0-plugins-bad \
       gstreamer1.0-plugins-ugly gstreamer1.0-libav
   ```

3. **Service fails immediately (two-lidar)**
   ```bash
   # Check the status - it will show ROS launch logs
   just two-lidar status

   # View full logs
   just two-lidar logs

   # Common issue: missing config files
   # Verify config files exist in src/ros2/lctk_launch/config/
   ```

4. **Service management issues**
   ```bash
   # Stop all services
   just lidar-camera stop
   just sample-sensor-data stop
   just two-lidar stop

   # Check systemd service status
   systemctl --user status lctk-calibration
   systemctl --user status lctk-lidar-camera-data
   systemctl --user status lctk-two-lidar
   ```

5. **No detections found**
   - Verify calibration board is visible in sensor data
   - Check configuration files in `src/ros2/lctk_launch/config/`
   - Enable debug mode to see intermediate processing steps
   - Ensure ArUco markers are clearly visible to camera

6. **ROS 2 service timeouts with DDS discovery**

   There is a known timing issue with ROS 2 DDS discovery, particularly when using CycloneDDS with localhost-only configuration. Services may become temporarily unreachable during the discovery lease renewal window (around 5 seconds after startup).

   **Symptoms:**
   - Services are discoverable with `ros2 service list` but calls timeout
   - Service calls work immediately after launch but fail after ~5 seconds
   - Random service availability when testing

   **Solution:**
   ```bash
   # Wait for service to be fully available before calling
   ros2 service wait /your/service/name --timeout 30

   # In Python clients, use longer timeouts:
   if not client.wait_for_service(timeout_sec=10.0):
       print("Service not available")
   ```

   **For testing scripts:**
   ```bash
   # Add explicit wait in your scripts:
   source install/setup.sh
   export ROS_DOMAIN_ID=109
   ros2 service wait /calibration/lidar_board_detector/get_bbox_params
   # Then run your test
   ```

   This is a DDS configuration issue, not related to the service implementation. The `lidar_board_detector` uses lock-free arc-swap for optimal performance.

7. **empy version compatibility**

   ROS 2 Humble requires empy < 4.0. If you see errors like `AttributeError: module 'string' has no attribute 'split'`:
   ```bash
   # Remove pip-installed empy and use system package
   pip3 uninstall empy
   sudo apt-get install python3-empy
   ```

   The `.envrc` file (if using direnv) will warn you if an incompatible empy version is detected.

## Development

### Using direnv (Optional)

For automatic environment setup when entering the project directory:

```bash
# Install direnv
sudo apt install direnv

# Add to your shell config (~/.bashrc or ~/.zshrc)
eval "$(direnv hook bash)"  # for bash
eval "$(direnv hook zsh)"   # for zsh

# Allow the .envrc file
direnv allow .

# Now the ROS environment will be sourced automatically when you cd into LCTK/
```

The `.envrc` file automatically:
- Sources the correct ROS distribution based on Ubuntu version
- Sources the workspace overlay from `install/setup.bash`
- Sets up OpenCV environment variables
- Checks for empy version compatibility

### Available Just Commands

```bash
# View all available commands
just --list

# View detailed help
just help

# Build commands
just build                 # Build all ROS packages
just clean                 # Clean build artifacts
just lint                  # Run linters
just test                  # Run tests

# Service management
just lidar-camera {start|stop|restart|status|logs}
just sample-sensor-data {start|stop|restart|status|logs}
just two-lidar {start|stop|restart|status|logs}

# Tools
just rviz                              # Launch RViz
just run-advanced-solver-controller    # Interactive solver controller
```

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines, code structure, and contribution process.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors

This software is created and maintained by NEWSLAB, National Taiwan University.

- Lin Hsiang-Jui (2022-)
- philly12399 (2022-2023)
