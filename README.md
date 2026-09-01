# LCTK - LiDAR Camera Toolkit

A comprehensive toolkit for calibrating LiDAR and camera systems, implemented in Rust with ROS 2 integration.

## Quick Start

```bash
# Clone the repository
git clone https://github.com/NEWSLabNTU/LCTK.git
cd LCTK

# Setup development environment
./setup.sh                 # Install system dependencies

# Build the project
just build

# Test with sample data (two terminals)
just sample-data           # Terminal 1: sample data playback
just demo                  # Or: sample data + calibration pipeline in one command
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
        SOLVER[lidar_to_camera_solver]
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
# Interactive setup (installs all dependencies)
./setup.sh

# Show what is already installed
./setup.sh status

# Run a single recipe (e.g. only ROS 2), or see all options
./setup.sh ros2
./setup.sh --help
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
# Build everything (always use this — it also builds the conflux packages the
# solver nodes depend on, which a plain `colcon build` does not)
just build
```

## Usage

LCTK supports two data input methods:
1. **Sample Data**: Pre-recorded test data included in the repository
2. **ROS Bag Playback**: Your own sensor recordings in ROS bag format

### Using Sample Data

Test the calibration pipeline with included sample data:

```bash
# Terminal 1: sample data playback (LiDAR + camera)
just sample-data

# Terminal 2: calibration pipeline
just lidar-camera

# Or run playback + pipeline together in one command
just demo

# Disable debug topics / set a custom log level
just debug_mode=false log_level=info lidar-camera

# Launch RViz for visualization
just rviz
```

The launch runs in the foreground. Monitor node status in the play_launch web UI
at <http://localhost:8000>, and stop it with `Ctrl+C` in the launching terminal
(see [Stopping a launch](#stopping-a-launch) if child processes are left behind).

### Configuration Variables

You can customize the calibration pipeline behavior using configuration variables:

```bash
# Available justfile variables (with defaults):
debug_mode=true            # Enable debug topics
log_level=info             # ROS log level (debug/info/warn/error)
rviz_enabled=true          # Launch RViz
mode=offline               # offline (RELIABLE QoS, rosbags) or realtime (BEST_EFFORT, live)
solver_mode=continuous     # continuous (latest pair), manual (multi-pose buffer), assisted (auto-capture + web review)
enable_overlay=true        # Point cloud / image overlay for visual validation
enable_judge=true          # Calibration judge (IoU metrics)

# Example usage (override with just var=value <recipe>):
just debug_mode=true rviz_enabled=true log_level=debug lidar-camera
```

### Using Your Own Data

One directory describes one run — where the data comes from and everything needed to
calibrate against it. Copy the closest shipped session from `sessions/` and edit it; see
the [Calibration Sessions](book/src/user-guide/sessions.md) guide.

```bash
# Scaffold from a shipped session, then edit its session.yaml
ros2 run lctk_launch lctk_session new ~/calib/rig-b \
    --from $(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne

# Validate before launching anything: resolves every path, checks the data exists,
# verifies bag topics, and prints the topics and frames each device will use
ros2 run lctk_launch lctk_session check ~/calib/rig-b

# Run it end to end -- LCTK starts the bag or pcap playback itself
ros2 launch lctk_launch session.launch.py session:=~/calib/rig-b
```

`session:=` is always an explicit path; `just run <name-or-path>` adds bare-name lookup on
top. If you would rather play the data yourself, run only the calibration half:

```bash
# Play your rosbag in one terminal
ros2 bag play your_data.bag

# Run the config-driven pipeline against your session manifest in another terminal
just calibrate ~/calib/rig-b/session.yaml

# For live sensors, use realtime QoS
just mode=realtime calibrate /path/to/your_config.yaml
```

Camera info is auto-derived from each image topic (image_pipeline convention).

For sample data topics:
- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2)
- `/sensing/camera/front_center/image_raw` (sensor_msgs/Image)
- `/sensing/camera/front_center/camera_info` (sensor_msgs/CameraInfo)

#### Two LiDAR Calibration

```bash
# Launch two-LiDAR calibration (config-driven, aligns two LiDAR sensors)
just two-lidar

# Stop with Ctrl+C in the launching terminal
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

### Manual Solver Tools

```bash
# Interactive TUI to drive the manual multi-pose solver
just extrinsic-solver-controller
```

### Running and stopping a launch

The `just` launch recipes run `play_launch` in the **foreground** — there is no
background service to manage.

```bash
# Monitor node status while a launch runs: open the play_launch web UI
#   http://localhost:8000

# Stop a launch: Ctrl+C in the terminal that started it
```

<a id="stopping-a-launch"></a>
If `Ctrl+C` leaves orphaned nodes behind, kill the whole `play_launch` process
group (note the leading `-` before the PGID):

```bash
ps -o pid,pgid,cmd | grep play_launch   # find the PGID
kill -9 -<PGID>
```

## Customization

### Configuration Files

All calibration parameters and settings are stored in configuration files:

```bash
# Configuration file locations
ros/lctk_launch/config/
├── targets/            # Target Definitions: physical plate geometry, cutouts,
│   │                   # fiducial layout (the geometric truth)
│   └── hollow_1000_aruco_4_v1.json5
├── board/              # Detector Tuning presets: sensor-specific, geometry-free
│   └── hollow_1000/
│       └── velodyne.json5      # ICP and RANSAC settings
├── aruco/              # ArUco detector tuning (corner refinement, adaptive threshold)
│   └── aruco_detector.json5
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
just log_level=debug debug_mode=true lidar-camera

# View debug topics in another terminal
ros2 topic list | grep debug
ros2 topic echo /calibration/lidar_board_detector/debug/icp_stats
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
   cat ros/lctk_launch/launch/camera.launch.xml

   # Test the GStreamer pipeline manually
   gst-launch-1.0 filesrc location=sessions/sample3-hollow-velodyne/data/video.avi ! \
       decodebin ! videoconvert ! autovideosink

   # Install missing plugins if needed
   sudo apt install gstreamer1.0-plugins-bad \
       gstreamer1.0-plugins-ugly gstreamer1.0-libav
   ```

3. **two-lidar exits immediately**
   ```bash
   # Watch the launch output in the foreground terminal for the error.
   # Common issue: missing config files -- verify they exist under
   # ros/lctk_launch/config/
   ```

4. **Orphaned nodes after Ctrl+C**
   ```bash
   # Kill the whole play_launch process group (see "Running and stopping a launch")
   ps -o pid,pgid,cmd | grep play_launch
   kill -9 -<PGID>
   ```

5. **No detections found**
   - Verify calibration board is visible in sensor data
   - Check configuration files in `ros/lctk_launch/config/`
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

# Build / check
just build                 # Build all ROS packages (+ conflux)
just clean                 # Clean build artifacts
just lint                  # Run linters
just test                  # Run tests

# Launch (foreground; Ctrl+C to stop)
just sample-data           # Sample data playback
just demo                  # Sample data + calibration pipeline
just lidar-camera          # LiDAR-camera calibration
just two-lidar             # Two-LiDAR calibration
just calibrate <config>    # Config-driven calibration for your own sensors

# Tools
just rviz                          # Launch RViz
just extrinsic-solver-controller      # Interactive manual-solver TUI
```

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines, code structure, and contribution process.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors

This software is created and maintained by NEWSLAB, National Taiwan University.

- Lin Hsiang-Jui (2022-)
- philly12399 (2022-2023)
