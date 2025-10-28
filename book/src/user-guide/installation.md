# Installation

Get LCTK running on Ubuntu 22.04 LTS in three steps.

## System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Memory**: 8GB RAM minimum, 16GB recommended
- **Disk Space**: ~10GB for dependencies and build artifacts
- **Network**: Internet connection for downloading dependencies

## Quick Installation

### Step 1: Clone Repository

```bash
cd ~
mkdir -p repos && cd repos
git clone https://github.com/your-org/LCTK.git
cd LCTK
```

### Step 2: Run Setup Script

The setup script installs all dependencies automatically:

```bash
# Interactive installation (recommended first time)
make prepare

# Or non-interactive (for automation)
./setup-dev-env.sh -y

# Minimal install (no CUDA, no dev tools)
./setup-dev-env.sh -y --minimal
```

**What gets installed:**
- ROS 2 Humble
- Rust toolchain (stable + nightly)
- OpenCV 4.5.4+
- GStreamer and plugins
- SFCGAL library
- Python dependencies

**Time required:** ~15-20 minutes depending on your internet speed.

### Step 3: Build Project

```bash
make build
```

This runs a three-pass build process:
1. ROS 2 Rust foundation (~3 min)
2. Interface types (~1 min)
3. LCTK applications (~5 min)

**First build takes ~10 minutes.** Subsequent builds are much faster (~1-2 min).

## Verify Installation

Test with sample data:

```bash
make launch_lidar_camera_sample_data
```

In another terminal:
```bash
ros2 topic list
```

You should see topics like:
- `/sensing/lidar/top/pointcloud_raw`
- `/sensing/camera/front_center/image_raw`

If you see these topics, installation succeeded!

## Optional: GPU Acceleration

For CUDA support (NVIDIA GPUs only):

```bash
# Run setup with CUDA (default on first install)
./setup-dev-env.sh -y

# Verify CUDA installation
nvidia-smi
```

CUDA is optional. The toolkit works fine without GPU acceleration.

## Troubleshooting

**Build fails with "memory file not found":**
```bash
sudo apt-get install libstdc++-12-dev libclang-dev
```

**"SFCGAL not found" errors:**
```bash
sudo apt-get install libsfcgal-dev
```

**"Command not found" after build:**
```bash
source install/setup.bash
```

**ROS 2 daemon unresponsive:**
```bash
pkill -9 -f ros2-daemon
```

For more issues, see [Troubleshooting](./troubleshooting.md).

## Next Steps

- **Try the tutorial**: [Quick Start](./quickstart.md)
- **Calibrate sensors**: [LiDAR-Camera](./lidar-camera.md) or [Multi-LiDAR](./multi-lidar.md)
- **Adjust settings**: [Configuration](./configuration.md)
