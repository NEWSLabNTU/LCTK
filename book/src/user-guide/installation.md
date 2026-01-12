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
./setup.sh
```

This runs an interactive setup. Follow the prompts to select optional components (CUDA, dev tools).

**What gets installed:**
- ROS 2 Humble
- Rust toolchain (stable + nightly)
- OpenCV 4.5.4+
- GStreamer and plugins
- SFCGAL library
- Python dependencies
- colcon-cargo-ros2 for Rust ROS 2 integration

**Time required:** ~15-20 minutes depending on your internet speed.

### Step 3: Build Project

After setup completes, reload your shell and build:

```bash
source ~/.bashrc
just build
```

**First build takes ~5-10 minutes.** Subsequent builds are much faster (~1-2 min).

## Verify Installation

Test with sample data:

```bash
just demo
```

Open a web browser to `http://localhost:8080` to see the web UI.

In another terminal, check running topics:
```bash
source install/setup.bash
ros2 topic list
```

You should see topics like:
- `/sensing/lidar/top/pointcloud_raw`
- `/sensing/camera/front_center/image_raw`

If you see these topics, installation succeeded!

## Optional: GPU Acceleration

For CUDA support (NVIDIA GPUs only), select "Install CUDA toolkit" during interactive setup, or run:

```bash
./setup.sh cuda
```

Verify CUDA installation:
```bash
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

**Check setup status:**
```bash
./setup.sh status
```

For more issues, see [Troubleshooting](./troubleshooting.md).

## Next Steps

- **Try the tutorial**: [Quick Start](./quickstart.md)
- **Calibrate sensors**: [LiDAR-Camera](./lidar-camera.md) or [Multi-LiDAR](./multi-lidar.md)
- **Adjust settings**: [Configuration](./configuration.md)
