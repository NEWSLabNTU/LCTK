# Installation

Set up LCTK on Ubuntu 22.04 LTS with ROS 2 Humble.

## System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Disk space**: several GB for dependencies and build artifacts; allow more if you
  install the optional CUDA toolkit
- **Network**: Internet connection for downloading dependencies

## Quick Installation

### Step 1: Clone the repository

```bash
mkdir -p ~/repos
cd ~/repos
git clone --recurse-submodules https://github.com/NEWSLabNTU/LCTK.git
cd LCTK
```

If you cloned the repository without `--recurse-submodules`, initialize its submodules
from the repository root before continuing:

```bash
git submodule update --init --recursive
```

### Step 2: Run the setup script

```bash
./setup.sh
```

The script prints an installation plan and asks whether to proceed. It installs `just`
if necessary and may request `sudo` access. The default plan includes:

- ROS 2 Humble, `rosdep`, and the workspace's ROS dependencies
- Stable and nightly Rust, `cargo-ament-build`, and `cargo-nextest`
- `colcon-cargo-ros2` and `play_launch`
- OpenCV, GStreamer, packet-capture libraries, and SFCGAL
- The system Python scientific stack and Python lint tools
- Optional development and documentation tools that are selected by default

CUDA is optional and is not in the default plan. Use `./setup.sh --yes` to accept the
default plan without the confirmation prompt, or `./setup.sh --dry-run` to print the
resolved plan and stop. Use `--only` or `--skip` when the default plan needs
to be narrowed.

After setup completes, reload the shell:

```bash
source ~/.bashrc
```

To inspect or verify the setup, use flags—not positional subcommands:

```bash
./setup.sh --status
./setup.sh --verify
```

### Step 3: Build the project

```bash
just build
source install/setup.bash
```

Always use `just build`; it applies the workspace's required ROS and Rust build
configuration.

## Verify the Installation

First validate the shipped sample session without launching it:

```bash
just check sample3-hollow-velodyne
```

Then run the sample playback and calibration graph:

```bash
just demo
```

`just demo` runs in the foreground. While it is running, open
`http://localhost:8000` to view the `play_launch` status UI. In another terminal,
source the workspace and inspect the sample topics:

```bash
source install/setup.bash
ros2 topic list
```

The shipped session should publish at least:

- `/sensing/lidar/top/pointcloud_raw`
- `/sensing/camera/front_center/image_raw`

For a slower end-to-end assertion that detections flow through the shipped session,
run `just smoke` after stopping the demo.

## Optional: CUDA Toolkit

Nothing in LCTK's default build path requires CUDA. To install the optional CUDA step
and its declared dependencies, use:

```bash
./setup.sh --only cuda
source ~/.bashrc
```

Verify the toolkit with:

```bash
nvcc --version
```

`nvidia-smi` checks the NVIDIA driver, not whether the CUDA toolkit was installed. The
bundled installer currently targets the Ubuntu 22.04 x86_64 CUDA repository; do not use
this step unchanged on Jetson/aarch64 systems.

## Troubleshooting

**Missing C/C++ build tools or headers:**

```bash
./setup.sh --only build-tools
```

**`SFCGAL not found` errors:**

```bash
./setup.sh --only geometric-libs
```

**Commands are not found after setup or build:**

```bash
source ~/.bashrc
source install/setup.bash
```

Keep the apt-provided Python, NumPy, SciPy, and setuptools versions used by ROS 2 and
OpenCV; `pip3 install --user` versions can shadow them and break the build or runtime.

**ROS 2 daemon is unresponsive:**

```bash
pkill -9 -f ros2-daemon
```

**Check setup status:**

```bash
./setup.sh --status
```

For more issues, see [Troubleshooting](./troubleshooting.md).

## Next Steps

- **Try the tutorial**: [Quick Start](./quickstart.md)
- **Calibrate sensors**: [LiDAR-Camera](./lidar-camera.md) or [Multi-LiDAR](./multi-lidar.md)
- **Adjust settings**: [Configuration](./configuration.md)
