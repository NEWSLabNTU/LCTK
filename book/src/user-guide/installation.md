# Installation

LCTK is built from its source repository. The supported baseline is Ubuntu 22.04 LTS
with ROS 2 Humble.

## 1. Clone the repository

```bash
mkdir -p ~/repos
cd ~/repos
git clone --recurse-submodules https://github.com/NEWSLabNTU/LCTK.git
cd LCTK
```

If you already cloned without `--recurse-submodules`, initialize the required
submodules from the repository root:

```bash
git submodule update --init --recursive
```

## 2. Install dependencies and build

```bash
./setup.sh
```

Review the installation plan before proceeding. Setup installs system packages and
development tools and may ask for your password through `sudo`. When setup finishes,
open a new login shell (or log out and back in) if `just` or `cargo` is not found; this
loads the command paths added by setup. Then build and source the workspace:

```bash
just build
source install/setup.bash
```

`just build` builds the ROS 2 workspace with the repository's required settings.

To preview the setup plan without installing, use `./setup.sh --dry-run`. To inspect
or check the installed dependencies later, use `./setup.sh --status` or
`./setup.sh --verify`.

## 3. Try the sample session

```bash
just check sample3-hollow-velodyne
just demo
```

The demo plays the included LiDAR and camera recording and starts the calibration
graph. Its launch status page is at <http://localhost:8000>; assisted capture review
is at <http://localhost:8080>. The sample recording shows that the pipeline runs; it
contains only one board placement and is not a field-validated calibration.

## Optional CUDA toolkit

CUDA is not required by the default build. If you need the optional CUDA setup step,
run:

```bash
./setup.sh --only cuda
```

This installer targets Ubuntu 22.04 x86_64. It is not the Jetson/aarch64 installation
path.

For calibration workflows, see the [Quick Start](./quickstart.md). For setup problems,
see [Troubleshooting](./troubleshooting.md).
