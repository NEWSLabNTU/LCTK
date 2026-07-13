# LCTK Build System
#
# This justfile provides build and launch commands for LCTK.

set shell := ["bash", "-uc"]

# Default configuration values
debug_mode := "true"
log_level := "info"
rviz_enabled := "true"
# Processing mode: "offline" (RELIABLE QoS, perfect sync) or "realtime" (BEST_EFFORT QoS, no buffering)
mode := "offline"
use_advanced_solver := "false"
enable_overlay := "true"
enable_judge := "true"

# Show available commands
default:
    @just --list

# Build all ROS packages using colcon and cargo-ros2
# The conflux packages LCTK depends on (conflux_cpp + conflux_py) are built
# first by `build-conflux`; the rest of ros/conflux (conflux, conflux-ros2)
# is excluded because it uses a git rclrs that conflicts with our crates.io rclrs.
build: build-conflux
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build \
        --base-paths ros \
        --packages-ignore conflux conflux_cpp conflux_py \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --cargo-args --profile=test-release

# Build the conflux packages LCTK needs at runtime.
# conflux_cpp builds the libconflux_ffi.so that conflux_py loads via ctypes; the
# solver nodes import conflux_py and fail to start without it. Only these two
# packages are selected so the git-rclrs conflux/conflux-ros2 packages are skipped.
build-conflux: _check-setuptools
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build \
        --base-paths ros \
        --packages-select conflux_cpp conflux_py \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Guard: ROS 2 Humble's ament_python builds need the apt setuptools (59.6.0).
# A pip `--user` setuptools shadows it on sys.path, and setuptools >= 80 removed the
# `setup.py develop --editable` step colcon uses for --symlink-install. Without this
# check the build dies deep inside colcon with a bare
# "error: option --editable not recognized", which points nowhere near the cause.
_check-setuptools:
    #!/usr/bin/env bash
    set -eo pipefail
    location=$(python3 -c 'import setuptools; print(setuptools.__file__)')
    version=$(python3 -c 'import setuptools; print(setuptools.__version__)')
    if [[ "$location" != /usr/lib/python3/dist-packages/* ]]; then
        echo "error: setuptools $version ($location) shadows the apt setuptools." >&2
        echo "       ROS 2 Humble's ament_python packages need the apt version (59.6.0);" >&2
        echo "       every ament_python package will fail with" >&2
        echo "       'error: option --editable not recognized'." >&2
        echo "" >&2
        echo "       Fix with:  pip3 uninstall -y setuptools" >&2
        exit 1
    fi

# Set up development environment (install all dependencies)
setup *args:
    ./setup.sh {{ args }}

# Clean all build artifacts
clean:
    rm -rf build install log target

# Format code (Rust + Python)
format:
    cargo +nightly fmt
    ruff format ros/

# Run formatting and linting checks (Rust + Python)
lint:
    cargo +nightly fmt --check
    cargo clippy --all-targets --
    ruff check ros/
    ruff format --check ros/

# Run all tests (Rust + Python)
test:
    #!/usr/bin/env bash
    set -eo pipefail
    cargo nextest run --cargo-profile test-release --no-fail-fast
    source install/setup.bash
    pytest ros/lctk_launch/test/ -v --no-header

# Launch LiDAR-camera calibration (config-driven)
lidar-camera:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    CONFIG=$(ros2 pkg prefix lctk_launch --share)/config/examples/sample_data.yaml
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$CONFIG \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch two-LiDAR calibration (config-driven)
two-lidar:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    CONFIG=$(ros2 pkg prefix lctk_launch --share)/config/examples/two_lidar.yaml
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$CONFIG \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }}

# Launch sample data playback only
sample-data:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        lctk_sample_data lidar_camera.launch.xml

# Launch demo (sample data + calibration pipeline)
demo:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch demo.launch.py \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch RViz for calibration visualization
rviz:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run rviz2 rviz2

# Launch config-driven calibration pipeline
# Usage: just calibrate /path/to/config.yaml
# Example: just calibrate $(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/sample_data.yaml
calibrate config_file:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:={{ config_file }} \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch interactive advanced solver controller
advanced-solver-controller:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run interactive_solver_controller interactive_solver_controller
