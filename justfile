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
# Note: ros/conflux is built separately (it uses git rclrs with DynamicMessage support)
build:
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build \
        --base-paths ros \
        --ignore-paths ros/conflux \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --cargo-args --profile=test-release

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
    CONFIG=$(ros2 pkg prefix lctk_launch --share)/config/examples/seyond_left.yaml
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
    SHARE=$(ros2 pkg prefix lctk_launch --share)
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$SHARE/config/examples/two_lidar.yaml \
        rviz_config:=$SHARE/config/rviz/two_lidar_calibration.rviz \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }}

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
        use_advanced_solver:={{ use_advanced_solver }}

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

republish:
    ros2 run image_transport republish compressed raw \
      --ros-args \
      -r in/compressed:=/camera/left/image_raw/compressed \
      -r out:=/camera/left/image_raw