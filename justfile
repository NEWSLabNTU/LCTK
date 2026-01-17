# LCTK Build System
#
# This justfile provides build and launch commands for LCTK.

set shell := ["bash", "-uc"]

# Default configuration values
debug_mode := "true"
enable_icp_iteration_debug := "true"
enable_evaluator := "true"
enable_overlay := "true"
log_level := "info"
rviz_enabled := "true"
use_best_effort_qos := "true"
use_advanced_solver := "true"
use_synchronized_input := "false"
camera_topic := "/sensing/camera/zedxm/right/color/rect/image"
pointcloud_topic := "/sensing/lidar/concatenated/pointcloud"

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

# Clean all build artifacts
clean:
    rm -rf build install log target

# Format code with rustfmt
format:
    cargo +nightly fmt

# Run formatting and linting checks
lint:
    cargo +nightly fmt --check
    cargo clippy --config build/ros2_cargo_config.toml --all-targets --

# Run tests with cargo nextest
test:
    cargo nextest run --config build/ros2_cargo_config.toml --cargo-profile test-release --no-fail-fast

# Launch LiDAR-camera calibration
lidar-camera:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch lidar_camera_calibration.launch.xml \
        debug_mode:={{ debug_mode }} \
        enable_icp_iteration_debug:={{ enable_icp_iteration_debug }} \
        enable_evaluator:={{ enable_evaluator }} \
        enable_overlay:={{ enable_overlay }} \
        enable_rviz:={{ rviz_enabled }} \
        log_level:={{ log_level }} \
        use_best_effort_qos:={{ use_best_effort_qos }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        use_synchronized_input:={{ use_synchronized_input }} \
        camera_topic:={{ camera_topic }} \
        pointcloud_topic:={{ pointcloud_topic }}

# Launch two-LiDAR calibration
two-lidar:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch two_lidar_calibration.launch.xml

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
        lctk_launch lidar_camera_demo.launch.xml \
        debug_mode:={{ debug_mode }} \
        enable_icp_iteration_debug:={{ enable_icp_iteration_debug }} \
        enable_judge:={{ enable_evaluator }} \
        enable_overlay:={{ enable_overlay }} \
        enable_rviz:={{ rviz_enabled }} \
        log_level:={{ log_level }} \
        use_best_effort_qos:={{ use_best_effort_qos }} \
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
        use_best_effort_qos:={{ use_best_effort_qos }}

# Launch interactive advanced solver controller
advanced-solver-controller:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run interactive_solver_controller interactive_solver_controller
