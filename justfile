# LCTK Build System using cargo-ros2 and colcon-cargo-ros2
#
# This justfile replaces the three-pass build system (ros2_rust → interface → packages)
# with a simplified single-pass build using cargo-ros2's automatic binding generation.

# Build all ROS packages using colcon and cargo-ros2
build:
    #!/usr/bin/env bash
    set -e
    echo "Building LCTK packages with cargo-ros2..."
    source /opt/ros/jazzy/setup.bash
    export OPENCV_PKGCONFIG_NAME=opencv4
    export RUST_LOG=debug
    mkdir -p build_logs
    colcon build --symlink-install \
        --base-paths ros \
        2>&1 | tee build_logs/colcon_build.log
    echo "✓ Build complete!"

# Clean all build artifacts
clean:
    #!/usr/bin/env bash
    echo "Cleaning build artifacts..."
    rm -rf build install log target .cargo build_logs
    echo "✓ Clean complete!"
