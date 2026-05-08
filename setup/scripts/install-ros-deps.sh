#!/usr/bin/env bash
# Install ROS dependencies from package.xml files

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "Installing ROS dependencies from workspace..."

source /opt/ros/humble/setup.bash

cd "$PROJECT_ROOT"

# Skip keys that don't have rosdep definitions:
# - rclrs: Rust ROS 2 client library (installed via cargo)
# - ament_python: Not a real rosdep key
# - calibration_evaluator: Local package
SKIP_KEYS="rclrs ament_python calibration_evaluator ament_cargo"

# Install dependencies from ros/ directory
if [[ -d "$PROJECT_ROOT/ros" ]]; then
    rosdep install --from-paths ros --ignore-src -y --skip-keys "$SKIP_KEYS"
else
    echo "Warning: ros/ directory not found at $PROJECT_ROOT/ros"
fi

echo "ROS dependencies installation complete."
