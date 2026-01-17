#!/usr/bin/env bash
# Install colcon-cargo-ros2 and play_launch for Rust ROS 2 integration
# Converted from ansible/roles/lctk.dev_env.colcon_rust/tasks/main.yaml

set -e

echo "Checking for conflicting packages..."

# Remove conflicting packages if present
if pip3 list 2>/dev/null | grep -qE '^colcon-cargo\s'; then
    echo "Removing conflicting colcon-cargo..."
    pip3 uninstall -y colcon-cargo
fi

if pip3 list 2>/dev/null | grep -qE '^colcon-ros-cargo\s'; then
    echo "Removing conflicting colcon-ros-cargo..."
    pip3 uninstall -y colcon-ros-cargo
fi

echo "Installing colcon-cargo-ros2..."
pip3 install --user colcon-cargo-ros2

echo "Installing play_launch..."
pip3 install --user 'play_launch>=0.5.0'

# Remove pip-installed empy if present (can conflict with system package)
echo "Removing pip-installed empy if present..."
pip3 uninstall -y empy 2>/dev/null || true

echo "Colcon Rust integration installation complete."
