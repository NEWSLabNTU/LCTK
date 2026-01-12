#!/usr/bin/env bash
# Setup rosdep and initialize rosdep database

set -e

echo "Setting up rosdep..."

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check if rosdep is initialized
if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
    echo "Initializing rosdep (requires sudo)..."
    sudo rosdep init
fi

# Update rosdep database
echo "Updating rosdep database..."
rosdep update

echo "Rosdep setup complete."
