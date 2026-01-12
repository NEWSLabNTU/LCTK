#!/usr/bin/env bash
# Setup rosdep and install ROS dependencies
# Converted from ansible/roles/lctk.dev_env.rosdep/tasks/main.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "Setting up rosdep..."

# Check if rosdep is initialized
if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
    echo "Initializing rosdep (requires sudo)..."
    sudo rosdep init
fi

# Update rosdep database
echo "Updating rosdep database..."
rosdep update

# Install ROS dependencies from package.xml files
if [[ -d "$PROJECT_ROOT/src" ]] || [[ -d "$PROJECT_ROOT/ros" ]]; then
    echo "Installing ROS dependencies from workspace..."
    source /opt/ros/humble/setup.bash

    # Try both src/ and ros/ directories
    if [[ -d "$PROJECT_ROOT/ros" ]]; then
        rosdep install --from-paths "$PROJECT_ROOT/ros" --ignore-src -y --rosdistro humble || true
    fi
    if [[ -d "$PROJECT_ROOT/src" ]]; then
        rosdep install --from-paths "$PROJECT_ROOT/src" --ignore-src -y --rosdistro humble || true
    fi
fi

echo "Rosdep setup complete."
