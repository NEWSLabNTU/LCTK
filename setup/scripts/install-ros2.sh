#!/usr/bin/env bash
# Install ROS 2 Humble
# Converted from ansible/roles/lctk.dev_env.ros2/tasks/main.yaml

set -e

ROSDISTRO="${ROSDISTRO:-humble}"

echo "Installing ROS 2 ${ROSDISTRO}..."

# Check if ROS 2 is already installed
if [[ -d "/opt/ros/${ROSDISTRO}" ]]; then
    echo "ROS 2 ${ROSDISTRO} is already installed."
else
    # Install locales
    sudo apt-get update
    sudo apt-get install -y locales software-properties-common curl

    # Enable universe repository
    sudo add-apt-repository -y universe
    sudo apt-get update

    # ros-apt-source is PINNED to a known-good release (L-09): the old behavior of
    # curling api.github.com for `releases/latest` made a fresh setup depend on GitHub
    # API availability, rate limits, and response-format stability, and could silently
    # pick up an untested release. Override with ROS_APT_VERSION=x.y.z if needed.
    ROS_APT_VERSION="${ROS_APT_VERSION:-1.2.0}"
    UBUNTU_CODENAME=$(source /etc/os-release && echo "$VERSION_CODENAME")

    echo "Using ros-apt-source version: ${ROS_APT_VERSION}"

    # Download and install ros-apt-source
    ROS_APT_DEB="/tmp/ros2-apt-source.deb"
    if ! curl -fSL --retry 3 -o "${ROS_APT_DEB}" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.${UBUNTU_CODENAME}_all.deb"; then
        echo "error: failed to download ros-apt-source ${ROS_APT_VERSION} for ${UBUNTU_CODENAME}." >&2
        echo "       Check https://github.com/ros-infrastructure/ros-apt-source/releases and" >&2
        echo "       rerun with ROS_APT_VERSION=<good version> if the pin has gone stale." >&2
        exit 1
    fi

    sudo apt-get install -y "${ROS_APT_DEB}"
    sudo apt-get update

    # Install ROS 2 desktop
    sudo apt-get install -y ros-${ROSDISTRO}-desktop ros-${ROSDISTRO}-ros-base
fi

# Install additional ROS 2 packages
echo "Installing additional ROS 2 packages..."
sudo apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-${ROSDISTRO}-gscam \
    ros-${ROSDISTRO}-velodyne \
    ros-${ROSDISTRO}-velodyne-driver \
    ros-${ROSDISTRO}-velodyne-pointcloud \
    ros-${ROSDISTRO}-vision-msgs \
    ros-${ROSDISTRO}-tf2-ros \
    ros-${ROSDISTRO}-tf2-tools \
    ros-${ROSDISTRO}-interactive-markers \
    ros-${ROSDISTRO}-image-transport \
    ros-${ROSDISTRO}-cv-bridge \
    ros-${ROSDISTRO}-image-geometry \
    ros-${ROSDISTRO}-test-msgs

echo "ROS 2 ${ROSDISTRO} installation complete."
