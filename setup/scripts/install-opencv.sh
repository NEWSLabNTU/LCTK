#!/usr/bin/env bash
# Install OpenCV and related libraries
# Converted from ansible/roles/lctk.dev_env.opencv/tasks/main.yaml

set -e

echo "Installing OpenCV and related libraries..."

sudo apt-get update
sudo apt-get install -y \
    libopencv-dev \
    opencv-data \
    python3-opencv

# Set OpenCV environment variable in bashrc if not already present
if ! grep -q 'export OPENCV_PKGCONFIG_NAME=opencv4' "$HOME/.bashrc"; then
    echo 'export OPENCV_PKGCONFIG_NAME=opencv4' >> "$HOME/.bashrc"
fi

echo "OpenCV installation complete."
