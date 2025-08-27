#!/bin/bash

# LCTK System Dependencies Installation Script for Ubuntu 22.04
# This script installs all required system dependencies for the LCTK project

set -e  # Exit on any error

echo "Installing LCTK system dependencies on Ubuntu 22.04..."

# Check if ROS 2 is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/"
    echo "Please install ROS 2 Humble first, then run this script."
    echo "See: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "Found ROS 2 Humble installation"

# Update system
echo "Updating system packages..."
sudo apt update

# Install core build tools
echo "Installing core build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    curl \
    wget

# Install mathematical libraries
echo "Installing mathematical libraries..."
sudo apt install -y \
    libeigen3-dev \
    libomp-dev \
    libtbb-dev \
    libfmt-dev

# Install Python environment
echo "Installing Python environment..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools

# Install Rust FFI dependencies
echo "Installing Rust FFI dependencies..."
sudo apt install -y \
    libclang-dev \
    llvm-dev

# Install additional utilities
echo "Installing additional utilities..."
sudo apt install -y \
    moreutils

# Install geometric computation libraries
echo "Installing geometric computation libraries..."
sudo apt install -y \
    libsfcgal-dev \
    libsfcgal1

# Install OpenCV
echo "Installing OpenCV..."
sudo apt install -y \
    libopencv-dev \
    opencv-data

# Install Python libraries for TUI applications
echo "Installing Python libraries for GUI/TUI applications..."
sudo apt install -y \
    python3-numpy \
    python3-scipy

# Install development libraries for testing (optional)
echo "Installing optional development libraries..."
sudo apt install -y \
    libgtest-dev \
    lcov \
    pybind11-dev \
    python3-pybind11

# Check if Rust is installed, if not install it
if ! command -v cargo &> /dev/null; then
    echo "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source $HOME/.cargo/env
else
    echo "Rust is already installed"
fi

# Check if Poetry is installed, if not install it
if ! command -v poetry &> /dev/null; then
    echo "Installing Poetry..."
    curl -sSL https://install.python-poetry.org | python3 -
    export PATH="$HOME/.local/bin:$PATH"
else
    echo "Poetry is already installed"
fi

# Install Python dependencies for colcon
echo "Installing colcon Rust extensions..."
pip3 install --user git+https://github.com/colcon/colcon-cargo.git
pip3 install --user git+https://github.com/colcon/colcon-ros-cargo.git

# Install basic Python dependencies
echo "Installing Python dependencies..."
pip3 install --user \
    cffi>=1.0.0 \
    setuptools>=45 \
    wheel

echo ""
echo "System dependencies installation completed!"
echo ""
echo "Next steps:"
echo "1. Source ROS 2 environment: source /opt/ros/humble/setup.bash"
echo "2. Install ROS dependencies: rosdep update && rosdep install --from-paths src --ignore-src -r -y"
echo "3. Source Rust environment: source ~/.cargo/env (if just installed)"
echo "4. Add Poetry to PATH: export PATH=\"\$HOME/.local/bin:\$PATH\" (if just installed)"
echo "5. Build the project: make build"
