#!/usr/bin/env bash
# Install C++ development and build tools
# Converted from ansible/roles/lctk.dev_env.build_tools/tasks/main.yaml

set -e

echo "Installing C++ development tools..."

sudo apt-get update
sudo apt-get install -y \
    libstdc++-12-dev \
    libclang-dev \
    llvm-dev \
    clang \
    clang-format \
    clang-tidy

echo "Installing mathematical libraries..."
sudo apt-get install -y \
    libeigen3-dev \
    libomp-dev \
    libtbb-dev \
    libfmt-dev

echo "Installing additional utilities..."
sudo apt-get install -y \
    moreutils \
    jq \
    htop

echo "Build tools installation complete."
