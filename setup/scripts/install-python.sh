#!/usr/bin/env bash
# Install Python environment
# Converted from ansible/roles/lctk.dev_env.python/tasks/main.yaml

set -e

echo "Installing Python environment..."

sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    python3-empy

# Remove pip-installed empy if present (conflicts with system package)
echo "Removing pip-installed empy if present..."
pip3 uninstall -y empy 2>/dev/null || true

echo "Installing Python libraries for scientific computing..."
sudo apt-get install -y \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-pandas

echo "Installing Python build dependencies..."
pip3 install --user \
    "cffi>=1.0.0" \
    "setuptools>=45" \
    wheel

echo "Python environment installation complete."
