#!/usr/bin/env bash
# Install geometric computation libraries
# Converted from ansible/roles/lctk.dev_env.geometric_libs/tasks/main.yaml

set -e

echo "Installing SFCGAL for geometric computations..."

sudo apt-get update
sudo apt-get install -y \
    libsfcgal-dev \
    libsfcgal1

echo "Geometric libraries installation complete."
