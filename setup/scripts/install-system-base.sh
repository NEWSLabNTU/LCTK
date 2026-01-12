#!/usr/bin/env bash
# Install basic system packages
# Converted from ansible/roles/lctk.dev_env.system_base/tasks/main.yaml

set -e

echo "Installing basic system packages..."

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    curl \
    wget \
    software-properties-common \
    lsb-release \
    gnupg \
    ca-certificates

echo "Basic system packages installation complete."
