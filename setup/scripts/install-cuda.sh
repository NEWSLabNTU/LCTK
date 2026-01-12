#!/usr/bin/env bash
# Install CUDA toolkit (optional)
# Converted from ansible/roles/lctk.dev_env.cuda/tasks/main.yaml

set -e

echo "Installing CUDA toolkit..."

# Check if CUDA is already installed
if [[ -d "/usr/local/cuda" ]]; then
    echo "CUDA is already installed."
else
    echo "Adding NVIDIA package repositories..."
    cd /tmp
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
    sudo dpkg -i cuda-keyring_1.0-1_all.deb
    rm cuda-keyring_1.0-1_all.deb

    echo "Updating apt cache..."
    sudo apt-get update

    echo "Installing CUDA toolkit 11.8..."
    sudo apt-get install -y cuda-toolkit-11-8
fi

# Set CUDA environment variables in bashrc if not already present
if ! grep -q '# LCTK CUDA' "$HOME/.bashrc"; then
    cat >> "$HOME/.bashrc" << 'EOF'

# LCTK CUDA configuration
export CUDA_PATH=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
EOF
fi

echo "CUDA installation complete."
