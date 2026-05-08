#!/usr/bin/env bash
# Install Rust toolchain
# Converted from ansible/roles/lctk.dev_env.rust/tasks/main.yaml

set -e

echo "Installing Rust toolchain..."

# Check if Rust is already installed
if command -v cargo &> /dev/null; then
    echo "Rust is already installed."
else
    # Download and install Rust
    echo "Downloading Rust installer..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
fi

# Ensure cargo is in PATH for this script
export PATH="$HOME/.cargo/bin:$PATH"

# Add Rust to PATH in bashrc if not already present
if ! grep -q '.cargo/bin' "$HOME/.bashrc"; then
    echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> "$HOME/.bashrc"
fi

# Install nightly toolchain
echo "Installing Rust nightly toolchain..."
rustup toolchain install nightly

# Install components
echo "Installing Rust components..."
rustup component add rustfmt clippy

# Install cargo-ament-build for ROS 2 integration
echo "Installing cargo-ament-build..."
if ! command -v cargo-ament-build &> /dev/null; then
    cargo install cargo-ament-build
fi

# Install cargo-nextest for testing
echo "Installing cargo-nextest..."
if ! command -v cargo-nextest &> /dev/null; then
    cargo install --locked cargo-nextest
fi

echo "Rust toolchain installation complete."
