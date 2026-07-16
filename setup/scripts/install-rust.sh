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
    curl --proto '=https' --tlsv1.2 -sSf --retry 3 https://sh.rustup.rs | sh -s -- -y
    # The toolchain itself is pinned by rust-toolchain.toml at the repo root;
    # rustup respects it on first `cargo` invocation inside the workspace.
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

# Cargo tools are PINNED to the versions this workspace is known to build with (L-09);
# an unpinned `cargo install` floats to whatever released last night. Override via env
# (e.g. CARGO_AMENT_BUILD_VERSION=0.1.12) to move a pin deliberately.
CARGO_AMENT_BUILD_VERSION="${CARGO_AMENT_BUILD_VERSION:-0.1.11}"
CARGO_NEXTEST_VERSION="${CARGO_NEXTEST_VERSION:-0.9.137}"

# Install cargo-ament-build for ROS 2 integration
echo "Installing cargo-ament-build ${CARGO_AMENT_BUILD_VERSION}..."
if ! command -v cargo-ament-build &> /dev/null; then
    cargo install --locked --version "${CARGO_AMENT_BUILD_VERSION}" cargo-ament-build
fi

# Install cargo-nextest for testing
echo "Installing cargo-nextest ${CARGO_NEXTEST_VERSION}..."
if ! command -v cargo-nextest &> /dev/null; then
    cargo install --locked --version "${CARGO_NEXTEST_VERSION}" cargo-nextest
fi

echo "Rust toolchain installation complete."
