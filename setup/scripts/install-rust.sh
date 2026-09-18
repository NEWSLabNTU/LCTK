#!/usr/bin/env bash
# Install Rust toolchain
# Converted from ansible/roles/lctk.dev_env.rust/tasks/main.yaml

set -e

echo "Installing Rust toolchain..."

# Rustup installs Cargo and its subcommands under this directory.  Put it on PATH
# before checking for an existing installation so a fresh setup shell does not
# reinstall Rust merely because ~/.cargo/bin has not been loaded from ~/.bashrc yet.
CARGO_BIN="${CARGO_HOME:-$HOME/.cargo}/bin"
export PATH="${CARGO_BIN}:$PATH"

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

# Add Rust to PATH in bashrc if not already present
if ! grep -q '.cargo/bin' "$HOME/.bashrc"; then
    printf 'export PATH="%s:$PATH"\n' "$CARGO_BIN" >> "$HOME/.bashrc"
fi

# Install nightly toolchain
echo "Installing Rust nightly toolchain..."
rustup toolchain install nightly

# Install components
echo "Installing Rust components..."
rustup component add rustfmt clippy

# Cargo tools are PINNED to the versions this workspace is known to build with (L-09);
# an unpinned `cargo install` floats to whatever released last night. Override via env
# (e.g. CARGO_NEXTEST_VERSION=0.9.138) to move a pin deliberately.
CARGO_NEXTEST_VERSION="${CARGO_NEXTEST_VERSION:-0.9.137}"

# Install cargo-nextest for testing
echo "Installing cargo-nextest ${CARGO_NEXTEST_VERSION}..."
if ! command -v cargo-nextest &> /dev/null; then
    cargo install --locked --version "${CARGO_NEXTEST_VERSION}" cargo-nextest
fi

echo "Rust toolchain installation complete."
