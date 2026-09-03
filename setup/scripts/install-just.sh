#!/usr/bin/env bash
# Install the `just` command runner as a prebuilt binary.
#
# Not `cargo install just`: cargo comes from the `rust` step, which is part of the setup
# that cannot start without just. Not apt either -- there is no `just` package on jammy.
# The upstream installer resolves x86_64/aarch64 musl targets, so one command covers the
# workstations and the Jetson hosts with no glibc or toolchain dependency.

set -eu

# Pinned per L-09; override to move the pin deliberately.
JUST_VERSION="${JUST_VERSION:-1.58.0}"
JUST_INSTALL_DIR="${JUST_INSTALL_DIR:-$HOME/.local/bin}"

if command -v just >/dev/null 2>&1; then
    echo "just already installed: $(just --version)"
    exit 0
fi

# Ubuntu's ~/.profile only adds ~/.local/bin to PATH when the directory exists at login,
# so create it before installing or the binary lands somewhere PATH never looks.
mkdir -p "$JUST_INSTALL_DIR"

echo "Installing just ${JUST_VERSION} into ${JUST_INSTALL_DIR}..."
curl --proto '=https' --tlsv1.2 -sSf --retry 3 https://just.systems/install.sh \
    | bash -s -- --tag "${JUST_VERSION}" --to "${JUST_INSTALL_DIR}"

if ! command -v just >/dev/null 2>&1; then
    echo ""
    echo "just is installed at ${JUST_INSTALL_DIR}/just but is not on PATH."
    echo "For this shell:  export PATH=\"${JUST_INSTALL_DIR}:\$PATH\""
    echo "Permanently:     log out and back in (~/.profile picks it up)."
fi

echo "just installation complete."
