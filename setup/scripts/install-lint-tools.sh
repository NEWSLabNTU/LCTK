#!/usr/bin/env bash
# Install ruff and uv (L-25).
#
# `just lint` / `just lint-py` run ruff, and regenerating the board-cluster-detector
# parity fixtures needs `uv run python tools/export_golden.py`. Neither has an apt
# package on jammy, and nothing in setup used to install them, so a freshly set-up
# machine could not lint and started with two failing Rust tests whose cause was
# documented only in a fixtures README.
#
# Both ship as self-contained static binaries with no Python dependencies, so they
# cannot drag in the setuptools/numpy/scipy that CLAUDE.md Known Issue 3 warns about.
# musl builds are used for both so the same command works on the Jetson hosts.

set -euo pipefail

# Pinned per L-09; override to move a pin deliberately.
RUFF_VERSION="${RUFF_VERSION:-0.16.3}"
UV_VERSION="${UV_VERSION:-0.12.5}"
LINT_TOOLS_DIR="${LINT_TOOLS_DIR:-$HOME/.local/bin}"

TARGET="$(uname -m)-unknown-linux-musl"

# Ubuntu's ~/.profile only adds ~/.local/bin to PATH when the directory exists at login.
mkdir -p "$LINT_TOOLS_DIR"

# Each release tarball holds a single <target>/ directory, so strip one component and
# the binary lands at the top of the temp dir.
install_release() {
    local name="$1" version="$2" repo="$3" version_env="$4"
    local url="https://github.com/${repo}/releases/download/${version}/${name}-${TARGET}.tar.gz"

    if command -v "$name" >/dev/null 2>&1; then
        echo "${name} already installed: $("$name" --version)"
        return 0
    fi

    echo "Installing ${name} ${version}..."
    local tmp
    tmp="$(mktemp -d)"
    if ! curl -fsSL --retry 3 -o "$tmp/archive.tar.gz" "$url"; then
        rm -rf "$tmp"
        echo "error: failed to download ${name} ${version}." >&2
        echo "       URL: ${url}" >&2
        echo "       Check https://github.com/${repo}/releases and rerun with" >&2
        echo "       ${version_env}=<good version> if the pin has gone stale." >&2
        return 1
    fi
    tar -xzf "$tmp/archive.tar.gz" -C "$tmp" --strip-components=1
    install -m 0755 "$tmp/$name" "$LINT_TOOLS_DIR/$name"
    rm -rf "$tmp"
}

install_release ruff "$RUFF_VERSION" astral-sh/ruff RUFF_VERSION
install_release uv "$UV_VERSION" astral-sh/uv UV_VERSION

if ! command -v ruff >/dev/null 2>&1 || ! command -v uv >/dev/null 2>&1; then
    echo ""
    echo "Installed into ${LINT_TOOLS_DIR}, which is not on this shell's PATH."
    echo "For this shell:  export PATH=\"${LINT_TOOLS_DIR}:\$PATH\""
    echo "Permanently:     log out and back in (~/.profile picks it up)."
fi

echo "Lint tooling installation complete."
