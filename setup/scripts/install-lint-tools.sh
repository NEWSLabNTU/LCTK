#!/usr/bin/env bash
# Install ruff (L-25).
#
# `just lint` / `just lint-py` run ruff. It has no apt package on jammy, and
# nothing in setup used to install it, so a freshly set-up machine could not lint.
#
# Ruff ships as a self-contained static binary with no Python dependencies, so it
# cannot drag in the setuptools/numpy/scipy that AGENTS.md Known Issue 3 warns about.
# The musl build is used so the same command works on the Jetson hosts.

set -euo pipefail

# Pinned per L-09; override to move a pin deliberately.
RUFF_VERSION="${RUFF_VERSION:-0.16.3}"
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

if ! command -v ruff >/dev/null 2>&1; then
    echo ""
    echo "Installed into ${LINT_TOOLS_DIR}, which is not on this shell's PATH."
    echo "For this shell:  export PATH=\"${LINT_TOOLS_DIR}:\$PATH\""
    echo "Permanently:     log out and back in (~/.profile picks it up)."
fi

echo "Lint tooling installation complete."
