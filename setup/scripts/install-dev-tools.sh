#!/usr/bin/env bash
# Install development and testing tools (optional)
# Converted from ansible/roles/lctk.dev_env.dev_tools/tasks/main.yaml

set -e

echo "Installing development and testing tools..."

sudo apt-get update
sudo apt-get install -y \
    gdb \
    valgrind \
    strace \
    ltrace \
    perf-tools-unstable \
    linux-tools-generic \
    libgtest-dev \
    lcov \
    pybind11-dev \
    python3-pybind11

echo "Installing code quality tools..."
sudo apt-get install -y \
    cppcheck \
    iwyu \
    ccache

echo "Installing documentation tools..."
sudo apt-get install -y \
    doxygen \
    graphviz

echo "Installing mdbook for documentation..."
# Install mdbook and mermaid preprocessor via cargo
if command -v cargo &> /dev/null; then
    cargo install mdbook mdbook-mermaid

    # Install mermaid JS files if book directory exists
    if [[ -d "${PROJECT_ROOT:-..}/book" ]]; then
        echo "Setting up mermaid for mdbook..."
        cd "${PROJECT_ROOT:-..}/book"
        mdbook-mermaid install .
    fi
else
    echo "Warning: cargo not found, skipping mdbook installation"
fi

echo "Development tools installation complete."
