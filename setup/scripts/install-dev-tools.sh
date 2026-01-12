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

echo "Development tools installation complete."
