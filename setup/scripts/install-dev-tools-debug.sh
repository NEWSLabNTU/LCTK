#!/usr/bin/env bash
# Debuggers, profilers and code-quality tools.
#
# Split from the old combined `dev-tools` step, which also installed mdbook via cargo.
# That step declared only `system-base` as a dependency, so on a machine without cargo
# it printed "Warning: cargo not found, skipping mdbook installation", exited 0, and was
# marked done forever. Documentation tooling now lives in install-dev-tools-docs.sh with
# an explicit `rust` dependency.

set -e

echo "Installing debuggers and profilers..."
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

echo "Installing diagram/documentation system packages..."
sudo apt-get install -y \
    doxygen \
    graphviz

echo "Debug tooling installation complete."
