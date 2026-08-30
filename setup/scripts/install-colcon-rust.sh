#!/usr/bin/env bash
# Install colcon-cargo-ros2 and play_launch for Rust ROS 2 integration
# Converted from ansible/roles/lctk.dev_env.colcon_rust/tasks/main.yaml

set -e

echo "Checking for conflicting packages..."

# Remove conflicting packages if present
if pip3 list 2>/dev/null | grep -qE '^colcon-cargo\s'; then
    echo "Removing conflicting colcon-cargo..."
    pip3 uninstall -y colcon-cargo
fi

if pip3 list 2>/dev/null | grep -qE '^colcon-ros-cargo\s'; then
    echo "Removing conflicting colcon-ros-cargo..."
    pip3 uninstall -y colcon-ros-cargo
fi

# Version-pinned per L-09, with an env override to move the floor deliberately
# (e.g. COLCON_CARGO_ROS2_VERSION='>=0.6.0'). A floor rather than an exact pin:
# 0.5.3 is the first release this workspace builds against, and older versions
# are silently wrong rather than loudly broken.
COLCON_CARGO_ROS2_VERSION="${COLCON_CARGO_ROS2_VERSION:->=0.5.3}"

echo "Installing colcon-cargo-ros2 ${COLCON_CARGO_ROS2_VERSION}..."
pip3 install --user --upgrade "colcon-cargo-ros2${COLCON_CARGO_ROS2_VERSION}"

echo "Installing play_launch..."
pip3 install --user 'play_launch>=0.5.0'

# Remove pip-installed empy if present (can conflict with system package)
echo "Removing pip-installed empy if present..."
pip3 uninstall -y empy 2>/dev/null || true

# Remove pip-installed setuptools if present so the Ubuntu apt version is used.
# ROS 2 Humble's ament_python builds assume the apt setuptools (59.6.0); a newer
# pip setuptools (>=80, which the pip installs above can pull in) breaks colcon's
# `setup.py develop --editable` step with "error: option --editable not
# recognized", failing every ament_python package (conflux_py, the solver nodes).
echo "Removing pip-installed setuptools if present (use apt setuptools for Humble)..."
pip3 uninstall -y setuptools 2>/dev/null || true

# Remove pip-installed numpy if present so the Ubuntu apt version is used, for the
# same reason as setuptools: a newer user-pip numpy (e.g. 2.x) shadows the apt numpy
# that ROS 2 Humble's Python packages are built against. The build's
# `_check-python-env` guard rejects a shadowing numpy, so drop it here.
echo "Removing pip-installed numpy if present (use apt numpy for Humble)..."
pip3 uninstall -y numpy 2>/dev/null || true

# Same again for scipy: a user-pip scipy (>=1.15) requires numpy >= 1.23 while apt ships
# 1.21, so importing scipy.optimize dies with "TypeError: 'numpy._DTypeMeta' object is
# not subscriptable". Use apt's python3-scipy.
echo "Removing pip-installed scipy if present (use apt scipy for Humble)..."
pip3 uninstall -y scipy 2>/dev/null || true

echo "Colcon Rust integration installation complete."
