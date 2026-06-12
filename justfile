# LCTK Build System
#
# This justfile provides build and launch commands for LCTK.

set shell := ["bash", "-uc"]

# Default configuration values
debug_mode := "true"
log_level := "info"
rviz_enabled := "true"
# Processing mode: "offline" (RELIABLE QoS, perfect sync) or "realtime" (BEST_EFFORT QoS, no buffering)
mode := "offline"
use_advanced_solver := "false"
enable_overlay := "true"
enable_judge := "true"

# Show available commands
default:
    @just --list

# Build all ROS packages using colcon and cargo-ros2
# The conflux packages LCTK depends on (conflux_cpp + conflux_py) are built
# first by `build-conflux`; the rest of ros/conflux (conflux, conflux-ros2)
# is excluded because it uses a git rclrs that conflicts with our crates.io rclrs.
build: build-conflux
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    # L-16 guard: colcon-cargo-ros2 generates Rust bindings once, then marks itself done
    # with build/.colcon/bindgen.lock and never re-checks its outputs. After a partial
    # clean (rm -rf build/<pkg>) the lock survives, generation is skipped, and every
    # Rust package fails with "failed to read .../rosidl_cargo/.../Cargo.toml".
    # Drop the lock whenever any binding path pinned in .cargo/config.toml is missing.
    if [[ -f build/.colcon/bindgen.lock ]]; then
        while read -r path; do
            if [[ ! -f "$path/Cargo.toml" ]]; then
                echo "bindgen output missing ($path); removing stale bindgen.lock"
                rm -f build/.colcon/bindgen.lock
                break
            fi
        done < <(grep -oP 'path = "\K[^"]+' .cargo/config.toml)
    fi
    # L-29 guard: --symlink-install symlinks package data files into build/ and install/
    # instead of copying them. When a source file is later deleted -- a launch file dropped
    # in a rebase, say -- the symlink is left behind pointing at nothing, and the next build
    # fails with "can't copy '<path>': doesn't exist or not a regular file". The path it
    # names still shows up in `ls`, because a dangling symlink is a directory entry without
    # a target, so the message reads as nonsense until you know to look for that.
    #
    # A broken symlink is never useful: colcon recreates the ones that should exist. Prune
    # them before building rather than making the developer decode the error.
    for tree in build install; do
        if [[ -d "$tree" ]]; then
            pruned=$(find "$tree" -xtype l -print -delete 2>/dev/null | wc -l)
            if [[ "$pruned" -gt 0 ]]; then
                echo "removed $pruned dangling symlink(s) under $tree/ (L-29)"
            fi
        fi
    done

    colcon build \
        --base-paths ros \
        --packages-ignore conflux conflux_cpp conflux_py \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --cargo-args --profile=test-release

# Build the conflux packages LCTK needs at runtime.
# conflux_cpp builds the libconflux_ffi.so that conflux_py loads via ctypes; the
# solver nodes import conflux_py and fail to start without it. Only these two
# packages are selected so the git-rclrs conflux/conflux-ros2 packages are skipped.
build-conflux: _check-python-env
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon build \
        --base-paths ros \
        --packages-select conflux_cpp conflux_py \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Guard: this project runs against the SYSTEM python that ROS 2 Humble and apt's OpenCV were
# built against. A pip `--user` install lands in ~/.local/lib/python3.10/site-packages, which
# precedes /usr/lib/python3/dist-packages on sys.path and silently shadows the apt package.
# Three of these have already bitten, and all fail far from the cause:
#
#   setuptools >= 80  removed the `setup.py develop --editable` step colcon uses for
#                     --symlink-install  ->  "error: option --editable not recognized",
#                     which kills every ament_python package at BUILD time.
#   numpy >= 2        breaks the ABI apt's cv2 was compiled against  ->  "ImportError:
#                     numpy.core.multiarray failed to import", which kills every solver
#                     node at RUN time, after a clean build.
#   scipy >= 1.15     requires numpy >= 1.23 (apt has 1.21)  ->  "TypeError:
#                     'numpy._DTypeMeta' object is not subscriptable" inside scipy at
#                     TEST/RUN time, wherever scipy.optimize is imported.
#
# Never `pip3 install --user` setuptools, numpy, or scipy on this machine.
_check-python-env:
    #!/usr/bin/env bash
    set -eo pipefail
    fail=0

    for pkg in setuptools numpy scipy; do
        location=$(python3 -c "import $pkg; print($pkg.__file__)" 2>/dev/null) || continue
        version=$(python3 -c "import $pkg; print($pkg.__version__)" 2>/dev/null) || continue
        if [[ "$location" != /usr/lib/python3/dist-packages/* ]]; then
            echo "error: $pkg $version shadows the apt package that ROS 2 Humble needs." >&2
            echo "       found: $location" >&2
            echo "       Fix with:  pip3 uninstall -y $pkg" >&2
            echo "" >&2
            fail=1
        fi
    done

    # The failure that actually bites at runtime: cv2 cannot import under a numpy it was not
    # built against. Check it directly rather than inferring it from version numbers.
    if ! python3 -c 'import cv2' 2>/dev/null; then
        echo "error: 'import cv2' fails. The solver nodes import cv2 and will crash at startup." >&2
        python3 -c 'import cv2' 2>&1 | tail -1 | sed 's/^/       /' >&2
        echo "       Usually a pip numpy shadowing apt's; fix with:  pip3 uninstall -y numpy" >&2
        fail=1
    fi

    [[ $fail -eq 0 ]]

# Set up development environment (install all dependencies)
setup *args:
    ./setup.sh {{ args }}

# Clean all build artifacts
clean:
    rm -rf build install log target

# Format code (Rust + Python)
format:
    cargo +nightly fmt
    ruff format ros/

# Run formatting and linting checks (Rust + Python)
lint:
    cargo +nightly fmt --check
    cargo clippy --all-targets --
    ruff check ros/
    ruff format --check ros/

# Fast Python-only lint (skips the multi-minute clippy step)
lint-py:
    ruff check ros/
    ruff format --check ros/

# Audit Rust dependencies for RUSTSEC advisories (Phase 4).
# Must run in the sourced build env: a bare `cargo audit` re-resolves the wildcard ROS
# message crates against crates.io and hits the yanked sensor_msgs. Tracked exceptions
# live in .cargo/audit.toml with justification.
audit:
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    cargo audit

# Run all tests (Rust + Python)
test:
    #!/usr/bin/env bash
    set -eo pipefail
    cargo nextest run --cargo-profile test-release --no-fail-fast
    source install/setup.bash
    # L-28: invoke pytest as a module. apt's python3-pytest installs the package but no
    # `pytest` executable, so the bare form exits 127 and the Python half of the suite
    # never ran.
    python3 -m pytest ros/lctk_launch/test/ ros/advanced_extrinsic_solver/test/ ros/lctk_quality/test/ ros/lctk_autoware_export/test/ ros/calibration_judge/test/ -v --no-header

# Launch LiDAR-camera calibration (config-driven)
lidar-camera:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    CONFIG=$(ros2 pkg prefix lctk_launch --share)/config/examples/seyond_left.yaml
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$CONFIG \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch two-LiDAR calibration (config-driven)
two-lidar:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    SHARE=$(ros2 pkg prefix lctk_launch --share)
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$SHARE/config/examples/two_lidar.yaml \
        rviz_config:=$SHARE/config/rviz/two_lidar_calibration.rviz \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }}

# Launch sample data playback only
sample-data:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        lctk_sample_data lidar_camera.launch.xml

# Launch demo (sample data + calibration pipeline)
demo:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch demo.launch.py \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch RViz for calibration visualization
rviz:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run rviz2 rviz2

# Launch config-driven calibration pipeline
# Usage: just calibrate /path/to/config.yaml
# Example: just calibrate $(ros2 pkg prefix lctk_launch)/share/lctk_launch/config/examples/sample_data.yaml
calibrate config_file:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:={{ config_file }} \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        use_advanced_solver:={{ use_advanced_solver }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch interactive advanced solver controller
advanced-solver-controller:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run interactive_solver_controller interactive_solver_controller

republish:
    ros2 run image_transport republish compressed raw \
      --ros-args \
      -r in/compressed:=/camera/left/image_raw/compressed \
      -r out:=/camera/left/image_raw