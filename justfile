# LCTK Build System
#
# This justfile provides build and launch commands for LCTK.

set shell := ["bash", "-uc"]

# Default configuration values
debug_mode := "true"
log_level := "info"
rviz_enabled := "true"
# Transport QoS only: "offline" (RELIABLE) or "realtime" (BEST_EFFORT). The synchronizer
# window, queue size and drop policy come from the config's required `sync:` section.
mode := "offline"
solver_mode := "continuous"
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
    # M-18: colcon-cargo-ros2 writes its [patch.crates-io] block only into per-package
    # .cargo/config.toml files, so at the workspace root cargo has no patches and dies
    # on the yanked sensor_msgs. Synthesise a root config from a per-package one before
    # the L-16 guard below, which reads it. On a never-yet-built tree there is no
    # per-package config to copy either; that is fine, colcon is about to create them
    # and the post-build sync will pick them up.
    ./setup/scripts/sync-root-cargo-config.sh || \
        echo "no per-package cargo config yet; root config will be synthesised after colcon"
    # L-16 guard: colcon-cargo-ros2 generates Rust bindings once, then marks itself done
    # with build/.colcon/bindgen.lock and never re-checks its outputs. After a partial
    # clean (rm -rf build/<pkg>) the lock survives, generation is skipped, and every
    # Rust package fails with "failed to read .../rosidl_cargo/.../Cargo.toml".
    # Compare persistent source manifests, not mtimes, so additions, edits and
    # deletions all invalidate the cached generated wrapper.  The helper accepts
    # only build/<pkg>/rosidl_cargo/<same-pkg> as a deletion target.
    ./setup/scripts/guard-rosidl-bindings.sh --check

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
    # Refresh from what colcon just wrote. Never hand-maintain this file: a *stale*
    # root config fails the build with "Unable to update .../install/.../rust"
    # (CLAUDE.md Known Issue 1).
    ./setup/scripts/sync-root-cargo-config.sh
    ./setup/scripts/guard-rosidl-bindings.sh --record

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

# Check that every relative link in the docs resolves.
# Part of the release gate: CLAUDE.md requires that when an issue closes and
# moves into docs/issues/archive/, every link crossing the move is repaired in
# both directions -- which only holds if something checks it.
check-docs:
    python3 setup/scripts/check-doc-links.py

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

# Guard (M-18): prove the Rust suite can be COLLECTED before running it.
#
# The failure this exists to catch is silent. When the workspace root has no
# .cargo/config.toml, cargo re-resolves the wildcard ROS message crates against
# crates.io and aborts on the yanked sensor_msgs 4.2.3 -- it never compiles a
# single test. That is easy to mistake for an environment hiccup and work around
# by cd'ing into a package dir, which is exactly how the lidar_board_detector
# tests stayed broken and unnoticed from commit 2a4fd49 until 2026-08-11.
# `cargo nextest list` compiles every test target without running any, so a
# non-zero test count is proof the suite is real before `just test` reports on it.
_check-rust-tests-collectable:
    #!/usr/bin/env bash
    set -eo pipefail

    if ! ./setup/scripts/sync-root-cargo-config.sh; then
        echo "" >&2
        echo "=====================================================================" >&2
        echo " RUST TESTS COULD NOT BE COLLECTED: no root .cargo/config.toml" >&2
        echo "=====================================================================" >&2
        echo " The root config could not be synthesised (see the error above)." >&2
        echo " Nothing was tested. Run 'just build' first." >&2
        exit 1
    fi

    # Keep stderr OUT of $listing: cargo writes "Compiling ..."/"Finished" there, and
    # counting those would make an empty suite look populated.
    errlog=$(mktemp)
    trap 'rm -f "$errlog"' EXIT
    if ! listing=$(cargo nextest list --workspace --cargo-profile test-release 2>"$errlog"); then
        cat "$errlog" >&2
        echo "" >&2
        echo "=====================================================================" >&2
        echo " RUST TESTS COULD NOT BE COLLECTED: 'cargo nextest list' failed" >&2
        echo "=====================================================================" >&2
        echo " The test targets did not compile, so NO Rust test ran and none can." >&2
        echo " This is NOT a passing suite -- see the compiler/cargo error above." >&2
        echo "" >&2
        echo " If it is 'sensor_msgs = \"*\" ... is yanked', the root" >&2
        echo " .cargo/config.toml is missing or stale (M-18). Fix with: just build" >&2
        exit 1
    fi

    count=$(grep -c '[^[:space:]]' <<<"$listing" || true)
    if [[ "${count:-0}" -eq 0 ]]; then
        echo "" >&2
        echo "=====================================================================" >&2
        echo " RUST TESTS COULD NOT BE COLLECTED: zero tests found" >&2
        echo "=====================================================================" >&2
        echo " 'cargo nextest list --workspace' succeeded but listed no tests." >&2
        echo " An empty suite passes vacuously; refusing to report that as green." >&2
        exit 1
    fi
    echo "rust test collection OK: $count tests"

# Run all tests (Rust + Python)
test: _check-rust-tests-collectable
    #!/usr/bin/env bash
    set -eo pipefail
    ./setup/scripts/test-guard-rosidl-bindings.sh
    cargo nextest run --workspace --cargo-profile test-release --no-fail-fast
    source install/setup.bash
    # `python3 -m pytest`, not `pytest`: apt's python3-pytest installs the module and a
    # `pytest-3` script but no bare `pytest` on PATH, so the plain name exits 127 and the
    # Python half never runs. Same failure class as M-18 -- a suite that silently is not
    # reached. Must be the system python3 (see _check-python-env).
    python3 -m pytest ros/lctk_target/test/ ros/lctk_launch/test/ ros/lctk_sync/test/ ros/lidar_to_camera_solver/test/ ros/lidar_to_lidar_solver/test/ ros/lctk_quality/test/ ros/lctk_autoware_export/test/ ros/calibration_judge/test/ -v --no-header

# Launch LiDAR-camera calibration (config-driven)
lidar-camera CONFIG='seyond_left.yaml':
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    SHARE=$(ros2 pkg prefix lctk_launch --share)
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$SHARE/config/examples/{{ CONFIG }} \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        rviz_config:=$SHARE/config/rviz/lidar_camera.rviz \
        solver_mode:={{ solver_mode }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Development only, remove later
solid CONFIG='solid_600_handheld.yaml':
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    SHARE=$(ros2 pkg prefix lctk_launch --share)
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$SHARE/config/examples/{{ CONFIG }} \
        debug_mode:=true \
        log_level:=info \
        mode:=realtime \
        enable_rviz:=true \
        rviz_config:=$SHARE/config/rviz/solid_board.rviz \
        solver_mode:=manual \
        enable_overlay:=true \
        enable_judge:=false

# `solver_mode` stays a switch: `just solver_mode=continuous lidar-camera` and
# `just solver_mode=manual lidar-camera` still run the original paths unchanged.
# Launch assisted calibration (auto-capture + review page on :8080)
assisted CONFIG='seyond_left.yaml':
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    SHARE=$(ros2 pkg prefix lctk_launch --share)
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$SHARE/config/examples/{{ CONFIG }} \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        rviz_config:=$SHARE/config/rviz/lidar_camera.rviz \
        solver_mode:=assisted \
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
        solver_mode:={{ solver_mode }}

# Launch sample data playback only
sample-data:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        lctk_sample_data lidar_camera.launch.xml

# List available sessions
sessions:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run lctk_launch lctk_session list

# Validate a session without launching it: just check <path-or-name>
check SESSION:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    # Resolve first, into a variable: under `set -e` a failing command substitution
    # aborts an assignment but NOT an argument, so inlining it would swallow the
    # "no session" message and run the tool against an empty path instead.
    session_path=$(just _session-path {{ SESSION }})
    ros2 run lctk_launch lctk_session check "$session_path"

# Scaffold a new session: just new <new-path> [<template-path-or-name>]
new TARGET FROM='sample3-hollow-velodyne':
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    template_path=$(just _session-path {{ FROM }})
    ros2 run lctk_launch lctk_session new {{ TARGET }} --from "$template_path"

# Run a session end to end: just run <path-or-name>
run SESSION:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    session_path=$(just _session-path {{ SESSION }})
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch session.launch.py \
        session:="$session_path" \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        solver_mode:={{ solver_mode }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Run the shipped sample-data session end to end
demo:
    #!/usr/bin/env bash
    set -eo pipefail
    just run sample3-hollow-velodyne

# Resolve a session name to a path. Name lookup lives HERE, in the alias layer --
# the ros2 interface takes an explicit path and makes no assumption about where
# sessions live or where the user is standing.
_session-path SESSION:
    #!/usr/bin/env bash
    set -eo pipefail
    if [[ -e "{{ SESSION }}" ]]; then
        realpath "{{ SESSION }}"
    elif [[ -e "sessions/{{ SESSION }}" ]]; then
        realpath "sessions/{{ SESSION }}"
    else
        source install/setup.bash
        SHARE=$(ros2 pkg prefix lctk_launch --share)
        if [[ -e "$SHARE/sessions/{{ SESSION }}" ]]; then
            echo "$SHARE/sessions/{{ SESSION }}"
        else
            echo "no session '{{ SESSION }}' as a path, in ./sessions/, or in $SHARE/sessions/" >&2
            exit 1
        fi
    fi

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
        solver_mode:={{ solver_mode }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Launch interactive manual solver controller
extrinsic-solver-controller:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run interactive_solver_controller interactive_solver_controller

republish which:
    ros2 run image_transport republish compressed raw \
      --ros-args \
      -r in/compressed:=/camera/{{ which }}/image_raw/compressed \
      -r out:=/camera/{{ which }}/image_raw

republish-zed:
    ros2 run image_transport republish compressed raw \
    --ros-args \
    -r in/compressed:=/sensing/camera/zed/rgb/color/rect/image/compressed \
    -r out:=/sensing/camera/zed/rgb/color/rect/image
