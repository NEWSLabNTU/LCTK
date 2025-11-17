# LCTK Build System using cargo-ros2 and colcon-cargo-ros2
#
# This justfile replaces the three-pass build system (ros2_rust → interface → packages)
# with a simplified single-pass build using cargo-ros2's automatic binding generation.

# Default configuration values
debug_mode := "true"
enable_icp_iteration_debug := "true"
enable_evaluator := "true"
enable_overlay := "true"
log_level := "info"
rviz_enabled := "true"
use_best_effort_qos := "true"
use_advanced_solver := "false"
camera_topic := "/sensing/camera/zedxm/zed_node/left_raw/image_raw_color"
pointcloud_topic := "/sensing/lidar/concatenated/pointcloud"

# Show available commands
default:
    @just help

# Show this help message
help:
    @echo "LCTK - LiDAR and Camera Toolkit"
    @echo ""
    @echo "Build Commands:"
    @echo "  just build                 - Build all ROS packages using colcon and cargo-ros2"
    @echo "  just clean                 - Clean all build artifacts"
    @echo "  just lint                  - Run formatting and linting checks"
    @echo "  just test                  - Run tests with cargo nextest"
    @echo ""
    @echo "LiDAR-Camera Calibration Service:"
    @echo "  just lidar-camera start [ARGS...]   - Start calibration service"
    @echo "  just lidar-camera stop              - Stop calibration service"
    @echo "  just lidar-camera restart           - Restart calibration service"
    @echo "  just lidar-camera status            - Show service status"
    @echo "  just lidar-camera logs [ARGS...]    - View service logs (e.g., -f, --since '5 min ago')"
    @echo ""
    @echo "Two-LiDAR Calibration Service:"
    @echo "  just two-lidar start [ARGS...]      - Start two-LiDAR calibration service"
    @echo "  just two-lidar stop                 - Stop two-LiDAR calibration service"
    @echo "  just two-lidar restart              - Restart two-LiDAR calibration service"
    @echo "  just two-lidar status               - Show service status"
    @echo "  just two-lidar logs [ARGS...]       - View service logs"
    @echo ""
    @echo "Sample Sensor Data Service:"
    @echo "  just sample-sensor-data start [ARGS...]  - Start sample data playback"
    @echo "  just sample-sensor-data stop             - Stop sample data playback"
    @echo "  just sample-sensor-data restart          - Restart sample data playback"
    @echo "  just sample-sensor-data status           - Show service status"
    @echo "  just sample-sensor-data logs [ARGS...]   - View service logs"
    @echo ""
    @echo "Visualization & Tools:"
    @echo "  just rviz                              - Launch RViz for calibration visualization"
    @echo "  just run-advanced-solver-controller    - Run interactive solver controller"
    @echo ""
    @echo "Configuration Variables (set with VAR=value):"
    @echo "  debug_mode              - Enable debug topics (default: true)"
    @echo "  enable_icp_iteration_debug - Enable ICP iteration debug (default: true)"
    @echo "  enable_evaluator        - Enable calibration evaluator (default: true)"
    @echo "  enable_overlay          - Enable point cloud overlay (default: true)"
    @echo "  log_level               - ROS log level: debug/info/warn/error (default: info)"
    @echo "  rviz_enabled            - Launch RViz (default: true)"
    @echo "  use_best_effort_qos     - Use best effort QoS (default: true)"
    @echo "  use_advanced_solver     - Use advanced solver (default: false)"
    @echo "  camera_topic            - Camera topic (default: /sensing/camera/zedxm/...)"
    @echo "  pointcloud_topic        - Point cloud topic (default: /sensing/lidar/concatenated/pointcloud)"
    @echo ""
    @echo "Examples:"
    @echo "  just lidar-camera start"
    @echo "  just debug_mode=false log_level=debug lidar-camera start"
    @echo "  just camera_topic=/camera/image pointcloud_topic=/lidar/points lidar-camera start"
    @echo "  just lidar-camera logs -f"
    @echo "  just sample-sensor-data start"

# Build all ROS packages using colcon and cargo-ros2
build:
    colcon build \
        --base-paths ros \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --cargo-args --profile=test-release

# Clean all build artifacts
clean:
    rm -rf build install log target


lint:
    cargo +nightly fmt --check
    cargo clippy --all-targets --

test:
    cargo nextest run --cargo-profile test-release --no-fail-fast

# LiDAR-Camera calibration service management
lidar-camera action *args='':
    #!/usr/bin/env bash
    set -eo pipefail
    SERVICE_NAME="lctk-calibration"

    case "{{ action }}" in
        start)
            # Stop existing service if it exists and reset failed state
            systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
            systemctl --user reset-failed "$SERVICE_NAME" 2>/dev/null || true
            systemd-run --user --unit="$SERVICE_NAME" \
                --setenv=RUST_LOG=debug \
                --working-directory="$PWD" \
                bash -c "source install/setup.bash && ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
                    debug_mode:={{ debug_mode }} \
                    enable_icp_iteration_debug:={{ enable_icp_iteration_debug }} \
                    enable_evaluator:={{ enable_evaluator }} \
                    enable_overlay:={{ enable_overlay }} \
                    enable_rviz:={{ rviz_enabled }} \
                    log_level:={{ log_level }} \
                    use_best_effort_qos:={{ use_best_effort_qos }} \
                    use_advanced_solver:={{ use_advanced_solver }} \
                    camera_topic:={{ camera_topic }} \
                    pointcloud_topic:={{ pointcloud_topic }} \
                    {{ args }}"
            ;;
        stop)
            systemctl --user stop "$SERVICE_NAME"
            ;;
        restart)
            systemctl --user restart "$SERVICE_NAME"
            ;;
        status)
            systemctl --user status "$SERVICE_NAME"
            ;;
        logs)
            journalctl --user -u "$SERVICE_NAME" {{ args }}
            ;;
        *)
            echo "Usage: just lidar-camera {start|stop|restart|status|logs} [ARGS...]"
            exit 1
            ;;
    esac

# Sample sensor data service management
sample-sensor-data action *args='':
    #!/usr/bin/env bash
    set -eo pipefail
    SERVICE_NAME="lctk-lidar-camera-data"

    case "{{ action }}" in
        start)
            # Stop existing service if it exists and reset failed state
            systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
            systemctl --user reset-failed "$SERVICE_NAME" 2>/dev/null || true
            systemd-run --user --unit="$SERVICE_NAME" \
                --setenv=RUST_LOG=debug \
                --working-directory="$PWD" \
                bash -c "source install/setup.bash && ros2 launch lctk_sample_data lidar_camera.launch.xml {{ args }}"
            ;;
        stop)
            systemctl --user stop "$SERVICE_NAME"
            ;;
        restart)
            systemctl --user restart "$SERVICE_NAME"
            ;;
        status)
            systemctl --user status "$SERVICE_NAME"
            ;;
        logs)
            journalctl --user -u "$SERVICE_NAME" {{ args }}
            ;;
        *)
            echo "Usage: just sample-sensor-data {start|stop|restart|status|logs} [ARGS...]"
            exit 1
            ;;
    esac

# Two-LiDAR calibration service management
two-lidar action *args='':
    #!/usr/bin/env bash
    set -eo pipefail
    SERVICE_NAME="lctk-two-lidar"

    case "{{ action }}" in
        start)
            # Stop existing service if it exists and reset failed state
            systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
            systemctl --user reset-failed "$SERVICE_NAME" 2>/dev/null || true
            systemd-run --user --unit="$SERVICE_NAME" \
                --setenv=RUST_LOG=debug \
                --working-directory="$PWD" \
                bash -c "source install/setup.bash && ros2 launch lctk_launch two_lidar_calibration.launch.xml {{ args }}"
            ;;
        stop)
            systemctl --user stop "$SERVICE_NAME" 2>/dev/null || echo "Service not running"
            ;;
        restart)
            systemctl --user restart "$SERVICE_NAME" 2>/dev/null || echo "Service not found. Use 'start' instead."
            ;;
        status)
            if systemctl --user is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
                systemctl --user status "$SERVICE_NAME"
            else
                echo "Service $SERVICE_NAME is not running or failed."
                echo ""
                echo "Checking recent ROS launch logs..."
                LATEST_LOG=$(find ~/.ros/log -maxdepth 1 -type d -name "$(date +%Y-%m-%d)*" 2>/dev/null | sort -r | head -1)
                if [ -n "$LATEST_LOG" ] && [ -f "$LATEST_LOG/launch.log" ]; then
                    echo "Latest log: $LATEST_LOG/launch.log"
                    echo ""
                    tail -30 "$LATEST_LOG/launch.log"
                else
                    echo "No recent ROS logs found"
                fi
            fi
            ;;
        logs)
            # Try to show logs for the unit, fall back to ROS log files
            if systemctl --user list-units --all "$SERVICE_NAME.service" 2>/dev/null | grep -q "$SERVICE_NAME"; then
                journalctl --user -u "$SERVICE_NAME" {{ args }}
            else
                echo "Service unit not found. Showing ROS launch logs:"
                echo ""
                LATEST_LOG=$(find ~/.ros/log -maxdepth 1 -type d -name "$(date +%Y-%m-%d)*" 2>/dev/null | sort -r | head -1)
                if [ -n "$LATEST_LOG" ] && [ -f "$LATEST_LOG/launch.log" ]; then
                    echo "Log: $LATEST_LOG/launch.log"
                    echo ""
                    cat "$LATEST_LOG/launch.log"
                    # Also show node-specific logs if they exist
                    find "$LATEST_LOG" -name "*.log" ! -name "launch.log" -exec echo "" \; -exec echo "=== {} ===" \; -exec cat {} \;
                else
                    echo "No ROS logs found. Try: ls ~/.ros/log/"
                fi
            fi
            ;;
        *)
            echo "Usage: just two-lidar {start|stop|restart|status|logs} [ARGS...]"
            exit 1
            ;;
    esac

# Launch RViz for calibration visualization
rviz:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    export RUST_LOG=debug
    ros2 launch lctk_launch rviz.launch.xml

# Run interactive advanced solver controller
run-advanced-solver-controller:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run interactive_solver_controller interactive_solver_controller
