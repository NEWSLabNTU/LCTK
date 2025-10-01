#!/usr/bin/env bash
# Launch script for LiDAR-Camera calibration pipeline
# Direct launch using ros2 launch (runs in foreground)

set -e

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Default configuration values (can be overridden by environment variables)
DEBUG_MODE="${DEBUG_MODE:-true}"
ENABLE_ICP_ITERATION_DEBUG="${ENABLE_ICP_ITERATION_DEBUG:-true}"
LOG_LEVEL="${LOG_LEVEL:-info}"
RVIZ="${RVIZ:-false}"
USE_BEST_EFFORT_QOS="${USE_BEST_EFFORT_QOS:-true}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/sensing/camera/zedxm/zed_node/left_raw/image_raw_color}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/concatenated/pointcloud}"

# Print configuration
echo "==================================="
echo "LiDAR-Camera Calibration Launcher"
echo "==================================="
echo "Configuration:"
echo "  DEBUG_MODE: ${DEBUG_MODE}"
echo "  ENABLE_ICP_ITERATION_DEBUG: ${ENABLE_ICP_ITERATION_DEBUG}"
echo "  LOG_LEVEL: ${LOG_LEVEL}"
echo "  RUST_LOG: debug (for Rust debug! output)"
echo "  RVIZ: ${RVIZ}"
echo "  USE_BEST_EFFORT_QOS: ${USE_BEST_EFFORT_QOS}"
echo "  CAMERA_TOPIC: ${CAMERA_TOPIC}"
echo "  POINTCLOUD_TOPIC: ${POINTCLOUD_TOPIC}"
echo "==================================="
echo ""

# Change to project root
cd "${PROJECT_ROOT}"

# Source ROS2 setup
if [ ! -f "install/setup.sh" ]; then
    echo "Error: install/setup.sh not found. Please build the project first with 'make build'."
    exit 1
fi

source install/setup.sh


# Launch the calibration pipeline using ros2 launch
echo "Launching calibration pipeline..."
echo "Press Ctrl+C to stop."
echo ""

# Export RUST_LOG to enable debug logging for Rust nodes
export RUST_LOG=debug

timeout 20 ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
    debug_mode:="${DEBUG_MODE}" \
    enable_icp_iteration_debug:="${ENABLE_ICP_ITERATION_DEBUG}" \
    enable_rviz:="${RVIZ}" \
    log_level:="${LOG_LEVEL}" \
    use_best_effort_qos:="${USE_BEST_EFFORT_QOS}" \
    camera_topic:="${CAMERA_TOPIC}" \
    pointcloud_topic:="${POINTCLOUD_TOPIC}"

