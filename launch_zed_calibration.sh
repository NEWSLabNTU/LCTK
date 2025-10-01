#!/bin/bash

# LiDAR-Camera Calibration Launch Script for ZED Camera
# This script launches the lidar-camera calibration with ZED camera topics
# Usage: ./launch_zed_calibration.sh [options]
#
# Default topics:
#   Camera: /sensing/camera/zedxm/zed_node/left_raw/image_raw_color
#   Camera Info: /sensing/camera/zedxm/zed_node/left_raw/camera_info (auto-derived)
#   Pointcloud: /sensing/lidar/concatenated/pointcloud

set -e

# Default values
DEBUG_MODE="true"
ENABLE_RVIZ="false"
USE_BEST_EFFORT_QOS="true"
CAMERA_TOPIC="/sensing/camera/zedxm/zed_node/left_raw/image_raw_color"
POINTCLOUD_TOPIC="/sensing/lidar/concatenated/pointcloud"
LOG_LEVEL="info"
TIMEOUT="10"

# Function to show help
show_help() {
    echo "ZED LiDAR-Camera Calibration Launch Script"
    echo "=========================================="
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --debug-mode=true/false     Enable debug mode (default: false)"
    echo "  --rviz=true/false          Enable RViz (default: false)"
    echo "  --use-best-effort-qos=true/false  Use best effort QoS (default: true)"
    echo "  --camera-topic=TOPIC       Camera topic (default: /sensing/camera/zedxm/zed_node/left_raw/image_raw_color)"
    echo "  --pointcloud-topic=TOPIC   Pointcloud topic (default: /sensing/lidar/concatenated/pointcloud)"
    echo "  --log-level=LEVEL          ROS log level (default: info)"
    echo "  --timeout=SECONDS          Timeout in seconds (default: 10)"
    echo "  --help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Launch with default ZED topics"
    echo "  $0 --debug-mode=true --rviz=true     # Launch with debug mode and RViz"
    echo "  $0 --timeout=30                      # Launch with 30 second timeout"
    echo ""
    echo "Note: Camera info topic is automatically derived from camera topic."
    echo "Camera info will be: /sensing/camera/zedxm/zed_node/left_raw/camera_info"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug-mode=*)
            DEBUG_MODE="${1#*=}"
            shift
            ;;
        --rviz=*)
            ENABLE_RVIZ="${1#*=}"
            shift
            ;;
        --use-best-effort-qos=*)
            USE_BEST_EFFORT_QOS="${1#*=}"
            shift
            ;;
        --camera-topic=*)
            CAMERA_TOPIC="${1#*=}"
            shift
            ;;
        --pointcloud-topic=*)
            POINTCLOUD_TOPIC="${1#*=}"
            shift
            ;;
        --log-level=*)
            LOG_LEVEL="${1#*=}"
            shift
            ;;
        --timeout=*)
            TIMEOUT="${1#*=}"
            shift
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate boolean parameters
for param in DEBUG_MODE ENABLE_RVIZ USE_BEST_EFFORT_QOS; do
    value=$(eval echo \$$param)
    if [[ "$value" != "true" && "$value" != "false" ]]; then
        echo "Error: $param must be 'true' or 'false', got '$value'"
        exit 1
    fi
done

# Validate log level
if [[ "$LOG_LEVEL" != "debug" && "$LOG_LEVEL" != "info" && "$LOG_LEVEL" != "warn" && "$LOG_LEVEL" != "error" && "$LOG_LEVEL" != "fatal" ]]; then
    echo "Error: log_level must be one of: debug, info, warn, error, fatal, got '$LOG_LEVEL'"
    exit 1
fi

# Validate timeout
if ! [[ "$TIMEOUT" =~ ^[0-9]+$ ]] || [ "$TIMEOUT" -lt 1 ]; then
    echo "Error: timeout must be a positive integer, got '$TIMEOUT'"
    exit 1
fi

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

echo "ZED LiDAR-Camera Calibration Launch Script"
echo "=========================================="
echo ""
echo "Configuration:"
echo "  Debug mode: $DEBUG_MODE"
echo "  RViz enabled: $ENABLE_RVIZ"
echo "  Best effort QoS: $USE_BEST_EFFORT_QOS"
echo "  Camera topic: $CAMERA_TOPIC"
echo "  Camera info topic: $(dirname $CAMERA_TOPIC)/camera_info (auto-derived)"
echo "  Pointcloud topic: $POINTCLOUD_TOPIC"
echo "  Log level: $LOG_LEVEL"
echo "  Timeout: ${TIMEOUT}s"
echo ""

if [[ "$DEBUG_MODE" = "true" ]]; then
    echo "Debug mode enabled - additional debug topics will be published"
fi

echo "Starting LiDAR-Camera calibration pipeline..."
echo ""

# Change to project root directory
cd "$PROJECT_ROOT"

# Source the setup script and launch with timeout
echo "Sourcing setup script..."
source install/setup.sh

echo "Launching calibration pipeline with ${TIMEOUT}s timeout..."
RUST_LOG=debug timeout $TIMEOUT ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
    debug_mode:="$DEBUG_MODE" \
    enable_rviz:="$ENABLE_RVIZ" \
    use_best_effort_qos:="$USE_BEST_EFFORT_QOS" \
    camera_topic:="$CAMERA_TOPIC" \
    pointcloud_topic:="$POINTCLOUD_TOPIC" \
    log_level:="$LOG_LEVEL" || {
    exit_code=$?
    if [ $exit_code -eq 124 ]; then
        echo ""
        echo "Calibration pipeline timed out after ${TIMEOUT} seconds (as expected for testing)."
        echo "This is normal behavior when testing the launch."
    else
        echo ""
        echo "Calibration pipeline exited with error code: $exit_code"
        exit $exit_code
    fi
}

echo ""
echo "Calibration pipeline finished."
echo ""
echo "Note: This script is configured for ZED camera topics."
echo "Camera info topic is automatically derived from the image topic."
