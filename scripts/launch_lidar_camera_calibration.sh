#!/bin/bash

# LiDAR-Camera Calibration Launch Script
# This script launches the lidar-camera calibration using ros2 launch instead of ros2 systemd
# Usage: ./launch_lidar_camera_calibration.sh [options]
#
# Options:
#   --debug-mode=true/false     Enable debug mode (default: false)
#   --rviz=true/false          Enable RViz (default: false)
#   --use-best-effort-qos=true/false  Use best effort QoS (default: true)
#   --camera-topic=TOPIC       Camera topic (default: /sensing/camera/front_center/image_raw)
#   --pointcloud-topic=TOPIC   Pointcloud topic (default: /sensing/lidar/top/pointcloud_raw)
#   --help                     Show this help message

set -e

# Default values
DEBUG_MODE="false"
ENABLE_RVIZ="false"
USE_BEST_EFFORT_QOS="true"
CAMERA_TOPIC="/sensing/camera/front_center/image_raw"
POINTCLOUD_TOPIC="/sensing/lidar/top/pointcloud_raw"

# Function to show help
show_help() {
    echo "LiDAR-Camera Calibration Launch Script"
    echo "======================================"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --debug-mode=true/false     Enable debug mode (default: false)"
    echo "  --rviz=true/false          Enable RViz (default: false)"
    echo "  --use-best-effort-qos=true/false  Use best effort QoS (default: true)"
    echo "  --camera-topic=TOPIC       Camera topic (default: /sensing/camera/front_center/image_raw)"
    echo "  --pointcloud-topic=TOPIC   Pointcloud topic (default: /sensing/lidar/top/pointcloud_raw)"
    echo "  --help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Launch with default settings"
    echo "  $0 --debug-mode=true --rviz=true     # Launch with debug mode and RViz"
    echo "  $0 --camera-topic=/my_camera/image   # Launch with custom camera topic"
    echo ""
    echo "Note: This script launches the calibration pipeline directly using ros2 launch."
    echo "To publish sample data, run 'make launch_lidar_camera_sample_data' separately."
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

# Get the script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "LiDAR-Camera Calibration Launch Script"
echo "======================================"
echo ""
echo "Configuration:"
echo "  Debug mode: $DEBUG_MODE"
echo "  RViz enabled: $ENABLE_RVIZ"
echo "  Best effort QoS: $USE_BEST_EFFORT_QOS"
echo "  Camera topic: $CAMERA_TOPIC"
echo "  Pointcloud topic: $POINTCLOUD_TOPIC"
echo ""

if [[ "$DEBUG_MODE" = "true" ]]; then
    echo "Debug mode enabled - additional debug topics will be published"
fi

echo "Starting LiDAR-Camera calibration pipeline..."
echo ""

# Change to project root directory
cd "$PROJECT_ROOT"

# Source the setup script and launch
source install/setup.sh && \
ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
    debug_mode:="$DEBUG_MODE" \
    enable_rviz:="$ENABLE_RVIZ" \
    use_best_effort_qos:="$USE_BEST_EFFORT_QOS" \
    camera_topic:="$CAMERA_TOPIC" \
    pointcloud_topic:="$POINTCLOUD_TOPIC"

echo ""
echo "Calibration pipeline finished."
echo ""
echo "Note: This only starts the calibration pipeline. To publish sample data,"
echo "run 'make launch_lidar_camera_sample_data' separately."
