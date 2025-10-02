#!/bin/bash

# LCTK LiDAR-Camera Calibration Launch Script
# This script launches the calibration pipeline using ros2 launch directly

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameter values (can be overridden via environment variables)
DEBUG_MODE="${debug_mode:-true}"
ENABLE_ICP_ITERATION_DEBUG="${enable_icp_iteration_debug:-true}"
LOG_LEVEL="${log_level:-info}"
RVIZ="${rviz:-true}"
USE_BEST_EFFORT_QOS="${use_best_effort_qos:-true}"
USE_ADVANCED_SOLVER="${use_advanced_solver:-true}"
CAMERA_TOPIC="${camera_topic:-/sensing/camera/zedxm/zed_node/left_raw/image_raw_color}"
POINTCLOUD_TOPIC="${pointcloud_topic:-/sensing/lidar/concatenated/pointcloud}"

# Print usage information
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Launch the LCTK LiDAR-Camera calibration pipeline.

Options:
    -h, --help              Show this help message
    -d, --debug             Enable debug mode (default: $DEBUG_MODE)
    --no-debug              Disable debug mode
    --icp-debug             Enable ICP iteration debug (default: $ENABLE_ICP_ITERATION_DEBUG)
    --no-icp-debug          Disable ICP iteration debug
    -l, --log-level LEVEL   Set log level (default: $LOG_LEVEL)
    -r, --rviz              Enable RViz (default: $RVIZ)
    --no-rviz               Disable RViz
    --qos-best-effort       Use best effort QoS (default: $USE_BEST_EFFORT_QOS)
    --qos-reliable          Use reliable QoS (for rosbag playback)
    --advanced-solver       Use advanced solver (default: $USE_ADVANCED_SOLVER)
    --basic-solver          Use basic solver
    -c, --camera TOPIC      Camera topic (default: $CAMERA_TOPIC)
    -p, --pointcloud TOPIC  Pointcloud topic (default: $POINTCLOUD_TOPIC)

Environment Variables:
    All options can also be set via environment variables:
    debug_mode, enable_icp_iteration_debug, log_level, rviz,
    use_best_effort_qos, use_advanced_solver, camera_topic, pointcloud_topic

Examples:
    # Launch with defaults
    $0

    # Launch with debug logging and no RViz
    $0 --log-level debug --no-rviz

    # Launch for rosbag playback (reliable QoS)
    $0 --qos-reliable

    # Launch with custom topics
    $0 --camera /my/camera/topic --pointcloud /my/lidar/topic

    # Using environment variables
    debug_mode=false rviz=false $0
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -d|--debug)
            DEBUG_MODE="true"
            shift
            ;;
        --no-debug)
            DEBUG_MODE="false"
            shift
            ;;
        --icp-debug)
            ENABLE_ICP_ITERATION_DEBUG="true"
            shift
            ;;
        --no-icp-debug)
            ENABLE_ICP_ITERATION_DEBUG="false"
            shift
            ;;
        -l|--log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        -r|--rviz)
            RVIZ="true"
            shift
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --qos-best-effort)
            USE_BEST_EFFORT_QOS="true"
            shift
            ;;
        --qos-reliable)
            USE_BEST_EFFORT_QOS="false"
            shift
            ;;
        --advanced-solver)
            USE_ADVANCED_SOLVER="true"
            shift
            ;;
        --basic-solver)
            USE_ADVANCED_SOLVER="false"
            shift
            ;;
        -c|--camera)
            CAMERA_TOPIC="$2"
            shift 2
            ;;
        -p|--pointcloud)
            POINTCLOUD_TOPIC="$2"
            shift 2
            ;;
        *)
            echo "Error: Unknown option: $1"
            echo "Run '$0 --help' for usage information."
            exit 1
            ;;
    esac
done

# Check if workspace is built
if [ ! -f "$WORKSPACE_DIR/install/setup.sh" ]; then
    echo "Error: Workspace not built. Please run 'make build' first."
    exit 1
fi

# Print configuration
echo "========================================"
echo "LCTK LiDAR-Camera Calibration Pipeline"
echo "========================================"
echo "Debug mode:             $DEBUG_MODE"
echo "ICP iteration debug:    $ENABLE_ICP_ITERATION_DEBUG"
echo "Log level:              $LOG_LEVEL"
echo "RViz:                   $RVIZ"
echo "QoS:                    $([ "$USE_BEST_EFFORT_QOS" = "true" ] && echo "Best Effort" || echo "Reliable")"
echo "Solver:                 $([ "$USE_ADVANCED_SOLVER" = "true" ] && echo "Advanced" || echo "Basic")"
echo "Camera topic:           $CAMERA_TOPIC"
echo "Pointcloud topic:       $POINTCLOUD_TOPIC"
echo "========================================"
echo ""

# Source the ROS2 workspace
cd "$WORKSPACE_DIR" || exit 1
source install/setup.sh

# Set environment variables
export RUST_LOG=debug

# Launch the calibration pipeline
echo "Launching calibration pipeline..."
echo "Press Ctrl+C to stop"
echo ""

ros2 launch lctk_launch lidar_camera_calibration.launch.xml \
    debug_mode:="$DEBUG_MODE" \
    enable_icp_iteration_debug:="$ENABLE_ICP_ITERATION_DEBUG" \
    enable_rviz:="$RVIZ" \
    log_level:="$LOG_LEVEL" \
    use_best_effort_qos:="$USE_BEST_EFFORT_QOS" \
    use_advanced_solver:="$USE_ADVANCED_SOLVER" \
    camera_topic:="$CAMERA_TOPIC" \
    pointcloud_topic:="$POINTCLOUD_TOPIC"
