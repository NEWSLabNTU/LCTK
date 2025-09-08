#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Integration test launch file for multi_wayside_node basic calibration scenario.

    This launch file:
    1. Starts multi_wayside_node with test parameters
    2. Plays back test rosbag data
    3. Launches RViz for visualization
    4. Runs validation scripts
    """

    # Declare launch arguments
    test_bag_arg = DeclareLaunchArgument(
        "test_bag",
        default_value="scenario_1_perfect_boards.bag",
        description="Test bag file to play",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz for visualization"
    )

    auto_validate_arg = DeclareLaunchArgument(
        "auto_validate",
        default_value="true",
        description="Run automatic validation after calibration",
    )

    # Package paths
    pkg_share = FindPackageShare("multi_wayside_node")
    test_data_dir = PathJoinSubstitution([pkg_share, "test_data"])
    config_dir = PathJoinSubstitution([pkg_share, "config"])

    # Multi-wayside node with test parameters
    multi_wayside_node = Node(
        package="multi_wayside_node",
        executable="multi_wayside_node",
        name="multi_wayside_test",
        parameters=[
            PathJoinSubstitution([config_dir, "test_params.yaml"]),
            {
                # Test-specific parameter overrides
                "max_queue_size": 50,
                "sync_tolerance_ms": 200,
                "auto_calibrate": True,
                "min_detections_for_calibration": 3,
                "quality_threshold": 0.6,
                "roi_box_size_x": 3.0,
                "roi_box_size_y": 3.0,
                "roi_box_size_z": 1.5,
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    # ROS bag playback
    bag_playback = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            PathJoinSubstitution([test_data_dir, LaunchConfiguration("test_bag")]),
            "--rate",
            "0.5",  # Slow playback for processing
            "--loop",  # Loop for continuous testing
        ],
        name="test_bag_playback",
        output="screen",
    )

    # RViz for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_test",
        arguments=["-d", PathJoinSubstitution([config_dir, "rviz_test_config.rviz"])],
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # Calibration validation script (delayed start)
    validation_script = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([pkg_share, "scripts", "validate_calibration.py"]),
            "--timeout",
            "60",  # Wait up to 60 seconds for calibration
            "--expected_translation_range",
            "1.0,5.0",  # Expected transform magnitude
            "--expected_rotation_range",
            "0.0,0.5",  # Expected rotation (radians)
        ],
        name="calibration_validator",
        condition=launch.conditions.IfCondition(LaunchConfiguration("auto_validate")),
        output="screen",
    )

    # Delayed start for validation (wait for system to initialize)
    delayed_validation = TimerAction(
        period=10.0, actions=[validation_script]  # Wait 10 seconds
    )

    return LaunchDescription(
        [
            test_bag_arg,
            use_rviz_arg,
            auto_validate_arg,
            multi_wayside_node,
            bag_playback,
            rviz_node,
            delayed_validation,
        ]
    )
