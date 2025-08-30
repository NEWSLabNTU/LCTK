#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import launch


def generate_launch_description():
    """
    Integration test launch file for testing multiple calibration scenarios sequentially.

    This launch file:
    1. Runs through all test scenarios automatically
    2. Validates each scenario
    3. Generates a comprehensive test report
    """

    # Declare launch arguments
    test_scenarios_arg = DeclareLaunchArgument(
        "test_scenarios",
        default_value="scenario_1_perfect_boards,scenario_2_noisy_data,scenario_3_partial_occlusion,scenario_4_multi_boards",
        description="Comma-separated list of test scenarios to run",
    )

    generate_report_arg = DeclareLaunchArgument(
        "generate_report",
        default_value="true",
        description="Generate HTML test report after completion",
    )

    timeout_per_scenario_arg = DeclareLaunchArgument(
        "timeout_per_scenario",
        default_value="120",
        description="Timeout in seconds for each scenario",
    )

    # Package paths
    pkg_share = FindPackageShare("multi_wayside_node")
    test_data_dir = PathJoinSubstitution([pkg_share, "test_data"])
    config_dir = PathJoinSubstitution([pkg_share, "config"])
    results_dir = PathJoinSubstitution([pkg_share, "test_results"])

    # Multi-wayside node
    multi_wayside_node = Node(
        package="multi_wayside_node",
        executable="multi_wayside_node",
        name="multi_wayside_test",
        parameters=[
            PathJoinSubstitution([config_dir, "test_params.yaml"]),
            {
                # Optimized for automated testing
                "auto_calibrate": True,
                "min_detections_for_calibration": 3,
                "calibration_timeout_seconds": 20,
                "quality_threshold": 0.5,  # Lower threshold for noisy scenarios
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    # Test scenario executor script
    scenario_executor = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([pkg_share, "scripts", "execute_test_scenarios.py"]),
            "--scenarios",
            LaunchConfiguration("test_scenarios"),
            "--test_data_dir",
            test_data_dir,
            "--results_dir",
            results_dir,
            "--timeout",
            LaunchConfiguration("timeout_per_scenario"),
        ],
        name="scenario_executor",
        output="screen",
    )

    # Report generator (runs after scenarios complete)
    report_generator = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([pkg_share, "scripts", "generate_test_report.py"]),
            "--results_dir",
            results_dir,
            "--output",
            PathJoinSubstitution([results_dir, "test_report.html"]),
        ],
        name="report_generator",
        condition=IfCondition(LaunchConfiguration("generate_report")),
        output="screen",
    )

    # Delayed report generation
    delayed_report = TimerAction(
        period=300.0,  # Wait up to 5 minutes for all scenarios
        actions=[report_generator],
    )

    return LaunchDescription(
        [
            test_scenarios_arg,
            generate_report_arg,
            timeout_per_scenario_arg,
            multi_wayside_node,
            scenario_executor,
            delayed_report,
        ]
    )
