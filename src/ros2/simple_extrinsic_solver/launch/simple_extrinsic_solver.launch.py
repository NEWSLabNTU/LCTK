#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'parent_frame',
            default_value='lidar',
            description='Parent frame for the extrinsic transform'
        ),
        DeclareLaunchArgument(
            'child_frame',
            default_value='camera',
            description='Child frame for the extrinsic transform'
        ),
        DeclareLaunchArgument(
            'aruco_pattern_file',
            default_value='',
            description='Path to ArUco pattern configuration file'
        ),
        DeclareLaunchArgument(
            'aruco_config_file',
            default_value='config/aruco_pattern.json5',
            description='Path to ArUco pattern configuration JSON/JSON5 file (relative to workspace root)'
        ),
        DeclareLaunchArgument(
            'enable_quality_assessment',
            default_value='true',
            description='Enable calibration quality assessment'
        ),
        DeclareLaunchArgument(
            'board_detector_file',
            default_value='config/board_detector.json5',
            description='Path to board detector JSON5 file (relative to workspace root)'
        ),
        DeclareLaunchArgument(
            'intrinsics_file',
            default_value='config/intrinsics.yaml',
            description='Path to camera intrinsics YAML file (relative to workspace root)'
        ),
        # Topic arguments
        DeclareLaunchArgument(
            'aruco_detections',
            default_value='/calibration/aruco_locator/aruco_detections',
            description='Input ArUco detections topic'
        ),
        DeclareLaunchArgument(
            'board_detections',
            default_value='/calibration/calibration_board_locator/calibration_board_detections',
            description='Input calibration board detections topic'
        ),
        DeclareLaunchArgument(
            'image_with_detections',
            default_value='/calibration/aruco_locator/image_with_detections',
            description='Input debug overlay image topic'
        ),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'extrinsic_transform',
            default_value='/calibration/extrinsic_solver/extrinsic_transform',
            description='Output extrinsic transform topic'
        ),
        DeclareLaunchArgument(
            'calibration_quality',
            default_value='/calibration/extrinsic_solver/calibration_quality',
            description='Output calibration quality topic'
        ),
        
        # Simple extrinsic solver node
        Node(
            package='simple_extrinsic_solver',
            executable='simple_extrinsic_solver',
            name='simple_extrinsic_solver',
            output='screen',
            parameters=[{
                'parent_frame': LaunchConfiguration('parent_frame'),
                'child_frame': LaunchConfiguration('child_frame'),
                'aruco_pattern_file': LaunchConfiguration('aruco_pattern_file'),
                'aruco_config_file': LaunchConfiguration('aruco_config_file'),
                'board_detector_file': LaunchConfiguration('board_detector_file'),
                'intrinsics_file': LaunchConfiguration('intrinsics_file'),
                'enable_quality_assessment': LaunchConfiguration('enable_quality_assessment'),
            }],
            remappings=[
                ('aruco_detections', LaunchConfiguration('aruco_detections')),
                ('calibration_board_detections', LaunchConfiguration('board_detections')),
                ('image_with_detections', LaunchConfiguration('image_with_detections')),
                ('camera_info', LaunchConfiguration('camera_info')),
                ('extrinsic_transform', LaunchConfiguration('extrinsic_transform')),
                ('calibration_quality', LaunchConfiguration('calibration_quality')),
            ]
        ),
    ])

