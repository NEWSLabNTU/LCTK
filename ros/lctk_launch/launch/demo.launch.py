"""Demo launch file that runs both sample data playback and calibration pipeline.

This combines:
- Sample sensor data playback (LiDAR + camera from lctk_sample_data)
- Config-driven calibration pipeline (calibrate.launch.py with sample_data.yaml)

Usage:
    ros2 launch lctk_launch demo.launch.py
    ros2 launch lctk_launch demo.launch.py debug_mode:=true enable_rviz:=true
    ros2 launch lctk_launch demo.launch.py mode:=realtime  # For live data
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    debug_mode_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="true",
        description="Enable debug logging and visualization",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level (debug, info, warn, error, fatal)",
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="offline",
        description="Processing mode: 'offline' (RELIABLE QoS) or 'realtime' (BEST_EFFORT QoS)",
    )

    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Launch RViz for calibration visualization",
    )

    use_advanced_solver_arg = DeclareLaunchArgument(
        "use_advanced_solver",
        default_value="false",
        description="Use advanced multi-pose solver vs standard solver",
    )

    # Debug: Log the mode and derived QoS setting
    # This helps diagnose if the parameter is being passed correctly
    mode_log = LogInfo(
        msg=["Demo launch: mode=", LaunchConfiguration("mode"),
             ", use_sensor_data_qos=",
             PythonExpression(["'true' if '", LaunchConfiguration("mode"), "' == 'realtime' else 'false'"])]
    )

    # Sample data playback
    # In realtime mode, use BEST_EFFORT QoS for camera topics to match the subscriber
    sample_data_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lctk_sample_data"), "launch", "lidar_camera.launch.xml"]
            )
        ),
        launch_arguments={
            # use_sensor_data_qos=True in realtime mode (BEST_EFFORT), False in offline (RELIABLE)
            # Note: XML launch files expect string "true"/"false" for boolean params
            "use_sensor_data_qos": PythonExpression(
                ["'true' if '", LaunchConfiguration("mode"), "' == 'realtime' else 'false'"]
            ),
        }.items(),
    )

    # Config-driven calibration pipeline using sample_data.yaml
    calibration_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lctk_launch"), "launch", "calibrate.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [FindPackageShare("lctk_launch"), "config", "examples", "sample_data.yaml"]
            ),
            "debug_mode": LaunchConfiguration("debug_mode"),
            "log_level": LaunchConfiguration("log_level"),
            "mode": LaunchConfiguration("mode"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
            "use_advanced_solver": LaunchConfiguration("use_advanced_solver"),
        }.items(),
    )

    return LaunchDescription([
        debug_mode_arg,
        log_level_arg,
        mode_arg,
        enable_rviz_arg,
        use_advanced_solver_arg,
        mode_log,  # Debug logging of mode and QoS settings
        sample_data_launch,
        calibration_launch,
    ])
