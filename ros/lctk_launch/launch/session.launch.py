"""Run a whole calibration session: its data source, then the calibration graph.

Generalises the deleted `demo.launch.py`, which hard-coded dataset 3 and
`sample_data.yaml` in its body and took no argument to change either.

Both halves are handed the *same* manifest -- the data launch by directory, the
calibration launch by file. That is the design's one guarantee: the topics the
player publishes and the topics the graph subscribes to come from one source.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lctk_launch.session import resolve_session

_FORWARDED = (
    ("debug_mode", "false", "Enable debug topics"),
    ("log_level", "info", "ROS log level"),
    (
        "mode",
        "offline",
        "Transport QoS: 'offline' (RELIABLE) or 'realtime' (BEST_EFFORT)",
    ),
    ("enable_rviz", "true", "Launch RViz alongside the pipeline"),
    ("solver_mode", "continuous", "'continuous', 'manual' or 'assisted'"),
    ("enable_overlay", "false", "Launch pointcloud_image_overlay"),
    ("enable_judge", "false", "Launch the calibration quality judge"),
)


def _share(*parts):
    return PathJoinSubstitution([FindPackageShare("lctk_launch"), *parts])


def generate_session(context, *args, **kwargs) -> list:
    session = resolve_session(LaunchConfiguration("session").perform(context))
    forwarded = {
        name: LaunchConfiguration(name).perform(context) for name, _, _ in _FORWARDED
    }
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(_share("launch", "session_data.launch.py")),
            launch_arguments={"session": str(session.directory)}.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(_share("launch", "calibrate.launch.py")),
            launch_arguments={
                "config_file": str(session.manifest),
                **forwarded,
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "session",
            description=(
                "Explicit path to a session directory or its session.yaml. "
                "There is no search path: a session may live anywhere."
            ),
        )
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in _FORWARDED
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=generate_session)])
