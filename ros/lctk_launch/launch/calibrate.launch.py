"""Turn a calibration configuration into a running graph.

Thin by design. Which nodes exist, what each is called and namespaced, every
parameter and every remapping are decided by `lctk_launch.node_plan`, which
imports no launch types and can be tested without staging a context. This file
maps that plan onto launch actions and does nothing else.

Usage:
    ros2 launch lctk_launch calibrate.launch.py config_file:=/path/to/config.yaml

The config file describes:
- Devices (lidars and cameras with their topics, frame IDs and optional `qos:`)
- Markers (calibration boards with their Target Definition and detector tuning)
- Calibration pairs (which devices to calibrate together using which marker)
- A required `sync:` section (tolerance_ms, queue_size, drop_policy) for the
  Conflux synchronizer -- see config_parser.py's `_parse_sync_tolerance_ms`

Transport reliability is not an argument here. It is a property of what
publishes each sensor topic, resolved per device by `lctk_launch.transport`.

See sessions/ (installed at share/lctk_launch/sessions/) for maintained
configurations; a session manifest is a calibration config plus a `data:`
section, which this launch file does not read.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lctk_launch.node_plan import (
    JudgeInclude,
    Message,
    NodeSpec,
    RunSettings,
    build_node_plan,
)
from lctk_launch.session import DEFAULT_RVIZ_CONFIG_PARTS


def _judge_include(judge: JudgeInclude) -> IncludeLaunchDescription:
    """The quality judge ships as a launch file, so it is included, not spawned."""
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("lctk_launch"),
                    "launch",
                    "calibration_judge.launch.xml",
                ]
            )
        ),
        launch_arguments={
            "transform_topic": judge.transform_topic,
            "namespace": judge.namespace,
            "log_level": judge.log_level,
        }.items(),
    )


def _action(entry):
    """One plan entry, as the launch action that realises it."""
    if isinstance(entry, Message):
        return LogInfo(msg=entry.text)
    if isinstance(entry, NodeSpec):
        return Node(
            package=entry.package,
            executable=entry.executable,
            name=entry.name,
            namespace=entry.namespace,
            output="screen",
            arguments=list(entry.arguments),
            parameters=[entry.parameters],
            remappings=[tuple(pair) for pair in entry.remappings],
        )
    return _judge_include(entry)


def generate_nodes(context, *args, **kwargs) -> list:
    """Read the configuration, build the plan, and realise it as launch actions."""
    from lctk_launch.config_parser import parse_config

    def configuration(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    settings = RunSettings(
        solver_mode=configuration("solver_mode"),
        debug_mode=configuration("debug_mode") == "true",
        log_level=configuration("log_level"),
        enable_overlay=configuration("enable_overlay") == "true",
        enable_judge=configuration("enable_judge") == "true",
    )
    pipeline = parse_config(configuration("config_file"))
    return [_action(entry) for entry in build_node_plan(pipeline, settings)]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description."""
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "config_file",
                description="Path to calibration configuration YAML file",
            ),
            DeclareLaunchArgument(
                "debug_mode",
                default_value="false",
                description="Enable debug logging and visualization",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level (debug, info, warn, error, fatal)",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz for calibration visualization",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                # Shared with session.launch.py via lctk_launch.session so the
                # two cannot drift; see DEFAULT_RVIZ_CONFIG_PARTS for why the
                # default has to be named in both places rather than only here.
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lctk_launch"), *DEFAULT_RVIZ_CONFIG_PARTS]
                ),
                description="Path to RViz config file. Override for different setups, e.g. two_lidar_calibration.rviz",
            ),
            DeclareLaunchArgument(
                "solver_mode",
                default_value="continuous",
                description=(
                    "LiDAR-camera solver behaviour: 'continuous' (auto-publishes the "
                    "latest pair), 'manual' (service-driven multi-pose buffer), or "
                    "'assisted' (auto-captures still, novel poses and serves a review "
                    "page)"
                ),
            ),
            DeclareLaunchArgument(
                "enable_overlay",
                default_value="false",
                description="Enable pointcloud-image overlay visualization (one per lidar-camera pair)",
            ),
            DeclareLaunchArgument(
                "enable_judge",
                default_value="false",
                description="Enable calibration quality judge (ground truth comparison, one per lidar-camera pair)",
            ),
            # Dynamic node generation
            OpaqueFunction(function=generate_nodes),
            # RViz visualization (optional)
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("lctk_launch"), "launch", "rviz.launch.xml"]
                    )
                ),
                launch_arguments={
                    "rviz_config": LaunchConfiguration("rviz_config")
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
