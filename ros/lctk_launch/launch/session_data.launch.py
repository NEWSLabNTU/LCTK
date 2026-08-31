"""Start the data source a session manifest describes.

Thin by design: the rules live in `lctk_launch.session`, and for `pcap_avi` this
includes the existing `lidar_camera.launch.xml` rather than reimplementing the
velodyne driver and gscam wiring. The topics it passes are the same derived
values `config_parser` gives the calibration graph, which is the whole point --
one source, so the two halves cannot disagree.
"""

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lctk_launch.session import (
    derived_camera_topics,
    derived_lidar_topics,
    parse_data,
    resolve_session,
)


def generate_data_source(context, *args, **kwargs) -> list:
    session = resolve_session(LaunchConfiguration("session").perform(context))
    manifest = yaml.safe_load(session.manifest.read_text(encoding="utf-8"))
    source = parse_data(manifest.get("data"), session.directory)

    devices = manifest.get("devices") or {}
    lidars = list((devices.get("lidars") or {}).items())
    cameras = list((devices.get("cameras") or {}).items())

    if source.kind == "live":
        return [
            LogInfo(
                msg=(
                    f"  session '{session.directory.name}': live sensors, "
                    "no playback started"
                )
            )
        ]

    if source.kind == "bag":
        return [
            LogInfo(msg=f"  session '{session.directory.name}': playing {source.path}"),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", str(source.path), "--clock"],
                output="screen",
            ),
        ]

    # pcap_avi: one lidar and one camera, played by lctk_sample_data.
    if len(lidars) != 1 or len(cameras) != 1:
        raise RuntimeError(
            f"data.kind 'pcap_avi' plays exactly one lidar and one camera; this "
            f"session declares {len(lidars)} lidar(s) and {len(cameras)} camera(s)"
        )
    lidar_name, lidar_config = lidars[0]
    camera_name, camera_config = cameras[0]
    lidar_topics = derived_lidar_topics(lidar_name)
    camera_topics = derived_camera_topics(camera_name)

    info_url = source.camera_info_url
    if info_url and not info_url.startswith("file://"):
        info_url = f"file://{info_url}"

    return [
        LogInfo(
            msg=f"  session '{session.directory.name}': playing {source.directory}"
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lctk_sample_data"),
                        "launch",
                        "lidar_camera.launch.xml",
                    ]
                )
            ),
            launch_arguments={
                "pcap_file": str(source.directory / "lidar.pcap"),
                "video_file": str(source.directory / "video.avi"),
                "pointcloud_topic": lidar_topics["pointcloud"],
                "velodyne_packets_topic": lidar_topics["packets"],
                "lidar_frame_id": lidar_config["frame_id"],
                "rpm": str(source.lidar_rpm),
                "camera_name": camera_name,
                "camera_namespace": camera_topics["namespace"],
                "camera_frame_id": camera_config["frame_id"],
                **({"camera_info_url": info_url} if info_url else {}),
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "session",
                description=(
                    "Explicit path to a session directory or its session.yaml. "
                    "There is no search path: a session may live anywhere."
                ),
            ),
            OpaqueFunction(function=generate_data_source),
        ]
    )
