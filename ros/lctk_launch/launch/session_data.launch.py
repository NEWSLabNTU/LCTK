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
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lctk_launch.session import (
    derived_camera_topics,
    derived_lidar_topics,
    parse_data,
    resolve_session,
)


def _wait_for_arguments(source, lidars) -> list[str]:
    """Topics whose subscribers mark the graph as ready to receive the bag.

    The LiDAR topics are consumed directly by the board detectors. For the
    camera, the topic to watch is the *compressed* one the bag publishes rather
    than the raw one the graph reads: its subscriber is the republish bridge, and
    the bridge coming up is what makes the rest of the camera chain reachable.
    """
    topics = [
        config["pointcloud_topic"]
        for _, config in lidars
        if config.get("pointcloud_topic")
    ]
    topics += [compressed for compressed, _ in source.republish]
    return [f"--wait-for={topic}" for topic in topics]


def _republish_nodes(source) -> list:
    """Bridge each declared CompressedImage topic to a raw one.

    Nothing in this tree subscribes to `sensor_msgs/CompressedImage`, so a bag
    that records only the compressed stream starves the camera half of the
    pipeline while every node reports itself healthy. Running the bridge from
    the session removes the second terminal, and with it the chance of
    forgetting it.
    """
    return [
        Node(
            package="image_transport",
            executable="republish",
            name=f"republish_{index}",
            arguments=["compressed", "raw"],
            remappings=[("in/compressed", compressed), ("out", raw)],
            output="screen",
        )
        for index, (compressed, raw) in enumerate(source.republish)
    ]


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
            ),
            # Not dead code for a `live` session: an operator replaying bags by
            # hand into a standing graph still needs the CompressedImage bridge,
            # and `live` is exactly how a session says "someone else supplies
            # the data".
            *_republish_nodes(source),
        ]

    if source.kind == "bag":
        return [
            LogInfo(msg=f"  session '{session.directory.name}': playing {source.path}"),
            # A `Node`, not an `ExecuteProcess`, and that is load-bearing.
            # `play_launch` -- which every `just` recipe uses -- runs the launch
            # tree twice: a recording pass, then a replay that starts the nodes.
            # It records `Node` actions only. An `ExecuteProcess` therefore runs
            # during the *recording* pass, when no node exists yet, and is absent
            # from the replay entirely: the whole recording plays into an empty
            # graph and the detectors come up after it has finished. `just demo`
            # never caught this because the pcap_avi path is all Node actions.
            #
            # Not a bare `ros2 bag play` either: the player is ready in about a
            # second while the Rust detectors are still loading, and the bag
            # topics replay BEST_EFFORT/VOLATILE, so everything sent into that
            # gap is lost to a subscriber that has not appeared yet. With RViz,
            # the overlay and the judge enabled the gap can swallow a short
            # recording entirely. `lctk_bag_play` waits for the subscriptions to
            # exist rather than for a guessed number of seconds.
            Node(
                package="lctk_launch",
                executable="lctk_bag_play",
                name="bag_player",
                arguments=[
                    str(source.path),
                    *_wait_for_arguments(source, lidars),
                    "--play-arg=--clock",
                ],
                output="screen",
            ),
            *_republish_nodes(source),
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
