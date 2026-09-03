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
from lctk_launch.session import (
    DEFAULT_RVIZ_CONFIG_PARTS,
    SESSION_RVIZ_NAME,
    resolve_session,
)

# An RViz layout is a per-experiment thing -- which displays are open, which
# topics they point at -- so a session may ship its own next to its manifest.
SESSION_RVIZ = SESSION_RVIZ_NAME

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


def _rviz_config(context, session_dir) -> dict:
    """Decide which RViz layout the calibration graph should open.

    Three sources, in falling order of how specific they are to this run: what
    the operator typed, what the session ships, and the repo-wide fallback.
    Typing `rviz_config:=` means it, so an explicit value wins over the session
    file.

    All three branches return a concrete path. An earlier version returned `{}`
    for the third, meaning "let `calibrate.launch.py` apply its own default",
    which does not work: this file declares `rviz_config` (so it can tell an
    untyped argument from a typed one), and a launch configuration set in a
    parent scope is inherited by every included launch file, so the empty value
    won and RViz opened with no layout at all. The default is shared as parts
    via `lctk_launch.session` instead, so naming it here cannot drift.
    """
    explicit = LaunchConfiguration("rviz_config").perform(context).strip()
    if explicit:
        return {"rviz_config": explicit}
    session_layout = session_dir / SESSION_RVIZ
    if session_layout.is_file():
        return {"rviz_config": str(session_layout)}
    # Name the fallback rather than dropping the key. Returning `{}` here looks
    # like it leaves `calibrate.launch.py` to apply its own default, but a
    # launch configuration set in a parent scope is inherited by everything it
    # includes, so the empty `rviz_config` declared below wins and RViz opens
    # with `-d ""` -- the stock layout, not this repo's. Both files read the
    # same parts from `lctk_launch.session`, so there is still one default.
    return {"rviz_config": _share(*DEFAULT_RVIZ_CONFIG_PARTS)}


def generate_session(context, *args, **kwargs) -> list:
    session = resolve_session(LaunchConfiguration("session").perform(context))
    forwarded = {
        name: LaunchConfiguration(name).perform(context) for name, _, _ in _FORWARDED
    }
    forwarded.update(_rviz_config(context, session.directory))
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
    arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            # Deliberately empty, so "the operator passed nothing" stays
            # distinguishable from "the operator passed a path" -- only the
            # former may be overridden by a session's own rviz.rviz.
            #
            # This empty value is also why `_rviz_config` must name a fallback
            # rather than staying silent: declaring the argument here puts it in
            # the parent scope, and every included launch file inherits it, so
            # an unset value is not "unset" downstream. It is "".
            default_value="",
            description=(
                "Path to an RViz config. Empty means: use the session's "
                f"{SESSION_RVIZ} if it ships one, else the calibrate default."
            ),
        )
    )
    return LaunchDescription([*arguments, OpaqueFunction(function=generate_session)])
