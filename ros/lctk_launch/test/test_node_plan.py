"""The calibration graph, asserted as a value rather than as a launch context.

`test_calibrate_launch_graph.py` covers the same ground through
`generate_nodes`, staging a launch context and reading `launch_ros.Node`
internals for every assertion. These tests exist to show what the interface is
now: a manifest and a `RunSettings` in, a list of plain values out, with no
launch import anywhere in this file.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from lctk_launch.config_parser import parse_config
from lctk_launch.node_plan import (
    JudgeInclude,
    Message,
    NodeSpec,
    RunSettings,
    build_node_plan,
)

SESSIONS = Path(__file__).resolve().parents[3] / "sessions"


def plan_for(session: str, **settings) -> list:
    pipeline = parse_config(str(SESSIONS / session / "session.yaml"))
    return build_node_plan(pipeline, RunSettings(**settings))


def nodes_named(plan, executable: str) -> list[NodeSpec]:
    return [
        entry
        for entry in plan
        if isinstance(entry, NodeSpec) and entry.executable == executable
    ]


def parameters_of(node: NodeSpec) -> dict:
    return node.parameters


def test_a_lidar_camera_session_produces_the_four_expected_nodes():
    plan = plan_for("seyond-left")
    assert len(nodes_named(plan, "lidar_board_detector")) == 1
    assert len(nodes_named(plan, "aruco_locator_node")) == 1
    assert len(nodes_named(plan, "lidar_to_camera_solver")) == 1
    # One lidar-camera pair is one tree edge, so the broadcaster is present.
    assert len(nodes_named(plan, "tf_tree_broadcaster")) == 1
    assert not nodes_named(plan, "lidar_to_lidar_solver")


def test_two_lidars_produce_two_detectors_and_a_lidar_lidar_solver():
    plan = plan_for("twolidar-vlp32-falcon")
    assert len(nodes_named(plan, "lidar_board_detector")) == 2
    assert len(nodes_named(plan, "lidar_to_lidar_solver")) == 1
    # No camera in this recording, so nothing camera-side is generated.
    assert not nodes_named(plan, "aruco_locator_node")
    assert not nodes_named(plan, "lidar_to_camera_solver")


def test_each_detector_carries_its_own_devices_reliability():
    """TWO_LIDAR_1 records a RELIABLE Falcon beside a BEST_EFFORT VLP-32.

    Reading this off a value is the whole point: the old test had to build a
    launch context and reach into `Node._Node__parameters` to see it.
    """
    plan = plan_for("twolidar-vlp32-falcon")
    by_namespace = {
        node.namespace: parameters_of(node)["use_best_effort_qos"]
        for node in nodes_named(plan, "lidar_board_detector")
    }
    assert set(by_namespace.values()) == {True, False}


def test_the_optional_viewers_are_absent_unless_asked_for():
    plan = plan_for("seyond-left")
    assert not nodes_named(plan, "overlay_node")
    assert not [entry for entry in plan if isinstance(entry, JudgeInclude)]


def test_enabling_the_viewers_adds_one_of_each_per_solver():
    plan = plan_for("seyond-left", enable_overlay=True, enable_judge=True)
    solvers = nodes_named(plan, "lidar_to_camera_solver")
    assert len(nodes_named(plan, "overlay_node")) == len(solvers)
    judges = [entry for entry in plan if isinstance(entry, JudgeInclude)]
    assert len(judges) == len(solvers)
    assert judges[0].transform_topic == parameters_of(solvers[0]).get(
        "extrinsic_transform", judges[0].transform_topic
    )


def test_each_node_is_announced_by_the_message_before_it():
    """The order is part of what the plan describes: a log is read top to bottom."""
    plan = plan_for("seyond-left")
    first_detector = next(
        index
        for index, entry in enumerate(plan)
        if isinstance(entry, NodeSpec) and entry.executable == "lidar_board_detector"
    )
    announcement = plan[first_detector - 1]
    assert isinstance(announcement, Message)
    assert "Board detector" in announcement.text


def test_the_sync_section_reaches_both_solver_kinds_unchanged():
    """Not a preset derived from anything: the numbers in the file, verbatim."""
    pipeline = parse_config(str(SESSIONS / "twolidar-vlp32-falcon" / "session.yaml"))
    plan = build_node_plan(pipeline, RunSettings())
    for node in nodes_named(plan, "lidar_to_lidar_solver"):
        assert parameters_of(node)["sync_tolerance_ms"] == pipeline.sync.tolerance_ms
        assert parameters_of(node)["sync_queue_size"] == pipeline.sync.queue_size
        assert parameters_of(node)["sync_drop_policy"] == pipeline.sync.drop_policy


def test_solver_mode_reaches_the_solver():
    plan = plan_for("seyond-left", solver_mode="assisted")
    solver = nodes_named(plan, "lidar_to_camera_solver")[0]
    assert parameters_of(solver)["solver_mode"] == "assisted"


def test_an_unknown_solver_mode_is_refused_before_any_node_is_built():
    """A typo must not silently ship a different solver policy.

    Validating in `RunSettings` rather than in the launch file means the refusal
    is reachable without a launch context -- and that it happens before any of
    the plan exists, rather than partway through building it.
    """
    with pytest.raises(RuntimeError) as excinfo:
        RunSettings(solver_mode="automatic")
    message = str(excinfo.value)
    assert "automatic" in message
    for mode in ("continuous", "manual", "assisted"):
        assert mode in message


def test_debug_mode_reaches_both_observer_kinds():
    plan = plan_for("seyond-left", debug_mode=True)
    detector = nodes_named(plan, "lidar_board_detector")[0]
    locator = nodes_named(plan, "aruco_locator_node")[0]
    assert parameters_of(detector)["enable_debug"] is True
    assert parameters_of(locator)["debug_mode"] is True


def test_the_log_level_reaches_every_node_that_takes_arguments():
    plan = plan_for("seyond-left", log_level="debug")
    for node in plan:
        if isinstance(node, NodeSpec) and node.arguments:
            assert node.arguments == ("--ros-args", "--log-level", "debug")


def test_a_plan_holds_no_launch_types():
    """The module's whole claim: these are values, not actions.

    If a launch type ever leaks in here, this file stops being testable without
    a context and the seam has moved back into the launch file.
    """
    import lctk_launch.node_plan as module

    plan = plan_for("seyond-left", enable_overlay=True, enable_judge=True)
    for entry in plan:
        assert isinstance(entry, Message | NodeSpec | JudgeInclude)
        assert type(entry).__module__ == "lctk_launch.node_plan"

    # Read the imports rather than grepping the text: the module's own docstring
    # names `launch_ros.Node` while explaining what it does not use.
    import ast

    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    for name in imported:
        root = name.split(".")[0]
        assert root not in ("launch", "launch_ros"), (
            f"node_plan imports {name!r}; the plan must stay free of launch types "
            "or it can no longer be built without a context"
        )
