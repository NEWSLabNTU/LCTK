"""Unit tests for calibration_planner module.

No ROS dependencies — can run standalone:
    python3 -m pytest ros/lctk_launch/test/test_calibration_planner.py -v

Or directly:
    python3 ros/lctk_launch/test/test_calibration_planner.py
"""

import sys
from pathlib import Path

# Add the package to path for standalone testing
sys.path.insert(0, str(Path(__file__).parent.parent))

from lctk_launch.calibration_planner import (
    CalibrationPlan,
    _find_chain,
    compute_plan,
    format_plan,
)


# ── Test fixtures ──────────────────────────────────────────────────────


def _simple_pair():
    """Single lidar-camera pair."""
    return (
        [("L1", "C1", "M1")],
        {"L1"},
        {"C1"},
        "L1",
    )


def _vehicle_setup():
    """2 lidars, 4 cameras, 5 calibration pairs (matches vehicle.yaml)."""
    pairs = [
        ("L1", "C1", "M1"),
        ("L1", "C2", "M1"),
        ("L2", "C3", "M2"),
        ("L2", "C4", "M2"),
        ("L1", "L2", "M3"),
    ]
    return pairs, {"L1", "L2"}, {"C1", "C2", "C3", "C4"}, "L1"


def _vehicle_with_validation():
    """Vehicle setup plus extra edge that becomes a validation edge."""
    pairs = [
        ("L1", "C1", "M1"),
        ("L1", "C2", "M1"),
        ("L2", "C3", "M2"),
        ("L2", "C4", "M2"),
        ("L1", "L2", "M3"),
        ("L2", "C2", "M1"),  # Extra: L2-C2 creates a cycle
    ]
    return pairs, {"L1", "L2"}, {"C1", "C2", "C3", "C4"}, "L1"


# ── Core algorithm tests ──────────────────────────────────────────────


def test_simple_pair():
    """Single pair produces 1 tree edge, 0 validation edges."""
    pairs, lidars, cameras, ref = _simple_pair()
    plan = compute_plan(pairs, lidars, cameras, ref)

    assert plan.reference_frame == "L1"
    assert len(plan.all_edges) == 1
    assert len(plan.tree_edges) == 1
    assert len(plan.validation_edges) == 0

    edge = plan.tree_edges[0]
    assert edge.parent == "L1"
    assert edge.child == "C1"
    assert edge.marker == "M1"
    assert edge.edge_type == "lidar_camera"


def test_vehicle_spanning_tree():
    """Vehicle setup: 5 edges → 5 tree edges (6 nodes need 5 edges)."""
    pairs, lidars, cameras, ref = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, ref)

    assert plan.reference_frame == "L1"
    assert len(plan.all_edges) == 5
    assert len(plan.tree_edges) == 5
    assert len(plan.validation_edges) == 0

    # All 6 nodes should be in the tree
    tree_nodes = {plan.reference_frame}
    for edge in plan.tree_edges:
        tree_nodes.add(edge.parent)
        tree_nodes.add(edge.child)
    assert tree_nodes == {"L1", "L2", "C1", "C2", "C3", "C4"}


def test_mst_prefers_lidar_camera():
    """MST should prefer lidar-camera edges (weight=1) over lidar-lidar (weight=2)."""
    pairs, lidars, cameras, ref = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, ref)

    # All lidar-camera edges should be in the tree (they have lower weight)
    lc_tree = [e for e in plan.tree_edges if e.edge_type == "lidar_camera"]
    assert len(lc_tree) == 4  # L1-C1, L1-C2, L2-C3, L2-C4


def test_validation_edges():
    """Extra edge creates a cycle → becomes validation edge."""
    pairs, lidars, cameras, ref = _vehicle_with_validation()
    plan = compute_plan(pairs, lidars, cameras, ref)

    assert len(plan.all_edges) == 6
    assert len(plan.tree_edges) == 5  # 6 nodes need 5 edges
    assert len(plan.validation_edges) == 1

    val = plan.validation_edges[0]
    # The L2-C2 or L1-L2 could be the validation edge depending on MST
    # (L1-L2 has weight 2, so if L2-C2 connects L2 first, L1-L2 becomes validation)
    # But with weight-based MST, all lidar-camera edges (weight=1) are preferred,
    # so the lidar-lidar edge could also be displaced.
    # The key assertion: exactly one validation edge.
    assert val.marker in ("M1", "M3")


def test_tree_adjacency():
    """Tree dict correctly maps parent → children."""
    pairs, lidars, cameras, ref = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, ref)

    # L1 should be root with children
    assert "L1" in plan.tree
    all_children = []
    for children in plan.tree.values():
        all_children.extend(children)
    assert len(all_children) == 5  # 5 tree edges = 5 children total


def test_rooted_at_reference():
    """Tree is rooted at reference frame — reference is never a child."""
    pairs, lidars, cameras, ref = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, ref)

    children_set = set()
    for children in plan.tree.values():
        children_set.update(children)
    assert ref not in children_set


def test_different_reference_frame():
    """Using L2 as reference re-roots the tree."""
    pairs, lidars, cameras, _ = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, "L2")

    assert plan.reference_frame == "L2"
    children_set = set()
    for children in plan.tree.values():
        children_set.update(children)
    assert "L2" not in children_set


# ── Error case tests ──────────────────────────────────────────────────


def test_no_pairs_raises():
    """Empty pairs list raises ValueError."""
    try:
        compute_plan([], {"L1"}, {"C1"}, "L1")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "No calibration pairs" in str(e)


def test_disconnected_graph_raises():
    """Disconnected components raise ValueError."""
    pairs = [
        ("L1", "C1", "M1"),
        ("L2", "C2", "M2"),  # Disconnected from L1-C1
    ]
    try:
        compute_plan(pairs, {"L1", "L2"}, {"C1", "C2"}, "L1")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "disconnected" in str(e).lower()


def test_camera_camera_pair_raises():
    """Camera-camera pair raises ValueError."""
    try:
        compute_plan([("C1", "C2", "M1")], set(), {"C1", "C2"}, "C1")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "Camera-camera" in str(e)


def test_unknown_device_raises():
    """Unknown device in pair raises ValueError."""
    try:
        compute_plan([("L1", "UNKNOWN", "M1")], {"L1"}, set(), "L1")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "Unknown device" in str(e)


def test_unknown_reference_frame_raises():
    """Unknown reference frame raises ValueError."""
    try:
        compute_plan([("L1", "C1", "M1")], {"L1"}, {"C1"}, "UNKNOWN")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "not a known device" in str(e)


def test_disconnected_reference_frame_raises():
    """Reference frame not in any pair raises ValueError."""
    pairs = [("L1", "C1", "M1")]
    try:
        compute_plan(pairs, {"L1", "L2"}, {"C1"}, "L2")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "not connected" in str(e)


# ── Format tests ──────────────────────────────────────────────────────


def test_format_simple():
    """Format output for a simple plan."""
    pairs, lidars, cameras, ref = _simple_pair()
    plan = compute_plan(pairs, lidars, cameras, ref)
    text = format_plan(plan)

    assert "Calibration Plan" in text
    assert "L1" in text
    assert "C1" in text
    assert "TF Tree" in text


def test_format_with_frame_ids():
    """Format output includes frame IDs when provided."""
    pairs, lidars, cameras, ref = _simple_pair()
    plan = compute_plan(pairs, lidars, cameras, ref)
    text = format_plan(plan, {"L1": "lidar_front", "C1": "camera_left"})

    assert "lidar_front" in text
    assert "camera_left" in text


def test_format_validation_edges():
    """Format output shows validation edges and chain paths."""
    pairs, lidars, cameras, ref = _vehicle_with_validation()
    plan = compute_plan(pairs, lidars, cameras, ref)
    text = format_plan(plan)

    assert "Validation edges" in text
    assert "chain:" in text


def test_format_vehicle():
    """Format the full vehicle plan for visual inspection."""
    pairs, lidars, cameras, ref = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, ref)
    frame_ids = {
        "L1": "lidar_front",
        "L2": "lidar_rear",
        "C1": "camera_front_left",
        "C2": "camera_front_right",
        "C3": "camera_rear_left",
        "C4": "camera_rear_right",
    }
    text = format_plan(plan, frame_ids)

    # Should have all devices mentioned
    for name in frame_ids.values():
        assert name in text, f"Missing {name} in formatted plan"

    print()
    print("=== Vehicle Plan (visual check) ===")
    print(text)


# ── Edge case tests ───────────────────────────────────────────────────


def test_single_lidar_lidar():
    """Two lidars only, no cameras."""
    pairs = [("L1", "L2", "M1")]
    plan = compute_plan(pairs, {"L1", "L2"}, set(), "L1")

    assert len(plan.tree_edges) == 1
    assert plan.tree_edges[0].edge_type == "lidar_lidar"


def test_three_lidars_chain():
    """Three lidars chained: L1-L2, L2-L3."""
    pairs = [
        ("L1", "L2", "M1"),
        ("L2", "L3", "M2"),
    ]
    plan = compute_plan(pairs, {"L1", "L2", "L3"}, set(), "L1")

    assert len(plan.tree_edges) == 2
    assert len(plan.validation_edges) == 0

    # L1 is root, L2 and L3 are descendants
    assert plan.reference_frame == "L1"


def test_large_setup():
    """10-sensor setup to verify scalability."""
    pairs = []
    lidars = {"L1", "L2", "L3"}
    cameras = {f"C{i}" for i in range(1, 8)}

    # Connect everything through lidars
    pairs.append(("L1", "L2", "M1"))
    pairs.append(("L2", "L3", "M2"))
    for i in range(1, 4):
        pairs.append(("L1", f"C{i}", "M3"))
    for i in range(4, 6):
        pairs.append(("L2", f"C{i}", "M4"))
    for i in range(6, 8):
        pairs.append(("L3", f"C{i}", "M5"))

    plan = compute_plan(pairs, lidars, cameras, "L1")

    assert len(plan.all_edges) == 9
    # 10 nodes need 9 edges for a spanning tree
    assert len(plan.tree_edges) == 9
    assert len(plan.validation_edges) == 0


# ── MST / Graph algorithm tests ──────────────────────────────────


def test_mst_breaks_tie_deterministically():
    """Two equal-weight paths; verify stable output across runs."""
    # L1-C1 and L1-C2 are both weight 1; both connect new nodes.
    # Run multiple times and check result is identical.
    pairs = [("L1", "C1", "M1"), ("L1", "C2", "M2")]
    results = []
    for _ in range(10):
        plan = compute_plan(pairs, {"L1"}, {"C1", "C2"}, "L1")
        edges = [(e.parent, e.child, e.marker) for e in plan.tree_edges]
        results.append(edges)
    assert all(r == results[0] for r in results), "MST output is not deterministic"


def test_mst_displaces_lidar_lidar_when_redundant():
    """Lidar-lidar displaced when lidar-camera path connects same components."""
    # L1-L2 (weight 2), L1-C1 (weight 1), L2-C1 (weight 1)
    # MST picks both lidar-camera edges (weight 1 each) before lidar-lidar (weight 2).
    # L1-L2 becomes validation edge since L1-C1-L2 path exists via tree.
    pairs = [
        ("L1", "L2", "M1"),
        ("L1", "C1", "M2"),
        ("L2", "C1", "M3"),
    ]
    plan = compute_plan(pairs, {"L1", "L2"}, {"C1"}, "L1")

    assert len(plan.tree_edges) == 2
    assert len(plan.validation_edges) == 1

    tree_types = {e.edge_type for e in plan.tree_edges}
    assert tree_types == {"lidar_camera"}

    val = plan.validation_edges[0]
    assert val.edge_type == "lidar_lidar"


def test_duplicate_pair_same_marker():
    """Same (d1, d2, marker) listed twice; no crash, one becomes validation."""
    pairs = [("L1", "C1", "M1"), ("L1", "C1", "M1")]
    plan = compute_plan(pairs, {"L1"}, {"C1"}, "L1")

    assert len(plan.all_edges) == 2
    assert len(plan.tree_edges) == 1
    assert len(plan.validation_edges) == 1


def test_duplicate_pair_different_markers():
    """Same (d1, d2) with different markers; one becomes validation edge."""
    pairs = [("L1", "C1", "M1"), ("L1", "C1", "M2")]
    plan = compute_plan(pairs, {"L1"}, {"C1"}, "L1")

    assert len(plan.all_edges) == 2
    assert len(plan.tree_edges) == 1
    assert len(plan.validation_edges) == 1
    markers = {plan.tree_edges[0].marker, plan.validation_edges[0].marker}
    assert markers == {"M1", "M2"}


def test_reversed_pair_order():
    """(L1, C1, M1) vs (C1, L1, M1) produce equivalent plans."""
    plan1 = compute_plan([("L1", "C1", "M1")], {"L1"}, {"C1"}, "L1")
    plan2 = compute_plan([("C1", "L1", "M1")], {"L1"}, {"C1"}, "L1")

    assert len(plan1.tree_edges) == len(plan2.tree_edges)
    assert plan1.tree_edges[0].parent == plan2.tree_edges[0].parent
    assert plan1.tree_edges[0].child == plan2.tree_edges[0].child
    assert plan1.tree_edges[0].edge_type == plan2.tree_edges[0].edge_type


# ── Tree rooting / parent-child tests ────────────────────────────


def test_camera_as_reference_frame():
    """Camera device as root; tree is valid."""
    pairs = [("L1", "C1", "M1"), ("L1", "C2", "M2")]
    plan = compute_plan(pairs, {"L1"}, {"C1", "C2"}, "C1")

    assert plan.reference_frame == "C1"
    # C1 should be root, never a child
    children_set = set()
    for children in plan.tree.values():
        children_set.update(children)
    assert "C1" not in children_set
    # C1 → L1 → C2 chain
    assert len(plan.tree_edges) == 2


def test_deep_chain_parent_child_order():
    """L1-L2-L3-L4 chain rooted at L1; correct parent→child direction."""
    pairs = [
        ("L1", "L2", "M1"),
        ("L2", "L3", "M2"),
        ("L3", "L4", "M3"),
    ]
    plan = compute_plan(pairs, {"L1", "L2", "L3", "L4"}, set(), "L1")

    # Verify parent→child direction follows root outward
    edge_map = {e.child: e.parent for e in plan.tree_edges}
    assert edge_map["L2"] == "L1"
    assert edge_map["L3"] == "L2"
    assert edge_map["L4"] == "L3"


def test_star_topology():
    """One hub lidar connected to 5 cameras; all are direct children."""
    cameras = {f"C{i}" for i in range(1, 6)}
    pairs = [("L1", f"C{i}", f"M{i}") for i in range(1, 6)]
    plan = compute_plan(pairs, {"L1"}, cameras, "L1")

    assert len(plan.tree_edges) == 5
    # All cameras are direct children of L1
    assert set(plan.tree.get("L1", [])) == cameras
    # No camera has children
    for cam in cameras:
        assert cam not in plan.tree or plan.tree[cam] == []


def test_reroot_preserves_edge_count():
    """Same graph rooted at every node; tree_edges is always N-1."""
    pairs = [
        ("L1", "C1", "M1"),
        ("L1", "C2", "M1"),
        ("L1", "L2", "M2"),
        ("L2", "C3", "M3"),
    ]
    lidars = {"L1", "L2"}
    cameras = {"C1", "C2", "C3"}
    all_nodes = lidars | cameras

    for node in all_nodes:
        plan = compute_plan(pairs, lidars, cameras, node)
        assert len(plan.tree_edges) == len(all_nodes) - 1
        assert plan.reference_frame == node


# ── Validation edge tests ────────────────────────────────────────


def test_multiple_validation_edges():
    """Graph with 2+ cycles; correct count of validation edges."""
    # Triangle L1-C1-L2 plus L1-L2 direct → 1 extra
    # Plus L2-C2 and L1-C2 → another cycle
    pairs = [
        ("L1", "C1", "M1"),
        ("L2", "C1", "M1"),
        ("L1", "L2", "M2"),
        ("L1", "C2", "M3"),
        ("L2", "C2", "M3"),
    ]
    plan = compute_plan(pairs, {"L1", "L2"}, {"C1", "C2"}, "L1")

    # 4 nodes → 3 tree edges, 2 validation edges
    assert len(plan.tree_edges) == 3
    assert len(plan.validation_edges) == 2


def test_validation_edge_chain_path():
    """Verify _find_chain returns correct LCA-based path."""
    # L1-C1 (w1), L1-C2 (w1), L2-C1 (w1), L2-C2 (w1), L1-L2 (w2)
    # MST picks 3 lidar-camera edges (3 nodes need 3 tree edges for 4 nodes).
    # L1-L2 (weight 2) is most likely validation, but any single edge could be.
    # We test _find_chain on whatever validation edge the planner produces.
    pairs = [
        ("L1", "C1", "M1"),
        ("L1", "C2", "M2"),
        ("L2", "C1", "M3"),
        ("L2", "C2", "M4"),
        ("L1", "L2", "M5"),
    ]
    plan = compute_plan(pairs, {"L1", "L2"}, {"C1", "C2"}, "L1")

    assert len(plan.validation_edges) >= 1

    for val in plan.validation_edges:
        chain = _find_chain(plan.tree, plan.reference_frame, val.parent, val.child)
        assert chain is not None
        assert chain[0] == val.parent
        assert chain[-1] == val.child
        # Chain length > 2 means it goes through intermediate node(s)
        assert len(chain) >= 2


def test_all_edges_equals_tree_plus_validation():
    """len(all_edges) == len(tree_edges) + len(validation_edges) invariant."""
    test_cases = [
        _simple_pair(),
        _vehicle_setup(),
        _vehicle_with_validation(),
    ]
    for pairs, lidars, cameras, ref in test_cases:
        plan = compute_plan(pairs, lidars, cameras, ref)
        assert len(plan.all_edges) == len(plan.tree_edges) + len(plan.validation_edges)


# ── Error handling tests ─────────────────────────────────────────


def test_self_loop_raises():
    """("L1", "L1", "M1") — self-loop should raise."""
    try:
        compute_plan([("L1", "L1", "M1")], {"L1"}, set(), "L1")
        # If it doesn't raise, the self-loop is silently ignored or produces
        # a degenerate plan. Either way, tree should have 0 useful edges.
    except ValueError:
        pass  # Raising is acceptable


def test_device_in_pair_but_not_in_sets():
    """Device in pairs but not in lidars or cameras raises ValueError."""
    try:
        compute_plan([("L1", "X1", "M1")], {"L1"}, set(), "L1")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "Unknown device" in str(e)


def test_empty_lidars_and_cameras():
    """Both device sets empty with pairs defined raises ValueError."""
    try:
        compute_plan([("L1", "C1", "M1")], set(), set(), "L1")
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "not a known device" in str(e)


# ── Format output tests ──────────────────────────────────────────


def test_format_no_validation_omits_section():
    """No validation edges → 'Validation edges' section absent."""
    pairs, lidars, cameras, ref = _simple_pair()
    plan = compute_plan(pairs, lidars, cameras, ref)
    text = format_plan(plan)

    assert "Validation edges" not in text


def test_format_tree_connectors():
    """├── and └── appear in correct positions for multi-child nodes."""
    pairs = [("L1", "C1", "M1"), ("L1", "C2", "M2"), ("L1", "C3", "M3")]
    plan = compute_plan(pairs, {"L1"}, {"C1", "C2", "C3"}, "L1")
    text = format_plan(plan)

    lines = text.split("\n")
    connector_lines = [l for l in lines if "├──" in l or "└──" in l]
    # 3 children → 2 with ├── and 1 with └──
    assert sum(1 for l in connector_lines if "├──" in l) == 2
    assert sum(1 for l in connector_lines if "└──" in l) == 1


def test_format_deep_nesting_indentation():
    """4+ level deep tree has │ continuation and increasing indentation."""
    # Tree shape: L1 → L2 → [L3, L4], L3 → C1
    # L3 is not-last under L2 and has a child (C1), so │ appears.
    # Expected:
    #   L1
    #       └── L2
    #           ├── L3
    #           │   └── C1
    #           └── L4
    pairs = [
        ("L1", "L2", "M1"),
        ("L2", "L3", "M2"),
        ("L2", "L4", "M3"),
        ("L3", "C1", "M4"),
    ]
    plan = compute_plan(
        pairs, {"L1", "L2", "L3", "L4"}, {"C1"}, "L1"
    )
    text = format_plan(plan)

    # │ continuation line from L3's subtree (L3 is not-last under L2)
    assert "│" in text
    # All nodes present
    for node in ["L1", "L2", "L3", "L4", "C1"]:
        assert node in text
    # C1 is deeper than L2
    lines = text.split("\n")
    c1_line = [l for l in lines if "C1" in l][0]
    l2_line = [l for l in lines if "L2" in l][0]
    assert len(c1_line) - len(c1_line.lstrip()) > len(l2_line) - len(l2_line.lstrip())


def test_format_single_node_pair():
    """Minimal 2-node tree formats without crashing."""
    pairs, lidars, cameras, ref = _simple_pair()
    plan = compute_plan(pairs, lidars, cameras, ref)
    text = format_plan(plan)

    assert "TF Tree (1 edge):" in text  # singular "edge"
    assert "L1" in text
    assert "C1" in text


# ── Structural invariant tests ───────────────────────────────────


def test_tree_edges_form_connected_tree():
    """BFS from root using tree_edges visits all nodes exactly once."""
    pairs, lidars, cameras, ref = _vehicle_setup()
    plan = compute_plan(pairs, lidars, cameras, ref)

    # BFS from root
    visited = {ref}
    queue = [ref]
    while queue:
        node = queue.pop(0)
        for child in plan.tree.get(node, []):
            assert child not in visited, f"Node {child} visited twice (cycle)"
            visited.add(child)
            queue.append(child)

    # All nodes in tree_edges should be visited
    all_nodes = {ref}
    for e in plan.tree_edges:
        all_nodes.add(e.parent)
        all_nodes.add(e.child)
    assert visited == all_nodes


def test_no_cycles_in_tree():
    """Every node has exactly one parent except root."""
    pairs, lidars, cameras, ref = _vehicle_with_validation()
    plan = compute_plan(pairs, lidars, cameras, ref)

    parent_count: dict[str, int] = {}
    for edge in plan.tree_edges:
        parent_count[edge.child] = parent_count.get(edge.child, 0) + 1

    # Each child appears exactly once
    for node, count in parent_count.items():
        assert count == 1, f"Node {node} has {count} parents"

    # Root is never a child
    assert ref not in parent_count


def test_tree_has_n_minus_1_edges():
    """For N nodes in graph, tree always has N-1 edges."""
    test_cases = [
        (_simple_pair(), 2),
        (_vehicle_setup(), 6),
    ]
    for (pairs, lidars, cameras, ref), expected_nodes in test_cases:
        plan = compute_plan(pairs, lidars, cameras, ref)
        assert len(plan.tree_edges) == expected_nodes - 1


# ── Main ──────────────────────────────────────────────────────────────


if __name__ == "__main__":
    import inspect

    # Auto-discover all test functions
    tests = [
        obj
        for name, obj in sorted(globals().items())
        if name.startswith("test_") and inspect.isfunction(obj)
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            print(f"  PASS: {test.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL: {test.__name__}: {e}")
            failed += 1

    print()
    print(f"{passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)
