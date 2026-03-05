"""
Calibration planner for multi-sensor setups.

Computes a minimum spanning tree over the sensor graph to determine:
1. Which direct calibrations form the TF tree (tree edges)
2. Which calibrations serve as validation (non-tree edges)

Pure Python — no ROS dependencies. Fully unit-testable.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field


# Edge weights: lower = preferred (higher accuracy)
EDGE_WEIGHTS = {
    "lidar_camera": 1,
    "lidar_lidar": 2,
}


@dataclass
class CalibrationEdge:
    """A single calibration between two devices."""

    parent: str  # Parent device name (closer to root in tree)
    child: str  # Child device name
    marker: str
    edge_type: str  # "lidar_camera" | "lidar_lidar"


@dataclass
class CalibrationPlan:
    """Complete calibration plan with TF tree structure."""

    reference_frame: str
    all_edges: list[CalibrationEdge] = field(default_factory=list)
    tree_edges: list[CalibrationEdge] = field(default_factory=list)
    validation_edges: list[CalibrationEdge] = field(default_factory=list)
    tree: dict[str, list[str]] = field(default_factory=dict)  # parent → [children]


class _UnionFind:
    """Union-Find (disjoint set) data structure for Kruskal's algorithm."""

    def __init__(self, nodes: set[str]):
        self.parent = {n: n for n in nodes}
        self.rank = {n: 0 for n in nodes}

    def find(self, x: str) -> str:
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]  # path compression
            x = self.parent[x]
        return x

    def union(self, x: str, y: str) -> bool:
        """Union two sets. Returns True if they were in different sets."""
        rx, ry = self.find(x), self.find(y)
        if rx == ry:
            return False
        if self.rank[rx] < self.rank[ry]:
            rx, ry = ry, rx
        self.parent[ry] = rx
        if self.rank[rx] == self.rank[ry]:
            self.rank[rx] += 1
        return True


def _classify_edge(
    device1: str,
    device2: str,
    lidars: set[str],
    cameras: set[str],
) -> str:
    """Classify a device pair as lidar_camera or lidar_lidar."""
    d1_lidar = device1 in lidars
    d2_lidar = device2 in lidars
    d1_camera = device1 in cameras
    d2_camera = device2 in cameras

    if d1_camera and d2_camera:
        raise ValueError(
            f"Camera-camera calibration is not supported: {device1}, {device2}"
        )

    if (d1_lidar and d2_camera) or (d1_camera and d2_lidar):
        return "lidar_camera"
    if d1_lidar and d2_lidar:
        return "lidar_lidar"

    raise ValueError(f"Unknown device types for pair: {device1}, {device2}")


def compute_plan(
    pairs: list[tuple[str, str, str]],
    lidars: set[str],
    cameras: set[str],
    reference_frame: str,
) -> CalibrationPlan:
    """
    Compute a calibration plan from device pairs.

    Args:
        pairs: List of (device1, device2, marker_name) tuples.
        lidars: Set of lidar device names.
        cameras: Set of camera device names.
        reference_frame: Root device for the TF tree.

    Returns:
        CalibrationPlan with tree edges, validation edges, and adjacency tree.

    Raises:
        ValueError: If the graph is disconnected, no pairs defined,
                    camera-camera pair found, or reference_frame is unknown.
    """
    all_devices = lidars | cameras

    if reference_frame not in all_devices:
        raise ValueError(
            f"Reference frame '{reference_frame}' is not a known device. "
            f"Known devices: {sorted(all_devices)}"
        )

    if not pairs:
        raise ValueError("No calibration pairs defined")

    # Build weighted edges and classify
    weighted_edges: list[
        tuple[int, str, str, str, str]
    ] = []  # (weight, d1, d2, marker, edge_type)
    for d1, d2, marker in pairs:
        if d1 not in all_devices:
            raise ValueError(f"Unknown device in pair: {d1}")
        if d2 not in all_devices:
            raise ValueError(f"Unknown device in pair: {d2}")
        edge_type = _classify_edge(d1, d2, lidars, cameras)
        weight = EDGE_WEIGHTS[edge_type]
        weighted_edges.append((weight, d1, d2, marker, edge_type))

    # Collect all nodes that appear in pairs
    nodes_in_graph: set[str] = set()
    for _, d1, d2, _, _ in weighted_edges:
        nodes_in_graph.add(d1)
        nodes_in_graph.add(d2)

    if reference_frame not in nodes_in_graph:
        raise ValueError(
            f"Reference frame '{reference_frame}' is not connected to any calibration pair"
        )

    # Kruskal's MST
    weighted_edges.sort(key=lambda e: e[0])  # sort by weight
    uf = _UnionFind(nodes_in_graph)
    mst_edges: list[tuple[str, str, str, str]] = []  # (d1, d2, marker, edge_type)
    non_tree_edges: list[tuple[str, str, str, str]] = []

    for weight, d1, d2, marker, edge_type in weighted_edges:
        if uf.union(d1, d2):
            mst_edges.append((d1, d2, marker, edge_type))
        else:
            non_tree_edges.append((d1, d2, marker, edge_type))

    # Check connectivity: all nodes in graph should share a root
    roots = {uf.find(n) for n in nodes_in_graph}
    if len(roots) > 1:
        # Find which components exist for a useful error message
        components: dict[str, list[str]] = defaultdict(list)
        for n in nodes_in_graph:
            components[uf.find(n)].append(n)
        comp_strs = [str(sorted(v)) for v in components.values()]
        raise ValueError(
            f"Sensor graph is disconnected. Cannot build a single TF tree. "
            f"Components: {', '.join(comp_strs)}"
        )

    # Root the MST at reference_frame via BFS → directed parent-child tree
    adjacency: dict[str, list[tuple[str, str, str]]] = defaultdict(
        list
    )  # node → [(neighbor, marker, edge_type)]
    for d1, d2, marker, edge_type in mst_edges:
        adjacency[d1].append((d2, marker, edge_type))
        adjacency[d2].append((d1, marker, edge_type))

    tree_edges: list[CalibrationEdge] = []
    tree_children: dict[str, list[str]] = defaultdict(list)
    visited: set[str] = {reference_frame}
    queue: deque[str] = deque([reference_frame])

    while queue:
        node = queue.popleft()
        for neighbor, marker, edge_type in adjacency[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
                tree_edges.append(
                    CalibrationEdge(
                        parent=node,
                        child=neighbor,
                        marker=marker,
                        edge_type=edge_type,
                    )
                )
                tree_children[node].append(neighbor)

    # Build all_edges (every pair gets a solver)
    all_edges: list[CalibrationEdge] = []
    for d1, d2, marker, edge_type in mst_edges + non_tree_edges:
        all_edges.append(
            CalibrationEdge(parent=d1, child=d2, marker=marker, edge_type=edge_type)
        )

    # Build validation edges with proper parent/child from tree perspective
    validation_edges: list[CalibrationEdge] = []
    for d1, d2, marker, edge_type in non_tree_edges:
        validation_edges.append(
            CalibrationEdge(parent=d1, child=d2, marker=marker, edge_type=edge_type)
        )

    return CalibrationPlan(
        reference_frame=reference_frame,
        all_edges=all_edges,
        tree_edges=tree_edges,
        validation_edges=validation_edges,
        tree=dict(tree_children),
    )


def _find_chain(
    tree: dict[str, list[str]],
    reference_frame: str,
    source: str,
    target: str,
) -> list[str] | None:
    """Find the path between two nodes through the tree (via LCA)."""
    # Build parent map
    parent_map: dict[str, str | None] = {reference_frame: None}
    queue: deque[str] = deque([reference_frame])
    while queue:
        node = queue.popleft()
        for child in tree.get(node, []):
            parent_map[child] = node
            queue.append(child)

    # Find path from source to root
    def path_to_root(node: str) -> list[str]:
        path = []
        current: str | None = node
        while current is not None:
            path.append(current)
            current = parent_map.get(current)
        return path

    source_path = path_to_root(source)
    target_path = path_to_root(target)

    source_set = set(source_path)
    # Find LCA
    lca = None
    for node in target_path:
        if node in source_set:
            lca = node
            break

    if lca is None:
        return None

    # Build path: source → LCA → target
    path_up = []
    for node in source_path:
        path_up.append(node)
        if node == lca:
            break

    path_down = []
    for node in target_path:
        if node == lca:
            break
        path_down.append(node)

    return path_up + list(reversed(path_down))


def format_plan(
    plan: CalibrationPlan,
    device_frame_ids: dict[str, str] | None = None,
) -> str:
    """
    Format a calibration plan as an ASCII tree for display.

    Args:
        plan: The calibration plan to format.
        device_frame_ids: Optional mapping of device names to frame_ids for display.

    Returns:
        Multi-line string with ASCII tree representation.
    """
    if device_frame_ids is None:
        device_frame_ids = {}

    # Build edge lookup: (parent, child) → (marker, edge_type)
    edge_info: dict[tuple[str, str], tuple[str, str]] = {}
    for edge in plan.tree_edges:
        edge_info[(edge.parent, edge.child)] = (edge.marker, edge.edge_type)

    lines: list[str] = []
    lines.append(f"Calibration Plan (reference: {plan.reference_frame})")
    lines.append("")

    num_tree = len(plan.tree_edges)
    num_val = len(plan.validation_edges)
    lines.append(f"TF Tree ({num_tree} edge{'s' if num_tree != 1 else ''}):")

    def _format_node(node: str) -> str:
        frame_id = device_frame_ids.get(node, "")
        if frame_id:
            return f"{frame_id} [{node}]"
        return node

    def _render_tree(node: str, prefix: str, is_last: bool, is_root: bool) -> None:
        if is_root:
            lines.append(f"  {_format_node(node)}")
        else:
            connector = "\u2514\u2500\u2500 " if is_last else "\u251c\u2500\u2500 "
            marker, _ = edge_info.get((parent_map[node], node), ("?", "?"))
            arrow_label = f"{parent_map[node]}-{node} via {marker}"
            lines.append(
                f"  {prefix}{connector}{_format_node(node)}   \u2190 {arrow_label}"
            )

        children = plan.tree.get(node, [])
        for i, child in enumerate(children):
            is_child_last = i == len(children) - 1
            child_prefix = prefix + ("    " if is_last or is_root else "\u2502   ")
            _render_tree(child, child_prefix, is_child_last, False)

    # Build parent map for display
    parent_map: dict[str, str] = {}
    for edge in plan.tree_edges:
        parent_map[edge.child] = edge.parent

    _render_tree(plan.reference_frame, "", True, True)

    if plan.validation_edges:
        lines.append("")
        lines.append(f"Validation edges ({num_val}):")
        for edge in plan.validation_edges:
            chain = _find_chain(
                plan.tree, plan.reference_frame, edge.parent, edge.child
            )
            chain_str = ""
            if chain:
                chain_str = f"  (chain: {' → '.join(chain)})"
            lines.append(f"  {edge.parent}-{edge.child} via {edge.marker}{chain_str}")

    return "\n".join(lines)
