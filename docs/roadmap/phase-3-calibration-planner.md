# Phase 3: Calibration Planner & TF Tree Broadcasting

## Overview

Automate calibration planning for multi-sensor setups. Users declare sensors and which pairs overlap in FOV (via markers). The system computes a minimum spanning tree to determine the TF hierarchy and identifies redundant edges for validation.

Previously, users manually enumerated every `calibration_pairs` entry. For large setups (10+ sensors), reasoning about which pairs to calibrate and how transforms chain together was error-prone. The planner eliminates this by deriving the optimal TF tree automatically.

## 3.1 Core Planner Implementation ✅ DONE

### Config Format

Calibration pairs are defined within marker definitions via `pairs` keys. Each marker lists the device pairs that will use it — grouping tasks by physical marker placement.

```yaml
devices:
  lidars:
    L1: { pointcloud_topic: ..., frame_id: lidar_front }
    L2: { pointcloud_topic: ..., frame_id: lidar_rear }
  cameras:
    C1: { image_topic: ..., frame_id: camera_front_left }
    C2: { image_topic: ..., frame_id: camera_front_right }

# Root of the TF tree (defaults to first lidar)
reference_frame: L1

markers:
  M1:
    type: hollow_board
    board_config: ...
    aruco_config: ...
    bbox_config: ...
    pairs:
      - [L1, C1]
      - [L1, C2]
  M3:
    type: hollow_board
    board_config: ...
    pairs:
      - [L1, L2]
```

All pairs get solver nodes. The spanning tree determines TF structure. Non-tree edges are validation edges.

The old `calibration_pairs` top-level key was removed (no backwards compatibility).

### Graph Algorithm

Pure-Python module `calibration_planner.py` (no ROS dependencies):

1. Collect all pairs from all markers → undirected weighted graph
2. Edge weights: lidar-camera = 1 (preferred), lidar-lidar = 2
3. Minimum spanning tree via Kruskal's algorithm (Union-Find with path compression and union-by-rank)
4. Root MST at `reference_frame` via BFS → directed parent-child tree
5. Non-tree edges flagged as validation edges

**Output — `CalibrationPlan`**:
```python
@dataclass
class CalibrationEdge:
    parent: str           # closer to root in tree
    child: str
    marker: str
    edge_type: str        # "lidar_camera" | "lidar_lidar"

@dataclass
class CalibrationPlan:
    reference_frame: str
    all_edges: list[CalibrationEdge]         # all pairs (all get solvers)
    tree_edges: list[CalibrationEdge]        # spanning tree subset (→ TF)
    validation_edges: list[CalibrationEdge]  # non-tree subset (→ validation)
    tree: dict[str, list[str]]              # adjacency: parent → [children]
```

**Error handling**: disconnected graph, no pairs, camera-camera pair, unknown device, unknown reference frame, reference frame not connected to any pair.

### Plan Display

On launch, the planner logs an ASCII tree:

```
Calibration Plan (reference: L1)

TF Tree (5 edges):
  lidar_front [L1]
  ├── camera_front_left [C1]   ← L1-C1 via M1
  ├── camera_front_right [C2]  ← L1-C2 via M1
  └── lidar_rear [L2]          ← L1-L2 via M3
      ├── camera_rear_left [C3]  ← L2-C3 via M2
      └── camera_rear_right [C4] ← L2-C4 via M2

Validation edges (1):
  L2-C2 via M1  (chain: L2 → L1 → C2)
```

### TF Tree Broadcaster

New lightweight ROS node `tf_tree_broadcaster` (executable within `lctk_launch` package):

- Subscribes to each solver's output `TransformStamped` topic (tree edges only)
- Broadcasts to `/tf_static` via `StaticTransformBroadcaster`
- TF2 natively computes chain transforms (A→C = A→B + B→C)
- Uses TRANSIENT_LOCAL QoS durability

Replaces per-solver `publish_tf` with a centralized broadcaster that understands the full tree.

### Integration

The planner runs on every config parse. All edges become `CalibrationPair` objects fed into the existing `_derive_pipeline()`. Downstream node spawning logic is unchanged.

```
markers[].pairs → CalibrationPlanner → CalibrationPlan
                                            ↓
                                     all_edges → [CalibrationPair]
                                            ↓
                                  _derive_pipeline() → PipelineConfig
                                            ↓
                          calibrate.launch.py (log plan + spawn tf_tree_broadcaster)
```

### Files Changed

| File                                                 | Action   | Description                                                       |
|------------------------------------------------------|----------|-------------------------------------------------------------------|
| `ros/lctk_launch/lctk_launch/calibration_planner.py` | NEW      | Graph algorithm, MST, plan formatting                             |
| `ros/lctk_launch/lctk_launch/tf_tree_broadcaster.py` | NEW      | Subscribes to solver transforms, broadcasts /tf_static            |
| `ros/lctk_launch/lctk_launch/config_parser.py`       | MODIFIED | Parse marker pairs, invoke planner, attach plan to PipelineConfig |
| `ros/lctk_launch/launch/calibrate.launch.py`         | MODIFIED | Log plan, spawn tf_tree_broadcaster for tree edges                |
| `ros/lctk_launch/setup.py`                           | MODIFIED | Register tf_tree_broadcaster entry point                          |
| `ros/lctk_launch/package.xml`                        | MODIFIED | Add tf2_ros_py dependency                                         |
| `ros/lctk_launch/lctk_launch/__init__.py`            | MODIFIED | Export planner types                                              |
| `ros/lctk_launch/config/examples/sample_data.yaml`   | MODIFIED | Converted to marker pairs format                                  |
| `ros/lctk_launch/config/examples/vehicle.yaml`       | MODIFIED | Converted to marker pairs format, added reference_frame           |
| `ros/lctk_launch/test/test_config_parser.py`         | MODIFIED | Added plan assertions                                             |

### Work Items

- [x] Implement `calibration_planner.py` (Kruskal's MST, Union-Find, BFS rooting, format_plan)
- [x] Implement `tf_tree_broadcaster.py` (subscribe to solver topics, broadcast /tf_static)
- [x] Modify `config_parser.py` (parse marker pairs, run planner, attach plan)
- [x] Modify `calibrate.launch.py` (log plan, spawn tf_tree_broadcaster)
- [x] Update `setup.py`, `package.xml`, `__init__.py`
- [x] Convert `sample_data.yaml` and `vehicle.yaml` to marker pairs format
- [x] Remove old `calibration_pairs` support
- [x] Update `test_config_parser.py` with plan assertions
- [x] Build passes (`just build`)

---

## 3.2 Planner Unit Tests

### Goal

Comprehensive test coverage for `calibration_planner.py`. The module is pure Python with no ROS dependencies, so tests run with plain pytest.

```bash
PYTHONPATH=ros/lctk_launch pytest ros/lctk_launch/test/test_calibration_planner.py -v
```

### Existing Tests (20)

Basic coverage from initial implementation:

- [x] `test_simple_pair` — Single lidar-camera pair
- [x] `test_vehicle_spanning_tree` — 6-node vehicle setup
- [x] `test_mst_prefers_lidar_camera` — Weight preference
- [x] `test_validation_edges` — Extra edge becomes validation
- [x] `test_tree_adjacency` — Parent→children mapping
- [x] `test_rooted_at_reference` — Reference frame is never a child
- [x] `test_different_reference_frame` — Re-rooting at L2
- [x] `test_no_pairs_raises` — Empty pairs list
- [x] `test_disconnected_graph_raises` — Disconnected components
- [x] `test_camera_camera_pair_raises` — Invalid pair type
- [x] `test_unknown_device_raises` — Device not in lidars/cameras
- [x] `test_unknown_reference_frame_raises` — Reference not a known device
- [x] `test_disconnected_reference_frame_raises` — Reference not in any pair
- [x] `test_format_simple` — Basic format output
- [x] `test_format_with_frame_ids` — Frame IDs in display
- [x] `test_format_validation_edges` — Validation section with chain paths
- [x] `test_format_vehicle` — Full vehicle format (visual check)
- [x] `test_single_lidar_lidar` — Two lidars only
- [x] `test_three_lidars_chain` — L1-L2-L3 chain
- [x] `test_large_setup` — 10-sensor scalability

### New Tests (22) ✅ DONE

#### MST / Graph Algorithm

- [x] `test_mst_breaks_tie_deterministically` — Two equal-weight paths between nodes; verify stable output across multiple runs
- [x] `test_mst_displaces_lidar_lidar_when_redundant` — Lidar-lidar edge (weight 2) displaced by lidar-camera path (weight 1) when both connect the same components
- [x] `test_duplicate_pair_same_marker` — Same (d1, d2, marker) listed twice; verify no crash or duplicate edges
- [x] `test_duplicate_pair_different_markers` — Same (d1, d2) with different markers; one becomes validation edge
- [x] `test_reversed_pair_order` — (L1, C1, M1) vs (C1, L1, M1) produce equivalent plans

#### Tree Rooting / Parent-Child

- [x] `test_camera_as_reference_frame` — Camera device as root; tree is valid with camera at root
- [x] `test_deep_chain_parent_child_order` — L1-L2-L3-L4 chain rooted at L1; each edge has correct parent→child direction
- [x] `test_star_topology` — One hub lidar connected to 5+ cameras; all are direct children of root
- [x] `test_reroot_preserves_edge_count` — Same graph rooted at every possible node; tree_edges count is always N-1

#### Validation Edges

- [x] `test_multiple_validation_edges` — Graph with 2+ cycles; correct count of validation edges
- [x] `test_validation_edge_chain_path` — Verify `_find_chain` returns correct LCA-based path for a validation edge
- [x] `test_all_edges_equals_tree_plus_validation` — `len(all_edges) == len(tree_edges) + len(validation_edges)` invariant

#### Error Handling

- [x] `test_self_loop_raises` — `("L1", "L1", "M1")` raises or degenerates gracefully
- [x] `test_device_in_pair_but_not_in_sets` — Device in pairs but not in lidars or cameras
- [x] `test_empty_lidars_and_cameras` — Both sets empty with pairs defined

#### Format Output

- [x] `test_format_no_validation_omits_section` — No validation edges → "Validation edges" section absent
- [x] `test_format_tree_connectors` — `├──` and `└──` in correct positions for multi-child nodes
- [x] `test_format_deep_nesting_indentation` — 4+ level deep tree has correct indentation with `│` continuation
- [x] `test_format_single_node_pair` — Minimal 2-node tree formats without crashing

#### Structural Invariants

- [x] `test_tree_edges_form_connected_tree` — BFS from root using tree_edges visits all nodes exactly once
- [x] `test_no_cycles_in_tree` — Tree edges contain no cycles (every node has exactly one parent except root)
- [x] `test_tree_has_n_minus_1_edges` — For N nodes in graph, tree always has N-1 edges

### Acceptance Criteria

- [x] All existing 20 tests still pass
- [x] All 22 new tests pass
- [x] `pytest -v` runs in <1s (42 tests in 0.05s)
- [x] No test depends on execution order
- [x] Edge cases for `_find_chain` (LCA computation) are covered
