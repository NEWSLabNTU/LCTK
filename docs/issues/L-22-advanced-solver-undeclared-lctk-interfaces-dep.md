# L-22 · `advanced_extrinsic_solver` imports `lctk_interfaces` without declaring the dependency

- **Severity:** Low
- **Area:** advanced_extrinsic_solver (packaging)
- **Status:** Open
- **Verified:** By code trace (2026-08-14) while scoping Phase 2 of the corner-aligned board frame

## Problem

`advanced_extrinsic_solver/main.py` imports ten service types from `lctk_interfaces.srv` —
`AddDetection`, `ClearBuffer`, `GetStatus`, `ListBuffer`, `RemoveDetection`, `DumpDetections`,
`LoadDetections`, `AdjustTransform`, `ResetTransform`, `GetPoseInfo` — and its entire service surface
depends on them. Its `package.xml` declares no dependency on `lctk_interfaces` of any kind; grepping
the manifest for the name returns nothing.

The import succeeds today only because every workspace build happens to produce `lctk_interfaces`
before anyone runs the node, and because the node is always launched from a fully-built workspace.
Nothing in the package's own metadata says it needs the interface package, so:

- `colcon` has no declared edge to order the two, and relies on incidental ordering;
- a partial build, a per-package rebuild, or an install tree assembled without `lctk_interfaces`
  produces a package that imports cleanly at build time and raises `ModuleNotFoundError` at node
  startup — the failure surfaces far from its cause, in the same class as CLAUDE.md Known Issue 3;
- anyone reading the manifest to understand what this package needs is misled.

The same trace found that `ros/lctk_launch/package.xml` *does* `exec_depend` on both solver packages,
so the dependency graph is otherwise maintained — this is a single omission, not a general practice.

## Suggested fix

Add `<exec_depend>lctk_interfaces</exec_depend>` to `ros/advanced_extrinsic_solver/package.xml`.

Note this is **fixed incidentally** by Phase 2: the spec at
[`2026-08-14-lidar-to-camera-solver-diamond-frame.md`](../superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md)
migrates this package to `lidar_to_camera_solver` and requires the new manifest to declare it. If that
work lands first, close this issue against it rather than patching a package slated for deletion.

Worth checking the other ament_python packages for the same omission while in the area —
`interactive_solver_controller` also consumes `lctk_interfaces` service types.

## Related

- [H-11](./H-11-camera-solvers-stale-board-frame.md) — the Phase 2 work that supersedes this package
- [L-23](./L-23-debug-mode-parameter-never-read.md) — the other dead-metadata finding from the same trace
