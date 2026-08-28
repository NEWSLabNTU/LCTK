# L-26 · `extrinsic_solver_node.launch.xml` (lidar_to_camera_solver) cannot start the node

- **Severity:** Low
- **Area:** lidar_to_camera_solver / launch
- **Status:** Open
- **Verified:** By code trace (2026-08-28) against `main.py` and the launch file's current
  contents, plus `git show` on the commit before the identity-gate change to confirm the gap
  predates it

## Problem

`ros/lidar_to_camera_solver/launch/extrinsic_solver_node.launch.xml` declares and passes only:
`parent_frame`, `child_frame`, `camera_topic`, `debug_mode`, `publishing_rate`, plus remappings for
`aruco_detections`, `calibration_board_detections` and `extrinsic_transform`. It passes **no
target parameter of any kind** — no `target_config`, no legacy `aruco_config_file`.

`lidar_to_camera_solver/main.py` declares `target_config` with default `""`
(`_declare_parameters`, line 250) and, in `__init__`, unconditionally calls
`self._load_target_definition(target_config_file)` (line 150). `_load_target_definition` (line
1038) does:

```python
target_config_file = target_config_file.strip()
if not target_config_file:
    raise ValueError("target_config is required")
```

So launching this file raises `target_config is required` during node construction — the node
never starts.

## This is not new

The gap predates the target-identity-gate work (`f97156e`, "feat(solver): gate target identity"):
at the commit before it, the launch file already declared no `aruco_config_file` arg or param, and
`main.py` already declared `("aruco_config_file", "")` and unconditionally loaded it — same
failure, different parameter name. The launch file has never, at any point in its history under
either package name (`advanced_extrinsic_solver` → `lidar_to_camera_solver`), passed the parameter
its own node requires to start.

## Why this has gone unnoticed

The file appears to be orphaned. It is not referenced by any `just` recipe, any other launch file,
or any `README.md`/`book/` instruction (`grep -rn "extrinsic_solver_node.launch.xml"` across
`*.py`, `*.xml`, `*.md`, and `justfile` turns up only a mention in
`docs/superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md` listing it among files
slated for cleanup). The maintained manual-mode workflow (`just solver_mode=manual lidar-camera` +
`just manual-solver-controller`, documented in CLAUDE.md) goes through `calibrate.launch.py` /
`lctk_launch`, not this file. Anyone who *does* discover it — it's named identically to the launch
file in the deprecated `ros/extrinsic_solver_node/` package, and sits inside the currently
maintained `lidar_to_camera_solver` package, which invites confusion about which one is live — and
tries `ros2 launch lidar_to_camera_solver extrinsic_solver_node.launch.xml` directly will hit the
`ValueError` above.

## Suggested fix

Delete it. It duplicates functionality `calibrate.launch.py` already provides for
`lidar_to_camera_solver` (with a working target parameter), nothing references it, and keeping a
same-named, non-functional twin of the deprecated `ros/extrinsic_solver_node/` package's launch
file next to the maintained solver is actively confusing. If a standalone single-node launch entry
point is still wanted for manual testing, it needs a `target_config` arg/param wired through like
`calibrate.launch.py`'s.
