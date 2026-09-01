# L-23 · `debug_mode` is declared by `lidar_to_camera_solver` and read by neither

- **Severity:** Low
- **Area:** `lidar_to_camera_solver` / solver parameter surface
- **Status:** Open
- **Verified:** By code trace (2026-08-14) while scoping Phase 2 of the corner-aligned board frame

## Problem

The maintained `lidar_to_camera_solver` declares a `debug_mode` parameter but never reads it. The
identifier appears exactly once — on its own `declare_parameter` line — and never again.

An operator setting `debug_mode:=true` therefore gets no additional output, no warning, and no
indication that the parameter does nothing. The parameter is also plumbed through the launch layer,
so it looks supported from the outside: it appears in the node's parameter list under `ros2 param
list`, and reads as a documented knob.

This is the same failure shape as several findings closed this month — a control that appears to work
and silently does nothing. It is harmless in isolation, but it costs an operator a debugging session
the first time they reach for it while diagnosing something else, which is exactly when they can least
afford it.

Note the justfile carries its own `debug_mode` variable used for other nodes; that one is live. The
solver parameter is dead, which makes the trap worse — the name works elsewhere.

## Suggested fix

Either wire it up or delete it, but do not leave it declared.

Wiring it up is the more useful option and is cheap: both solvers already emit per-solve logging at
`info` and rate-limit some messages; gating the verbose paths on `debug_mode` gives the parameter the
meaning its name promises. Deleting it is acceptable if nothing wants the extra output, but then it
must also be removed wherever launch forwards it, or the launch layer will pass a parameter the node
no longer declares.

The former `extrinsic_solver_node` half is resolved by H-11 Stage 3 deletion; no effort should be
spent wiring a deleted package back up.

## Related

- [H-11](./archive/H-11-camera-solvers-stale-board-frame.md) — the Phase 2 work that replaced the legacy package
- [L-22](./archive/L-22-advanced-solver-undeclared-lctk-interfaces-dep.md) — the other dead-metadata finding from the same trace

## Historical update (2026-08-28) — pointer repaired; finding unchanged at that time

`ros/advanced_extrinsic_solver` no longer existed — it was renamed to `ros/lidar_to_camera_solver` in
`ecba23c`. Unlike L-22 (fixed by the rename), this finding survived that rename:
`debug_mode` was still declared and still never read.

- `ros/lidar_to_camera_solver/lidar_to_camera_solver/main.py:251` — `("debug_mode", True)` in the
  parameter declaration list; `grep -rn debug_mode ros/lidar_to_camera_solver/` finds no other
  occurrence in the package.
- `ros/extrinsic_solver_node/extrinsic_solver_node/main.py:102` — `self.declare_parameter("debug_mode",
  True)`, also unread at that time. The package was superseded and pending deletion per the
  diamond-frame plan, so this half needed no separate action.

Status and severity were unchanged then; only the `advanced_extrinsic_solver` →
`lidar_to_camera_solver` pointer needed repair.

## Update (2026-09-01) — legacy half resolved by deletion

Stage 3 of H-11 deleted `ros/extrinsic_solver_node/`. Its unread `debug_mode` declaration therefore
needs no separate fix. The remaining live finding is the unchanged declaration in
`ros/lidar_to_camera_solver/lidar_to_camera_solver/main.py:251`; L-23 remains open for that half.
