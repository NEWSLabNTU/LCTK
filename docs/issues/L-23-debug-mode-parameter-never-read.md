# L-23 · `debug_mode` is declared by both solvers and read by neither

- **Severity:** Low
- **Area:** extrinsic_solver_node, advanced_extrinsic_solver
- **Status:** Open
- **Verified:** By code trace (2026-08-14) while scoping Phase 2 of the corner-aligned board frame

## Problem

Both LiDAR-camera solvers declare a `debug_mode` parameter. Neither reads it. In each file the
identifier appears exactly once — on its own `declare_parameter` line — and never again.

An operator setting `debug_mode:=true` therefore gets no additional output, no warning, and no
indication that the parameter does nothing. The parameter is also plumbed through the launch layer,
so it looks supported from the outside: it appears in the node's parameter list under `ros2 param
list`, and reads as a documented knob.

This is the same failure shape as several findings closed this month — a control that appears to work
and silently does nothing. It is harmless in isolation, but it costs an operator a debugging session
the first time they reach for it while diagnosing something else, which is exactly when they can least
afford it.

Note the justfile carries its own `debug_mode` variable used for other nodes; that one is live. Only
the two solvers' parameter is dead, which makes the trap worse — the name works elsewhere.

## Suggested fix

Either wire it up or delete it, but do not leave it declared.

Wiring it up is the more useful option and is cheap: both solvers already emit per-solve logging at
`info` and rate-limit some messages; gating the verbose paths on `debug_mode` gives the parameter the
meaning its name promises. Deleting it is acceptable if nothing wants the extra output, but then it
must also be removed wherever launch forwards it, or the launch layer will pass a parameter the node
no longer declares.

For `extrinsic_solver_node` specifically, prefer deletion: Phase 2
([spec](../superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md)) deletes that package
entirely, so any effort spent wiring it up is discarded.

## Related

- [H-11](./H-11-camera-solvers-stale-board-frame.md) — the Phase 2 work that replaces both packages
- [L-22](./L-22-advanced-solver-undeclared-lctk-interfaces-dep.md) — the other dead-metadata finding from the same trace
