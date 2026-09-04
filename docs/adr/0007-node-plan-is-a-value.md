# 0007. The calibration graph is built as a value, and the launch file only realises it

- **Date:** 2026-09-04
- **Status:** accepted

## Context

`calibrate.launch.py` decided the entire calibration graph inside one function,
`generate_nodes(context)`: which nodes exist, what each is called and namespaced, every parameter,
every remapping, the log lines, and the order of all of it. At its largest it was 356 lines in one
scope, in a 498-line file.

Its only interface was a launch callback, so every question about the graph had to be asked through
a launch context. `test_calibrate_launch_graph.py` reached 1083 lines doing that — staging a
`_LaunchContext` stub and reading `launch_ros.Node` internals (`_Node__parameters`,
`_Node__node_name`) to assert things as simple as "a two-lidar config produces two board detectors".
The module those assertions are really about had no interface of its own to test.

This is the shape ADR-0001 and ADR-0002 already moved away from elsewhere: a procedure whose
behaviour can only be observed by running the thing that hosts it.

## Decision

`ros/lctk_launch/lctk_launch/node_plan.py` builds the graph as a value:

```python
build_node_plan(pipeline: PipelineConfig, settings: RunSettings) -> list[PlanEntry]
```

A `PlanEntry` is a `Message`, a `NodeSpec` (package, executable, name, namespace, parameters,
remappings, arguments) or a `JudgeInclude`. The list is **ordered**, because the order is part of
what it describes: each node is preceded by the line announcing it, and a replay's log is read top
to bottom.

The module imports no launch types, and a test asserts that by reading its imports — not by
grepping its text, since its docstring names `launch_ros.Node` while explaining what it does not
use.

`generate_nodes` keeps only the mapping from entries to launch actions. It has no decisions left:
read five launch configurations into a `RunSettings`, parse the config, map each entry. The launch
file is 173 lines.

`RunSettings.__post_init__` validates `solver_mode`, so the refusal is reachable without a launch
context and happens before any of the plan exists rather than partway through building it. It
raises `RuntimeError` rather than `ValueError` to match the guard it replaced.

## Consequences

**Easy.** A question about the graph is answered by reading a list of dataclasses.
`test_node_plan.py` asserts what the old graph tests assert, in a twelfth of the lines, with no
launch import. Adding a node kind means adding a `NodeSpec` to a small function, not editing a
356-line one.

**Hard.** Two files now describe the graph rather than one, and a reader chasing a parameter must
know the plan is where decisions live and the launch file is where they become actions. The launch
file's docstring says so in its first sentence.

**Deliberately not done.** `CalibrationPlan` in `calibration_planner.py` was not extended to carry
this. It answers a different question — which edges form the TF spanning tree and which are
validation edges — and `node_plan` consumes its answer. Merging them would put graph topology and
node parameters behind one name.

**Not changed.** `test_calibrate_launch_graph.py` stays as it is. It now covers the adapter
end to end, which is exactly what it should cover, and it passed unchanged through this move —
which is what makes the move demonstrably faithful rather than merely plausible.
