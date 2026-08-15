# L-19 · `conflux_py/__init__.py` swallows real ImportErrors, hiding `ROS2Synchronizer`

- **Severity:** Low
- **Area:** conflux_py
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/conflux_py/conflux_py/__init__.py:29-35`

## Problem

```python
try:
    from .synchronizer import ROS2Synchronizer, SyncStatistics  # noqa: F401
    __all__.extend(["ROS2Synchronizer", "SyncStatistics"])
except ImportError:
    pass
```

The intent is "skip the ROS2 wrapper when rclpy is absent". What it actually does is swallow
**every** `ImportError` raised anywhere in `synchronizer.py` — a typo in a module-level
import, a renamed symbol, a missing message package, a partially built workspace.

This is the Pokemon-exception pattern the project's own CLAUDE.md prohibits, and the same
class of finding as the archived L-06.

## Failure scenario

A genuine breakage inside `synchronizer.py` makes `ROS2Synchronizer` silently vanish from the
package. The user's node then fails at `from conflux_py import ROS2Synchronizer` with a bare
ImportError naming *conflux_py*, pointing at the wrong file entirely. The real cause never
appears in any traceback.

## Suggested fix

Narrow the guard to the condition actually being probed:

```python
try:
    import rclpy  # noqa: F401
except ImportError:
    pass
else:
    from .synchronizer import ROS2Synchronizer, SyncStatistics  # noqa: F401
    __all__.extend(["ROS2Synchronizer", "SyncStatistics"])
```

Any failure inside `synchronizer.py` then propagates with its real traceback.

Related: L-06 (archived, same pattern).

## Resolution (2026-08-15)

Fixed in conflux (`jerry73204/conflux`@0a9c901; LCTK pins it). The guard now probes for the
condition it actually means:

```python
try:
    import rclpy  # noqa: F401
except ImportError:
    pass
else:
    from .synchronizer import ROS2Synchronizer, SyncStatistics  # noqa: F401
    __all__.extend(["ROS2Synchronizer", "SyncStatistics"])
```

Any `ImportError` raised inside `synchronizer.py` now propagates with its own traceback, instead
of making `ROS2Synchronizer` silently vanish and surfacing later as an ImportError naming
`conflux_py`.
