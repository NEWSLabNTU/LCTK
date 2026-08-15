# L-18 · `last_push_result` returns an opaque int; `ConfluxResult` is neither exported nor constructible

- **Severity:** Low
- **Area:** conflux_py (API ergonomics)
- **Status:** Open
- **Verified:** Reproduced in a Python session (2026-08-15)
- **Location:** `ros/conflux/conflux_py/conflux_py/_core.py:219-226`,
  `ros/conflux/conflux_py/conflux_py/__init__.py:25-27`

## Problem

`Synchronizer.last_push_result` is a public, documented property whose stated purpose is to
let callers "distinguish a real buffer overflow (BUFFER_FULL) from a late / out-of-order
drop". It returns a bare `int`.

The enum needed to interpret that int is not exported:

```python
from conflux_py import ConfluxResult       # ImportError
from conflux_py._ffi import ConfluxResult  # works — private module
ConfluxResult(2)                           # TypeError: ConfluxResult() takes no arguments
```

`__init__.py` exports `DropPolicy`, `SyncConfig`, `SyncGroup`, `Synchronizer` and (when rclpy
is present) `ROS2Synchronizer`, `SyncStatistics` — but not `ConfluxResult`. And the class is a
plain constant holder, not an `IntEnum`, so it cannot be called on a value to get a name.

The property also reads a private attribute of another object
(`self._ffi_sync._last_result`), so there is no supported way to get at it other than the one
that does not work.

## Failure scenario

Anyone acting on push results — the intended use — must import a private module and hardcode
integer literals, which then silently rot if the FFI codes are renumbered.

## Suggested fix

Make `ConfluxResult` an `enum.IntEnum` in `_ffi.py`, export it from `conflux_py/__init__.py`,
and have `last_push_result` return the enum member rather than the raw int (an `IntEnum` stays
backward-compatible with existing `== 2` comparisons). Give `_ffi.FFISynchronizer` a public
`last_result` accessor so `_core` stops reaching into a private attribute.

Related: L-20.
