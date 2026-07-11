# H-02 · Conflux Python binding drops the first message (msg_id 0 → NULL pointer)

- **Severity:** High
- **Area:** conflux synchronizer (Python FFI)
- **Status:** Fixed (2026-07-11, conflux submodule)
- **Verified:** Yes (confirmed against live source, 2026-07-09)
- **Location:** `ros/conflux/conflux_py/conflux_py/_ffi.py:211, 237-245, 273-279`

## Problem

`_next_id` starts at `0`. The first message gets `msg_id = 0`, passed to the core as `ctypes.c_void_p(0)` — which is a NULL pointer. When the poll callback receives it back as a `c_void_p`, ctypes maps NULL to `None`, so `msg_id in self._message_refs` is `False` and the stored Python message is not found.

## Failure scenario

The first sync group that needs the id-0 message fails: the consumer's `group["/topic"]` raises `KeyError`, or the group resolves to empty and is silently swallowed as "no match". Only the very first message per binding is affected, so it is intermittent and easy to miss.

## Suggested fix

Start `self._next_id = 1` (and never reuse 0). A one-line change.

## Resolution (2026-07-11)

Fixed in the conflux submodule (`jerry73204/conflux`, branch `fix/h02-h05-stats`,
commit da9f101): `_next_id` now starts at 1. LCTK pins the updated submodule.
Verified with a functional test (`tmp/test_h02_first_message.py`) that drives the
low-level `Synchronizer` and confirms the very first pushed message survives into
the synchronized group. Should be upstreamed to conflux `main` via PR.
