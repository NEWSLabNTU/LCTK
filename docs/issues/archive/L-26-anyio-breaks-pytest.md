# L-26 · A pip `--user` `anyio` breaks pytest workspace-wide before collection

- **Severity:** Low
- **Area:** development environment
- **Status:** Fixed (2026-08-15)
- **Verified:** `pytest conflux_py/test/` runs with plugin autoload enabled after removal
- **Location:** `CLAUDE.md` Known Issue 3

## Problem

`anyio >= 4.3` ships a pytest plugin that imports `_pytest.scope`, a module added in pytest 7.
Ubuntu 22.04 / ROS 2 Humble ship pytest 6.2.5 from apt. Plugin autoload pulls the anyio plugin
into **every** pytest invocation, which aborts during startup:

```
File "/home/aeon/.local/lib/python3.10/site-packages/anyio/pytest_plugin.py", line 15, in <module>
    from _pytest.scope import Scope
ModuleNotFoundError: No module named '_pytest.scope'
```

No test is collected, in any package. Fourth instance of the pip-shadowing failure class
already documented in CLAUDE.md alongside setuptools, numpy and scipy.

## Failure scenario

Every pytest suite in the workspace fails identically, with a traceback pointing at anyio
rather than at anything the user was working on. Under `colcon test` the failure was invisible
entirely, because colcon was running unittest instead (M-25).

## Resolution (2026-08-15)

- `pip3 uninstall -y anyio`.
- CLAUDE.md Known Issue 3 gained a row for the symptom, the root cause (anyio's plugin needs
  pytest 7; apt ships 6.2.5), and the `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` escape hatch that
  works without uninstalling.

**Caveat recorded in CLAUDE.md:** anyio was a dependency of the pip `--user` `starlette`,
which this removal breaks. Anything needing starlette/fastapi should install it into a venv —
a bare `pip3 install --user anyio` re-breaks pytest workspace-wide.

Related: M-25, L-06.
