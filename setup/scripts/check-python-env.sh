#!/usr/bin/env bash
# Guard against pip --user packages shadowing the apt ones ROS 2 Humble was built
# against (CLAUDE.md Known Issue 3).
#
# ~/.local/lib/python3.10/site-packages precedes /usr/lib/python3/dist-packages on
# sys.path, so a pip --user install silently replaces the apt package and fails far from
# the cause: --editable errors at build time, numpy ABI errors at node startup, a scipy
# TypeError in tests, or a pytest that cannot start at all.
#
# Single implementation, called by both `just _check-python-env` and the setup engine's
# `python-guard` step, so the two cannot drift.

set -uo pipefail

fail=0

for pkg in setuptools numpy scipy; do
    location=$(python3 -c "import $pkg; print($pkg.__file__)" 2>/dev/null) || continue
    version=$(python3 -c "import $pkg; print($pkg.__version__)" 2>/dev/null) || continue
    if [[ "$location" != /usr/lib/python3/dist-packages/* ]]; then
        echo "error: $pkg $version shadows the apt package that ROS 2 Humble needs." >&2
        echo "       found: $location" >&2
        echo "       Fix with:  pip3 uninstall -y $pkg" >&2
        echo "" >&2
        fail=1
    fi
done

# anyio >= 4.3 ships a pytest plugin importing _pytest.scope, added in pytest 7; apt
# ships pytest 6.2.5 and plugin autoload pulls it in on every pytest invocation, so a
# shadowing anyio breaks the whole workspace's Python suite before collection starts.
# Only >= 4.3 does this, so check the version rather than the location alone.
anyio_version=$(python3 -c "from importlib.metadata import version; print(version('anyio'))" 2>/dev/null) || anyio_version=""
if [[ -n "$anyio_version" ]]; then
    anyio_location=$(python3 -c "import anyio; print(anyio.__file__)" 2>/dev/null) || anyio_location=""
    if [[ "$anyio_location" != /usr/lib/python3/dist-packages/* ]]; then
        if ! python3 -c "
import sys
v = [int(p) for p in '${anyio_version}'.split('.')[:2]]
sys.exit(0 if v < [4, 3] else 1)
" 2>/dev/null; then
            echo "error: anyio $anyio_version breaks every pytest run in this workspace." >&2
            echo "       found: $anyio_location" >&2
            echo "       Fix with:  pip3 uninstall -y anyio" >&2
            echo "       Escape hatch:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest ..." >&2
            echo "" >&2
            fail=1
        fi
    fi
fi

# The failure that actually bites at runtime: cv2 cannot import under a numpy it was not
# built against. Check it directly rather than inferring it from version numbers.
if ! python3 -c 'import cv2' 2>/dev/null; then
    echo "error: 'import cv2' fails. The solver nodes import cv2 and will crash at startup." >&2
    python3 -c 'import cv2' 2>&1 | tail -1 | sed 's/^/       /' >&2
    echo "       Usually a pip numpy shadowing apt's; fix with:  pip3 uninstall -y numpy" >&2
    fail=1
fi

exit "$fail"
