# L-24: Board-geometry import test rejects unrelated Jetson EGL stdout

**Severity:** Low

**Status:** 🟢 Fixed (2026-08-18)

## Symptom

`just test` fails at `test_module_imports_without_rclpy` on Jetson even though the
fresh interpreter prints `False`, proving that importing `board_geometry` did not
import `rclpy`.

NVIDIA's runtime appends this unrelated line to the subprocess stdout:

```text
nvbufsurftransform: Could not get EGL display connection
```

The test compares the entire stripped stdout with `False`, so harmless platform
diagnostics fail the assertion.

## Resolution criteria

- Give the probe output a unique sentinel and assert that sentinel as one complete
  line, without accepting `rclpy=True`.
- Keep checking the subprocess return code and stderr.
- Verify `just test` on Jetson.

## Resolution

The subprocess now prints a unique `LCTK_BOARD_GEOMETRY_RCLPY=<bool>` sentinel. The test selects
that complete line and requires its value to be `False`, so unrelated NVIDIA EGL diagnostics no
longer corrupt the assertion while `rclpy=True` still fails.

Verified on Jetson: `just test` collected and passed 240 Rust tests and 181 Python tests.
