# Phase 11: revisioned assisted review

Implements [ADR 0008](../adr/0008-revisioned-assisted-review-session.md).

The review heartbeat carries live synchronization and stillness information.
Capture quality, scene geometry, and matched evidence change independently.
This work gives those changes explicit revisions and makes the browser paint
available state before waiting for evidence.

## Implementation

- Cache the Detection Buffer snapshot and quality projection across unchanged
  reads; serialize review snapshots and revision publication.
- Track evidence completion/eviction separately from capture quality and use
  conditional HTTP for state and scene representations.
- Put browser cache ownership, retries, selection, and async invalidation in
  `ReviewSession`; keep `ReviewApi` as the HTTP adapter.
- Render keyed capture rows and footer sections only when their inputs change.
  Preserve local parameter drafts and the existing orbit convention.
- Exercise delayed hydration, conditional requests, cache reuse, reset/drop,
  stale responses, quality changes, and unchanged DOM through automated tests.

## Verification record

Baseline on `8c77c83`: 760 Python tests passed, one skipped. The baseline is
retained in `tmp/orbit-sign-final.txt` on the development machine.

The deliberate assertion failure was observed through `just test` (exit 1),
then restored. Build/test/lint use the repository recipes; test logs and ROS
runtime files stay under `tmp/`.

An attempted Firefox integration check found only the Ubuntu snap launcher;
the Firefox snap/runtime is absent in the current environment. Browser visual
verification therefore remains an operator check; automated JavaScript tests
cover the state and rendering contracts without requiring WebGL.
