#!/usr/bin/env python3
"""C-02 verification: message references must not leak under DropOldest.

Under DropOldest (realtime mode), the core evicts buffered messages silently
(push still returns Ok, poll never matches them), so the Python binding used to
retain one reference per evicted message forever. Push many messages that all
get evicted and assert the internal _message_refs table stays bounded.
"""
import sys


def main():
    from conflux_py import DropPolicy, SyncConfig, Synchronizer

    cfg = SyncConfig(
        window_size_ms=50, buffer_size=2, drop_policy=DropPolicy.DROP_OLDEST
    )
    sync = Synchronizer(["/a", "/b"], cfg)

    N = 3000
    for i in range(N):
        # push only to /a: /b stays empty so nothing ever matches -> every /a
        # message is eventually evicted by DropOldest.
        sync.push("/a", (i + 1) * 1_000_000, object())
        sync.poll()

    refs = sync._ffi_sync._message_refs
    n = len(refs)
    # Bounded by buffer occupancy (2) plus at most one reconcile interval (16).
    assert n <= 40, f"C-02 leak: {n} references retained after {N} pushes"
    print(f"C-02 PASS: {n} refs retained after {N} pushes (bounded, no leak)")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001
        print(f"C-02 FAIL: {e}")
        sys.exit(1)
