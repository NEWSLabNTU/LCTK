#!/usr/bin/env python3
"""H-02 verification: the first pushed message must not be dropped.

Message IDs were previously assigned starting at 0; id 0 becomes a NULL
c_void_p across the FFI, which ctypes maps back to None in the poll callback,
so the first message could never be looked up and was silently lost. IDs now
start at 1. This drives the low-level Synchronizer and asserts the very first
pushed message survives into the synchronized group.
"""
import sys


def main():
    from conflux_py import SyncConfig, Synchronizer

    # Infinite window (0), generous buffer: pure ordering/grouping, no drops.
    cfg = SyncConfig(window_size_ms=0, buffer_size=100)
    sync = Synchronizer(["/a", "/b"], cfg)

    # The FIRST push in the process -- previously assigned msg_id 0 (NULL).
    assert sync.push("/a", 1000, "a_first") is True, "first push rejected"
    assert sync.push("/b", 1000, "b_first") is True, "second push rejected"

    group = sync.poll()
    assert group is not None, "expected a synchronized group, got None"

    # SyncGroup[topic] returns the stored message object directly.
    a = group["/a"]
    b = group["/b"]
    # unwrap (timestamp, message) if the binding returns a tuple
    if isinstance(a, tuple):
        a = a[-1]
    if isinstance(b, tuple):
        b = b[-1]
    assert a == "a_first", f"first message lost: got {a!r} (H-02 regression)"
    assert b == "b_first", f"second message wrong: got {b!r}"

    print("H-02 PASS: first message survived (got a_first / b_first)")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001 - test harness surfaces the error
        print(f"H-02 FAIL: {e}")
        sys.exit(1)
