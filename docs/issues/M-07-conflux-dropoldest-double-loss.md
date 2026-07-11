# M-07 · Conflux DropOldest can destroy a good buffered message and reject the new one

- **Severity:** Medium
- **Area:** conflux-core
- **Status:** Open
- **Verified:** Static review
- **Location:** `ros/conflux/crates/conflux-core/src/state.rs:308-329`

## Problem

Under DropOldest, eviction (`pop_front`) happens **before** the monotonic-timestamp check. Given a buffer `[100, 200]` and an incoming message at `150`: it evicts `100`, then `try_push(150)` fails with `OutOfOrder` (last_ts is 200). Net result: `100` is destroyed **and** `150` is rejected — strictly worse than RejectNew.

## Failure scenario

Out-of-order arrivals (normal under BEST_EFFORT) cause double message loss. Compounds C-02 (leak) and H-05 (miscount).

## Suggested fix

Perform the ordering/acceptance check before evicting, so a message that would be rejected does not first destroy a valid buffered one. Only evict when the new message will actually be accepted.
