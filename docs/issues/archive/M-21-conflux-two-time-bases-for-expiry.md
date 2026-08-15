# M-21 · Expiry is defined in two incompatible time bases (wall clock vs message stamp)

- **Severity:** Medium
- **Area:** conflux-core
- **Status:** Fixed (2026-08-15)
- **Verified:** Static review (2026-08-15)
- **Location:** `ros/conflux/crates/conflux-core/src/staleness.rs:139`, `:211` (wall clock)
  vs `ros/conflux/crates/conflux-core/src/state.rs:283-290` and
  `ros/conflux/crates/conflux-core/src/sync.rs:118-125` (message time)

## Problem

conflux expires messages through two mechanisms that measure time differently:

| Mechanism | Reference | Type |
|-----------|-----------|------|
| `StalenessDetector` / `ConstrainedHeap` / `TimerWheel` | `Instant::now()` — wall clock | `Instant` |
| `State::drop_expired_messages`, driven from the poll loop with `commit_ts` | the newest committed **message stamp** | `Duration` |

Both are described as "expiration" in the API and both consume `WithTimestamp::timeout()` as
their budget, but they answer different questions: one asks "how long since this arrived in
real time", the other "how far has the message clock advanced past it".

## Failure scenario

For recorded playback the two diverge without bound. `lctk_sample_data` replays a pcap/avi
whose stamps are unrelated to the current wall clock — and offline mode is the LCTK default.
A timeout that behaves correctly against message time is meaningless against wall time and
vice versa; playback faster or slower than real time makes the wall-clock path expire
everything or nothing.

This is the same class of defect as M-04 (L2L staleness compared wall clock against sensor
stamps), which was fixed in the LCTK node but never in conflux itself.

## Suggested fix

Pick message time as the single reference, since it is the only one meaningful for recorded
data, and drive all expiry from the highest observed stamp:

- Convert the staleness subsystem to `Duration`-based deadlines keyed off the stream clock.
- Where wall-clock behaviour is genuinely wanted (live sensors with a stalled publisher), make
  it an explicit, separately named policy rather than an implementation detail.
- Document which clock each knob uses in `StalenessConfig`.

Related: M-04 (archived, same class), H-11, M-17.

## Resolution (2026-08-15) — removed

Closed by removing the staleness subsystem entirely (`jerry73204/conflux`@014a2c9; LCTK pins it),
per [Phase 8](../../roadmap/phase-8-conflux-staleness-subsystem.md) Stage 0. `ConstrainedHeap`,
`TimerWheel`, `StalenessDetector` and the placeholder background task are gone — about 700 lines
of source and 22 tests of the deleted machinery.

The decision rested on three facts, not on this defect alone:

- **Nothing reached it.** The FFI hardcoded `staleness_detector: None`, so no binding — and
  therefore no LCTK node — ever executed this code.
- **Every part of it was defective.** M-17 through M-21 were found in a single reading pass, and
  H-11 (expiry anchored to construction time) meant the subsystem had never worked correctly at all.
- **It was built on the wrong clock.** M-21: expiry ran on `Instant` while the rest of the pipeline
  runs on message time. For recorded playback — LCTK's default mode — wall-clock expiry is
  meaningless, so repair would have meant a rewrite rather than a patch.

**What remains** is `Buffer::drop_expired` / `State::drop_expired_messages`, driven by
`WithTimestamp::timeout`: message-time expiry, which is the semantics recorded data needs. Its
contract was pinned by `timeout_tests.rs` *before* the deletion and is unchanged after it.

The `tokio` feature on `conflux-core` went too, since it gated nothing else; `conflux-ros2` no
longer requests it. The `staleness:` block was dropped from `conflux_node`'s YAML schema — a
leftover key in an old config is ignored, and `config/example.yaml` records why.

Note that M-21 was the decisive one: it is the reason the answer was "remove" rather than "repair".
The retained message-time path is, by construction, on the single correct clock.
