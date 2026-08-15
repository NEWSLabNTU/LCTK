# M-21 · Expiry is defined in two incompatible time bases (wall clock vs message stamp)

- **Severity:** Medium
- **Area:** conflux-core
- **Status:** Open
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
