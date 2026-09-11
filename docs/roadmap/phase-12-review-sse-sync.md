# Phase 12: split assisted-review reads and SSE invalidation

Implements the amended [ADR 0008](../adr/0008-revisioned-assisted-review-session.md)
on branch `feat/review-sse-sync`.

## Objective

Keep the assisted-review page responsive without transferring or rebuilding the
detection list when only live stillness/synchronization status changes. The
browser receives a small revision hint over Server-Sent Events (SSE), then
fetches only the representation whose revision changed. A tiny revisions
endpoint is the recovery path when the stream is unavailable.

`GET /api/state` remains an atomic bootstrap and compatibility endpoint. It is
not the steady-state transport for the new page.

## Contract

The server exposes independently revisioned projections:

- `GET /api/captures`: pair list, solve/quality values, diversity gauges, and
  evidence availability. `captures_revision` covers all of these; the
  existing `capture_revision` keeps its Detection Buffer-only meaning.
- `GET /api/live`: stillness, synchronization, identity/error notices, and the
  confirmed parameter/export status used by the footer and advanced settings.
- `GET /api/scene`: world geometry and camera pose, as before.
- `GET /api/revisions`: only the complete vector
  `{session_epoch, live_revision, captures_revision, scene_revision}`.
- `GET /api/events`: `text/event-stream` carrying the same vector as an
  invalidation hint, never pair data, evidence bytes, or raw LiDAR. A client
  receives the current vector on connect/reconnect; pending notifications are
  coalesced to the newest value. Subscribers have bounded latest-value
  storage, keepalives, and disconnect cleanup.

Each projection has an independent ETag. A matching validator returns `304`.
The vector's `session_epoch` replaces every projection and asset cache on a
session restart; stale asynchronous responses are discarded.

## Ordered work

- [x] Publish the feature branch and this phase record.
- [x] Amend ADR 0008 with the split-read/SSE decision and retained constraints.
- [ ] Add read-model revisions, mutation notifications, and endpoint snapshots.
- [ ] Add the bounded SSE hub and `/api/live`, `/api/captures`, and
      `/api/revisions` routes.
- [ ] Replace the browser heartbeat with bootstrap + SSE, independent fetches,
      and `/api/revisions` fallback.
- [ ] Add regression tests for bandwidth/DOM isolation, reconnect/coalescing,
      epochs, evidence completion, and independent ETags.
- [ ] Run deliberate-failure test checks, `just build`, `just test`, and
      `just lint`; restore any generated `Cargo.lock` changes.
- [ ] Publish every verified checkpoint and record the command/results below.

## Progress log

| Date | Checkpoint | Result |
|------|------------|--------|
| 2026-09-11 | Baseline branch | Started from `origin/main` at `59ec995`; branch published. |
| 2026-09-11 | Design | Read-only Astra review confirmed split projections, SSE hints, epoch guards, and fallback contract. |

## Verification record

The previous implementation baseline was 777 tests passed and one skipped,
with `just build` and `just lint` successful. New verification entries will be
added here as each checkpoint lands. Browser visual verification remains an
operator check when the Firefox runtime is available; JavaScript tests cover
the transport and rendering contracts without requiring WebGL.
