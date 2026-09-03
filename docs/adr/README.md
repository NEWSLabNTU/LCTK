# Architecture Decision Records

An ADR records **one decision** about the shape of this codebase: what was decided, why, and what it
rules out. It exists so that a later reader — human or agent — does not re-open a settled question
without knowing what closed it.

An ADR is not a design doc and not a review. Those live elsewhere:

| Kind | Home | Answers |
|---|---|---|
| Decision record (ADR) | `docs/adr/NNNN-slug.md` | why the code is shaped this way |
| Design doc | `docs/superpowers/specs/` | how a planned change will work |
| Phase doc | `docs/roadmap/` | what a larger remediation contains |
| Finding / bug | `docs/issues/` | what is wrong today |
| Architecture review | `docs/adr/YYYY-MM-DD-architecture-review.md` | where the friction is, before any decision |

## Format

```markdown
# NNNN. <the decision, as a sentence>

- **Date:** YYYY-MM-DD
- **Status:** proposed | accepted | superseded by ADR-NNNN

## Context
What forced the decision. Prefer measurements and file references over adjectives.

## Decision
What we are doing, stated so that a reader can tell whether a given change obeys it.

## Consequences
What this makes easy, what it makes hard, and what it deliberately rules out.
```

Numbering is sequential from `0001`. A decision that replaces another marks the old one
**superseded** rather than deleting it — the reasoning that was true then is the reason the change
was needed.

## Vocabulary

Architecture discussion in these records uses one fixed vocabulary, so that the same idea is not
argued twice under two names:

- **Module** — anything with an interface and an implementation, at any scale.
- **Interface** — everything a caller must know to use it correctly: signature, invariants, ordering,
  error modes, configuration.
- **Depth** — behaviour available per unit of interface a caller must learn. **Deep**: a lot behind a
  little. **Shallow**: the interface is nearly as large as the implementation.
- **Seam** — the place where a module's interface lives; where behaviour can be changed without
  editing in that place.
- **Adapter** — a concrete thing filling a slot at a seam.
- **Leverage** — what callers gain from depth. **Locality** — what maintainers gain: change, bugs and
  verification concentrate in one place.

Two tests recur: the **deletion test** (if this module vanished, would complexity concentrate or just
scatter?) and **the interface is the test surface** (if a test has to reach past the interface, the
module is the wrong shape).

## Index

| ADR | Decision | Status |
|---|---|---|
| [0001](./0001-detection-pair-source.md) | The synchronized detection pair is one module, and it refuses an infinite window | accepted |
| [0002](./0002-detection-buffer.md) | The detection buffer owns every estimate derived from its captures | accepted |
| [0003](./0003-single-source-target-definition.md) | Calibration targets share one authoritative definition, bound once per launch | accepted |
| [0004](./0004-lidar-camera-solver-owns-camera-board-pose.md) | The LiDAR-to-camera solver owns camera-frame board pose | accepted |

Reviews:

- [2026-08-15 architecture review](./2026-08-15-architecture-review.md) — five deepening candidates
  across the detector and solver nodes.
