# 0006. One manifest, one strictness: every section refuses a key nothing reads

- **Date:** 2026-09-04
- **Status:** accepted

## Context

A session manifest is parsed by two modules. `session.py` owns `data:` and has refused unknown keys
there since sessions existed. `config_parser.py` owns everything else — `devices:`, `markers:`,
`sync:`, the top level — and refused nothing, with one exception (`assisted:`, whose accepted set is
derived from a dataclass).

So one file had two rules for the same mistake. A misspelled `sync_tolernace_ms` was discarded in
silence, and the section was then reported as *missing* `tolerance_ms` — a message that is true and
useless, because the key is in the file, spelled wrong. A stray top-level `mode:` — the obvious
thing to try after [ADR-0005](./0005-session-owns-transport-reliability.md) deleted the argument —
parsed and did nothing.

This is the same failure shape the manifest exists to remove. Every refusal `session.py` performs
was added because the alternative was a graph that launches cleanly and produces nothing.

## Decision

`reject_unknown_keys` becomes public in `session.py` and is applied by `config_parser.py` at every
level: the top level, `devices`, each `devices.lidars.<name>`, each `devices.cameras.<name>`, each
`markers.<name>`, and `sync`. The accepted sets are module constants next to the parser.

**Retired keys are deliberately absent from those sets.** `board_config`, `type`, `aruco_config` and
`stability_window_frames` are checked first, by name, and raise an error saying what replaced them
and why there is no automatic migration. Listing them as accepted, or letting them fall through to
the generic "unknown key" message, would lose that.

**`name:` and `description:` stay accepted although nothing reads them.** Every shipped manifest
carries them and they are what an operator reads first. A section may be documentation without being
configuration.

## Consequences

**Easy.** A typo anywhere in the manifest fails at parse time naming the offender and listing what
the section accepts, so the error is its own fix. Adding a key — `qos:` was the immediate case — no
longer ships a new way to fail quietly.

**Hard.** Every new key must be added to its set as well as to the parser, or a valid manifest is
refused. The sets sit directly above the code that reads them so the omission is visible, and the
shipped sessions are parsed by `test_sessions_shipped.py`, which fails if one stops validating.

**Ruled out.** Deriving the accepted set by reflection over the parser, which is what `assisted:`
does through `fields(AssistedSettings)`. It works there because that section maps one-to-one onto a
dataclass. The rest do not: `devices.lidars.<name>` feeds three different fields and two overrides,
and reflection would silently accept whatever the implementation happened to look at.

**Not changed.** `assisted:` keeps its dataclass-derived set rather than being converted, because
the one-to-one mapping is real and the derivation cannot drift from it.
