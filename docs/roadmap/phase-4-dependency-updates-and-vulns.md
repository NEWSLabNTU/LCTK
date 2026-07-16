# Phase 4: Dependency Updates & Vulnerability Remediation

## Overview

This document outlines the plan for bringing LCTK's dependencies up to date and
clearing the security vulnerabilities reported by GitHub Dependabot. As of
2026-07-11, Dependabot reports **23 open alerts** against the default branch
(8 high, 9 moderate, 6 low), all in the Rust dependency tree.

The work is complicated by the fact that the workspace's ROS message crates are
**not** real crates.io dependencies — they are generated at build time — which
blocks a plain `cargo update` from a bare shell. This document explains the
constraint and the remediation procedure that works around it.

## Problem Statement

### The vulnerabilities

Every open alert is a Rust crate. Only `rand` is a direct workspace dependency;
the rest are transitive (pulled in through `rclrs`, `quinn`, `tokio`, and the
arrow/datafusion chain).

**High severity** — all fixable with semver-compatible (in-range) version bumps:

| Crate | Locked | Fixed in | Advisories |
|-------|--------|----------|------------|
| `openssl` | 0.10.75 | 0.10.80 | GHSA-xp3w-r5p5-63rr, GHSA-pqf5-4pqq-29f5, GHSA-8c75-8mhr-p7r9, GHSA-hppc-g8h3-xhp3, GHSA-ghm9-cr32-g9qj |
| `rustls-webpki` | 0.103.9 | 0.103.13 | GHSA-82j2-j2ch-gfr8 |
| `lz4_flex` | 0.11.5 | 0.11.6 | GHSA-vvp9-7p8x-rfvv |
| `quinn-proto` | 0.11.13 | 0.11.14 | GHSA-6xvm-j4wr-6v98 |

**Moderate severity:**

| Crate | Locked | Fixed in | Notes |
|-------|--------|----------|-------|
| `openssl` | 0.10.75 | 0.10.80 | GHSA-phqj-4mhp-q6mq, GHSA-xv59-967r-8726 (same bump as above) |
| `tar` | 0.4.44 | 0.4.46 | GHSA-3pv8-6f4r-ffg2, GHSA-gchp-q4r4-x4ff, GHSA-j4xf-2g29-59ph |
| `rustls-webpki` | 0.103.9 | 0.103.13 | GHSA-pwjx-qhcg-rvj4 |
| `time` | 0.3.45 | 0.3.47 | GHSA-r6v5-fh4h-64xc |
| `bytes` | 1.11.0 | 1.11.1 | GHSA-434x-w66g-qw3r |
| `thrift` | 0.17.0 | > 0.22.0 | GHSA-2f9f-gq7v-9h6m — **major bump, transitive** |

**Low severity:**

| Crate | Locked | Fixed in | Notes |
|-------|--------|----------|-------|
| `rand` | 0.8.5 | 0.8.6 | GHSA-cq8v-f236-94qc — **direct dependency** |
| `rand` | 0.9.2 | 0.9.3 | GHSA-cq8v-f236-94qc (second copy in tree) |
| `rustls-webpki` | 0.103.9 | 0.103.13 | GHSA-xgp8-3hg3-c2mh, GHSA-965h-392x-2mh5 |
| `lru` | 0.13.0 | 0.16.3 | GHSA-rhfx-m35p-ff5j — **major bump, transitive** |

### The blocker: ROS message crates are generated, not published

The root `Cargo.toml` declares the ROS interface crates with wildcard versions:

```toml
geometry_msgs = "*"
sensor_msgs = "*"
std_msgs = "*"
vision_msgs = "*"
visualization_msgs = "*"
builtin_interfaces = "*"
tf2_msgs = "*"
```

On crates.io, `sensor_msgs` is only a `0.0.0` placeholder. The real versions
(`sensor_msgs 4.2.4`, etc.) are produced by `rosidl_cargo` during the colcon
build at `build/<pkg>/rosidl_cargo/` and injected via a path/registry override
that is only active inside the `just build` environment (related to the
`.cargo/config.toml` note in CLAUDE.md's Known Issues).

Consequently, running `cargo update` (even targeted, `-p <crate> --precise …`)
from a **bare shell** re-resolves the wildcard message requirements against
crates.io, finds the yanked `sensor_msgs 4.2.3`, and aborts:

```
error: failed to select a version for the requirement `sensor_msgs = "*"`
  version 4.2.3 is yanked
```

Pinning the wildcards to `"4"` does not help — the maximum published crates.io
version is the yanked one. **Dependency updates must therefore be run inside the
colcon build environment**, where the generated message crates resolve via their
path override.

## Goals

1. Clear all 8 high-severity alerts (and the moderates that share the same bump).
2. Update remaining transitive vulns where an in-range fix exists.
3. Address the two major-version transitive vulns (`thrift`, `lru`) or document
   why they cannot be moved yet.
4. Make dependency updates reproducible — a documented command sequence and,
   ideally, CI/Dependabot that works with the generated-message constraint.

## Non-Goals

- Upgrading `rclrs` off 0.7 or changing the ROS distro.
- Replacing the `colcon-cargo-ros2` build flow.

## Implementation Plan

### Step 1 — Update the lockfile inside the build environment

Run from the project root **after sourcing the workspace** so the generated
message crates are resolvable:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # requires a prior `just build`

# High + moderate severity (in-range, safe)
cargo update -p openssl --precise 0.10.80
cargo update -p rustls-webpki --precise 0.103.13
cargo update -p lz4_flex --precise 0.11.6
cargo update -p quinn-proto --precise 0.11.14
cargo update -p tar --precise 0.4.46
cargo update -p time --precise 0.3.47
cargo update -p bytes --precise 1.11.1

# Low severity
cargo update -p rand@0.8.5 --precise 0.8.6
cargo update -p rand@0.9.2 --precise 0.9.3
```

If a `--precise` bump is rejected because a parent constraint caps the version,
record it and move it to Step 3.

### Step 2 — Rebuild and test

```bash
just build
just test
```

Confirm the workspace still builds under `--profile=test-release` and the Rust +
Python test suites pass. Pay attention to `openssl` (native TLS linkage) and
`quinn-proto` / `rustls-webpki` (QUIC/TLS) since those touch networking.

### Step 3 — Handle the major-version transitive vulns

`thrift` (0.17 → > 0.22) and `lru` (0.13 → 0.16.3) require major bumps and are
pulled transitively (arrow / datafusion chain). Options, in order of preference:

1. Update the direct parent crate to a release whose lockfile already uses the
   fixed `thrift` / `lru`.
2. If no parent release exists yet, leave the alert open with a tracking note and
   re-check on the next parent release.
3. As a last resort, a `[patch]` override — only if the fixed version is
   API-compatible for how the parent uses it.

### Step 4 — Prevent regression

- Add a `cargo audit` (or `cargo deny advisories`) check to CI, run inside the
  build environment so message crates resolve. This surfaces new advisories at PR
  time instead of via Dependabot after merge.
- Verify Dependabot can open PRs against this repo despite the generated-message
  wildcards; if it cannot resolve, document the manual procedure above as the
  canonical path and disable noisy auto-PRs.
- Consider pinning the ROS message crates to a concrete generated version in the
  lockfile-friendly way (or documenting why the `"*"` wildcards are required by
  `colcon-cargo-ros2`) so future `cargo update` runs are less fragile.

## Verification / Success Criteria

- [ ] `cargo update` sequence in Step 1 completes inside the build env.
- [ ] Dependabot high-severity count drops from 8 to 0.
- [ ] Moderate/low counts reduced to only the documented major-bump blockers.
- [ ] `just build` and `just test` pass after the bump.
- [ ] `cargo audit` runs clean (or only the tracked blockers remain).
- [ ] CI advisory check added.

## Risks

- **Build-env coupling:** the update procedure only works with a sourced
  workspace; a contributor running a bare `cargo update` will hit the yanked-crate
  error. Mitigated by documenting it here and in CONTRIBUTING.
- **TLS/native linkage:** `openssl` bumps can change system-library expectations;
  validate on the target Ubuntu 22.04 / Humble environment.
- **Transitive caps:** parent crates may pin vulnerable versions, blocking Step 1
  entries and pushing them to Step 3.

## References

- Audit findings index: [`docs/issues/README.md`](../issues/README.md)
- GitHub Dependabot: <https://github.com/NEWSLabNTU/LCTK/security/dependabot>
- CLAUDE.md → Known Issues (the `.cargo/config.toml` conflict is the same
  generated-message override mechanism referenced above).

## Execution Log (2026-07-16)

**Step 1 — done.** All in-range bumps applied inside the sourced build env exactly as
documented (openssl 0.10.80, rustls-webpki 0.103.13, lz4_flex 0.11.6, tar 0.4.46,
time 0.3.47, bytes 1.11.1, rand 0.8.6/0.9.3). Two advisories had moved since this doc
was drafted and were bumped further: quinn-proto → 0.11.15 (RUSTSEC-2026-0185) and
crossbeam-epoch → 0.9.20 (RUSTSEC-2026-0204). No `--precise` bump was rejected.

**Step 3 — resolved and blocked halves.**
- `lru`: fixed by bumping the parent — `rerun` 0.23.4 → 0.34.1 (dev-dependency of
  board-fitter, one example; compiled against 0.34 with zero code changes). lru now 0.18.1.
- `thrift` 0.17: still pinned by parquet even at parquet 58.3 (latest); upstream blocker,
  option 2 (tracking note, re-check on parquet releases).
- `quick-xml` <0.41 (RUSTSEC-2026-0194/0195, new since drafting): three parents inside
  rerun's tree (urdf-rs, minidom, wayland-scanner) all cap below the fix. Dev-dependency
  exposure only. Tracked as ignores in `.cargo/audit.toml` with justification.

**Step 4 — partially done.** `cargo audit` installed (0.22.2); new `just audit` recipe
runs it in the sourced build env; `.cargo/audit.toml` holds the two tracked quick-xml
ignores. `just audit` is green (8 informational warnings: unmaintained/unsound/yanked,
no fixes published). No CI pipeline exists in this repo yet, so the CI hook part waits
for CI to exist; Dependabot verification requires the GitHub UI.

**Step 2 — `just build` + `just test` pass after the bumps** (see commit).

### Success criteria status

- [x] `cargo update` sequence completes inside the build env
- [ ] Dependabot high-severity count drops to 0 — verify in the GitHub UI after push
- [x] Moderate/low reduced to documented major-bump blockers (thrift, quick-xml)
- [x] `just build` and `just test` pass after the bump
- [x] `cargo audit` runs clean apart from tracked blockers (`just audit`)
- [ ] CI advisory check — blocked on the repo having CI at all
