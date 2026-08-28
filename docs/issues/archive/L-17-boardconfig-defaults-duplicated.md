# L-17 · `BoardConfig` defaults defined twice — serde fns and `production_config` will drift

- **Severity:** Low
- **Area:** board-cluster-detector / config
- **Status:** Fixed (2026-08-28) — see Resolution below
- **Verified:** By code review (2026-08-11, standards axis)

## Problem

Every tunable in `BoardConfig` states its default **twice**: once in a `d_*()` serde default
function, and again as a literal inside `production_config()`. For example `d_cluster_eps()`
returns `0.15` while `production_config` writes `cluster_eps: 0.15`; the same pattern repeats for
all eleven knobs exposed in commit `2a4fd49` (`strip_plane_dist`, `strip_plane_min_frac`,
`merge_seed_min_points`, `merge_offset_tol`, `merge_dist_factor`, `patch_min_points`,
`patch_extent_lo_frac`, `patch_extent_hi_diag_frac`, `isolation_coplanar_tol`,
`isolation_band_lo`, `isolation_band_hi`).

Two sources of truth for one value. Changing a default in one place and not the other produces a
silent divergence between "config omitted the key" and "caller used the production preset", which
is exactly the class of mismatch that is painful to debug from a detection-rate symptom.

Note that `production_config` deliberately differs from the serde defaults for a few fields
(`flatness_rms_max`, `stance_floor`, `isolation`) — those are intentional overrides of the frozen
library defaults and must stay literal.

## Suggested fix

Have `production_config()` call the `d_*()` functions for every field where the two currently agree,
leaving explicit literals only where the preset intentionally departs from the library default (and
comment those, as the config file already does). A test asserting
`serde-defaulted BoardConfig == production_config(...)` on the agreeing subset would lock it in.

## Resolution (2026-08-28)

Both halves of the suggested fix are done, in the successor to `BoardConfig`/`production_config()`.

`rust/board-cluster-detector/src/config.rs`'s `production_tuning()` (the `DetectorTuning`-returning
replacement for the old `production_config()`) now calls the `d_*()` serde-default functions
directly for every field, and writes an explicit literal only for the three fields the issue itself
called out as intentional overrides:

```rust
pub fn production_tuning(up_axis: [f64; 3], cluster_min_points: usize) -> DetectorTuning {
    DetectorTuning {
        cluster_eps: d_cluster_eps(),
        side_tol: d_side_tol(),
        ...
        flatness_rms_max: 0.045, // production override of serde default
        stance_floor: 0.9,       // production override of serde default
        isolation: true,         // production override of serde default
        ...
    }
}
```

`rust/board-cluster-detector/tests/config.rs::production_tuning_reuses_serde_defaults_except_documented_overrides`
is exactly the lock-in test the suggested fix asked for: it builds a `DetectorTuning` from `{}` (pure
serde defaults) and asserts `production == defaults` field-by-field for every knob except
`flatness_rms_max`, `stance_floor`, and `isolation`, which it asserts against their documented
literal values instead. A future edit that reintroduces a duplicated literal for any other field
would fail this test.

This landed in two steps, both ahead of and including this phase: `e21aa01` (2026-08-23,
"refactor(clustering): expose neutral target evidence") introduced `production_tuning()` calling
`d_*()` directly, while the deprecated `BoardConfig`/`production_config()` pair the issue was
literally about — the one still writing literals — survived alongside it for migration. W5-E2
(`21142ac`, this phase, "refactor(rust): delete the hollow-board facade crates") deleted that
deprecated pair outright, so as of `21142ac` there is exactly one source of truth for these defaults,
not two. Verified by reading the current `config.rs` and `tests/config.rs` on this branch.

Closing 🟢 and archiving.
