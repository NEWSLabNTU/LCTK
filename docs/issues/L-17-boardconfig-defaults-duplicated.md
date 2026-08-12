# L-17 · `BoardConfig` defaults defined twice — serde fns and `production_config` will drift

- **Severity:** Low
- **Area:** board-cluster-detector / config
- **Status:** Open
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
