# L-21 · `find_correspondences` duplicated; the inline tests exercise the copy the node never runs

- **Severity:** Low
- **Area:** `rust/hollow-board-config`
- **Status:** **Fixed** 2026-08-13 — see [Resolution](#resolution-2026-08-13)
- **Verified:** 2026-08-13 — read from `lib.rs:169-652`; feature flags from `hollow-board-detector/Cargo.toml:19`, `lidar_board_detector/Cargo.toml:15`
- **Related:** [M-19](../M-19-debug-assertions-compiled-out.md), [M-20](./M-20-board-frame-edge-aligned-vs-diamond-naming.md)

## Problem

`rust/hollow-board-config/src/lib.rs` defines `find_correspondences` **twice**, textually identical:

- `#[cfg(feature = "parallel")]` at `:169-412`
- `#[cfg(not(feature = "parallel"))]` at `:414-652`

Three defects follow.

**1. The outer `#[cfg]` split is redundant.** Each copy *already contains both inner arms* — the
parallel copy has a dead `#[cfg(not(feature = "parallel"))]` dispatch at `:408-409`, and the serial
copy the mirror at `:650-651`. The bodies differ only in `into_par_iter()` vs `into_iter()`; the other
~240 lines are duplicated verbatim.

**2. The tests run the wrong body.** Both consumers enable the feature —
`rust/hollow-board-detector/Cargo.toml:19` and `ros/lidar_board_detector/Cargo.toml:15` both declare
`features = ["parallel"]` — but `hollow-board-config` has no default features. So the crate's own
inline tests (`lib.rs:781-1194`) compile the **serial** body while production runs the **parallel**
one. They are separate functions that today merely happen to share text; nothing enforces that.

**3. `cargo check -p hollow-board-config` checks the wrong one too**, for the same reason, so
per-crate lint output about this function is unreliable. (Observed in practice: a bare per-crate check
reported a `use log::debug` import as unused in a sibling crate because the `parallel` block was
`cfg`-ed out; removing it broke the node build.)

## Suggested fix

Hoist a single body and keep the split only over the iterator:

```rust
struct PlateGeometry { /* axes, centre, radii, hole centres, precomputed once */ }

impl BoardModel {
    fn plate_geometry(&self) -> PlateGeometry { ... }
    #[inline]
    fn correspondence_of(&self, g: &PlateGeometry, p: &Point3<f64>) -> Point3<f64> { ... }
}
```

then two thin `find_correspondences` wrappers differing only in `into_par_iter()` / `into_iter()`.
Keep the bound lists verbatim and keep the (always-`Some`) `Option` return so
`hollow-board-detector/src/algo.rs:281` is untouched. ~240 duplicated lines → ~15, and the tests then
exercise the same code the node runs.

Also delete both dead inner `#[cfg]` arms while there.

## Notes

Found while planning the board-frame change, where this function's boundary test is being rewritten —
duplicating that rewrite across two copies, only one of which is tested, would be an obvious hazard.
Related: `test_multi_marker_corners_basic` (`lib.rs:661-761`) never calls the API it is named after; it
recomputes the expected corners inline and asserts against its own arithmetic, so it passes while
testing nothing.

## Resolution (2026-08-13)

Fixed as step 2 of the corner-aligned board-frame work
([M-20](./M-20-board-frame-edge-aligned-vs-diamond-naming.md), spec
[`2026-08-13-corner-aligned-board-frame.md`](../../superpowers/specs/2026-08-13-corner-aligned-board-frame.md)),
deliberately *before* the frame flip so the boundary-projection rewrite landed once against a
tested body.

`rust/hollow-board-config/src/lib.rs` now hoists the per-point search into
`CorrespondenceContext::corresponding_point`, with the board geometry that does not vary per point
precomputed once in `BoardModel::correspondence_context`. The two `find_correspondences` wrappers
survive only as `#[cfg]`-split shells differing in `into_par_iter()` vs `into_iter()`; both dead
inner `#[cfg]` arms are gone. Bound lists and the (always-`Some`) `Option` return are unchanged, so
`hollow-board-detector/src/algo.rs` was untouched.

Defects 1 and 2 are therefore closed: there is one body, and the crate's own tests exercise it.
Defect 3 — `cargo check -p hollow-board-config` still checks the serial wrapper, because the crate
has no default features — is unchanged, but it is now a two-line shell rather than ~240 lines of
logic.

The related note about `test_multi_marker_corners_basic` never calling the API it is named for is
also resolved: it is replaced by `tests/marker_layout_golden.rs`, which calls
`multi_marker_corners` and asserts world-coordinate positions against a checked-in fixture keyed by
ArUco marker id.
