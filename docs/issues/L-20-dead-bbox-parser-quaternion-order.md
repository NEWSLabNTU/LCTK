# L-20 · Dead `BBox` JSON5 parser still reads the quaternion w-first

- **Severity:** Low
- **Area:** `ros/lidar_board_detector` / config parsing
- **Status:** Open
- **Verified:** 2026-08-13 — read from `bbox.rs:133-208`; caller search across `ros/` and `rust/`
- **Related:** [M-15 (archived)](./archive/M-15-bbox-quaternion-order-comment.md)

## Problem

`ros/lidar_board_detector/src/bbox.rs` contains **two** ways to parse a bbox JSON5 file, and they
disagree about quaternion component order.

**The live path is correct.** `load_bbox_config` (`main.rs:775-782`) calls
`load_json5_file::<BBox>`, which uses the derived `Deserialize` on
`BBox { pose: na::Isometry3<f64>, .. }` (`bbox.rs:7-11`). nalgebra serialises a `UnitQuaternion` as
`[i, j, k, w]`, i.e. **w-last** — matching how every config file is authored. This is what
[M-15](./archive/M-15-bbox-quaternion-order-comment.md) concluded, and it remains accurate.

**The dead path is wrong.** `BBox::from_json5_string` (`bbox.rs:133-181`) parses into a local struct
whose comment says `// [w, x, y, z]` and then destructures accordingly:

```rust
let [w, x, y, z] = bbox_json.pose.rotation;
let normalized_quat = na::UnitQuaternion::new_normalize(na::Quaternion::new(w, x, y, z));
```

Fed `bbox.json5`'s `[-0.0347, -0.1045, 0.0036, 0.9939]` — authored w-last, and reconciling with that
file's own `// euler: (-4,-12,0)` comment only under w-last — this yields a near-180° rotation instead
of the intended −12° pitch.

Its only caller is `BBox::load_from_file` (`bbox.rs:208`), which **has no callers anywhere** in `ros/`
or `rust/`. So the bug is currently unreachable.

## Secondary: two config files still carry the old ordering

`bbox_2_lidar_seyond.json5:7-11` and `bbox_2_lidar_vlp32.json5:7-11` both specify
`rotation: [1, 0, 0, 0]`. Read w-last (as the live path does) that is a **180° roll about X**, not the
identity those files intend.

It has never been noticed because it is genuinely invisible: a 180° X-roll maps `y → −y, z → −z`, and
the crop box is symmetric about its centre in both, so the selected region is identical. The
correct w-last identity is `[0, 0, 0, 1]`.

## Suggested fix

1. Delete `from_json5_string` and `load_from_file` (~50 lines, zero callers). Leaving a function that
   reads the quaternion backwards next to one that reads it correctly is a trap for the next person
   who needs to parse a bbox.
2. Change `bbox_2_lidar_{seyond,vlp32}.json5` to `[0, 0, 0, 1]` so the shipped files stop teaching the
   wrong order.
3. Make sure every bbox file's comment states `(x, y, z, w)`.

## Notes

Found while planning the board-frame change. An earlier reading of this code claimed M-15 had
regressed and that the live crop box was being rotated by ~176°; that was wrong — the live path goes
through nalgebra's serde and is correct. Only the dead parser carries the defect.
