# M-15 · `bbox.json5` documents the quaternion in the wrong order

- **Severity:** Medium
- **Area:** lctk_launch config → lidar_board_detector
- **Status:** Fixed (2026-07-13)
- **Verified:** Yes (confirmed against the running node, 2026-07-13)
- **Location:**
  - `ros/lctk_launch/config/board/bbox.json5:7-8` (the wrong comment)
  - `ros/lidar_board_detector/src/main.rs:270-282` (the log that exposes it)

## Problem

The shipped config says:

```json5
// Rotation as quaternion (w, x, y, z)
"rotation": [1.0, 0.0, 0.0, 0.0]
```

Read as `(w, x, y, z)`, that is the identity. But the node logs its parsed parameters in `(w, x, y,
z)` order (`main.rs:275-278` prints `rotation_w, rotation_x, rotation_y, rotation_z`), and at
runtime it prints:

```
[INFO] [lidar_board_detector]: BBox parameters: center=(2.600, 0.000, 0.350),
                               rotation=(0.000, 1.000, 0.000, 0.000), size=(3.1, 3.9, 2.2)
```

**`w = 0.000, x = 1.000`.** The array was consumed as `(x, y, z, w)` — w **last** — which is
nalgebra's serde representation for a quaternion. So the on-disk value is a **180° rotation about
X**, not the identity, and the comment describes an ordering the code does not use.

It is masked in this particular file only because the bounding box is symmetric in y and z about
its own centre, so a 180° X-rotation maps the box onto itself and the filtered volume is unchanged.

## Failure scenario

The mask disappears the moment anyone writes a *rotated* bbox — and the real field captures do
exactly that. `data/2022-10-14-otobrite-calibration/1/bbox.json5` carries
`rotation: [0.0, 0.0, -0.3256, 0.9455]`, which is only sensible as `(x, y, z, w)`: a −38° yaw. Read
through the shipped comment as `(w, x, y, z)`, that would be a 138° rotation about a mostly-Z axis —
a completely different box.

So anyone who trusts the comment and hand-writes a rotated bbox gets a filter volume pointing
somewhere unintended, the board falls outside it, and the detector emits empty detections — which,
per [C-04](./C-04-board-detector-gate-unreachable.md), it does **silently**. The trap is set
precisely for the person doing the thing the tool exists for: tuning the box to their scene.

## Suggested fix

Correct the comment to `(x, y, z, w)` in `bbox.json5` and in any other config that repeats it, and
say explicitly that this is nalgebra's serde order.

Better: make it unambiguous rather than merely documented. Accept a named form —
`{"x": …, "y": …, "z": …, "w": …}` — or express the rotation as `yaw_deg`, which is all any real
config has ever used. A bare 4-element array whose ordering convention is carried only in a comment
is a defect waiting to recur; it already silently disagrees with itself once in this repo.

## Resolution (2026-07-13)

Corrected the comment in `config/board/bbox.json5` to state the quaternion is
`(x, y, z, w)` — nalgebra's serde order, w **last** — and fixed the value: the
previous `[1, 0, 0, 0]` is a 180° rotation about X in that order (only masked by the
box's y/z symmetry), so it is now the true identity `[0, 0, 0, 1]`. The filtered
volume is unchanged for the shipped symmetric box, and the config no longer disagrees
with itself. `bbox.json5` was the only config carrying the wrong comment.

**Better (not done):** accept a named form (`{"x":…,"y":…,"z":…,"w":…}`) or a `yaw_deg`
scalar so the ordering can't be misread — a parser change to `lidar_board_detector`
left as a follow-up.
