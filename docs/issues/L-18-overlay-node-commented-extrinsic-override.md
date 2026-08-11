# L-18 · Overlay node ships commented-out hardcoded extrinsic overrides

- **Severity:** Low
- **Area:** pointcloud_image_overlay
- **Status:** Open
- **Verified:** By code review (2026-08-11, standards axis)

## Problem

`overlay_node.py`'s extrinsic callback carries two commented-out assignments that overwrite the
received `extrinsic_rvec` / `extrinsic_tvec` with hardcoded values, plus a trailing-whitespace
blank line. They arrived via the `2026golf` merge.

Commented-out debug overrides in a callback that consumes a solved calibration are a hazard: the
next person debugging an overlay mismatch may uncomment them, and a hardcoded extrinsic silently
produces a plausible-looking overlay that has nothing to do with the actual solve. This is the same
failure shape as [M-03](./archive/M-03-hardcoded-plane-normal-x.md) (hardcoded plane-normal flip).

Related, lower still: the node's `min_depth` / `max_depth` parameters are declared with hardcoded
defaults (`0.0` / `20.0`), mildly against CLAUDE.md's "All nodes require explicit config file
parameters (no hardcoded defaults)" — though consistent with this node's existing
`declare_parameter` style, so it is not obviously worth changing on its own.

## Suggested fix

Delete the commented-out override block and the stray whitespace. If a fixed extrinsic is genuinely
useful for debugging, make it an explicit, named, off-by-default parameter rather than commented
code — so that using it is a visible choice that shows up in the node's parameter dump.
