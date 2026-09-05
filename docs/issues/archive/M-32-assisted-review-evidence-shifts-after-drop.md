# M-32: Assisted review evidence changed owners after a drop

**Status:** Fixed in assisted-review chunk 1.

The review API used a capture's current list index as its image-cache key. Removing
index 0 shifted later captures down, but their JPEG cache entries kept the old indexes.
A surviving capture could therefore display another capture's image. Clearing or
restoring a buffer also reused indexes while evidence remained cached.

## Resolution

Prepared captures now carry process-local, monotonically allocated IDs. Snapshots
preserve those IDs across removals, and restored or replacement captures receive new
IDs. The review facade uses these IDs and refuses evidence for IDs outside its current
buffer. Removed evidence is pruned; an in-flight image cannot restore a dropped capture.
Manual services continue to use their documented positional indexes, and archive schema
and solve inputs remain unchanged.

## Verification

The detection-buffer contract test covers removal, clear, restoration and buffer
replacement. The assisted-review facade test checks that dropping an earlier capture
preserves the survivor's ID and preview and refuses the removed ID.

See the [implementation phase](../../roadmap/phase-10-assisted-review-3d-frontend.md)
and [approved design](../../superpowers/specs/2026-09-05-assisted-review-3d-frontend-design.md).
