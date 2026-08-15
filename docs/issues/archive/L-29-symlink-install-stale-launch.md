# L-29 · Deleting a launch file leaves a dangling symlink that breaks the next build

- **Severity:** Low
- **Area:** build tooling
- **Status:** Fixed (2026-08-16)
- **Verified:** Hit on a clean `main` checkout while building for M-12 (2026-08-15)
- **Location:** `build/lctk_launch/launch/` (artifact), `just build`

## Problem

`just build` uses `colcon build --symlink-install`, which symlinks package data files into
`build/` instead of copying them. When a launch file is later deleted from source, the symlink in
`build/` is left behind pointing at nothing, and the next build fails:

```
--- stderr: lctk_launch
error: can't copy '/home/aeon/repos/LCTK/build/lctk_launch/launch/lidar_camera_calibration.launch.xml':
       doesn't exist or not a regular file
---
Failed   <<< lctk_launch [0.70s, exited with code 1]
```

The message points at a path that `ls` *shows* as present, which is the confusing part: it is a
dangling symlink, so it exists as a directory entry but not as a regular file.

Six such stale entries were present — `extrinsic_calibration`, `image_processing`,
`lidar_camera_calibration`, `lidar_camera_demo`, `pointcloud_processing` and
`two_lidar_calibration` — all removed from source at some point, all still symlinked in `build/`.

## Failure scenario

A `git pull` that removes a launch file breaks the next build with an error naming a file the
developer never touched. It is recoverable in seconds once understood, but the message does not
suggest the cause, and this is the same family as the archived
[L-16](./L-16-bindgen-lock-stale-skip.md) (stale `bindgen.lock` skipping regeneration) and
[L-15](./L-15-build-dirties-worktree.md).

Workaround, and what was done here:

```bash
rm -rf build/lctk_launch install/lctk_launch && just build
```

## Suggested fix

- Have `just build` prune dangling symlinks under `build/*/` before invoking colcon — cheap, and
  it addresses the whole class rather than this one package.
- Or document it in CLAUDE.md's Known Issues alongside the other stale-artifact traps, so the
  error message is one search away from its cause.

## Resolution (2026-08-16)

`just build` now prunes broken symlinks from **both** `build/` and `install/` before invoking
colcon, and reports what it removed:

```
removed 1 dangling symlink(s) under build/ (L-29)
removed 3 dangling symlink(s) under install/ (L-29)
```

Pruning both trees was not over-caution: the first run found 1 stale link in `build/` and 3 in
`install/`, so `install/` had drifted as well.

There is nothing to weigh in this fix — a broken symlink is never useful, and colcon recreates the
ones that should exist. The guard sits next to the existing L-16 `bindgen.lock` check, which is
the same shape of problem: build state that survives a source change and then lies about it.

**Verified by reproducing the failure, not by reasoning about it.** Add a launch file, build
(symlink appears), delete the source, build again. Before the guard that second build fails with
the reported error; after it, the build prunes one link from each tree and succeeds.

CLAUDE.md gained a Known Issues entry (item 8) explaining why the error names a path that `ls`
still shows, plus the manual `find build install -xtype l -delete` for checkouts predating the
guard.
