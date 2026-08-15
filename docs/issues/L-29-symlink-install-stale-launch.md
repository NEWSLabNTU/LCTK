# L-29 · Deleting a launch file leaves a dangling symlink that breaks the next build

- **Severity:** Low
- **Area:** build tooling
- **Status:** Open
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
[L-16](./archive/L-16-bindgen-lock-stale-skip.md) (stale `bindgen.lock` skipping regeneration) and
[L-15](./archive/L-15-build-dirties-worktree.md).

Workaround, and what was done here:

```bash
rm -rf build/lctk_launch install/lctk_launch && just build
```

## Suggested fix

- Have `just build` prune dangling symlinks under `build/*/` before invoking colcon — cheap, and
  it addresses the whole class rather than this one package.
- Or document it in CLAUDE.md's Known Issues alongside the other stale-artifact traps, so the
  error message is one search away from its cause.
