# M-16 · LiDAR-to-LiDAR pipeline has never been run end-to-end

- **Severity:** Medium
- **Area:** lidar_to_lidar_solver / pipeline
- **Status:** Open
- **Verified:** By admission — CLAUDE.md and the L2L section both say "This pipeline is not yet tested"
- **Related:** [M-04](./archive/M-04-l2l-wallclock-staleness.md), [M-05](./archive/M-05-l2l-wrong-pose-field.md)

## Problem

`lidar_to_lidar_solver` replaced the deprecated `multi_wayside_node`, and the config-driven
launch generates it for every lidar-lidar pair — but nobody has ever run the two-LiDAR
pipeline end-to-end. Two real bugs were already found in it by static review alone
(M-04 wall-clock staleness, fixed; M-05 wrong pose field, closed as by-design after deeper
reading), which is strong evidence the untested remainder hides more.

The ingredients exist: `just two-lidar` launch recipe, `lctk_sample_data` dataset 3 + 4
(two VLP-32C pcaps), `two_lidar.launch.xml` playback, and a `config/examples/two_lidar.yaml`.

## Suggested fix

Run `just sample-data` (two-lidar variant) + `just two-lidar` on datasets 3/4, capture the
solved transform, and sanity-check it (the two lidars observed the same board; the transform
should reproduce the board correspondence within sensor noise). File whatever breaks; then
delete the "not yet tested" disclaimers from CLAUDE.md and the book.

Needs a human eye on RViz for the final geometric sanity check, so this is operator work,
not headless work.
