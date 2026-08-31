# M-29 · The shipped sample-data path was dead: a shared crop box and an ICP gate under the noise floor

- **Severity:** Medium
- **Area:** lctk_launch / config/board, calibration-target-detector
- **Status:** 🟢 Fixed (2026-08-31)
- **Verified:** By running `just sample-data` + `calibrate.launch.py` on dataset 3 and reading the detector's own log
- **Related:** [C-04 (archived)](./archive/C-04-board-detector-gate-unreachable.md), [M-27](./M-27-solid-600-handheld-topics-alias-sample-data.md)

## Problem

`config/examples/sample_data.yaml` — the example behind `just demo` and the one maintained
config still in `bbox` mode — produced **zero board detections**. The camera side was fine
(937 ArUco detections, 599 synchronized groups, nothing rejected or dropped), so the
synchronizer looked healthy while the pipeline produced nothing. Two independent causes,
stacked:

**1. A shared crop box retuned for a different rig.** `config/board/bbox.json5` was moved to
`translation [-1.04, -1.5, 7.0]`, `size [2, 4, 10]` by `b4fea62` ("update configuration for
new rosbags") for a Seyond rosbag capture. The sample data's board is nowhere near there, so
the detector logged, on every single frame:

```
bbox: no board selected — only 0 finite points in the configured box
```

One file was serving two recordings with different rig geometry. It is also the crop box the
`board-detection-2d` experiments treat as the pcap reference (`_PCAP_BBOX`,
`benchmark_e_loo.DEFAULT_BBOX_PATH`), so those were reading a box for a recording they do not
use.

**2. An ICP gate below the sensor noise floor.** With the box corrected the detector found
~2080 correspondences per frame and then rejected every one of them:

```
target rejected: reason=perforated_icp_failure rim_correspondences=250
                 iterations=50 total_correspondences=2081 best_loss_m=0.022562
```

`icp_rejection_threshold` was `0.008`. The measured loss is `0.0220–0.0231`, and `CLAUDE.md`'s
own profiling section records `0.026–0.029` as the **VLP-32C noise floor, not a bad fit** — a
sensor spec'd at ±3 cm range accuracy cannot do better. The gate could never pass.

This is [C-04](./archive/C-04-board-detector-gate-unreachable.md) recurring on a different
threshold. C-04 was `icp_good_fit_threshold` at `0.012`; this is `icp_rejection_threshold` at
`0.008`, and the same sentence in `CLAUDE.md` predicted it: *"`icp_good_fit_threshold` must sit
above this; it was once set to 0.012 and the detector then silently accepted nothing."* The
value was inherited unchanged from `main`, where the older detector did not apply it this way;
the perforated-surface path introduced by the selectable-target work does.

## Why nothing caught it

Both failures are silent by construction. The detector publishes a well-formed
`Detection3DArray` with `detections: []`, the synchronizer reports perfect statistics, and the
solver reports an empty buffer. Every component is behaving correctly and reporting success;
only the composition is dead. No test covers the shipped sample data end to end — the suites
are unit-level, and the one thing that would have caught this is running `just demo`, which
nothing automates.

## Resolution

- **`config/board/sample_data_bbox.json5`** is new, carrying the box the sample data actually
  needs (`[2.6, 0.0, 0.35]`, `[3.1, 3.94, 2.2]`), and `sample_data.yaml` points at it.
  `bbox.json5` is left alone for the rosbag workflow that retuned it — two recordings with
  different rig geometry no longer share one crop box.
- **`icp_rejection_threshold` raised to `0.035`** in `hollow_1000/velodyne_bbox.json5`, with a
  comment naming the noise floor and the failure mode so the next person does not tighten it
  back.

Verified after the fix on dataset 3: **zero rejections**, assisted mode auto-captures a pair
and serves a real 1920×1080 preview, and `continuous` solves and publishes the extrinsic at
14.3 Hz with 1.26–1.50 px reprojection error.

## Still open

The `board-detection-2d` experiment scripts still name `config/board/bbox.json5` as the pcap
reference. They should point at `sample_data_bbox.json5`; nobody has re-run those benchmarks
to see what the wrong box did to their numbers.

More generally: **nothing runs the shipped sample data end to end.** That is what let two
silent failures stack up unnoticed, and it is the real gap here. A smoke check that plays
dataset 3 and asserts a non-zero board-detection count would have caught both on the commit
that introduced them.
