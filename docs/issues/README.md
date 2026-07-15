# LCTK Issue Tracker

Findings from the 2026-07-09 workflow + correctness audit (sensor → config → build → runtime → solver → Autoware export), plus the 2026-07-12 extrinsic-stability audit (C-03, H-07…H-09, M-11…M-14, L-10…L-12). One file per finding, ranked by severity.

Status legend: 🔴 open · 🟡 in progress · 🟢 fixed · ⚪ won't fix / by-design

Closed issues (🟢 fixed, ⚪ won't-fix/by-design) are archived under [`archive/`](./archive/); open (🔴) and in-progress (🟡) issues stay here.

| ID | Sev | Title | Status |
|----|-----|-------|--------|
| [C-01](./archive/C-01-aruco-corners-discarded.md) | Critical | ArUco marker corners discarded; PnP uses axis-aligned bbox | 🟢 |
| [C-02](./archive/C-02-conflux-realtime-memory-leak.md) | Critical | Conflux realtime mode leaks a message object per dropped message | 🟢 |
| [C-03](./archive/C-03-double-undistortion.md) | Critical | Image undistorted twice before ArUco detection → every corner biased | 🟢 |
| [C-04](./archive/C-04-board-detector-gate-unreachable.md) | Critical | ICP accept gate set below the sensor noise floor → detector silently accepts nothing | 🟢 |
| [H-01](./archive/H-01-conflux-not-built.md) | High | `conflux_py` never built → solvers ImportError at startup | 🟢 |
| [H-02](./archive/H-02-conflux-drops-first-message.md) | High | Conflux Python binding drops the first message (msg_id 0 → NULL) | 🟢 |
| [H-03](./archive/H-03-pointcloud-datatype-endian.md) | High | Point cloud XYZ decoded as LE float32 without checking datatype/endianness | 🟢 |
| [H-04](./archive/H-04-board-detector-mandatory-params.md) | High | Detector declares params mandatory that launch adds only "if present" | 🟢 |
| [H-05](./archive/H-05-conflux-error-stats-collapse.md) | High | Conflux FFI collapses all push errors to BufferFull → corrupt stats | 🟢 |
| [H-06](./archive/H-06-config-schema-drift.md) | High | CLAUDE.md documents a config schema the parser does not accept | 🟢 |
| [H-07](./archive/H-07-no-pose-diversity-gate.md) | High | Degenerate pose sets accepted silently; extrinsic under-constrained | 🟢 |
| [H-08](./archive/H-08-no-subpixel-corner-refinement.md) | High | ArUco corners never sub-pixel refined (`CORNER_REFINE_NONE`) | 🟢 |
| [H-09](./archive/H-09-no-extrinsic-quality-metric.md) | High | The extrinsic solution has no quality metric of any kind | 🟢 |
| [H-10](./archive/H-10-dump-load-regresses-c01.md) | High | dump→load drops ArUco corners → silently re-introduces C-01 | 🟢 |
| [M-01](./M-01-transform-direction-inverted.md) | Medium | Transform frame labels inverted vs ROS TF semantics | 🟡 |
| [M-02](./archive/M-02-radians-degrees-mix.md) | Medium | Advanced solver adjust/pose API mixes radians and degrees | ⚪ |
| [M-03](./archive/M-03-hardcoded-plane-normal-x.md) | Medium | Hardcoded plane-normal flip to +X assumes sensor-forward-X | 🟢 |
| [M-04](./archive/M-04-l2l-wallclock-staleness.md) | Medium | L2L staleness check uses wall-clock vs sensor stamp | 🟢 |
| [M-05](./archive/M-05-l2l-wrong-pose-field.md) | Medium | L2L solver reads board pose from wrong message field | ⚪ |
| [M-06](./archive/M-06-detector-thread-no-panic-guard.md) | Medium | Board-detector thread has no panic guard → silent dead node | 🟢 |
| [M-07](./archive/M-07-conflux-dropoldest-double-loss.md) | Medium | DropOldest can destroy a good message and reject the new one | 🟢 |
| [M-08](./archive/M-08-conflux-ffi-no-locking.md) | Medium | Mutable Rust conflux State crosses FFI with no locking | 🟢 |
| [M-09](./archive/M-09-marker-ids-hard-index.md) | Medium | `marker_ids[0..3]` hard index → IndexError on short config | 🟢 |
| [M-10](./archive/M-10-multi-marker-config-collisions.md) | Medium | Multi-marker camera uses wrong ArUco config; duplicate pairs collide | 🟢 |
| [M-11](./archive/M-11-solvers-ignore-distortion.md) | Medium | Solvers hardcode `dist_coeffs = 0`, never read `camera_info.d` | 🟢 |
| [M-12](./M-12-no-robust-estimation-or-refinement.md) | Medium | No outlier rejection and no LM refinement in the extrinsic solve | 🟡 |
| [M-13](./archive/M-13-icp-quality-not-propagated.md) | Medium | Board-pose uncertainty measured, then discarded before the solver | 🟢 |
| [M-14](./M-14-corner-order-brittle.md) | Medium | Board origin corner picked by gravity; corner order duplicated, unchecked | 🟡 |
| [M-15](./archive/M-15-bbox-quaternion-order-comment.md) | Medium | `bbox.json5` documents the quaternion `(w,x,y,z)`; the wire format is `(x,y,z,w)` | 🟢 |
| [L-01](./archive/L-01-fit-board-icp-false-success.md) | Low | Library `fit_board_icp` reports non-converged fits as successful | 🟢 |
| [L-02](./archive/L-02-rust-panics-empty-nan.md) | Low | Pure-Rust panics on empty / NaN point sets | 🟢 |
| [L-03](./archive/L-03-pnp-solver-panic-distortion.md) | Low | `pnp-solver` panics on failed solve, truncates distortion | 🟢 |
| [L-04](./archive/L-04-hardcoded-2x2-board.md) | Low | `multi_marker_corners` hardcodes 2×2; unknown config fields ignored | 🟢 |
| [L-05](./archive/L-05-mode-typo-static-mut.md) | Low | `mode` typo silently offline; `static mut` counters race | 🟢 |
| [L-06](./archive/L-06-pokemon-exceptions.md) | Low | Pervasive `except Exception: pass` against project guideline | 🟢 |
| [L-07](./archive/L-07-tf-broadcaster-qos.md) | Low | tf_tree_broadcaster QoS may be incompatible with realtime publishers | 🟢 |
| [L-08](./archive/L-08-stale-readme-docs.md) | Low | Stale README & docs misdirect new users | 🟢 |
| [L-09](./L-09-setup-fragility-export-labeling.md) | Low | Setup fragility, no export tooling, dump JSON mislabeled | 🟡 |
| [L-10](./archive/L-10-solver-float32-precision.md) | Low | PnP correspondences and intrinsics cast to `float32` | 🟢 |
| [L-11](./archive/L-11-detector-param-block-bugs.md) | Low | Detector param block sets a field twice; tunes a disabled refiner | 🟢 |
| [L-12](./archive/L-12-dead-solver-crates.md) | Low | Dead crates (`pnp-solver`, `calibration-quality`) better than the live code | 🟢 |
| [L-13](./archive/L-13-calibration-metrics-msg-dead.md) | Low | `CalibrationMetrics.msg` built, unused, and IoU-shaped rather than residual-shaped | 🟢 |

## Three headline gaps

1. **Every calibration is biased** — C-01. The detector throws away real ArUco corners.
2. **The build omits a dependency the solvers need** — H-01. `conflux` is never built.
3. **The Autoware last mile is fully manual and undocumented** — see [gap-autoware-export.md](./gap-autoware-export.md) (+ M-01).

## The extrinsic-stability cluster (2026-07-12)

C-03, H-07, H-08, H-09, M-11–M-14 and L-10–L-12 are one story, not ten independent bugs. Together
they explain the standing symptom that the point-cloud overlay is correct on the board and on the
ArUco markers while the background points are tilted, and that caching more image/point-cloud pairs
does not fix it.

The short version: the PnP correspondences all live on a 500 mm coplanar patch, so a rotation about
the correspondence centroid is a near-null direction of the reprojection cost — it leaves the board
overlay untouched and tilts everything else. Nothing in the pipeline measures conditioning, so a
degenerate capture reports `"Calibration successful"` exactly like a good one.

- Root cause and geometry: [H-07](./archive/H-07-no-pose-diversity-gate.md)
- Why it is invisible: [H-09](./archive/H-09-no-extrinsic-quality-metric.md)
- The remediation plan: [docs/roadmap/phase-5-stable-extrinsic-solution.md](../roadmap/phase-5-stable-extrinsic-solution.md)

**Fix order matters.** [C-03](./archive/C-03-double-undistortion.md) made border-of-image poses carry the
largest systematic bias — so the standard remedy for H-07 ("spread the board across the field of
view") would have injected error rather than removing it. C-03 is now **fixed** (2026-07-12), which
unblocks the pose-diversity work.

## The pipeline was producing nothing at all (2026-07-13)

Worth stating plainly, because it reframes everything above: **`just demo` had been silently
producing zero calibrations.** The board detector's ICP accept gate (`icp_good_fit_threshold`) had
been tightened to `0.012` — *below the VLP-32C's ±3 cm range noise*, and below the 0.026–0.029 loss
that `CLAUDE.md`'s own profiling section records as normal. No fit could ever pass. The rejection
was logged at `debug`, so the detector emitted empty detections forever without a word.

That is [C-04](./archive/C-04-board-detector-gate-unreachable.md), now fixed: 0 → 1,049 board detections,
0 → 1,031 PnP solves on the shipped sample data.

The lesson generalises, and it is the same one [H-09](./archive/H-09-no-extrinsic-quality-metric.md) makes
about the extrinsic: **this system has no way to tell you it is not working.** A gate that can never
pass, a detector that publishes empty results, and a solver that reports `"Calibration successful"`
are all the same failure — silence where a number should be.

## Verified against live source
C-01, H-01, H-02 were confirmed by reading the current code during the audit; the rest of the
2026-07-09 findings are from static review and marked with their file:line anchors.
Every 2026-07-12 finding (C-03, H-07–H-09, M-11–M-14, L-10–L-12) was confirmed by reading the
current source.
