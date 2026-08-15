# LCTK Issue Tracker

Findings from the 2026-07-09 workflow + correctness audit (sensor → config → build → runtime → solver → Autoware export), plus the 2026-07-12 extrinsic-stability audit (C-03, H-07…H-09, M-11…M-14, L-10…L-12) and the 2026-08-15 conflux-core algorithm + API audit (C-05, H-11…H-13, M-17…M-25, L-17…L-26). One file per finding, ranked by severity.

Status legend: 🔴 open · 🟡 in progress · 🟢 fixed · ⚪ won't fix / by-design

Closed issues (🟢 fixed, ⚪ won't-fix/by-design) are archived under [`archive/`](./archive/); open (🔴) and in-progress (🟡) issues stay here.

| ID | Sev | Title | Status |
|----|-----|-------|--------|
| [C-01](./archive/C-01-aruco-corners-discarded.md) | Critical | ArUco marker corners discarded; PnP uses axis-aligned bbox | 🟢 |
| [C-02](./archive/C-02-conflux-realtime-memory-leak.md) | Critical | Conflux realtime mode leaks a message object per dropped message | 🟢 |
| [C-03](./archive/C-03-double-undistortion.md) | Critical | Image undistorted twice before ArUco detection → every corner biased | 🟢 |
| [C-04](./archive/C-04-board-detector-gate-unreachable.md) | Critical | ICP accept gate set below the sensor noise floor → detector silently accepts nothing | 🟢 |
| [C-05](./archive/C-05-conflux-ffi-sync-wedges.md) | Critical | Conflux FFI synchronizer wedges permanently after a stream divergence | 🟢 |
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
| [H-11](./archive/H-11-conflux-staleness-anchored-to-construction.md) | High | Conflux staleness expiry anchored to construction time, not message arrival | 🟢 |
| [H-12](./archive/H-12-conflux-two-divergent-pipelines.md) | High | conflux has two divergent pipelines; tests cover the one production does not use | 🟢 |
| [H-13](./archive/H-13-conflux-tokio-tests-never-compiled.md) | High | Conflux tokio tests had not compiled; `just test` reported green | 🟢 |
| [H-14](./archive/H-14-conflux-third-sync-implementation.md) | High | A third, independent sync implementation lives in `conflux-ros2` | 🟢 |
| [M-01](./archive/M-01-transform-direction-inverted.md) | Medium | Transform frame labels inverted vs ROS TF semantics | 🟢 |
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
| [M-12](./archive/M-12-no-robust-estimation-or-refinement.md) | Medium | No outlier rejection and no LM refinement in the extrinsic solve | 🟢 |
| [M-13](./archive/M-13-icp-quality-not-propagated.md) | Medium | Board-pose uncertainty measured, then discarded before the solver | 🟢 |
| [M-14](./archive/M-14-corner-order-brittle.md) | Medium | Board origin corner picked by gravity; corner order duplicated, unchecked | 🟢 |
| [M-15](./archive/M-15-bbox-quaternion-order-comment.md) | Medium | `bbox.json5` documents the quaternion `(w,x,y,z)`; the wire format is `(x,y,z,w)` | 🟢 |
| [M-16](./archive/M-16-l2l-pipeline-untested.md) | Medium | LiDAR-to-LiDAR pipeline has never been run end-to-end | 🟢 |
| [M-17](./M-17-judge-ground-truth-wrong-rig.md) | Medium | Judge ground truth does not describe the shipped sample data | 🟡 |
| [M-17](./archive/M-17-conflux-timer-wheel-loses-messages.md) | Medium | Staleness timer wheel skips slots and misplaces messages | 🟢 |
| [M-18](./archive/M-18-conflux-immediate-expiration-is-a-stub.md) | Medium | `enable_immediate_expiration` spawns a task that does nothing | 🟢 |
| [M-19](./archive/M-19-conflux-staleness-tracks-rejected-messages.md) | Medium | Staleness tracks rejected messages → ghosts can evict valid ones | 🟢 |
| [M-20](./archive/M-20-conflux-expiration-only-removes-front.md) | Medium | Expired messages removed only if at the buffer front | 🟢 |
| [M-21](./archive/M-21-conflux-two-time-bases-for-expiry.md) | Medium | Expiry defined in two incompatible time bases (wall clock vs stamp) | 🟢 |
| [M-22](./archive/M-22-conflux-last-ts-never-resets.md) | Medium | A stream whose clock goes backwards is permanently dead (`last_ts` never resets) | 🟢 |
| [M-23](./archive/M-23-conflux-stall-is-unobservable.md) | Medium | A stalled synchronizer is unobservable; statistics look perfect | 🟢 |
| [M-24](./archive/M-24-conflux-py-buffer-size-validation.md) | Medium | Invalid `buffer_size` raised a generic RuntimeError; `__del__` on partial init | 🟢 |
| [M-25](./archive/M-25-conflux-py-tests-never-ran.md) | Medium | `just test-python` collected zero tests and reported success | 🟢 |
| [L-01](./archive/L-01-fit-board-icp-false-success.md) | Low | Library `fit_board_icp` reports non-converged fits as successful | 🟢 |
| [L-02](./archive/L-02-rust-panics-empty-nan.md) | Low | Pure-Rust panics on empty / NaN point sets | 🟢 |
| [L-03](./archive/L-03-pnp-solver-panic-distortion.md) | Low | `pnp-solver` panics on failed solve, truncates distortion | 🟢 |
| [L-04](./archive/L-04-hardcoded-2x2-board.md) | Low | `multi_marker_corners` hardcodes 2×2; unknown config fields ignored | 🟢 |
| [L-05](./archive/L-05-mode-typo-static-mut.md) | Low | `mode` typo silently offline; `static mut` counters race | 🟢 |
| [L-06](./archive/L-06-pokemon-exceptions.md) | Low | Pervasive `except Exception: pass` against project guideline | 🟢 |
| [L-07](./archive/L-07-tf-broadcaster-qos.md) | Low | tf_tree_broadcaster QoS may be incompatible with realtime publishers | 🟢 |
| [L-08](./archive/L-08-stale-readme-docs.md) | Low | Stale README & docs misdirect new users | 🟢 |
| [L-09](./archive/L-09-setup-fragility-export-labeling.md) | Low | Setup fragility, no export tooling, dump JSON mislabeled | 🟢 |
| [L-10](./archive/L-10-solver-float32-precision.md) | Low | PnP correspondences and intrinsics cast to `float32` | 🟢 |
| [L-11](./archive/L-11-detector-param-block-bugs.md) | Low | Detector param block sets a field twice; tunes a disabled refiner | 🟢 |
| [L-12](./archive/L-12-dead-solver-crates.md) | Low | Dead crates (`pnp-solver`, `calibration-quality`) better than the live code | 🟢 |
| [L-13](./archive/L-13-calibration-metrics-msg-dead.md) | Low | `CalibrationMetrics.msg` built, unused, and IoU-shaped rather than residual-shaped | 🟢 |
| [L-14](./archive/L-14-lint-red-on-main.md) | Low | `just lint` is red on an untouched main checkout | 🟢 |
| [L-15](./archive/L-15-build-dirties-worktree.md) | Low | Every build dirties Cargo.lock + the conflux submodule | 🟢 |
| [L-16](./archive/L-16-bindgen-lock-stale-skip.md) | Low | `bindgen.lock` silently skips rosidl regeneration after partial cleanup | 🟢 |
| [L-17](./archive/L-17-conflux-is-empty-means-any-empty.md) | Low | `is_empty()` returns true when *any* buffer is empty | 🟢 |
| [L-18](./archive/L-18-conflux-result-not-exported.md) | Low | `last_push_result` returns an opaque int; `ConfluxResult` unexported | 🟢 |
| [L-19](./archive/L-19-conflux-py-swallows-import-error.md) | Low | `conflux_py/__init__.py` swallows real ImportErrors | 🟢 |
| [L-20](./archive/L-20-conflux-window-zero-sentinel.md) | Low | `window_size_ms = 0` is a magic sentinel for infinite window | 🟢 |
| [L-21](./archive/L-21-conflux-buf-size-min-unexplained.md) | Low | `buf_size >= 2` enforced without explanation | 🟢 |
| [L-22](./archive/L-22-conflux-cpp-has-no-tests.md) | Low | `just test-cpp` reports success while `conflux_cpp` has zero tests | 🟢 |
| [L-23](./archive/L-23-conflux-core-dead-code.md) | Low | Half-built feedback path, dead assert, large commented-out blocks | 🟢 |
| [L-24](./archive/L-24-conflux-sync-is-ready-latency.md) | Low | `sync()` holds a matched pair until every stream has two messages | 🟢 |
| [L-25](./archive/L-25-conflux-docs-stale-test-count.md) | Low | conflux CLAUDE.md documents a test count the suite does not have | 🟢 |
| [L-26](./archive/L-26-anyio-breaks-pytest.md) | Low | pip `--user` `anyio` breaks pytest workspace-wide before collection | 🟢 |
| [L-27](./archive/L-27-conflux-cpp-lint-red.md) | Low | `ament_lint` red on `conflux_cpp`, including generated and build-artifact files | 🟢 |
| [L-28](./archive/L-28-just-test-pytest-missing.md) | Low | `just test` invoked a bare `pytest`; the Python suites never ran | 🟢 |
| [L-29](./archive/L-29-symlink-install-stale-launch.md) | Low | Deleting a launch file leaves a dangling symlink that breaks the next build | 🟢 |

## Status (2026-08-16)

One issue in progress: **M-17**. Its silent half is fixed — the judge now detects a
reference recorded for a different rig and says so, instead of scoring 0/15 without
explanation. What remains needs knowledge this repo does not hold: a ground truth
actually recorded for `lctk_sample_data` dataset 3. Everything else is 🟢. All of the 2026-07-09, 2026-07-12 and 2026-08-15 audits are
closed, including the four that
had been carried as deferred or partially-fixed for a month: M-01 (transform direction), M-12
(robust estimation), M-14 (origin-corner disambiguation) and M-16 (the L2L pipeline).

Three of those four had been left open on the grounds that they needed something this environment
could not provide — a visual overlay check, board captures near 45° roll, an eye on RViz. In each
case the blocking premise turned out to be narrower than stated:

- **M-01** wanted "do the points still land on the image". That is a *numeric* property: project
  the same points through both paths and compare pixels. The test is stricter than the eyeball.
- **M-14** wanted 45°-roll captures. Those would validate the *improvement*; validating the
  *premise* — that the hole asymmetry separates the four candidate orientations at all — needs
  only synthetic points, and runs in 0.1 s.
- **M-16** wanted RViz. Repeatability across 81 independent solves is stronger and quantitative.
  What a visual check would still add is that the baseline matches the physical rig, which no
  self-consistency check can establish; that part is genuinely still operator work and is recorded
  as such.

The general lesson is worth keeping: "needs a human" is sometimes true of the *whole* task and
rarely true of *every part* of it. Splitting the verifiable part out is usually possible.

## Three headline gaps

1. **Every calibration is biased** — C-01. The detector throws away real ArUco corners.
2. **The build omits a dependency the solvers need** — H-01. `conflux` is never built.
3. **The Autoware last mile** — fixed 2026-07-16 by [Phase 6](../roadmap/phase-6-autoware-export.md)'s `lctk_autoware_export` (see [gap-autoware-export.md](./archive/gap-autoware-export.md)); transform-direction labeling (M-01) still open.

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

## The conflux cluster (2026-08-15)

**Status (2026-08-15):** every finding in this cluster is **fixed**
(`jerry73204/conflux`@bb490d9, @014a2c9 and @0a9c901) — Critical through Low.

Two further findings surfaced while closing it. **H-14** — a *third* independent
sync implementation in `conflux-ros2`, which H-12's "two pipelines" framing
missed — is now **fixed**: it is an adapter over the core, which became possible
only once the staleness removal retired the `T: Clone` bound that had forced the
duplication. **L-27** is fixed too: the C++ linters are green, the generated
header no longer oscillates between formatter and build, and `ament_uncrustify`
was dropped rather than allowed to fight clang-format forever.

**The conflux cluster is now closed end to end.** No conflux issue remains open.

- **C-05, H-12** — the matching policy now lives in one place (`State::advance`)
  that both drivers call, which closes the wedge and makes the two pipelines
  agree on identical input.
- **H-11, M-17–M-21** — the staleness subsystem was **removed**, not repaired
  (Phase 8 Stage 0). It was unreachable, defective in every part, and built on
  the wrong clock for recorded playback. Message-time expiry via
  `WithTimestamp::timeout` remains and is unchanged.
- **M-22** — `reset()` across core, the C ABI and Python, so a bag loop or
  sim-time reset no longer kills a stream permanently.
- **M-23** — `match_status()` reports why the matcher is not emitting, exported
  through the C ABI and Python, with a stall warning in `ROS2Synchronizer`.

- **L-17, L-19–L-25** — ergonomics and tooling: `is_empty` split into
  `has_empty_buffer`/`all_buffers_empty`, the swallowed ImportError narrowed, the
  `window_size_ms = 0` sentinel rejected, the `buf_size >= 2` floor explained,
  dead code removed, and `conflux_cpp` given its first tests behind a recipe that
  can actually fail.

The narrative below is kept in the past tense it was written in, because it
explains how the cluster arose.


C-05, H-11–H-13, M-17–M-25 and L-17–L-26 are one story about the message synchronizer every
solver node depends on, and it rhymes with the two above: **silence where a number should be.**

The structural cause is H-12. conflux ships *two* implementations of the pipeline over one
shared `State` — the pure-Rust `sync()` stream and the C ABI the bindings actually use. They
have different escape hatches, different emission gates, and different staleness support. All
LCTK nodes take the FFI path; conflux-core's 156 tests almost all exercise `sync()`.

That gap is what lets C-05 exist. `sync()` cannot wedge, because its poll loop calls
`drop_min()` when the buffers fill with unmatchable data. The FFI has no such call, so once
every buffer holds two messages and the spread stays under the window, it never emits again —
permanently, for the life of the process. Reproduced against the shipped realtime preset
(50 ms window, buffer 2): under `RejectNew` every subsequent push is refused; under
`DropOldest` every push is *accepted*, statistics report zero rejections and zero overflows,
and still nothing comes out.

Which is M-23's point. The observability surface describes inputs only — received, rejected,
buffer length. Nothing answers "why is the matcher not matching?", so the `DropOldest` wedge
presents as a perfectly healthy synchronizer that has silently stopped calibrating. Same
failure shape as [C-04](./archive/C-04-board-detector-gate-unreachable.md)'s unreachable gate
and [H-09](./archive/H-09-no-extrinsic-quality-metric.md)'s missing metric.

The staleness subsystem (H-11, M-17–M-21) is a separate matter: it is closer to a prototype
than a feature. Expiry is anchored to construction time rather than message arrival, so
messages are born stale (H-11); the `enable_immediate_expiration` background task is an
acknowledged placeholder that expires nothing (M-18); the timer wheel drains one slot per call
and inserts at the wrong offset (M-17); expired messages are only removed if they happen to
sit at a buffer front (M-20). It is unreachable today only because the FFI hardcodes
`staleness_detector: None`. The phase doc treats "repair or remove" as an open decision rather
than assuming repair.

**Why none of this was caught.** H-13 and M-25: both test suites were reporting green while
running nothing. `just test-rust` omitted `--features tokio`, so 20 staleness tests compiled
to nothing after the `Config` API changed under them; `just test-python` ran the tests through
colcon's unittest path, which collected 0 of 19 pytest-style tests and exited 0. Repairing the
Python suite immediately exposed M-24. `conflux_cpp` still has no tests at all (L-22).

- The remediation plan: [phase-7](../roadmap/phase-7-conflux-sync-correctness.md),
  [phase-8](../roadmap/phase-8-conflux-staleness-subsystem.md),
  [phase-9](../roadmap/phase-9-conflux-api-and-tooling.md)

## Verified against live source
C-01, H-01, H-02 were confirmed by reading the current code during the audit; the rest of the
2026-07-09 findings are from static review and marked with their file:line anchors.
Every 2026-07-12 finding (C-03, H-07–H-09, M-11–M-14, L-10–L-12) was confirmed by reading the
current source.
The 2026-08-15 conflux findings (C-05, H-11–H-13, M-17–M-25, L-17–L-26) were confirmed by
reading the current source; C-05, H-11 and M-24 were additionally reproduced at runtime
against the built `libconflux_ffi.so`.
