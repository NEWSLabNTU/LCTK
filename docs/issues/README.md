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
| [H-11](./archive/H-11-camera-solvers-stale-board-frame.md) | High | Camera solvers used the old edge-aligned board frame → extrinsic wrong by 45°, half of it silently | 🟢 |
| [H-12](./H-12-continuous-solver-forgets-prior-placements.md) | High | Continuous LiDAR-camera calibration forgets prior board placements | 🔴 |
| [H-13](./H-13-l2l-latest-board-pair-overwrites-extrinsic.md) | High | LiDAR-to-LiDAR calibration overwrites the extrinsic from one board-pose pair | 🔴 |
| [H-15](./H-15-perforated-icp-applies-correction-backwards.md) | High | Perforated ICP applied its Kabsch correction backwards → every iteration moved away from the fit | 🟢 |
| [H-16](./H-16-play-launch-does-not-replay-execute-process.md) | High | `play_launch` replays `Node` actions only, so a `kind: bag` session's `ExecuteProcess` player ran during the recording pass and the recording played into an empty graph | 🟢 |
| [H-17](./archive/H-17-solid-600-preset-detects-nothing.md) | High | The `solid_600` detector preset rejects every frame of real data | 🟢 |
| [M-01](./archive/M-01-transform-direction-inverted.md) | Medium | Transform frame labels inverted vs ROS TF semantics | 🟡 |
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
| [M-14](./archive/M-14-corner-order-brittle.md) | Medium | Board origin corner picked by gravity; corner order duplicated, unchecked | 🟡 |
| [M-15](./archive/M-15-bbox-quaternion-order-comment.md) | Medium | `bbox.json5` documents the quaternion `(w,x,y,z)`; the wire format is `(x,y,z,w)` | 🟢 |
| [M-16](./archive/M-16-l2l-pipeline-untested.md) | Medium | LiDAR-to-LiDAR pipeline has never been run end-to-end | 🔴 |
| [M-17](./M-17-initial-pose-rewrite-unverified-bbox-path.md) | Medium | Shared initial-pose rewrite leaves the bbox path's "unchanged" guarantee unproven | 🔴 |
| [M-18](./archive/M-18-root-cargo-config-missing-rust-tests-unrunnable.md) | Medium | No root `.cargo/config.toml` → Rust test suite unrunnable and the L-16 guard is inert | 🟢 |
| [M-19](./M-19-debug-assertions-compiled-out.md) | Medium | Every `debug_assert!` compiled out of `just build` and `just test`; the 51 in `hollow-board-config` are also rotation-invariant | 🔴 |
| [M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md) | Medium | Board model's axes run along edges while every accessor names a diamond → `initial_inplane_rotation_deg: 45.0` mandatory | 🟢 |
| [M-21](./archive/M-21-icp-stable-pose-exit-unreachable.md) | Medium | ICP's "stable pose" exit needs ~5× more iterations than any preset allows → `icp_pose_weight_threshold` is inert | 🟢 |
| [M-22](./archive/M-22-root-cargo-patch-block-single-source.md) | Medium | Root `.cargo/config.toml` copied from one package → clean clone cannot build at all | 🟢 |
| [M-26](./archive/M-26-two-lidar-example-topics-unreachable.md) | Medium | `two_lidar.yaml` names topics no in-repo data source ever publishes | 🟢 |
| [M-27](./archive/M-27-solid-600-handheld-topics-alias-sample-data.md) | Medium | `solid_600_handheld.yaml`'s placeholder topics alias the hollow-board sample-data playback | 🟢 |
| [M-28](./M-28-generator-geometry-cell-handedness-disagree.md) | Medium | ArUco generator and target geometry bind cells with opposite handedness (2x2 targets) | 🔴 |
| [M-29](./M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md) | Medium | Sample-data path dead: shared crop box retuned for another rig + ICP gate under the noise floor | 🟢 |
| [M-30](./M-30-bag-playback-qos-mismatch-is-silent.md) | Medium | A `kind: bag` session in the default `offline` mode silently receives no LiDAR: recorded BEST_EFFORT QoS cannot feed a RELIABLE subscriber | 🔴 |
| [M-31](./M-31-perforated-icp-parked-termination-findings.md) | Medium | Perforated ICP: `StablePose` publishes an unbounded residual and is reachable at a shipped budget; separation gate skipped on a lone success; two dead-code leftovers | 🔴 |
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
| [L-17](./archive/L-17-boardconfig-defaults-duplicated.md) | Low | `BoardConfig` defaults defined twice — serde fns and `production_config` will drift | 🟢 |
| [L-18](./L-18-overlay-node-commented-extrinsic-override.md) | Low | Overlay node ships commented-out hardcoded extrinsic overrides | 🔴 |
| [L-19](./archive/L-19-aruco-config-required-but-unused-for-lidar.md) | Low | `aruco_config` mandatory for LiDAR-only markers but never affects the LiDAR fit | 🟢 |
| [L-20](./archive/L-20-dead-bbox-parser-quaternion-order.md) | Low | Dead `BBox` JSON5 parser reads the quaternion w-first; two configs still carry `[1,0,0,0]` | 🟢 |
| [L-21](./archive/L-21-find-correspondences-duplicated-tests-wrong-body.md) | Low | `find_correspondences` duplicated; inline tests exercise the serial copy the node never runs | 🟢 |
| [L-22](./archive/L-22-advanced-solver-undeclared-lctk-interfaces-dep.md) | Low | `advanced_extrinsic_solver` imports `lctk_interfaces` without declaring the dependency | 🟢 |
| [L-23](./L-23-debug-mode-parameter-never-read.md) | Low | `debug_mode` declared by both solvers, read by neither | 🔴 |
| [L-24](./archive/L-24-board-geometry-import-test-egl-stdout.md) | Low | Board-geometry import test rejects unrelated Jetson EGL stdout | 🟢 |
| [L-25](./L-25-fresh-machine-bringup-deps-missing.md) | Low | `setup.sh` installs none of the tools `just test` and `just lint` need | 🔴 |
| [L-30](./archive/L-30-extrinsic-solver-launch-xml-missing-target-config.md) | Low | `extrinsic_solver_node.launch.xml` (lidar_to_camera_solver) never passes `target_config` → node can't start | 🟢 |
| [L-31](./L-31-plane-estimator-orphaned-crate.md) | Low | `rust/plane-estimator` is a live workspace member with zero consumers | 🔴 |

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

## The board frame changed under the camera solvers (2026-08-13)

The calibration board's canonical local frame was corner-aligned in Phase 1 of
[M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md): origin at the plate centre, in-plane
axes running corner to corner, matching how the board is physically hung. `initial_inplane_rotation_deg`
is now `0.0` in every config, and the magic `45.0` is gone. Phase 1 was **field-validated on the
two-LiDAR rig on 2026-08-14** — the board's `+Y` arrow points at the physically up-most plate corner,
which is what rules out the `−45°` conjugation that would have produced an identical-looking diamond
with the corner labels a quarter turn out — so M-20 is now 🟢.

The camera-side board-frame mismatch tracked by [H-11](./archive/H-11-camera-solvers-stale-board-frame.md)
is fixed. Stage 1 ported the maintained `lidar_to_camera_solver` to the corner-aligned,
plate-centre frame and Stage 2 put both operating modes on that backend. Stage 3 removed the
superseded `extrinsic_solver_node` and its stale references. The 45° error was real when filed;
it is no longer present on any supported config-driven calibration path.

Read the archived issue for the historical failure mode and validation caveat before running any
LiDAR-camera calibration from this tree. The board pose is a board→sensor transform and the solver
feeds board-local coordinates *into* it, so the convention sits on both sides of the product. A
mismatch would leave a 45° in-plane error that the symmetric 2×2 marker grid absorbs with a low
reprojection error — silent — alongside a ~707 mm origin shift that probably would be noticed.
LiDAR-to-LiDAR calibration is unaffected: both of its sides come from the same detector.

## Conflux submodule audit (2026-08-15) — merged from `main` 2026-08-31

These came in with the `main` merge and cover the **conflux submodule**, not LCTK itself.
They carry their own ID sequence, so several IDs appear twice in this directory with entirely
different meanings: this table's M-17/M-18, L-17/L-18/L-19, H-11/H-13/H-14, C-05, M-20 and
M-23 are *not* the LCTK findings of the same name listed above. Match by filename, never by ID.

All are 🟢 fixed; the remediation is written up in
[phase-7](../roadmap/phase-7-conflux-sync-correctness.md), [phase-8](../roadmap/phase-8-conflux-staleness-subsystem.md)
and [phase-9](../roadmap/phase-9-conflux-api-and-tooling.md).

| ID | Severity | Finding | Status |
|----|----------|---------|--------|
| [C-05](./archive/C-05-conflux-ffi-sync-wedges.md) | Critical | Conflux FFI synchronizer wedges permanently after a stream divergence | 🟢 |
| [H-11](./archive/H-11-conflux-staleness-anchored-to-construction.md) | High | Conflux staleness expiry is anchored to construction time, not message arrival | 🟢 |
| [H-12](./archive/H-12-conflux-two-divergent-pipelines.md) | High | conflux has two divergent pipelines; the test suite covers the one production does not use | 🟢 |
| [H-13](./archive/H-13-conflux-tokio-tests-never-compiled.md) | High | Conflux tokio integration tests had not compiled for an unknown period; `just test` hid it | 🟢 |
| [H-14](./archive/H-14-conflux-third-sync-implementation.md) | High | A third, independent synchronization implementation lives in `conflux-ros2` | 🟢 |
| [M-17](./archive/M-17-conflux-timer-wheel-loses-messages.md) | Medium | Conflux staleness timer wheel skips slots and misplaces messages | 🟢 |
| [M-17](./archive/M-17-judge-ground-truth-wrong-rig.md) | Medium | The checked-in judge ground truth does not describe the shipped sample data | 🟢 |
| [M-18](./archive/M-18-conflux-immediate-expiration-is-a-stub.md) | Medium | `enable_immediate_expiration` spawns a task that does nothing | 🟢 |
| [M-19](./archive/M-19-conflux-staleness-tracks-rejected-messages.md) | Medium | Staleness tracks messages before the ordering check, so rejected messages become ghost entries | 🟢 |
| [M-20](./archive/M-20-conflux-expiration-only-removes-front.md) | Medium | Expired messages are only removed if they sit at the front of a buffer | 🟢 |
| [M-21](./archive/M-21-conflux-two-time-bases-for-expiry.md) | Medium | Expiry is defined in two incompatible time bases (wall clock vs message stamp) | 🟢 |
| [M-22](./archive/M-22-conflux-last-ts-never-resets.md) | Medium | A stream whose clock goes backwards is permanently dead (`last_ts` never resets) | 🟢 |
| [M-23](./archive/M-23-conflux-stall-is-unobservable.md) | Medium | A stalled synchronizer is unobservable — statistics look perfect while nothing is emitted | 🟢 |
| [M-24](./archive/M-24-conflux-py-buffer-size-validation.md) | Medium | `conflux_py` reported an invalid `buffer_size` as a generic RuntimeError, and `__del__` ran on a partially built object | 🟢 |
| [M-25](./archive/M-25-conflux-py-tests-never-ran.md) | Medium | `just test-python` collected zero tests and reported success | 🟢 |
| [L-17](./archive/L-17-conflux-is-empty-means-any-empty.md) | Low | `is_empty()` returns true when *any* buffer is empty | 🟢 |
| [L-18](./archive/L-18-conflux-result-not-exported.md) | Low | `last_push_result` returns an opaque int; `ConfluxResult` is neither exported nor constructible | 🟢 |
| [L-19](./archive/L-19-conflux-py-swallows-import-error.md) | Low | `conflux_py/__init__.py` swallows real ImportErrors, hiding `ROS2Synchronizer` | 🟢 |
| [L-20](./archive/L-20-conflux-window-zero-sentinel.md) | Low | `window_size_ms = 0` is a magic sentinel for "infinite window" | 🟢 |
| [L-21](./archive/L-21-conflux-buf-size-min-unexplained.md) | Low | `buf_size >= 2` is enforced without explanation | 🟢 |
| [L-22](./archive/L-22-conflux-cpp-has-no-tests.md) | Low | `just test-cpp` reports success while `conflux_cpp` has zero tests | 🟢 |
| [L-23](./archive/L-23-conflux-core-dead-code.md) | Low | conflux-core carries a half-built feedback path, a dead assert, and large commented-out blocks | 🟢 |
| [L-24](./archive/L-24-conflux-sync-is-ready-latency.md) | Low | `sync()` holds a matched pair until every stream has two messages | 🟢 |
| [L-25](./archive/L-25-conflux-docs-stale-test-count.md) | Low | conflux CLAUDE.md documents a test count the suite does not have | 🟢 |
| [L-26](./archive/L-26-anyio-breaks-pytest.md) | Low | A pip `--user` `anyio` breaks pytest workspace-wide before collection | 🟢 |
| [L-27](./archive/L-27-conflux-cpp-lint-red.md) | Low | `ament_lint` is red on `conflux_cpp`, including generated and build-artifact files | 🟢 |
| [L-28](./archive/L-28-just-test-pytest-missing.md) | Low | `just test` invoked a bare `pytest`, so the Python suites never ran | 🟢 |
| [L-29](./archive/L-29-symlink-install-stale-launch.md) | Low | Deleting a launch file leaves a dangling symlink that breaks the next build | 🟢 |

## Verified against live source
C-01, H-01, H-02 were confirmed by reading the current code during the audit; the rest of the
2026-07-09 findings are from static review and marked with their file:line anchors.
Every 2026-07-12 finding (C-03, H-07–H-09, M-11–M-14, L-10–L-12) was confirmed by reading the
current source.
