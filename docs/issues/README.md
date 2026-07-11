# LCTK Issue Tracker

Findings from the 2026-07-09 workflow + correctness audit (sensor → config → build → runtime → solver → Autoware export). One file per finding, ranked by severity.

Status legend: 🔴 open · 🟡 in progress · 🟢 fixed · ⚪ won't fix / by-design

| ID | Sev | Title | Status |
|----|-----|-------|--------|
| [C-01](./C-01-aruco-corners-discarded.md) | Critical | ArUco marker corners discarded; PnP uses axis-aligned bbox | 🔴 |
| [C-02](./C-02-conflux-realtime-memory-leak.md) | Critical | Conflux realtime mode leaks a message object per dropped message | 🔴 |
| [H-01](./H-01-conflux-not-built.md) | High | `conflux_py` never built → solvers ImportError at startup | 🟢 |
| [H-02](./H-02-conflux-drops-first-message.md) | High | Conflux Python binding drops the first message (msg_id 0 → NULL) | 🔴 |
| [H-03](./H-03-pointcloud-datatype-endian.md) | High | Point cloud XYZ decoded as LE float32 without checking datatype/endianness | 🟢 |
| [H-04](./H-04-board-detector-mandatory-params.md) | High | Detector declares params mandatory that launch adds only "if present" | 🟢 |
| [H-05](./H-05-conflux-error-stats-collapse.md) | High | Conflux FFI collapses all push errors to BufferFull → corrupt stats | 🔴 |
| [H-06](./H-06-config-schema-drift.md) | High | CLAUDE.md documents a config schema the parser does not accept | 🟢 |
| [M-01](./M-01-transform-direction-inverted.md) | Medium | Transform frame labels inverted vs ROS TF semantics | 🔴 |
| [M-02](./M-02-radians-degrees-mix.md) | Medium | Advanced solver adjust/pose API mixes radians and degrees | 🔴 |
| [M-03](./M-03-hardcoded-plane-normal-x.md) | Medium | Hardcoded plane-normal flip to +X assumes sensor-forward-X | 🔴 |
| [M-04](./M-04-l2l-wallclock-staleness.md) | Medium | L2L staleness check uses wall-clock vs sensor stamp | 🔴 |
| [M-05](./M-05-l2l-wrong-pose-field.md) | Medium | L2L solver reads board pose from wrong message field | 🔴 |
| [M-06](./M-06-detector-thread-no-panic-guard.md) | Medium | Board-detector thread has no panic guard → silent dead node | 🔴 |
| [M-07](./M-07-conflux-dropoldest-double-loss.md) | Medium | DropOldest can destroy a good message and reject the new one | 🔴 |
| [M-08](./M-08-conflux-ffi-no-locking.md) | Medium | Mutable Rust conflux State crosses FFI with no locking | 🔴 |
| [M-09](./M-09-marker-ids-hard-index.md) | Medium | `marker_ids[0..3]` hard index → IndexError on short config | 🔴 |
| [M-10](./M-10-multi-marker-config-collisions.md) | Medium | Multi-marker camera uses wrong ArUco config; duplicate pairs collide | 🔴 |
| [L-01](./L-01-fit-board-icp-false-success.md) | Low | Library `fit_board_icp` reports non-converged fits as successful | 🔴 |
| [L-02](./L-02-rust-panics-empty-nan.md) | Low | Pure-Rust panics on empty / NaN point sets | 🔴 |
| [L-03](./L-03-pnp-solver-panic-distortion.md) | Low | `pnp-solver` panics on failed solve, truncates distortion | 🔴 |
| [L-04](./L-04-hardcoded-2x2-board.md) | Low | `multi_marker_corners` hardcodes 2×2; unknown config fields ignored | 🔴 |
| [L-05](./L-05-mode-typo-static-mut.md) | Low | `mode` typo silently offline; `static mut` counters race | 🔴 |
| [L-06](./L-06-pokemon-exceptions.md) | Low | Pervasive `except Exception: pass` against project guideline | 🔴 |
| [L-07](./L-07-tf-broadcaster-qos.md) | Low | tf_tree_broadcaster QoS may be incompatible with realtime publishers | 🔴 |
| [L-08](./L-08-stale-readme-docs.md) | Low | Stale README & docs misdirect new users | 🔴 |
| [L-09](./L-09-setup-fragility-export-labeling.md) | Low | Setup fragility, no export tooling, dump JSON mislabeled | 🔴 |

## Three headline gaps

1. **Every calibration is biased** — C-01. The detector throws away real ArUco corners.
2. **The build omits a dependency the solvers need** — H-01. `conflux` is never built.
3. **The Autoware last mile is fully manual and undocumented** — see [gap-autoware-export.md](./gap-autoware-export.md) (+ M-01).

## Verified against live source
C-01, H-01, H-02 were confirmed by reading the current code during the audit; the rest are from static review and marked with their file:line anchors.
