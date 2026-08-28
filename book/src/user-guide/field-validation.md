# Field Validation Runbook (Wave 7)

This is the operating guide for running the calibrator against real recordings and producing the
evidence Wave 7 requires. It assumes you have operated the pre-Phase-8 calibrator and wants to tell
you what is different, not what a calibration is.

Two things to know before anything else:

- **Nothing in Phase 8 has been run against real sensor data.** Every gate the phase passed was
  headless. Your first session is an experiment, not a regression check. Budget time for the
  pipeline not working the first time, and read [Reading the system](#reading-the-system) before you
  need it.
- **The evidence tooling has no bag reader and no command line.** `lctk_quality`'s
  `EvidenceCollector` is a library and a schema. There is no `ros2 run` that turns a bag into a
  report. See [Collecting evidence](#collecting-evidence-for-w7-b) — you will be recording
  observations by hand against a defined schema.

## What changed since your last session

Your workflow shape is unchanged: launch the graph, play a bag in a second terminal, drive the TUI,
watch RViz, read logs. These are the differences that will actually stop you.

| Change | What it means for you |
|---|---|
| Marker schema | Your own YAML needs `target_config` + `detector_config`. The old `type` / `board_config` / `aruco_config` keys are **refused with an error**, not ignored. |
| `sync:` section | Now **required** in every config. Missing it fails at parse time, before any node starts. |
| Target-identity gate | New, and **fail-closed**. Solvers admit nothing until observers announce a matching target. Looks like nothing happening. |
| Detection archives | Now version 5 and carry a Target Identity. **Your old saved dumps will not load** without an explicit migration. |
| Detector presets | Moved to `config/board/<target>/<sensor>.json5`. The old `board_detector*.json5` files are gone. |
| ICP correctness | Issue H-15: until recently the perforated ICP applied its correction *backwards*. If you tried the new detector before that fix, what you saw was that bug. |

Unchanged: the TUI, its key bindings, `just manual-solver-controller`, the solver services, and the
overall manual-mode capture loop.

## Before you start: fix your config

If you are reusing a YAML from before Phase 8, edit it before you get to the field.

1. **Add a `sync:` block** at top level. All three keys are required:

   ```yaml
   sync:
     tolerance_ms: 100        # finite and > 0
     queue_size: 100          # positive integer
     drop_policy: reject_new  # or drop_oldest
   ```

   Use `reject_new` for replaying a recording: it loses no recorded data. `drop_oldest` is for live
   sensors where the latest data matters more than completeness.

   For a **moving, hand-held board**, tighten `tolerance_ms`. A mis-paired camera frame and LiDAR
   sweep is not merely noisy — it is *wrong*, because the board is not where the other sensor saw
   it. `config/examples/solid_600_handheld.yaml` states 50 ms, and says plainly that the number is
   intent rather than measurement. Confirming it is one of your first tasks; see
   [the sync line](#the-sync-line).

2. **Rewrite each marker.** Delete `type`, `board_config`, `aruco_config`. Add:

   ```yaml
   markers:
     calibration_board:
       target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
       detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/seyond.json5
       pairs:
         - [seyond_lidar, left_camera]
   ```

   Available targets: `hollow_1000_aruco_4_v1.json5`, `solid_600_aruco_1_v1.json5`.
   Available presets: `hollow_1000/{velodyne,velodyne_bbox,seyond}.json5`,
   `solid_600/{velodyne,seyond}.json5`.

3. **Drop `bbox_config`** unless your chosen preset is `hollow_1000/velodyne_bbox.json5` — that is
   the only shipped preset in `bbox` mode. Every other preset is `bbox_free` and never reads a crop
   box.

4. **Per-LiDAR overrides**: rename `board_config:` to `detector_config:` under
   `devices.lidars.<name>`. This is how two differently-sampled LiDARs share one target;
   `config/examples/two_lidar.yaml` does exactly that.

Copy `config/examples/seyond_left.yaml` (LiDAR + camera) or `two_lidar.yaml` (two LiDARs) as your
template. Both are 45–50 lines and current.

**Check it parses before you travel.** The parser loads and validates the target manifest, so this
catches most mistakes without any sensor:

```bash
source install/setup.bash
ros2 launch lctk_launch calibrate.launch.py config_file:=/path/to/your.yaml enable_rviz:=false
```

## Running a session

Four terminals, same as before.

**Terminal 1 — the graph.** For a bring-your-own-bag config:

```bash
just solver_mode=manual enable_judge=false lidar-camera your_config.yaml
```

`lidar-camera` takes a **bare filename** resolved inside the installed
`config/examples/` directory. For a config living elsewhere, use the full path form:

```bash
just solver_mode=manual enable_judge=false calibrate /abs/path/to/your.yaml
```

> **Always pass `enable_judge=false` for a field session.** The justfile defaults it to `true`, and
> the launch file never passes a ground-truth file, so the judge falls back to a hardcoded matrix
> from one historical rig. On any other rig it scores your solve against someone else's extrinsic
> and prints confident-looking numbers that mean nothing.

Useful overrides: `mode=offline` (default; RELIABLE QoS, right for bags), `debug_mode=true`
(default; publishes the LiDAR `debug/*` clouds and the ArUco overlay image), `log_level=debug` (this
is what raises log verbosity — `debug_mode` does not), `rviz_enabled=false`.

**Terminal 2 — the data.** `just two-lidar` and `just lidar-camera` do **not** play anything. Play
your bag yourself:

```bash
ros2 bag play /path/to/your.bag --clock
```

If your bag carries `CompressedImage`, republish to raw first — the locator subscribes to `Image`:

```bash
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/camera/left/image_raw/compressed \
             -r out:=/camera/left/image_raw
```

Your bag must also carry `camera_info` alongside the image: the locator derives that topic from the
image topic's namespace.

**Terminal 3 — the TUI.**

```bash
just manual-solver-controller
```

It discovers the solver on the graph by itself. If more than one lidar-camera pair exists it offers
a numbered picker. If it reports no services, the solver is not in manual mode — services exist only
under `solver_mode=manual`. To bind explicitly (the justfile recipe forwards no arguments):

```bash
ros2 run interactive_solver_controller interactive_solver_controller \
  --service-base /calibration/<lidar>_<camera>
```

Key bindings are unchanged: `Space` add, `Backspace` remove last, `c` clear, `p` save
(`~/detections.json`), `o` load, `q/a w/s e/d` translate, `r/f t/g y/b` rotate, `]` `[` step size,
`0` re-solve from buffer, `ESC` exit. The footer shows the last service reply — that is where a
refusal message appears.

**Terminal 4 — RViz.** The shipped configs are wired to specific examples:

| Config | Fixed frame | Matches |
|---|---|---|
| `config/rviz/calibration.rviz` (default) | `seyond` | `seyond_left.yaml` / `seyond_right.yaml` |
| `config/rviz/two_lidar_calibration.rviz` | `velodyne` | `two_lidar.yaml` |

`just lidar-camera` always uses the first one regardless of the config you pass, and there is no
justfile variable to change it. For any other rig, run with `rviz_enabled=false` and start RViz
yourself, or use the launch file directly with `rviz_config:=<path>`.

Both configs also reference two topics no node publishes (`debug/initial_board_marker`,
`/calibration/icp_debug/correspondences`). Those displays stay empty; that is not a fault.

## Reading the system

### Topics

Namespacing is derived from your config's device and marker names:

```
/calibration/<lidar>_<marker>/calibration_board_detections
/calibration/<lidar>_<marker>/target_identity
/calibration/<lidar>_<marker>/debug/*              (debug_mode only, 11 topics)
/calibration/<camera>/aruco_detections
/calibration/<camera>/target_identity
/calibration/<camera>/image_with_detections        (debug_mode only)
/calibration/<lidar>_<camera>/extrinsic_transform
/calibration/<lidar>_<camera>/lidar_to_camera_solver/*   (services, manual mode only)
```

For `two_lidar.yaml` the solver output is
`/calibration/top_lidar_front_lidar/lidar_to_lidar_transform`.

### The identity gate — the failure that looks like silence

This is new and it is the one most likely to waste your time. Before any detection pair is admitted,
the solver requires that its own target and **both** observers' announced identities match exactly
on all five fields (`schema_version`, `target_id`, `revision`, `semantic_sha256`,
`board_frame_convention`).

**When it is working**, you see this once:

```
LiDAR, camera, and local Target Identities agree; Detection Pair admission enabled
```

If you never see that line, the gate never opened, and no `Space` press will ever capture anything.

**When it is not**, the messages are:

- `LiDAR Target Identity is missing` / `camera Target Identity is missing` — that observer has not
  announced. Usually it did not start, or its topic is not reaching the solver. Note that a solver
  logs *nothing* about an observer that never appears, so silence is itself the symptom.
- `... does not exactly match the local Target Identity (...)` — one observer is on a different
  target than the solver.
- `... Target Identities disagree; no Detection Pair will be accepted` — the two observers disagree
  with each other.
- `Cannot capture before Target Identity agreement: ...` — what the TUI footer shows when you press
  `Space` with the gate shut.

**One trap worth memorising:** if you change `target_config` and restart only the observer nodes,
the still-running solver sees a *changed* identity from a source and blocks **permanently** — it
cannot be recovered without restarting the solver. Restart the whole graph after any target change.
The same event clears any captures you had buffered.

### The sync line

`DetectionPairSource` prints this every 10 s, but only when it changes:

```
sync: groups=580; pair skew last=12.4ms max=31.8ms; aruco_detections: received=800 rejected=0 dropped=0; calibration_board_detections: received=400 rejected=0 dropped=0
```

- `groups` — time-matched pairs emitted. Stuck at 0 while `received` climbs means the window is too
  tight or the two streams' header stamps do not overlap.
- **`pair skew max=` is the number that tells you whether your `tolerance_ms` is right.** It should
  sit comfortably *below* your window. Pinned near the tolerance means the window is doing the work
  and is too wide for a moving board; far below means you have headroom to tighten. **This is how
  you confirm or correct the solid example's provisional 50 ms.** Write the observed value down —
  it is evidence.
- `received` — messages reaching this node at all; a wiring check.
- `rejected` / `dropped` — buffer overflow under your drop policy.

If nothing pairs for 10 s you get a diagnosis naming which stream stopped, or telling you both are
arriving but not pairing (compare header stamps). Looping a bag prints a reset notice; that is
normal.

In manual mode, `Space` refuses if the newest pair is older than 2 s — pressing it after playback
stops is expected to fail.

### Detector rejections

The detector logs one line per rejected frame at INFO, unthrottled — on a 10 Hz LiDAR that is 10
lines a second when nothing is detected. Plan to `grep`, not watch.

```
bbox_free: no board selected — <description>; measured=<m> vs threshold=<t> [<unit>]; candidates=N, foreground_pts=M
```

The reason tells you which gate to loosen: `NoClusters` (nothing survived foreground extraction),
`Flatness`, `Extent`, `SizeGate`, `SquareResidual`, `Stance` (board not standing corner-up enough),
`Isolation` (board embedded in coplanar clutter). Target-aware rejections appear as
`target rejected: target=<id>@<rev> reason=<code> ...` with codes including `board_up_alignment`,
`insufficient_outer_edge_evidence`, `ambiguous_cutout_evidence` and `perforated_icp_failure`.

Success looks like:

```
Target detection successful: target=<id>@<rev>, pose=(x, y, z)
```

**Background warmup.** Every `bbox_free` preset uses background subtraction with
`bg_warmup_frames: 20`. Until warmup completes the detector emits nothing at all, printing
`background warmup <seen>/<needed>`. **Your recording must begin with at least 20 consecutive
board-absent frames** — roughly 2 s at 10 Hz — before the board enters. A bag that opens with the
board already in view will never detect anything, and the symptom is silence.

## Saving and loading

`p` writes a version-5 archive to `~/detections.json`, atomically. It refuses when the buffer is
empty, or when the identity gate is shut.

**Old dumps do not load.** A v4 file is refused with a migration command; v3 and earlier need two
hops, because each hop is a separate claim you are making:

```bash
# v3 -> v4: assert the board-frame convention the file was CAPTURED in
ros2 run lidar_to_camera_solver migrate_detections \
  --input old-v3.json --output mid-v4.json \
  --assume-convention corner_aligned_plate_center_v1

# v4 -> v5: assert the Target Definition it was CAPTURED against
ros2 run lidar_to_camera_solver migrate_detections \
  --input mid-v4.json --output new-v5.json \
  --target-config $(ros2 pkg prefix lctk_launch --share)/config/targets/hollow_1000_aruco_4_v1.json5
```

Doing both in one invocation is refused deliberately. The v4→v5 step checks that every marker ID the
archive observed belongs to the target you named — it catches an obviously wrong choice, but it
cannot prove which physical board produced the recording. That remains your assertion.

## Collecting evidence for W7-B

**There is no tool that reads a bag and emits a report.** `lctk_quality`'s `EvidenceCollector` is a
library with no console script, and the bag adapter is explicitly deferred to a future packet — the
spec forbids fabricating one rather than documenting and verifying the topic/message mapping first.

So treat `ros/lctk_quality/lctk_quality/evidence.py` as **the schema you record against**, and
produce the report yourself.

### Run each preset separately

Validation status is per **sensor-target preset**, not per target. Velodyne-solid and Seyond-solid
are two independent campaigns with two separate reports.

### Label your intervals

Exactly three labels exist: `visible`, `absent`, `stationary`. Record start and end times in
nanoseconds. You need, per campaign:

- a **moving** interval (`visible`) — the primary evidence, a hand-held board moved slowly through
  range, tilt and image position;
- a **board-absent** interval (`absent`) — for false detections. This doubles as your background
  warmup;
- a short **stationary** interval (`stationary`) — for pose jitter. Hand-held motion must never be
  reported as estimator jitter, so jitter is only meaningful over an interval where the board is
  genuinely still.

### Record, per campaign

- **Detection coverage** over `visible`, always as a fraction with its denominator — `accepted` and
  `rejected` out of `frames`. Never quote a bare rate.
- **False detections** over `absent` — that interval's `accepted` count *is* the false-detection
  count.
- **Jitter** over `stationary` — translation and rotation spread.
- **Quadrant continuity** — any 90-degree flip in the reported board orientation. A temporal jump
  alone is not proof; it must be confirmed against synchronised ArUco orientation in a common frame.
  **A confirmed flip blocks promotion of that preset** until the cause is found.
- **Extrinsic self-consistency** — solve from independent, non-overlapping time windows or subsets
  and compare. No shipped tool computes this; do it by capturing separate buffers and comparing the
  solved transforms.
- **Overlay check** — LiDAR-camera projection looks right (`enable_overlay=true`, published on
  `/calibration/pointcloud_overlay`).
- **The observed `pair skew max=`**, and whether your sync window needed changing.
- **The preset values you ended up with.** Tuning point-count, voxel, cluster, square-fit and
  acceptance gates per operating profile is part of the work, not a side effect.

### What does not count

- Synthetic or fabricated clouds — never field evidence, at any point.
- The historical hollow bags as an A/B baseline — different motion, duration and target size make
  them a reference, not a comparison.
- Low ICP or reprojection residual on its own. A single-capture solve can post an excellent residual
  and still be wrong; that is why continuous mode is not the evidence path.
- Invented thresholds. Do not assert a pass/fail number before real baselines exist — report what
  you measured.

### Promotion

Promotion out of EXPERIMENTAL takes a published evidence report **plus explicit operator/maintainer
sign-off**, then a small config/docs commit editing the `// EXPERIMENTAL` header in
`config/board/solid_600/<sensor>.json5`. There is deliberately no automatic promotion, and no
universal thresholds have been invented ahead of your first datasets.

## Traps

- **`enable_judge=false`.** Covered above; the default is wrong for your rig.
- **`solid_600_handheld.yaml`'s topics alias the sample data.** Its placeholder topics are byte
  identical to what `just sample-data` publishes from the *hollow* dataset. Running both together
  connects a solid Target Definition to a hollow-board recording, and **the identity gate cannot
  catch this** — both observers are told by config to expect solid-600, so they agree. The mismatch
  is physical. Confirm your actual data source before trusting any solid run. Tracked as M-27.
- **`just two-lidar` plays no data** and is hardcoded to `two_lidar.yaml`, whose topics match no
  in-repo source (M-26). Supply and remap your own.
- **Restarting observers after a target change** permanently blocks a running solver. Restart
  everything.
- **Continuous mode discards prior placements** (H-12). Use `solver_mode=manual` for anything you
  intend to trust.
- **The published transform's frame labels are inverted** (M-01). Use the dump JSON's raw
  `rvec`/`tvec` for export, not the TF topic's labels.

## If you get stuck

The fastest sanity check that the pipeline works at all uses data already in the repo:

```bash
just sample-data                                     # terminal 1
just enable_judge=false calibrate \
  $(ros2 pkg prefix lctk_launch --share)/config/examples/sample_data.yaml   # terminal 2
```

Known target, known data, no bag preparation. If that produces detections and a solve, the pipeline
is healthy and the problem is in your config or your bag. If it does not, the problem is upstream of
you — and it would be the first time anyone has run this path on real data, so say so.
