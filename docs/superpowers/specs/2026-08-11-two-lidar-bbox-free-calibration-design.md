# Two-LiDAR Calibration with the Crop-Box-Free Board Detector

Status: ready-for-agent
Date: 2026-08-11

## Problem Statement

An operator wants to calibrate the two-LiDAR rig (a spinning Velodyne VLP-32C and a
solid-state Seyond Falcon) against the recorded `TWO_LIDAR_*` bags, using the crop-box-free
(`bbox_free`) board detector that has already been proven on the LiDAR-camera pipeline.

Today this cannot be done without hand-editing shared configuration:

1. The two-LiDAR example config still points at a legacy per-sensor detector config and at the
   neutral template for the second LiDAR, so the Seyond detector runs with the wrong sensor
   up-axis and its board pose comes out rotated.
2. The per-sensor presets select `background_subtraction` as the foreground method. That method
   needs a warmup period in which the calibration board is **absent** from the scene. Every
   `TWO_LIDAR_*` bag holds the board static for its entire duration, so warmup absorbs the board
   into the background model and the detector then reports zero detections for the whole run.
   The operator's only workaround is to edit the shared presets and remember to revert them.
3. There is no way to express "this device, in this deployment, needs a different foreground
   method" — foreground method is baked into the detector config file, which is shared across
   deployments.

The operator experiences this as: the two-LiDAR pipeline starts, both detector nodes come up,
and nothing is ever detected — with no obvious knob to turn.

## Solution

From the operator's perspective:

- The two-LiDAR example config names the correct per-sensor detector preset for each LiDAR, so
  each LiDAR is interpreted with its own up-axis convention out of the box.
- Any LiDAR device in a calibration config may optionally override the foreground method for
  that deployment. The two-LiDAR example sets the override to `plane_strip` — the method that
  requires no warmup — so the static-board bags work unmodified.
- The shared per-sensor presets keep `background_subtraction` (correct for live sensors) and
  remain the single source of truth for every other parameter. No preset editing, no revert step.
- If an operator supplies an unknown foreground method, the launch fails immediately with a clear
  message naming the offending device and the accepted values, rather than starting nodes that
  silently detect nothing.

## User Stories

1. As a calibration operator, I want the two-LiDAR example config to name a per-sensor detector
   preset for each LiDAR, so that each sensor's up-axis convention is applied without me editing
   anything.
2. As a calibration operator, I want the Seyond LiDAR in the two-LiDAR rig to use the Seyond
   preset, so that its board pose is not rotated off the board surface.
3. As a calibration operator, I want the Velodyne LiDAR in the two-LiDAR rig to use the Velodyne
   preset, so that its ring-gap bridging and ICP thresholds match a spinning sensor.
4. As a calibration operator, I want to run the crop-box-free detector against a recorded
   static-board bag, so that I can validate two-LiDAR calibration without staging a live scene.
5. As a calibration operator, I want to override the foreground method per LiDAR device in the
   calibration config, so that I can pick a warmup-free method for recorded data without editing
   shared presets.
6. As a calibration operator, I want the per-device override to be optional, so that existing
   configs that omit it keep behaving exactly as they do today.
7. As a calibration operator, I want the shared per-sensor presets to keep their live-sensor
   defaults, so that a bag-specific workaround never leaks into a live calibration.
8. As a calibration operator, I want an invalid foreground method to fail at launch with a message
   naming the device and the valid values, so that I fix a typo in seconds instead of debugging
   silent non-detection.
9. As a calibration operator, I want the legacy per-sensor detector config removed, so that I
   cannot accidentally select a stale configuration that no longer matches the current schema.
10. As a calibration operator, I want removing a configuration file to not break the next build,
    so that config cleanup is a safe operation.
11. As a calibration operator, I want both board detector nodes to publish detections from the
    same bag, so that the LiDAR-to-LiDAR solver has correspondences to work with.
12. As a calibration operator, I want the LiDAR-to-LiDAR solver to publish a transform between the
    two sensor frames, so that I can inspect and export the extrinsic.
13. As a calibration operator, I want the detector's diagnostic topics to work identically in the
    two-LiDAR deployment, so that I can debug a non-detecting sensor the same way in both
    pipelines.
14. As a calibration operator, I want the reject log to tell me which gate failed and by how much
    in the two-LiDAR run, so that tuning is data-driven rather than trial and error.
15. As a calibration operator, I want the rejected-candidate diagnostic topic available per LiDAR,
    so that I can see which cluster nearly passed on each sensor independently.
16. As a developer, I want the config parser to expose the per-device foreground method to the
    launch description, so that the detector node receives it as a parameter.
17. As a developer, I want the detector node to accept a foreground-method parameter that takes
    precedence over the value in its detector config file, so that deployment beats preset.
18. As a developer, I want the precedence rule to be explicit and documented, so that a future
    reader knows which value wins.
19. As a developer, I want the override validated where the config is parsed, so that the failure
    surfaces before any node is spawned.
20. As a developer, I want the two-LiDAR example config covered by the existing config-parser
    tests, so that a future edit to that example cannot silently break the pipeline.
21. As a developer, I want the stale frame assertions in the existing two-LiDAR parity test fixed,
    so that the suite reflects the current rosbag frame naming and stops failing.
22. As a developer, I want a test asserting the per-device override reaches the generated detector
    node, so that the plumbing cannot regress.
23. As a developer, I want a test asserting an unknown foreground method raises a clear error, so
    that the validation cannot be removed unnoticed.
24. As a developer, I want a config that omits the override to produce exactly the parameters it
    produces today, so that backward compatibility is proven rather than assumed.
25. As a maintainer, I want the example config to document why the override is set for recorded
    bags, so that nobody "fixes" it back to the live-sensor default.
26. As a maintainer, I want the aruco pattern requirement for LiDAR-only markers to remain
    satisfied, so that the existing validation continues to pass while its redundancy is tracked
    separately.

## Implementation Decisions

**Scope of change.** Three modules change: the launch configuration parser, the calibration launch
description, and the LiDAR board detector node. The per-sensor detector presets and the
crop-box-free detector library are unchanged.

**Per-device override, not a new preset.** The foreground method becomes an optional per-LiDAR
field in the calibration config's device definition, alongside the existing per-LiDAR detector
config and crop-box overrides. Duplicating presets purely to flip one field was rejected: it
doubles the number of files that must be kept in step with every future tuning change.

**Precedence.** The per-device override, when present, wins over the `foreground_method` value in
the detector config file. When absent, the detector config file's value is used, and behaviour is
identical to today. This ordering is chosen so that a deployment-specific fact (this bag has no
empty frames) overrides a sensor-specific default (this sensor normally runs background
subtraction), never the reverse.

**Validation placement.** The override is validated during config parsing, against the same set of
accepted foreground methods the detector library recognises. An unknown value raises an error that
names the offending device and lists the valid values. This mirrors the existing validation that
requires a crop-box config for a hollow-board marker used by a LiDAR — same shape, same failure
point, before any node is spawned.

**Transport to the node.** The parser carries the resolved override on its LiDAR board detector
description; the launch description passes it to the node as a parameter only when set, so a config
without the override produces a byte-identical parameter set to today's.

**Two-LiDAR example config.** Each LiDAR names its own per-sensor preset (Velodyne preset for the
spinning sensor, Seyond preset for the solid-state sensor). Both devices set the foreground-method
override to the warmup-free method, with a comment stating that the recorded bags hold the board
static so background subtraction can never warm up. The legacy per-sensor detector config is
deleted; the marker's aruco pattern reference is retained because the detector still requires it.

**Build hygiene.** Deleting a config file that is installed by a glob leaves a dangling symlink in
the build tree that breaks the next build until removed. The deletion must be accompanied by
clearing that stale artifact.

## Testing Decisions

**What makes a good test here.** Tests assert externally observable behaviour of the configuration
surface: given a calibration config, what pipeline description comes out — how many detector nodes,
which detector config each one is given, which parameters reach each node, and which malformed
inputs raise errors. Tests do not assert on the parser's internal data structures beyond the
pipeline description that the launch description actually consumes, and they do not reach into the
detector library's internals.

**Single seam.** All new tests go in the existing launch-config parser test suite, which already
runs as part of the project's test command. This is the highest seam that covers the change: the
override's entire journey — config file, parsing, validation, node parameters — is observable
there. No new seam is introduced.

**Prior art.** The existing suite already contains: a two-LiDAR config test asserting detector and
solver counts and the calibration plan; a two-LiDAR node-parity test asserting node counts, frames,
and detector-to-solver topic wiring; and a validation test asserting that a hollow-board marker used
by a LiDAR without a crop-box config raises an error matching the missing field's name. The new
tests follow these three shapes directly.

**Tests to add or fix.**

- Fix the stale frame assertions in the two-LiDAR node-parity test. It currently asserts the
  pre-rename frame identifiers and fails against the current example config, which uses the frame
  identifiers recorded in the bags.
- Assert that the two-LiDAR example gives each LiDAR its own per-sensor detector config, and that
  the two differ.
- Assert that a per-device foreground-method override reaches the generated detector node's
  parameters.
- Assert that a config omitting the override produces the same parameter set as before, so
  backward compatibility is explicit.
- Assert that an unknown foreground method raises an error naming the device and the accepted
  values.

**Detector behaviour is not retested here.** The crop-box-free detector's foreground methods are
already covered by the detector library's own test suite, including its parity fixtures. Adding a
static-scene detection fixture was considered and deliberately deferred — it requires exporting new
fixture data from a gitignored bag, and the configuration plumbing is what this change actually
introduces.

## Out of Scope

- **Synchronization timing accuracy.** The restored time-synchronizer is known to deliver paired
  detections; the accuracy of its time matching is unverified and untestable with static-board data,
  where a timing error has no observable effect. Tracked separately.
- **Detector tuning for the two-LiDAR bags.** Choosing cluster, gate, and ICP values that maximise
  detection rate on these specific bags is an empirical follow-up, not part of this change.
- **Verifying the crop-box path is unchanged.** A separate review finding notes that the shared
  initial-pose construction was rewritten and the legacy crop-box path's byte-identical guarantee
  is unproven. That verification is its own task.
- **Making the aruco pattern optional for LiDAR-only markers.** The pattern's paper size is carried
  into the board model but never used by the LiDAR fit, so the requirement is redundant. Removing it
  is a separate cleanup.
- **Automatic fallback between foreground methods.** Detecting a failed warmup and switching methods
  at runtime was rejected as hidden behaviour that masks misconfiguration.
- **The camera-based pipeline.** Unaffected by this change.
- **Obtaining the recorded bags.** They are gitignored and fetched out of band.

## Further Notes

- The project has no issue tracker document, so this spec is filed under the repository's design-doc
  directory rather than published to a tracker. The `ready-for-agent` label is recorded in the
  header above.
- The two-LiDAR node-parity test is failing before this work begins. The failure predates it: the
  recorded bags renamed the sensor frames, the example config followed, and the test assertions did
  not. Fixing it is included here because the same test must be extended anyway.
- The warmup constraint is a property of the data, not a defect: background subtraction is the
  correct method for a live sensor observing a scene the board later enters. The override exists so
  that recorded captures, which cannot satisfy that precondition, remain usable.
- The two foreground methods differ substantially in cost; the warmup-free method is the slower of
  the two. This is acceptable for offline bag replay and is a further reason not to make it the
  shared default.
