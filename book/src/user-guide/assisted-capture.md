# Assisted Capture

Assisted capture is the third `solver_mode` on `lidar_to_camera_solver`. It watches the
board detections, queues a pair by itself whenever the board is held still in a placement it
has not seen before, and serves a web page for reviewing what it captured.

`continuous` and `manual` are unchanged and still selectable — `solver_mode` remains the
switch, so you can run the older paths for comparison at any time.

## Why the mode exists

Capturing a multi-pose calibration by hand is three jobs done at once by one person: hold the
board still, decide by eye that it is still enough and different enough from what you already
have, and reach for a keyboard.

The middle job is the expensive one, and until now the tooling gave no help with it. The TUI
shows a buffer count and a pose table; it never shows the image the corners were measured in.
So when a capture turned out to be bad — motion blur, a partly occluded marker, glare across
the plate — nothing in the pipeline could tell you why, because **no solver subscribed to an
image at all**.

Assisted mode moves the two mechanical judgements into the node and the one real judgement
into a browser.

## Running it

Assisted mode is `solver_mode:=assisted` on any [session](./sessions.md):

```bash
ros2 launch lctk_launch session.launch.py \
    session:=/path/to/sessions/seyond-left \
    solver_mode:=assisted
```

Then open <http://localhost:8080>.

If the data is already flowing — a live rig, or a bag you are playing yourself — start only
the calibration half:

```bash
ros2 launch lctk_launch calibrate.launch.py \
    config_file:=/path/to/sessions/seyond-left/session.yaml \
    solver_mode:=assisted
```

The `just` shorthand resolves a bare session name and fills in `solver_mode:=assisted`:

```bash
just assisted                          # defaults to the seyond-left session
just assisted solid600-handheld-zed
just solver_mode=assisted run <session>   # the same thing, spelled generally
```

The review archive is written where the session's `assisted.review_archive_path` says,
conventionally `$(session-dir)/out/detections.json`.

## The workflow

1. **Start the pipeline and open the page.** It shows a stillness banner, a diversity meter,
   the queue, and the current solve.
2. **Walk the board around the scene.** Hold each pose for about a second. The banner turns
   green and the pair appears in the queue on its own; your hands never leave the board.
3. **Watch the diversity meter, not the residual.** It reads `placements`, `normal span`,
   `depth range` and `lateral span` against the collection targets, and prints what is
   missing in plain words — *"board normals span only 14 deg (aim for 20+); vary the board's
   yaw and pitch"*.
4. **Stop when the meter is satisfied**, not when the queue looks long.
5. **Review the queue.** Each row shows the frame the pair was measured in, with the detected
   ArUco corners drawn on and corner 0 marked, plus that pair's reprojection RMS. Rows are
   sorted worst-first. Drop anything blurred, occluded or glared; dropping re-solves
   immediately.
6. **Export.** *Export archive* writes the version-5 `detections.json`. *Export to Autoware*
   shows the diff first and writes only on a second click.

## The two gates

A pair is queued only if it passes both.

**Stillness** — the board's pose must stay within `stability_max_translation_m` and
`stability_max_rotation_deg` across `stability_window_frames` consecutive synchronized pairs.

The gate measures the **span across the window**, not the frame-to-frame delta. A board
drifting steadily at 1 mm per frame has a negligible per-frame delta and is plainly not
still; only the span sees it. Getting that wrong would auto-capture exactly the slow drift
you would then have to find by hand in review.

**Novelty** — the pose must form a new placement under
`lctk_quality.distinct_placements`, by default 5 cm and 5°.

This gate matters more than it looks. Measured on a real field capture, reprojection RMSE and
subset resampling both *invert*: a degenerate capture — one placement filmed nine times —
scores **better** on both, and reports a confident ±0.22° / ±9 mm. Only placement diversity
separates a good capture from a degenerate one. An auto-queueing loop without this gate would
manufacture that degenerate capture, and every quality number on the page would applaud it.

That is why the diversity meter is the prominent thing on the page and the residual is not.

## Configuration

All of these are optional; the defaults below apply when the config file has no `assisted:`
section.

```yaml
assisted:
  # Stillness gate
  stability_window_frames: 10        # consecutive synced pairs the pose must hold
  stability_max_translation_m: 0.005 # translation span allowed across the window
  stability_max_rotation_deg: 0.5    # rotation span allowed across the window
  stability_cooldown_s: 1.0          # minimum gap between two auto-captures

  # Novelty gate
  novelty_position_tol_m: 0.05       # defaults to lctk_quality's own tolerances
  novelty_orientation_tol_deg: 5.0

  # Review server
  review_bind_host: "127.0.0.1"      # see the warning below before changing this
  review_port: 8080                  # first pair; see the note below for multi-pair rigs
  review_jpeg_quality: 80
  review_max_previews: 64
  review_archive_path: ""            # where "Export archive" writes

  # Autoware export -- all three required before the Autoware button works
  export_autoware_target: ""         # path to sensor_kit_calibration.yaml
  export_camera_frame: ""            # e.g. camera0/camera_link
  export_lidar_frame: ""             # e.g. velodyne_top_base_link
```

On a rig with more than one LiDAR-camera pair, each pair gets its own solver and therefore
its own review server. The launch file offsets `review_port` by the pair's index — the first
pair keeps the configured port, the second gets `+1`, and so on — because the server binds
its port eagerly and two solvers sharing one would leave the second dead at startup, after
the graph had already reported itself launched.

If the board is being rejected, the banner says which gate refused it and by how much, so
tune from the reported number rather than by guesswork. A board that reads "held still" but
never queues is being refused by the novelty gate — move it somewhere new.

## Security

**The review server has no authentication.** Anyone who can reach the port can read the
queue, the camera previews and the solved extrinsic, and can trigger an export that writes
`sensor_kit_calibration.yaml` on the host.

The only protection is the bind address. It defaults to `127.0.0.1`, so by default the page
is reachable only from the machine running the node. Setting `review_bind_host` to anything
else opens it to whatever network the rig is attached to; the node logs a warning naming the
exposure when you do. That is the level of protection being claimed — it is a field tool on a
rig network, not a service.

## Relationship to the other modes

| mode | what it does | when |
|------|--------------|------|
| `continuous` | solves and publishes from each latest pair | quick visual checks; under-constrained by construction |
| `manual` | service-driven multi-pose buffer, driven by the TUI | full manual control |
| `assisted` | auto-captures still, novel poses; browser review | normal multi-pose capture sessions |

Assisted mode also creates the manual services, so `just extrinsic-solver-controller` still
attaches to it if you want the TUI alongside the page.

## Exporting

The archive is the version-5 `detections.json` described in
[Exporting to Autoware](./autoware-export.md) — the kept pairs, the solved transform, the
quality report and the full Target Identity.

The Autoware export is deliberately two clicks. That file reaches a vehicle, so the page
shows the entry it would write before writing anything, and the existing `.bak` behaviour is
kept. Changing the queue after previewing invalidates the confirmation: the diff you were
shown described a different calibration, so you are asked to preview again.
