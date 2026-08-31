# Calibration sessions

A **session** is one directory that describes one calibration run: where the data
comes from, and everything needed to calibrate against it. Before sessions existed
these two halves lived apart — a playback launch file that hard-coded a recording
and its topic names, and a separate calibration config that restated those topic
names by hand. When the two disagreed the pipeline still launched cleanly and
simply never detected anything. A session removes that gap by making both halves
read one file.

## What is in a session directory

```
sessions/<name>/
  session.yaml     # the manifest: data:, devices:, markers:, sync:
  README.md        # what the recording is, and whether the data ships
  bbox.json5       # optional, session-local: a crop box for this rig
  camera_info.yaml # optional, session-local: intrinsics for this camera
  rviz.rviz        # optional, session-local: the layout for this experiment
  out/             # run outputs (detection archives, exports) — gitignored
```

`session.yaml` is a normal calibration config plus a `data:` section, so anything
`calibrate.launch.py` accepts still works. Inside it, `$(session-dir)` expands to
the directory the manifest was loaded from. Use it for every session-local file:
a manifest that names no absolute path can be copied to another machine, or into
an operator's own tree, and still run. `$(find-pkg-share <pkg>)` works as before
for the shared presets under `lctk_launch/config/`.

### The `data:` section

| `kind` | Required keys | What LCTK starts |
|---|---|---|
| `pcap_avi` | `dir` — holds `lidar.pcap` and `video.avi` | the sample-data playback |
| `bag` | `path` — a rosbag2 directory with a `metadata.yaml` | `ros2 bag play` |
| `live` | none | nothing; the sensors are already publishing |

Optional under any kind: `lidar: {model, rpm}` and `camera: {info_url}`.

Which side owns the topic names follows from what is knowable:

- Under **`pcap_avi`** LCTK drives the playback, so the topics are **derived** from
  the device names (`/sensing/lidar/<name>/pointcloud_raw`,
  `/sensing/camera/<name>/image_raw`). Stating one is refused — that would put the
  name in two places again.
- Under **`bag`** and **`live`** the topic is a fact about the recording or the rig,
  so each device **states** it. Under `bag` the stated set is checked against the
  recording's `metadata.yaml` at startup, and a name the bag does not publish is
  refused with the list of names it does.

## Running a session

`session:=` is **always an explicit path** — to the directory, or to its
`session.yaml`. There is no search path and no `LCTK_SESSION_PATH`: an implicit
location would assume both where sessions live and where you are standing, and a
session may live anywhere.

```bash
ros2 launch lctk_launch session.launch.py session:=/abs/path/to/sessions/<name>
ros2 launch lctk_launch session_data.launch.py session:=/abs/path/to/<name>  # data only
```

Name lookup lives one layer up, in the justfile, which resolves a bare name
against `./sessions/` and then the installed share directory:

```bash
just sessions              # list what is available
just check <name-or-path>  # validate without launching a graph
just run <name-or-path>    # data source + calibration graph
just demo                  # alias for the shipped sample-data session
```

## Inspecting and creating sessions

`lctk_session` does all of this without `just`:

```bash
ros2 run lctk_launch lctk_session list [DIR ...]
ros2 run lctk_launch lctk_session check /path/to/session
ros2 run lctk_launch lctk_session new /path/to/new-session --from /path/to/template
```

`check` is the one worth running before a recording session. It resolves the
manifest, the data source and the whole pipeline and prints the topics and frames
each device will actually use — answering "why is nothing being detected" before
the run rather than after, without starting a graph. Every failure it reports
names the **resolved** path, never the `$(session-dir)` string that produced it.

`new` copies an existing session, skipping its `out/`, and refuses to overwrite an
existing directory. The copy is a good starting point for a new rig: edit the
device frames, topics and `data:` section, and leave the shared presets alone.
