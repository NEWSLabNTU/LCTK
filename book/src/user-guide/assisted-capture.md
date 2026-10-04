# Assisted Capture

Assisted mode watches synchronized LiDAR and camera observations and automatically
adds a Capture when the Calibration Target has been still long enough and represents a
new Board Placement. You review the captures in a browser before using the result.

## Run assisted mode

Validate the session and start the graph:

```bash
just check /path/to/session
just solver_mode=assisted enable_judge=false run /path/to/session
```

The `just` launch status page is at <http://localhost:8000>. Open assisted review at
<http://localhost:8080>.
For sessions with multiple LiDAR-camera pairs, the first pair uses the configured
review port and later pairs use subsequent ports; see the `review_port` setting in
[Configuration](./configuration.md).

If the sensors or recording are already publishing, start only the calibration graph:

```bash
just solver_mode=assisted enable_judge=false calibrate /path/to/session/session.yaml
```

## Capture and review

1. Make sure both sensors see the Calibration Target and that its marker sheet is
   visible to the camera.
2. Move the target through different positions and tilts, pausing briefly at each
   placement. The page reports whether it is ready to capture and how much geometric
   variety the current Detection Buffer contains.
3. Continue until the placement coverage is adequate for your rig. A long list of
   repeated observations from one position is not a substitute for varied geometry.
4. Review each Capture and the available image and fit evidence. Evidence may be
   unavailable for an individual capture; remove captures that are visibly blurred,
   occluded, or otherwise unsuitable. Removing a Capture updates the estimate and
   Quality Verdict for the remaining buffer.
5. Save the Detection Archive, and record the session and target files used to produce
   it. Set `review_archive_path` in the session before launching the graph; its default
   is empty, and the review page does not prompt for a path.

For example:

```yaml
assisted:
  review_archive_path: $(session-dir)/out/detections.json
```

To restore a saved Detection Archive, open the menu and choose **Load archive…**
directly below **Export archive…**. Enter its path on the machine running the solver;
the configured export path is offered as a starting point. Loading replaces the
current Detection Buffer and asks for confirmation when Captures already exist.
Cancel leaves the current buffer in place.

The running solver checks the archive against its Target Definition and current
CameraInfo. A successful load refreshes the Capture list, estimate, and scene;
saved Captures may have no image or cloud evidence. The action notice explains a
rejected archive or a failed review refresh. Load only archives recorded with the
same physical sensor mounting and compatible camera settings.

A numerical estimate and its Quality Verdict answer different questions. The estimate
is the transform computed from the current Detection Buffer; the verdict describes how
well those observations constrain it. Neither replaces checking the result against the
real rig and independent observations. See the [Field Validation Runbook](./field-validation.md).

## Network access

The review page has no authentication and defaults to listening on `127.0.0.1`, so it
is available only on the machine running LCTK. If you change `review_bind_host` to
allow access from another machine, anyone who can reach that port can view captures
and calibration results, load Detection Archives, and trigger configured exports.
Use only a network you trust. The setting is under the session's optional `assisted:` section; see
[Configuration](./configuration.md).

## Export

The review page can save a Detection Archive and, when configured, export to an
Autoware calibration YAML. Add these fields to the session's `assisted:` section
(alongside `review_archive_path` if you use it), using the existing target file and
the frame entries for your rig:

```yaml
assisted:
  export_autoware_target: /path/to/sensor_kit_calibration.yaml
  export_camera_frame: camera0/camera_link
  export_lidar_frame: velodyne_top_base_link
```

The LiDAR entry must already exist in the YAML. The review page previews the values
before writing and requires confirmation. See
[Exporting to Autoware](./autoware-export.md) for the file requirements and verification
steps.
