# 0011. Sensor detection headers own solver frame binding

- **Date:** 2026-10-04
- **Status:** accepted

## Context

A session frame override changes transform labels without changing detector coordinates.
The numerical solve can remain correct while a published or archived transform is labeled
incorrectly. Sensor headers already carry the coordinate labels of those observations.

## Decision

Remove device `frame_id` from session configuration and remove configured solver frame
labels. Bind frames from ordered synchronized detection headers. Require nonempty,
distinct sensor labels and a matching CameraInfo/camera detection frame. Changed binding
invalidates derived state and advances the generation that suppresses stale publication.
Restored archives establish binding from consistent headers and projection provenance.

Bag/live sources preserve publisher headers. Session-managed `pcap_avi` publishers use the
LiDAR device name and camera device name with `_optical_frame` appended; refuse collisions.
The logical `reference_frame` device selector retains its meaning. Keep numerical TF and
Autoware direction handling in the existing owning modules.

## Consequences

Saved provenance and published transforms describe the frames actually observed by the
solver. Operators configure labels at the publisher when necessary. Header consistency
does not prove that a publisher names its coordinates correctly or that sensor mounting
remains unchanged; those are physical deployment responsibilities.
