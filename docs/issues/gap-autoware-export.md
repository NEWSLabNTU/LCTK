# GAP · No export path from a calibration result to Autoware

- **Severity:** High (headline gap)
- **Area:** output / integration
- **Status:** Open
- **Verified:** Static review (whole-repo grep for "autoware", "sensor_kit", "static_transform" — zero hits)
- **Related:** [M-01](./M-01-transform-direction-inverted.md), [M-02](./M-02-radians-degrees-mix.md), [L-09](./L-09-setup-fragility-export-labeling.md)

## Problem

LCTK produces a calibration result only as:
1. a `geometry_msgs/TransformStamped` on a topic (and optionally `/tf_static` via `tf_tree_broadcaster`), and
2. a formatted **log line** with translation (m) + RPY (deg) + quaternion.

Nothing is written to disk automatically. The `dump_detections` / interactive-controller "Save" produces a solver **re-load** JSON (raw rvec/tvec correspondences), not a usable extrinsic file. The word "autoware" appears nowhere in the repo, and there is no export tool, template, or documentation for the last mile.

## What a user must do manually today

1. Run the pipeline and watch the terminal / `ros2 topic echo` the transform.
2. Hand-copy translation and quaternion (or the degrees-RPY line) out of the console.
3. **Invert the transform direction** — the published frame labels are backwards vs ROS TF (see M-01).
4. **Convert quaternion → roll/pitch/yaw (radians)** because Autoware `sensor_kit_calibration.yaml` uses `x,y,z,roll,pitch,yaw`.
5. Hand-write the values into the Autoware sensor-kit YAML. No template, no generator, no example.

## Suggested fix

Ship an exporter (service or CLI) that writes an Autoware-shaped `sensor_kit_calibration.yaml` snippet: correct transform direction (parent = base/sensor-kit frame), meters, radians RPY, with the parent/child frames taken from the config. Document it as an "Exporting Results" page in the book.
