# Quick Start

First complete [Installation](./installation.md). This quick start runs LCTK against
the included sample recording.

From the repository root, validate and run the sample session:

```bash
just check sample3-hollow-velodyne
just demo
```

The check confirms that the session files and configuration are available. The demo
plays the recorded LiDAR and camera data and launches the calibration graph. While it
runs, the launch status page is at <http://localhost:8000> and assisted capture review
is at <http://localhost:8080>. The demo enables RViz by default. If you are running
headless, disable it explicitly:

```bash
just rviz_enabled=false demo
```

This sample contains one Board Placement. Use it to see the data and detections flow;
do not treat its estimate as a multi-pose, field-validated calibration.

Press Ctrl+C to stop the run. To use your own recording or sensors, start with
[Calibration Sessions](./sessions.md), then follow the relevant
[LiDAR-camera](./lidar-camera.md) or [multi-LiDAR](./multi-lidar.md) workflow.
