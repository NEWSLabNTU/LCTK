# LCTK Documentation

LCTK helps robotics teams estimate the transforms between LiDAR and camera sensors,
and align multiple LiDARs, using observations of a physical Calibration Target.

## Choose a task

### Run a calibration

Start with [Installation](./user-guide/installation.md), then follow the
[Quick Start](./user-guide/quickstart.md) using the included sample data. For a field
run, see [LiDAR-Camera Calibration](./user-guide/lidar-camera.md),
[Multi-LiDAR Calibration](./user-guide/multi-lidar.md), and the
[Field Validation Runbook](./user-guide/field-validation.md).

### Connect a new rig

Use [Calibration Sessions](./user-guide/sessions.md) to describe the data source and
sensors, then see [Configuration](./user-guide/configuration.md) for Target Definitions,
Detector Tuning, camera inputs, and synchronization. Use
[Troubleshooting](./user-guide/troubleshooting.md) when a session does not behave as
expected.

### Review and use results

[Assisted Capture](./user-guide/assisted-capture.md) describes automatic capture and
browser review. [Extrinsic Validation](./user-guide/extrinsic-validation.md) checks a
saved candidate against held-out observations. The
[Exporting to Autoware](./user-guide/autoware-export.md) guide covers previewing,
writing, and checking the exported transform.

## Requirements

The current source installation path uses Ubuntu 22.04 LTS and ROS 2 Humble. It clones
and builds the LCTK repository; see [Installation](./user-guide/installation.md).
Choose a Target Definition that matches the physical Calibration Target used by the rig.
The shipped examples use different targets and sensors, so no single board or sensor
setup is universal.

## Developer Guide

For architecture, building, testing, and contributing, see the
[Developer Guide](./developer-guide/architecture.md).
