# Introduction

## What is LCTK?

LCTK (LiDAR and Camera Toolkit) helps you calibrate LiDAR and camera sensors for robotics and autonomous systems. It computes the precise transformation (position and orientation) between sensors by detecting a calibration board.

## When to Use LCTK

Use LCTK when you need to:
- **Fuse LiDAR and camera data** for perception systems
- **Calibrate multi-LiDAR setups** on mobile robots or infrastructure
- **Validate sensor alignment** after installation or maintenance
- **Overlay point clouds on camera images** for visualization

## What You'll Need

**Hardware:**
- 1m × 1m calibration board with 4 circular holes (150mm radius each)
- LiDAR sensor(s): Velodyne or similar
- Camera with known intrinsics (calibration file)

**Software:**
- Ubuntu 22.04 LTS
- ROS 2 Humble
- ~10 minutes to set up

## How This Guide Works

**Part 1: User Guide** — Get calibration results quickly
- Installation and quick start
- Step-by-step calibration workflows
- Configuration and troubleshooting

**Part 2: Developer Guide** — Extend and customize LCTK
- Architecture and design
- Build system and testing
- Contributing code

Start with [Installation](./installation.md) to set up your system.
