# Introduction

Welcome to the LCTK (LiDAR and Camera Toolkit) Developer Guide!

## What is LCTK?

LCTK is a comprehensive toolkit for calibrating LiDAR and camera systems, primarily implemented in Rust with ROS 2 integration. It provides robust tools for:

- **LiDAR-Camera Calibration**: Accurate extrinsic calibration between LiDAR and camera sensors
- **Multi-LiDAR Calibration**: Calibration between multiple LiDAR sensors
- **ArUco Marker Detection**: Computer vision-based marker detection for calibration
- **Point Cloud Processing**: Advanced algorithms for plane fitting and registration
- **Real-time Visualization**: RViz integration for monitoring calibration processes

## Key Features

- **Modular Architecture**: Reusable Rust libraries with clean interfaces
- **ROS 2 Integration**: Full ROS 2 Humble support with standardized message types
- **High Performance**: Rust implementation for speed and memory safety
- **Flexible Workflows**: Support for both online and offline calibration
- **Comprehensive Tooling**: From data recording to visualization

## Getting Started

This guide is organized into several sections:

1. **Architecture & Design**: Understanding the system structure and design decisions
2. **Development**: Building, testing, and contributing to LCTK
3. **Roadmap**: Current progress and future plans
4. **Reference**: API documentation and troubleshooting

## Prerequisites

- ROS 2 Humble or later
- Rust toolchain
- OpenCV 4.5.4 or 4.6.0
- C++ development headers

For detailed setup instructions, see the [Build System](./development/build.md) section.