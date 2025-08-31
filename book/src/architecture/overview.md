# System Architecture

## Overview

The LCTK project is structured as a collection of Rust libraries and binaries, with a strong emphasis on ROS 2 for communication and workflow orchestration. The architecture can be broadly divided into three layers:

1. **Core Libraries (`src/lib`)**: These are the fundamental building blocks of the system, providing reusable functionalities for various tasks related to LiDAR and camera calibration.

2. **ROS 2 Nodes (`src/bin`)**: These are executable applications that use the core libraries to perform specific tasks. They communicate with each other using ROS 2 topics and services.

3. **Launch Files (`src/bin/calib_launch/launch`)**: These files define the overall workflow by launching and configuring the ROS 2 nodes in the correct sequence.

## Design Principles

- **Modularity**: Each component has a single, well-defined responsibility
- **Reusability**: Core algorithms are separated from ROS 2 dependencies
- **Type Safety**: Rust's type system ensures correctness at compile time
- **Performance**: Critical paths are optimized for real-time processing
- **Flexibility**: Components can be easily swapped or extended

## System Layers

### Core Layer
The foundation of LCTK, consisting of pure Rust libraries that implement calibration algorithms, computer vision routines, and data structures.

### Application Layer
ROS 2 nodes that wrap the core libraries and provide standardized interfaces for inter-process communication.

### Orchestration Layer
Launch files and configuration that define complete calibration workflows by composing individual nodes.

## Key Technologies

- **Rust**: Primary implementation language for safety and performance
- **ROS 2 Humble**: Middleware for distributed computing and sensor integration
- **OpenCV**: Computer vision algorithms for ArUco detection
- **PCL/small_gicp**: Point cloud processing and registration
- **nalgebra**: Linear algebra operations