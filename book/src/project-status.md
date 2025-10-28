# Project Status

Current capabilities and future direction for LCTK.

## Current Status

**Version:** Active development (2025)
**Maturity:** Production-ready for LiDAR-camera and multi-LiDAR calibration

## Completed Features

### Core Calibration
- ✅ **LiDAR-Camera Calibration**: End-to-end pipeline with ArUco markers
- ✅ **Multi-LiDAR Calibration**: Real-time dual-LiDAR registration
- ✅ **Calibration Quality Evaluation**: IoU-based accuracy metrics
- ✅ **Point Cloud Overlay**: Real-time visualization

### ROS 2 Integration
- ✅ **Modern Node Architecture**: rclrs 0.5.x patterns
- ✅ **Message Synchronization**: Temporal alignment of multi-sensor data
- ✅ **Launch File Workflows**: Complete pipeline orchestration
- ✅ **Debug Visualization**: RViz markers and debug topics

### Detection Algorithms
- ✅ **ArUco Detection**: OpenCV-based marker detection
- ✅ **Board Detection**: RANSAC + ICP hollow board localization
- ✅ **PnP Solving**: Multiple algorithms (SQPNP, IPPE, ITERATIVE)
- ✅ **PCA-based Initialization**: Fast initial pose estimation

### Performance
- ✅ **Real-time Processing**: >10 Hz detection rates
- ✅ **GPU Acceleration**: CUDA support (optional)
- ✅ **Lock-free Updates**: arc-swap for configuration
- ✅ **Three-pass Build**: Clean dependency management

## Technology Stack

| Component | Technology | Version |
|-----------|------------|---------|
| Language | Rust | Stable |
| Middleware | ROS 2 | Humble |
| Vision | OpenCV | 4.5.4+ |
| Point Cloud | small_gicp | Latest |
| Linear Algebra | nalgebra | Latest |

## Architecture Highlights

- **Modular Design**: Core libraries separate from ROS nodes
- **Type Safety**: Rust compile-time guarantees
- **Testability**: Unit tests for all libraries
- **Configurability**: JSON5 configuration files
- **Scalability**: Multi-sensor support

## Known Limitations

- **45° tilt issue**: Corner ordering bug in pointcloud overlay (under investigation)
- **Single board target**: Multi-board detection at 33% success rate
- **Ubuntu 22.04 only**: Primary development platform

## Future Direction

### Performance Optimization
- Reduce ICP detection time to <1s
- Improve multi-board detection rate to 66%+
- Optimize GPU utilization

### New Capabilities
- Additional sensor support (thermal cameras, radar)
- Multi-target calibration
- Automated calibration workflows
- Enhanced visualization tools

### Platform Support
- Ubuntu 24.04 LTS support
- ROS 2 Jazzy migration
- Cross-platform compatibility

## Development Practices

- **Code Quality**: clippy, rustfmt, no silent errors
- **Testing**: Unit + integration + performance tests
- **Documentation**: rustdoc + user guides
- **Version Control**: Git with conventional commits
- **Community**: Open source, PRs welcome

## Getting Involved

**Ready to use:** [Installation](./user-guide/installation.md)
**Want to contribute:** [Contributing](./developer-guide/contributing.md)
**Have questions:** GitHub Discussions

## Release History

Major milestones:

- **Q1 2025**: Foundation phase - Core Rust libraries
- **Q2 2025**: ROS integration - Node implementation
- **Q3 2025**: Advanced features - Multi-sensor support
- **Q4 2025**: Performance optimization (ongoing)

---

LCTK is actively maintained and production-ready for robotics calibration workflows.
