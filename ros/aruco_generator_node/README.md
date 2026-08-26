# aruco_generator_node

A command-line tool for generating the exact fiducial paper in a Target Definition.

## Overview

This tool derives dictionary, marker IDs, paper size, quiet zone, marker size and layout from a
validated Target Definition. The target manifest is the physical source of truth; DPI only chooses
raster resolution.

## Requirements

- Rust 1.56 or later
- OpenCV 4.6.0

## Quick Start

```bash
# Generate the solid target's 600 mm paper
aruco_generator_node \
  --target-config ros/lctk_launch/config/targets/solid_600_aruco_1_v1.json5 \
  --output solid_600.png --dpi 300

# Generate with preview mode
aruco_generator_node --target-config target.json5 --output target.png --preview
```

## Command Line Options

- `--target-config`: Path to a Target Definition JSON5 file (required)
- `--output, -o`: Output image path (required)
- `--dpi`: Raster resolution (default: 300); it does not change physical geometry
- `--preview`: Display generated markers in preview window

## License

MIT License
