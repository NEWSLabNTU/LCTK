# aruco_generator_node

A command-line tool for generating ArUco markers from configuration files.

## Overview

This tool generates ArUco marker images based on specifications provided in TOML or JSON configuration files. It supports various ArUco dictionary types and allows customization of marker size, border width, and output format.

## Requirements

- Rust 1.56 or later
- OpenCV 4.6.0

## Quick Start

```bash
# Generate markers from configuration
aruco_generator_node --config config.toml

# Generate with preview mode
aruco_generator_node --config config.toml --preview
```

## Configuration Format

Create a TOML or JSON configuration file specifying marker parameters:

```toml
# Example config.toml
dictionary = "DICT_4X4_50"
marker_size = 200
border_bits = 1
output_dir = "./markers"

[[markers]]
id = 0
filename = "marker_0.png"

[[markers]]
id = 1
filename = "marker_1.png"
```

## Command Line Options

- `--config, -c`: Path to configuration file (required)
- `--preview`: Display generated markers in preview window

## License

MIT License