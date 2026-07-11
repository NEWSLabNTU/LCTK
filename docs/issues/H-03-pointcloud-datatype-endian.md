# H-03 · Point cloud XYZ decoded as little-endian FLOAT32 without checking datatype or endianness

- **Severity:** High
- **Area:** lidar_board_detector
- **Status:** Open
- **Verified:** Static review
- **Location:** `ros/lidar_board_detector/src/main.rs:1537-1587` (`convert_pointcloud2_to_points`)

## Problem

The converter reads x/y/z with `read_f32_le` unconditionally and never inspects `field.datatype` or `msg.is_bigendian`. It assumes every producer uses little-endian FLOAT32 XYZ.

## Failure scenario

A LiDAR driver that publishes FLOAT64 XYZ (`datatype == 8`) or a big-endian producer causes every point to be decoded from the wrong bytes. Result: garbage plane fit and ICP → a wrong-but-plausible calibration, with no error surfaced to the user.

## Suggested fix

Read `x`/`y`/`z` field offsets and datatypes from `msg.fields`, honor `msg.is_bigendian`, and support at least FLOAT32 and FLOAT64. Error clearly if an unsupported datatype is encountered.
