# Calibration Target fixtures

`solid_600_aruco_1_v1.json5` and `hollow_1000_aruco_4_v1.json5` are source-format
variants of the launch manifests. They prove that Target Identity is semantic: comments,
whitespace, key order and metre/millimetre spellings do not affect it.

The Rust and later Python target modules consume these files as their cross-language
schema and identity fixtures.

`marker_corners_world.golden.json` freezes both targets at one shared pose. It is the
cross-language marker-ID, cell-binding, corner-order, and board-frame contract after
lengths have been normalized to integer micrometres.
