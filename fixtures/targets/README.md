# Calibration Target fixtures

`solid_600_aruco_1_v1.json5` and `hollow_1000_aruco_4_v1.json5` are source-format
variants of the launch manifests. They prove that Target Identity is semantic: comments,
whitespace, key order and metre/millimetre spellings do not affect it.

The Rust and later Python target modules consume these files as their cross-language
schema and identity fixtures.

`marker_corners_world.golden.json` freezes both targets at one shared pose. It is the
cross-language marker-ID, cell-binding, corner-order, and board-frame contract after
lengths have been normalized to integer micrometres. Its consumers are
`rust/calibration-target/tests/geometry_contract.rs`, `ros/lctk_target/test/test_target.py`,
and `ros/lidar_to_camera_solver/test/test_marker_corners_world_golden.py`.

**Do not re-baseline it from implementation output** -- generating it from
`calibration-target` or `lctk_target` and overwriting the golden with the result would
make the contract check nothing: every consumer would agree with an implementation bug
that all three happened to share.

`generate_marker_corners_world.py` is how the golden is re-derived instead. It computes
every world coordinate from each manifest's stated geometry (plate side, paper
placement, cell layout, marker IDs, read as plain JSON5 data) and a stated physical
mounting, following the plate's documented corner-aligned frame convention from first
principles. It does not import or call the Rust crate or the Python target module --
see its module docstring for the full physical derivation. Run it and diff its output
against the committed file by hand:

```bash
python3 fixtures/targets/generate_marker_corners_world.py > /tmp/candidate.json
# then compare /tmp/candidate.json to marker_corners_world.golden.json numerically
# (exact JSON formatting need not match; corner coordinates should agree to float64
# precision, i.e. worst-case deviations on the order of 1e-15 m)
```

Regenerate only when a manifest's physical geometry changes (plate side, paper
placement, cell layout, or marker IDs) -- never to make a failing test pass. A
disagreement between the script's output and the committed golden is a finding about
one of them, not something to paper over by re-baselining.
