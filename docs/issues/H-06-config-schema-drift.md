# H-06 · CLAUDE.md documents a config schema the parser does not accept

- **Severity:** High
- **Area:** documentation ↔ lctk_launch config parser
- **Status:** Open
- **Verified:** Static review
- **Location:**
  - `ros/lctk_launch/lctk_launch/config_parser.py:217-232` (reads `markers.<name>.pairs` only)
  - `CLAUDE.md` (Config-Driven Calibration section) and `config_parser.py:7` docstring
  - `ros/lctk_launch/lctk_launch/calibration_planner.py:126-127` ("No calibration pairs defined")

## Problem

CLAUDE.md (and the parser's own module docstring) describe a **top-level** `calibration_pairs:` block with `devices: [a, b]` / `marker:` keys. The actual parser only reads pairs **nested inside each marker** as `pairs:`. There is no code path that reads a top-level `calibration_pairs`.

## Failure scenario

A user copies the documented example verbatim. The config parses with zero pairs, then the planner raises "No calibration pairs defined". A first-run footgun that contradicts the primary documentation.

## Suggested fix

Pick one schema. Either support the documented top-level `calibration_pairs:` in the parser, or update CLAUDE.md, the docstring, and `config/README.md` to the nested `markers.<name>.pairs` form used by the example configs. Add a validation error that names the expected key when no pairs are found.
