# H-06 · CLAUDE.md documents a config schema the parser does not accept

- **Severity:** High
- **Area:** documentation ↔ lctk_launch config parser
- **Status:** Fixed (2026-07-11)
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

## Resolution (2026-07-11)

Standardized on the nested `markers.<name>.pairs` form (already used by every
example config and the parser). Fixed the CLAUDE.md "Configuration Format" example
to show `pairs:` inside the marker instead of a top-level `calibration_pairs:`
block. The parser's own docstring was already correct. Improved the planner's
"No calibration pairs defined" error to name the expected schema, so a user who
wrote the old top-level form gets a pointer to the fix.

Note: `config/README.md` still has unrelated stale references (`calib_launch`,
`board_pattern.json5`) — tracked separately under [L-08](./L-08-stale-readme-docs.md).
