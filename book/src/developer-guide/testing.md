# Testing

Use the repository recipes so tests run with the ROS 2 and Cargo configuration the
workspace expects. Run commands from the repository root unless noted otherwise.

## Build before testing

After [setting up the development environment](./build-system.md), build the workspace
and source its installed packages:

```bash
just build
source install/setup.bash
```

## Run the test suites

```bash
just test
```

This runs Rust workspace tests with Cargo Nextest and the Python test suites registered
in the root `justfile`, including launch/session, synchronization, both solvers,
quality, target loading, export, judge, and setup checks. The recipe also checks that
Rust test targets can be collected before reporting a test result.

Rust tests live in each crate's `tests/` directory and inline `#[cfg(test)]` modules.
Python tests live under the relevant ROS package's `test/` directory. The repository
uses `just test` as the complete entry point rather than running `cargo test` or
`colcon test` against an assumed package layout.

## Exercise shipped sessions

`just test` does not replay every sample through the complete ROS graph. For the
slower launch/data integration checks, run:

```bash
just smoke
```

This runs the session smoke tests under `ros/lctk_launch/smoke/`; it requires a built
and sourced workspace and takes longer than the unit and package tests.

## Lint and documentation checks

```bash
just lint-rust  # nightly rustfmt check and clippy
just lint-py    # ruff checks and Python formatting check
just lint       # both Rust and Python lint checks
```

For documentation changes, build the mdBook from `book/`:

```bash
cd book
just build
```

`just check-docs` checks relative Markdown links in `docs/`, `book/src/`, and selected
repository README/CONTRIBUTING files. It strips anchors without validating that their
headings exist, so it is not a substitute for building the book.

## When a test or recipe changes

Keep each test reachable through the documented recipe, and verify that a deliberately
failing assertion makes that recipe exit non-zero before trusting newly wired test
coverage. The repository's root `AGENTS.md` records known pitfalls in the build and
test entry points.
