# M-22 · Root `.cargo/config.toml` copied from one package → clean clone cannot build at all

- **Severity:** Medium
- **Area:** build tooling
- **Status:** Fixed
- **Verified:** Reproduced 2026-08-27 on a freshly cloned tree — `just build` fails during the first Rust package with `error: no matching package named 'lctk_interfaces' found`
- **Fixed:** 2026-08-27 — `0df4f48`; root patch block is now the union of every per-package block, and `aruco_generator_node` declares the interface build dependency
- **Related:** [M-18](./M-18-root-cargo-config-missing-rust-tests-unrunnable.md), [L-16](./L-16-bindgen-lock-stale-skip.md), [L-22](./L-22-advanced-solver-undeclared-lctk-interfaces-dep.md)

## Problem

M-18 made `setup/scripts/sync-root-cargo-config.sh` synthesise the workspace-root
`.cargo/config.toml` from colcon-cargo-ros2's per-package output. It took that content from a single
source, chosen as the sorted-first per-package config:

```bash
src=${sources[0]}     # always ros/aruco_generator_node/.cargo/config.toml
```

Each per-package block only contains patches for the interfaces *that package* declares, so no
per-package block is a superset of the others:

| config | has | lacks |
|---|---|---|
| `aruco_generator_node` | 11 upstream message crates | `lctk_interfaces`, `vision_msgs`, `std_srvs`, `visualization_msgs` |
| `aruco_locator_node` | 13 incl. `lctk_interfaces`, `vision_msgs` | `std_srvs`, `visualization_msgs` |
| `lidar_board_detector` | 8 incl. `std_srvs`, `visualization_msgs` | `lifecycle_msgs`, `rcl_interfaces`, … |

Sorted-first is the one config with no `lctk_interfaces` entry, so the generated root config never
had that patch. Two consequences:

**1. A clean clone could not build.** `ros/*` and `rust/*` are members of one cargo workspace
(root `Cargo.toml`), so building *any* member resolves *every* member's dependencies. Building
`aruco_generator_node` therefore has to resolve `aruco_locator_node`'s `lctk_interfaces`
dependency. Neither that package's own config nor the root config supplied the patch, so cargo
looked on crates.io and died:

```
error: no matching package named `lctk_interfaces` found
location searched: crates.io index
required by package `aruco_locator_node v0.1.1 (/home/ubuntu/LCTK/ros/aruco_locator_node)`
```

The failure names a package that is not the one being built, which sends you looking in the wrong
place. It reproduces only on a tree with no prior `build/`, which is why a warm development machine
never saw it.

**2. Root-level cargo was quietly incomplete.** `cargo nextest`, `clippy` and `audit` run from the
repo root — exactly what M-18 existed to enable — were resolving without the `lctk_interfaces`
patch the whole time.

A second, independent ordering hazard sits behind the same failure: `colcon` had no declared edge
from `aruco_generator_node` to `lctk_interfaces`, so it scheduled them in parallel and the bindings
could still be absent when cargo ran. This is the same class as [L-22](./L-22-advanced-solver-undeclared-lctk-interfaces-dep.md),
but here the dependency is on the *cargo workspace resolution*, not on an import in the package's
own code.

## Fix

`sync-root-cargo-config.sh` now unions the `[patch.crates-io]` entries across every
`ros/*/.cargo/config.toml`, keyed by crate name, and refuses when two configs give one crate
different paths rather than silently picking a winner. `[build]` and `[env]` still come from one
deterministic source, since `target-dir`, `rustflags` and `AMENT_PREFIX_PATH` are per-package
absolute values that cannot be merged meaningfully.

`ros/aruco_generator_node/package.xml` gained `<build_depend>lctk_interfaces</build_depend>` with a
comment explaining why a package whose own code uses none of those messages still needs the bindings
generated first.

## Verification

Fresh clone, submodules initialised, no `build/` or `install/`: `just build` completes all 17 ROS
packages, and `just test` runs 317 Rust and 337 Python tests from the repo root.
