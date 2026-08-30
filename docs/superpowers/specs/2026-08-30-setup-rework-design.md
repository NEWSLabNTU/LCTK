# Setup rework: `./setup.sh` as the entry point, content-addressed steps, selectable TUI

- **Status:** Implemented 2026-08-30 on `feat/bbox-free-parity-validation`
- **Date:** 2026-08-30
- **Supersedes the shape of:** `setup.sh`, `setup/justfile`, `setup/scripts/*`
- **Related:** [L-25](../../issues/L-25-fresh-machine-bringup-deps-missing.md), L-09 (version pinning)

## Why

Bringing this branch up on a machine that had already "completed" setup cost four
separate build failures, each discovered only by running `just build` and reading a
compiler error. The failures were not exotic:

| symptom | root cause |
|---|---|
| `no matching package named vision_msgs found` | `ros-humble-vision-msgs` never installed |
| `ModuleNotFoundError: No module named 'json5'` | `python3-json5` never installed |
| `fatal error: SFCGAL/capi/sfcgal_c.h: No such file` | `libsfcgal-dev` never installed |
| `no fixtures found — run export_golden.py` | `uv` not installed by any step |

Every one of these is covered by an existing setup step. The steps had all been run.

### The core defect: markers record that a step *ran*, not that it *worked*

`setup/justfile`'s `_run` touches `setup/.markers/<name>` after a script exits 0, and
skips the step forever after. The marker is keyed on the step **name**, so it says
nothing about what that step installed. Two consequences, both observed on this
machine:

- `setup/.markers/geometric-libs` existed while `/usr/include/SFCGAL/capi/sfcgal_c.h`
  did not. `libsfcgal-dev` was added to the script after the marker was written, and
  the marker made the step unreachable.
- `setup/.markers/ros-deps` existed while four apt packages named by `package.xml`
  were missing. `ros-deps`' input is every `ros/*/package.xml` in the tree — a set that
  changes on most feature branches — so a durable marker on it is always wrong.

This is the same failure class CLAUDE.md's Testing Practices section describes: a step
that cannot fail converts "not installed" into "believed installed", which is the more
expensive state.

### The other four defects

1. **`./setup.sh` cannot bootstrap.** It exits if `just` is absent and tells you to run
   `cargo install just` — but `cargo` comes from the `rust` step, inside the setup it
   just refused to run. `just` has no apt package on jammy. A fresh machine is stuck.
2. **The wizard covers 3 of 15 steps.** Only submodules, CUDA and dev-tools are asked
   about. The other twelve are forced, in a fixed order, with no way to select a subset.
3. **A step can silently do nothing and still mark itself done.**
   `install-dev-tools.sh` ends with `echo "Warning: cargo not found, skipping mdbook"`
   and exits 0, so `_run` writes the marker. `mdbook` is then permanently unreachable
   and `just dev-tools` reports `[done]`. `dev-tools` declares `needs system-base`, not
   `needs rust`, so this is reachable, not theoretical.
4. **`status` reads markers, never reality.** It is a view of the same untrustworthy
   state, so it cannot catch any of the above.

## Target shape

### 1. `./setup.sh` is the entry point; `just` stays the engine

`setup.sh` is the single documented entry point. It gates on `just` and, when absent,
prints the exact pinned install command and offers to run it — see
[Bootstrapping `just`](#bootstrapping-just). Everything after that gate is driven by
`setup/justfile` as today, so `just -f setup/justfile <step>` keeps working unchanged.

The step table, marker logic and verifiers move into `setup/steps.py` (stdlib only,
Python 3.10 — no `tomllib` on jammy, so the table is a Python data module rather than
TOML). `justfile` recipes become one-line delegations to it, which keeps `just` as the
interface while giving the TUI and the marker engine a single source of truth to read.

```
./setup.sh                 # TUI selector, then install
./setup.sh --yes           # non-interactive, recommended defaults
./setup.sh --only rust,ros2
./setup.sh --skip cuda
./setup.sh --status        # what is installed, verified against reality
./setup.sh --verify        # run every verifier, install nothing
./setup.sh --dry-run       # print the plan
./setup.sh --log FILE      # tee a full transcript
```

`setup/justfile` stays, reduced to a thin alias layer (`just -f setup/justfile rust`
→ `./setup.sh --only rust`) so existing muscle memory and docs keep working. The step
table lives in exactly one place.

> **Decision to confirm.** The alternative is for `setup.sh` to bootstrap `just` and
> keep delegating to the justfile as today. I recommend against it: the marker engine
> is ~15 lines, and moving it out of `just` removes the bootstrap dependency outright
> rather than papering over it. Say the word if you would rather keep `just` as the
> setup engine.

### 1a. Bootstrapping `just`

`just` has no apt package on jammy and `cargo install just` needs the `rust` step that
setup has not run yet, so the gate installs a prebuilt binary:

```bash
JUST_VERSION="${JUST_VERSION:-1.58.0}"
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh \
  | bash -s -- --tag "$JUST_VERSION" --to ~/.local/bin
```

- The upstream installer resolves `x86_64-unknown-linux-musl` and
  `aarch64-unknown-linux-musl`, so one command covers x86_64 and the Jetson hosts, with
  no glibc or toolchain dependency. It needs only `curl` and `tar`.
- `--tag` is pinned per L-09 with a `JUST_VERSION` override, matching
  `CARGO_NEXTEST_VERSION` and friends.
- `--to ~/.local/bin` overrides the installer's `~/bin` default so `just` lands beside
  `ruff` and `uv`.
- Ubuntu's `~/.profile` adds `~/.local/bin` to `PATH` **only if the directory existed at
  login**, so the gate creates it first and tells the user to re-login or export `PATH`
  for the current shell. Without that, `just` installs and still looks missing.

Air-gapped fallback, no script:

```bash
mkdir -p ~/.local/bin
curl -fsSL https://github.com/casey/just/releases/download/1.58.0/just-1.58.0-$(uname -m)-unknown-linux-musl.tar.gz \
  | tar -xz -C ~/.local/bin just
```

Rejected: `cargo install just` (needs cargo, the circular case), `apt` (no jammy
package), `snap` (third-party publisher, classic confinement, snapd absent on many
server and Jetson images).

### 2. Steps are content-addressed and verified

Each step declares metadata in one table:

```python
Step(
    id="geometric-libs",
    title="SFCGAL geometry library",
    group="Media & sensors",
    needs=["system-base"],
    sudo=True,
    size_mb=40,
    script="install-geometric-libs.sh",
    verify="test -f /usr/include/SFCGAL/capi/sfcgal_c.h",
    why="sfcgal-sys builds aruco_locator_node's newslab-geom-algo dependency",
)
```

Two changes do the real work:

**Markers become content hashes.** The marker records
`sha256(script) + the step's pinned versions`. Edit `install-geometric-libs.sh` to add a
package and the marker invalidates itself on the next run. This alone would have
prevented the SFCGAL and vision_msgs failures.

**Every step carries a `verify`.** It runs after install, and on `--status` /
`--verify` without installing. A step whose script exits 0 but fails verification is a
hard error — never a marker. This closes the `dev-tools`-skips-mdbook hole, because
`verify` is `command -v mdbook`, not "the script printed a warning and exited 0".

Steps whose input is the working tree are declared `cache=never` and re-run every
time: `ros-deps` (reads every `package.xml`) and `submodules`. They are fast and
idempotent; caching them is what made them wrong.

### 3. Scrollable TUI with nested sub-options

Stdlib `curses` — no pip install, so it works on the bare machine the script is meant
to bootstrap. (`whiptail` is present on most Ubuntu images and was considered, but its
checklist cannot nest sub-options or show live per-step state.)

```
 LCTK setup                                    space toggle · → expand · ⏎ install · q quit

 [x] Core toolchain                                                          required
   [x] system-base              apt   ~50 MB                            ✓ installed
   [x] build-tools              apt  ~400 MB                            ✓ installed
   [x] python                   apt  ~200 MB                            ✓ installed
 [x] ROS 2 Humble                                                            required
   [x] ros2 (desktop + drivers) apt  ~2.5 GB                            ✓ installed
   [x] rosdep init + workspace deps                                     ! always re-runs
 [x] Rust
   [x] rustup + rustfmt/clippy       ~1 GB                              ✓ installed
   [x] cargo-ament-build        0.1.11                                  ✓ installed
   [x] cargo-nextest            0.9.137                                 ✓ installed
   [x] just                     1.57.0                                  ✓ installed
   [x] colcon-cargo-ros2        >=0.5.3                                 ✓ 0.5.3
 [x] Media & sensors
   [x] opencv                   apt  ~500 MB                            ✓ installed
   [ ] opencv 4.5 private prefix     JetPack hosts only          – not needed (x86_64)
   [x] gstreamer                apt  ~300 MB                            ✓ installed
   [x] network-libs (libpcap)   apt                                     ✓ installed
   [x] geometric-libs (SFCGAL)  apt   ~40 MB                            ✗ header missing
 [x] Test & lint tooling                                                          NEW
   [x] ruff                     0.16.3                                  ✓ installed
   [x] uv                       0.12.5                                  ✓ installed
 [ ] Optional
   [ ] cuda                          ~3 GB                              – skipped
   [x] dev-tools
     [x] debuggers & profilers  apt  ~500 MB                            ✓ installed
     [x] docs (mdbook)               needs rust                         ✗ missing
 [ ] Repo
   [ ] submodules                    discards local changes             ! ros/conflux dirty

 6 steps to install · ~1.1 GB · sudo required          ⏎ install   v verify only   q quit
```

Checking a parent expands its children; unchecking one skips the subtree. The right
column is live `verify` output, so the screen shows what is actually on the machine
rather than what a marker claims. `–` marks a step the host does not need (the
JetPack-only OpenCV prefix on x86_64).

### 4. Step reorganisation

| step | change |
|---|---|
| `python` | **dependency fix** — currently `needs ros2`, which forces a 2.5 GB ROS install before apt python. Should need only `system-base`. |
| `dev-tools` | **split** into `dev-tools-debug` (apt) and `dev-tools-docs` (mdbook, `needs rust`). Removes the silent-skip path and lets you take debuggers without a Rust build. |
| `rust` | `just` extracted into its own pinned step so `setup.sh` can install it before anything else needs it. |
| `network-libs` | `wireshark-common` prompts via debconf about non-root `dumpcap`; needs `DEBIAN_FRONTEND=noninteractive` or it hangs `--yes`. |
| `ros-deps` | `cache=never`. |
| `lint-tools` | **new** — `ruff` and `uv`, both pinned, both self-contained binaries with no Python deps (so they cannot drag in the setuptools/numpy/scipy that Known Issue 3 warns about). Closes L-25 #2 and #3. |
| `python-guard` | **new** — the pip-shadowing check currently smeared across the tails of `install-python.sh` and `install-colcon-rust.sh`, hoisted into one step that runs after every pip-installing step and is callable standalone. Same check as `just build`'s `_check-python-env`. |
| `verify` | **new** terminal step — runs every verifier and reports a single pass/fail, so setup ends by proving the machine is ready instead of asserting it. |

### 5. Version pinning completed (L-09)

`mdbook`, `mdbook-mermaid`, `ruff`, `uv` and `just` are currently unpinned or absent.
All get the established `NAME_VERSION="${NAME_VERSION:-x.y.z}"` env-override form
already used by `CARGO_AMENT_BUILD_VERSION` and `CARGO_NEXTEST_VERSION`.

## What this would have caught

Replaying this session's four failures against the target design:

| failure | caught by |
|---|---|
| `vision_msgs` missing | `ros-deps` is `cache=never`; runs on every setup |
| `python3-json5` missing | same |
| `libsfcgal-dev` missing | content-hashed marker invalidates when the script gains a package; `verify` checks the header |
| `uv` missing | new `lint-tools` step |
| (latent) `mdbook` silently skipped | `verify: command -v mdbook` fails the step instead of marking it done |

## Decisions

1. **Engine ownership** — **decided 2026-08-30: `just` stays the engine.** `setup.sh`
   gates on it and, when it is missing, offers to install a pinned prebuilt binary (see
   [Bootstrapping `just`](#bootstrapping-just)). The recommendation to replace it was
   not taken; the appendix below is kept as the record of what was weighed.
2. **TUI substrate** — **decided 2026-08-30: stdlib `curses`.**
3. **Scope of `verify`** — **decided 2026-08-30: cheap existence checks only**
   (`test -f`, `command -v`, `dpkg -s`). No `--verify=deep` build smoke test.

## Non-goals

- Supporting distributions other than Ubuntu 22.04. The warning-and-continue prompt for
  other versions stays as is.
- Replacing the root `justfile`. Only `setup/justfile` is in scope.
- Any change to `just build` / `just test`.

## Appendix: what "the driver replaces `just`" means

### What is actually being moved

`setup/` is 200 lines of `justfile` orchestration plus 588 lines of `install-*.sh` doing
the real work. **The install scripts do not change.** They keep their names, their
contents, and their standalone-runnable shape (`./setup/scripts/install-ros2.sh` works
today and still works after). Only the 200-line orchestration layer moves.

That layer does exactly seven things:

| # | `setup/justfile` provides | driver equivalent |
|---|---|---|
| 1 | step list — 15 recipes, each a one-line `@just _run <name> <script>` | rows in the `Step(...)` table |
| 2 | dependency edges (`build-tools: _init system-base`) | `needs=[...]` on each row |
| 3 | topological ordering | ~20 lines of toposort over `needs` |
| 4 | marker skip (`_run`) | the same check, now content-hashed |
| 5 | `status`, `clean-marker`, `clean-markers` | `--status`, `--only`, `--clean` |
| 6 | conditional steps (`_setup-cuda` et al, driven by 3 env vars) | deleted — the TUI selects steps directly |
| 7 | discovery (`just --list`) | `--help` / the TUI itself |

Row 6 is worth calling out: the current design needs three private wrapper recipes
(`_setup-submodules`, `_setup-cuda`, `_setup-dev-tools`) and three exported env vars
purely because a `justfile` cannot take a runtime-selected set of targets. With a step
table, selection is a set of ids and those wrappers stop existing.

### What the user types

Unchanged, because `setup/justfile` survives as an alias shim:

```
just -f setup/justfile ros2     ->  ./setup.sh --only ros2
just -f setup/justfile status   ->  ./setup.sh --status
```

The shim is a generated one-liner per step. Docs, muscle memory and
`CLAUDE.md` references keep working.

### Why this rather than bootstrapping `just`

The bootstrap path is not hypothetical hardening — it is the case that is broken today.
A fresh Ubuntu 22.04 machine has no `just` (no jammy apt package) and no `cargo`, so
`./setup.sh` stops at its own `command -v just` check and refers the user to
`cargo install just`, which the setup it just declined to run is what installs. Keeping
`just` as the engine means `setup.sh` must fetch and install a binary *before* it can
read its own step list — so the entry point ends up with two install mechanisms, one
hard-coded outside the step table and one inside it.

Making `just` an ordinary pinned step collapses that: the driver reads its step table
with nothing but bash and stdlib python3, and `just` gets installed by the same
verified, content-hashed machinery as `cargo-nextest`.

### The cost, stated plainly

This repo runs on `just` everywhere else, and this adds a second orchestration idiom in
one directory. That is a real cost, and it is the honest argument for the alternative.
It is bounded by the alias shim (nobody has to learn the new entry point to keep
working) and by the fact that setup is the one place where "requires a tool you cannot
install yet" is a contradiction rather than an inconvenience.

The root `justfile` — `just build`, `just test`, `just lint` — is untouched and stays
the primary interface for everything after setup.


## As built (2026-08-30)

| file | role |
|---|---|
| `setup.sh` -> `setup/setup.sh` | entry point: `just` gate, selector, plan, run, verify |
| `setup/steps.py` | step table + engine (resolve, markers, verify, run, status, clean) |
| `setup/tui.py` | stdlib-curses selector; writes the selection to `--out` |
| `setup/justfile` | one-line delegations per step; `status`, `verify`, `plan`, `clean-*` |
| `setup/test/test_steps.py` | 15 tests, wired into `just test` |
| `setup/scripts/install-just.sh` | new, pinned prebuilt binary |
| `setup/scripts/install-lint-tools.sh` | new: ruff + uv, pinned musl binaries |
| `setup/scripts/check-python-env.sh` | new: the pip-shadow guard, now shared with `just _check-python-env` |
| `setup/scripts/install-dev-tools-{debug,docs}.sh` | replace `install-dev-tools.sh` |

`tui.py` writes its result to a file rather than stdout: curses owns stdout while the
screen is up, so piping it breaks the display.

Verified on this machine:

- a script that exits 0 while its verifier fails returns non-zero and writes no marker
- editing a script or a verifier invalidates an existing marker
- `--status` reports `dev-tools-debug` and `dev-tools-docs` missing, which markers alone
  never showed, and flags the pre-existing markers as "installed by an older version of
  the script"
- the test suite fails when an assertion is deliberately broken and passes when restored
