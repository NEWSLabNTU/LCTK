# Contributing

The repository's root `AGENTS.md` is the canonical development guide. It records the
current build and test workflow, coding conventions, issue-tracking rules, and known
environment pitfalls. Read it before making a change; use this page as a short
contribution checklist.

## Choose and scope the work

- Check `docs/issues/README.md` before taking an issue. Do not take an issue another
  contributor has marked in progress.
- Work on a `fix/...`, `feat/...`, or `docs/...` branch.
- Follow the project's domain vocabulary in `AGENTS.md`. In particular, keep a
  Target Definition distinct from Detector Tuning, and a numerical solve distinct
  from its Quality Verdict.
- Keep changes at the layer that owns the behavior. See
  [Architecture](./architecture.md) for package responsibilities.

## Build, test, and lint

Use the repository entry points from the project root:

```bash
just build
just test
just lint
```

For Python-only edits, also run `just lint-py`. For focused iteration, `just lint-rust`
and `just lint-py` run the language-specific checks. See [Build System](./build-system.md)
and [Testing](./testing.md) for their scope. Do not substitute raw `colcon build` or
`cargo test` for the workspace recipes.

For book changes, build the mdBook separately:

```bash
cd book
just build
```

Before reporting completion, state which relevant checks passed and which could not be
run.

## Coding conventions

- Use named Rust format arguments, such as `println!("{error}")`.
- Prefer direct struct initialization when constructing a value.
- Clone `Arc` values in the local scope before moving them into closures.
- Avoid broad Python exception handlers that hide errors.
- Keep tests attached to the code they exercise and include them in the repository's
  test entry point.

The root `AGENTS.md` has more detailed examples and project-specific rules.

## Issues, documentation, and generated files

File each finding as one Markdown document under `docs/issues/` and update the tracker.
When closing an issue, add a resolution note, move it to `docs/issues/archive/`, and
repair links affected by the move. `just check-docs` checks relative Markdown links in
`docs/`, `book/src/`, and selected repository README/CONTRIBUTING files; it does not
validate heading anchors.

Do not commit build-generated `Cargo.lock` changes as dependency updates. Follow the
separate dependency-update procedure documented in `docs/roadmap/` and the root
`AGENTS.md`. The `ros/conflux` submodule is maintained separately; do not commit its
ignored lockfile churn upstream.

After verification, the repository workflow is to fast-forward the change branch into
`main`. Fetch and rebase before pushing so the branch includes the latest
`origin/main`.
