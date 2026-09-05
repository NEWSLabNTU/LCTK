# Assisted review: evidence and 3D frontend

Implementation of the [approved design](../superpowers/specs/2026-09-05-assisted-review-3d-frontend-design.md)
on `feat/assisted-review-3d-frontend`. Starting commit: `c1af8ed`.

## Ordered commits

1. Match captured evidence by source header stamp; rectify camera previews only when
   distortion is nonzero. Pin both defects through `StampedRing` and `EvidenceStore`.
2. Publish `debug/plane_inliers` unconditionally with RELIABLE QoS, subscribe in assisted
   mode, and serve each capture's packed float32 XYZ cloud.
3. Relocate the existing page verbatim into packaged Flask static assets.
4. Expose server-computed scene geometry and revision; add `ReviewApi`, vendored three.js,
   and `SceneModel` with picking, navigation, and resource disposal.
5. Apply the approved layout through `Chrome`, keeping selection in `main.js`.
6. Add loopback-only writes for the three named stability parameters.

Each chunk receives focused verification and a separate commit. No merge or push.
Continuous/manual behavior, other debug publishers, raw sweep capture, authentication,
and JavaScript build tooling remain outside this work.

## Verification

- Preserve clean-tree baseline and test command exit codes under
  `tmp/assisted-review/`. Use `JUST_TEMPDIR` there for just's temporary scripts.
- Before production edits, the synthetic edge-marker test must fail with the demo
  camera's real distortion and pass with zero distortion.
- Deliberately break an assertion, verify `just test` returns nonzero, restore it,
  and verify the final suite passes. Keep the failing assertion in the log.
- Run `just build`, `just test`, and `just lint`; exclude build-generated lockfile churn
  and concurrent unrelated edits from commits.
- Inspect the approved HTML in a browser. Exercise the shipped page and APIs with both
  `solid600-handheld-vlp` (motion) and `sample3-hollow-velodyne` (distorted edge markers).
  Record limitations explicitly where recordings do not contain the required geometry.

## Baseline and results

Pending verification. The controller's standalone flake8 and pep257 checks both fail on
the clean tree; they are not included in this checkout's `just test` recipe.
