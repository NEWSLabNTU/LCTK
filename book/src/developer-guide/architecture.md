# Architecture

LCTK uses a layered architecture that separates algorithms from ROS 2 integration.

## Design Principles

### Separation of Concerns

```
┌─────────────────────────────────────────────────────┐
│              ROS 2 Nodes (ros/)                     │
│   Thin wrappers: topics, services, parameters       │
├─────────────────────────────────────────────────────┤
│            Core Libraries (rust/)                   │
│   Pure Rust: algorithms, no ROS dependencies        │
└─────────────────────────────────────────────────────┘
```

**Core libraries** (`rust/`) contain detection and calibration algorithms. They are pure Rust with no ROS dependencies, testable with `cargo test`, and reusable outside ROS.

**ROS nodes** (`ros/`) are thin wrappers that handle communication (topics, services) and lifecycle. They delegate actual work to core libraries.

This separation enables:
- Fast iteration with `cargo test` (no ROS setup needed)
- Algorithm reuse in non-ROS contexts
- Clear boundaries for testing and maintenance

### Lock-Free Configuration

Nodes use `arc-swap` for runtime configuration updates without blocking callbacks:

```rust
// Service updates config atomically
config.store(Arc::new(new_config));

// Detection callback reads without locking
let current = config.load();
```

## LiDAR-Camera Calibration Pipeline

```mermaid
flowchart LR
    subgraph Input
        CAM[(Camera)]
        LID[(LiDAR)]
    end

    subgraph Detection
        ARU[ArUco Detector]
        BRD[Board Detector]
    end

    subgraph Calibration
        EXT[Extrinsic Solver]
    end

    subgraph Output
        OVL[Overlay]
        TF>Transform]
    end

    CAM -->|image| ARU
    LID -->|pointcloud| BRD

    ARU -->|2D corners| EXT
    BRD -->|3D pose| EXT

    EXT --> TF
    EXT --> OVL

    CAM -->|image| OVL
    LID -->|pointcloud| OVL

    classDef sensor fill:#e0e0e0,stroke:#333,color:#000
    classDef node fill:#4a90d9,stroke:#333,color:#fff
    classDef output fill:#2d6a4f,stroke:#333,color:#fff

    class CAM,LID sensor
    class ARU,BRD,EXT,OVL node
    class TF output
```

### Data Flow

1. **Camera** publishes images; **LiDAR** publishes point clouds
2. **ArUco Detector** finds marker corners in images (2D)
3. **Board Detector** finds calibration board pose in point clouds (3D)
4. **Extrinsic Solver** computes LiDAR-to-camera transform using 2D-3D correspondences
5. **Overlay** projects points onto images using the computed transform

### Board Detection Pipeline

The board detector uses a multi-stage approach:

```
Pointcloud → Stage 1: cluster selection → RANSAC/PCA Plane → PCA Initial Pose → ICP Refinement → Pose
```

Stage 1 selects the candidate board cluster, and its mode is chosen by
`detection_mode` in the marker's Detector Tuning preset (e.g.
`config/board/hollow_1000/velodyne.json5`):

- **`bbox`** (default): a fixed bounding box crops points to a region of
  interest. Simple, but the board must stay inside a hand-tuned box.
- **`bbox_free`**: no bounding box — the `board-cluster-detector`
  library isolates the board cluster from the whole cloud by projecting
  candidate planes to 2D and gating on board shape/size/stance. Its
  selected points feed the same downstream engine (Option C: the library's
  square-fit corners gate cluster selection only; the final pose is still
  the node's ICP, not the library's corners).

Downstream stages are identical for both modes:

1. **RANSAC / PCA** fits the dominant plane on the selected points (RANSAC,
   or PCA directly when `skip_ransac: true`)
2. **PCA** computes initial pose from plane inliers
3. **ICP** refines pose by matching model to observed points

#### The board model and its frame

Everything downstream of Stage 1 is expressed in the board's **canonical
local frame**, defined in `rust/calibration-target/src/lib.rs` (the crate
that superseded `hollow-board-config`). The plate is a square hung as a
**diamond** — it stands on one corner — and the frame's in-plane axes run
along its diagonals, corner to corner, so that every accessor name
(`top_corner`, `left_corner`, …) means what it says:

- **origin** — the plate's **centre**. A published board pose is therefore
  a pose *of the plate centre*.
- **+Z** — the board normal, pointing toward the sensor.
- **+Y** — from the centre toward the **top** corner.
- **+X** — `Y × Z`, from the centre toward the **left** corner.

Viewed from the sensor (so +Z comes out of the page), with `W` the plate's
edge length, `R = W/√2` its half-diagonal, `s` each cutout centre's offset
from the plate centre along the plate's edge (as configured per-cutout in
the Target Definition's `plate.surface.circular_cutouts`) and `d = s√2` its
distance from the centre along the diagonal:

```
                             ▲ +Y
                             ●  top corner (0, +R)
                            ╱ ╲
                           ╱   ╲
                          ╱  ○  ╲         top hole (0, +d)
                         ╱       ╲
   right corner (−R, 0) ●   ○ + ○  ●──▶ +X   left corner (+R, 0)
                         ╲       ╱
    right hole (−d, 0)    ╲     ╱          left hole (+d, 0)
                           ╲   ╱
                            ╲ ╱
                             ●  bottom corner (0, −R)
```

Two things in that picture look like mistakes and are not:

- **The "left" corner is on the observer's right.** The corner accessors
  are named from the *board's* point of view, not the sensor's. Renaming
  them would silently reorder the corner lists every downstream consumer
  depends on, so the naming is recorded and deliberately left alone.
- **Z is the normal, not X.** `board-cluster-detector` uses the REP-103
  convention, where X is the normal. Aligning the two is a separate change,
  because the quality metric and the detection publisher both read this
  rotation's third column as the normal.

The three holes are the *only* feature that resolves the square's 90°
symmetry: two sit on the horizontal diagonal at ±d, one on the vertical
diagonal at +d, and none at −d. Board-interior points carry no in-plane
information at all, so without that missing fourth hole the pose would be
unobservable within the board plane.

`calibration-target` also owns the **marker paper's** placement on the
plate. Paper coordinates run along the paper's *edges*, i.e. at 45° to the
board frame's axes; `marker_paper_point` is the single bridge between the
two, and where the sheet sits is configuration (`fiducial.paper_center` in
the Target Definition, e.g. `config/targets/hollow_1000_aruco_4_v1.json5`),
not a derived constant.

The contract above is enforced by
`rust/calibration-target/tests/geometry_contract.rs`, which checks named
boundary/interior/rim points and random query points under a fixed set of
posed transforms — cross-checking every projection distance against an
independently sampled surface (the same role `boundary_projection.rs` used
to play) — and verifies marker corners in **world** coordinates against a
golden fixture keyed by ArUco marker id
(`fixtures/targets/marker_corners_world.golden.json`, produced by an
independent Python generator, `fixtures/targets/generate_marker_corners_world.py`;
the same role `marker_layout_golden.rs` used to play).

> **Camera-side status.** The two Python solvers reimplement the marker
> layout independently and still use the **previous, edge-aligned** frame.
> Until that is corrected, LiDAR-camera calibration is not trustworthy —
> see `docs/issues/H-11-camera-solvers-stale-board-frame.md` in the repo.
> LiDAR-to-LiDAR calibration is unaffected, because both sides of it come
> from the same detector.

#### Crop-box-free foreground methods (`bbox_free`)

`foreground_method` picks how `bbox_free` extracts foreground points:

- **`background_subtraction`** (Method E, the shipping path, ~34–69 ms/frame):
  builds a background voxel model during a warmup phase (the first
  `warmup_frames` board-free clouds are observed, then the model is
  finalized), then keeps only points absent from that background. Re-run
  the warmup at runtime by calling the `~/reset_background`
  (`std_srvs/srv/Empty`) service — e.g. after moving the rig.
- **`plane_strip`** (Method B, ~157–202 ms/frame, over the ~100 ms budget):
  RANSAC-strips large background planes. No warmup/background needed, but
  slower; not the default.

When `bbox_free` finds nothing, the node logs the furthest-progressing
reject reason (no clusters / flatness / extent / size / square residual /
stance / isolation) instead of silently publishing an empty detection.

## Configuration Flow

```
Config Files (JSON5)
        ↓
Launch Parameters (XML)
        ↓
ROS Parameters
        ↓
Runtime Services (dynamic updates)
```

Configuration files in `ros/lctk_launch/config/` define:
- **Target Definition** (`targets/`): the physical truth — plate geometry, cutout layout,
  fiducial marker IDs and placement
- **Detector Tuning** (`board/<target>/`): sensor-specific, geometry-free ICP/RANSAC parameters
- **ArUco detector tuning** (`aruco/`): corner refinement, adaptive threshold
- **Bounding box** (`board/bbox*.json5`): region of interest for detection, when the tuning
  preset selects `detection_mode: "bbox"`

Launch files pass these to nodes as parameters. Some parameters (like bounding box) can be updated at runtime via services.

## Debug Mode

When `debug_mode=true`, nodes publish intermediate results for visualization:

- Filtered point clouds at each pipeline stage
- RANSAC plane visualization
- ICP iteration poses
- Detection statistics

Use RViz or the web UI to visualize these topics during development.

## Project Structure

```
LCTK/
├── rust/                    # Core libraries (pure Rust)
│   ├── aruco-detector/
│   ├── calibration-target/          # Target Definitions: geometry, identity
│   ├── calibration-target-detector/ # Pose estimation against a Target Definition
│   ├── board-cluster-detector/
│   ├── plane-estimator/
│   └── ...
├── ros/                     # ROS 2 packages
│   ├── aruco_locator_node/
│   ├── lidar_board_detector/
│   ├── extrinsic_solver_node/       # Superseded; pending deletion
│   ├── lidar_to_camera_solver/
│   ├── lctk_launch/         # Launch files and configs
│   │   └── config/
│   │       ├── targets/     # Target Definitions
│   │       ├── board/       # Detector Tuning presets
│   │       └── aruco/       # ArUco detector tuning
│   └── ...
├── book/                    # This documentation
└── justfile                 # Build commands
```

Explore the source code for implementation details. Each library and node has its own README and rustdoc comments.

## Extending LCTK

### Adding a New Detection Target

1. Implement detector algorithm in `rust/`
2. Create ROS node wrapper in `ros/`
3. Add configuration files and launch integration
4. Update extrinsic solver to accept new detection type

### Adding a New Solver Algorithm

1. Add algorithm to the solver library
2. Expose as a configuration option
3. Update node parameters

## Next Steps

- [Build System](./build-system.md) - How to build the project
- [Testing](./testing.md) - Testing strategies
- [Contributing](./contributing.md) - Development workflow
