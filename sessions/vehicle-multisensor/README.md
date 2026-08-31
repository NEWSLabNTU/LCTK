# vehicle-multisensor

Two LiDARs and four cameras across three markers, on a `reference_frame: L1` TF
tree. This is a **schema demonstration**: there is no recording and no rig behind
it. It exists so the multi-pair, multi-marker form of a manifest is documented by
a file that actually parses.

`data.kind` is `live` because a manifest must declare a source and there is no
recording to name — not because sensors are expected to appear.

| | |
|---|---|
| Data | `live` (nominal) |
| LiDARs | `L1` (`lidar_front`), `L2` (`lidar_rear`) |
| Cameras | `C1`–`C4`, the four corners |
| Markers | `M1` front, `M2` rear, `M3` visible to both LiDARs |

The graph it generates:

- 4 `lidar_board_detector` nodes — (L1,M1), (L1,M3), (L2,M2), (L2,M3)
- 4 `aruco_locator_node` nodes — C1–C4
- 4 LiDAR-camera solvers — L1-C1, L1-C2, L2-C3, L2-C4
- 1 LiDAR-LiDAR solver — L1-L2
- 1 `tf_tree_broadcaster`

Because it is the one session with both solver kinds and four LiDAR-camera pairs,
it is also the fixture that pins `sync:` reaching both kinds and each assisted-mode
solver getting its own review port.
