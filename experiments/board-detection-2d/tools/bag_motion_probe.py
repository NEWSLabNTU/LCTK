"""Does the board sit somewhere different in each bag?

Method E's cross-capture premise: the held-out capture contains an object
the others do not, at a location they do not occupy. If every bag places
the board identically, a consensus background absorbs it and LOO recall is
0 by construction -- a fact worth knowing BEFORE running a benchmark and
misreading the result as a detector failure.

For each bag, build a consensus background from the OTHER bags and report
what survives: how many points, and where their largest cluster sits.

NOTE (measured): the four TWO_LIDAR bags are only TWO distinct board
positions -- {TWO_LIDAR_1, TWO_LIDAR_2} share one, {TWO_LIDAR_3,
TWO_LIDAR_4} share the other. So a naive 4-bag leave-one-out is CONFOUNDED
(the held-out bag's twin sits in the background), and the real evaluation
merges each pair into one source for a clean 2-fold LOO at min_sources=1.
This probe still answers the gate question -- foreground survives at
min_sources>=2, i.e. the board is not in the same place across all four --
but read the per-bag numbers with the pairing in mind. See
docs/roadmap/side-track_method-e-background-subtraction.md.
"""
from __future__ import annotations

import numpy as np

from boarddet.background import BackgroundModel
from boarddet.geometry import downsample
from boarddet.ingest import load_bag_frames

BAGS = ["TWO_LIDAR_1", "TWO_LIDAR_2", "TWO_LIDAR_3", "TWO_LIDAR_4"]


def main() -> None:
    frames = {b: load_bag_frames(b, "vlp32", max_frames=40) for b in BAGS}
    dn = {b: [downsample(f.xyz, 0.03) for f in fr] for b, fr in frames.items()}
    for min_sources in (1, 2, 3):
        print(f"--- min_sources={min_sources}")
        for held in BAGS:
            m = BackgroundModel(voxel=0.06, dilation_radius=1,
                                min_sources=min_sources)
            for b in BAGS:
                if b == held:
                    continue
                m.observe(np.concatenate(dn[b], axis=0), source=b)
            m.finalize()
            fg = m.foreground_points(dn[held][0])
            if len(fg) == 0:
                print(f"  {held}: 0 foreground points "
                      f"(bg {m.n_voxels} voxels)")
                continue
            c = fg.mean(axis=0)
            print(f"  {held}: {len(fg):6d} fg pts  centroid "
                  f"({c[0]:6.2f},{c[1]:6.2f},{c[2]:6.2f})  "
                  f"bg {m.n_voxels} voxels")


if __name__ == "__main__":
    main()
