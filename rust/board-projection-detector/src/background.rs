//! Accumulated static-scene voxel occupancy for background/foreground diffing.
//!
//! Port of `experiments/board-detection-2d/src/boarddet/background.py`
//! (Method E of `docs/roadmap/side-track_auto-bounding-box.md`): a
//! static-mounted sensor sees the same room every frame, so voxels that are
//! reliably occupied are background and points landing in never-occupied
//! voxels are foreground.
//!
//! CONSENSUS (`min_sources`): sources are kept apart and a voxel becomes
//! background only once `>= min_sources` distinct sources have seen it. With
//! one source per recorded dataset, the shared static room (seen by every
//! source) survives while each source's own calibration board (seen by
//! exactly one) drops out.

use nalgebra::Point3;
use std::collections::{BTreeSet, HashMap};

/// Integer voxel indices bit-packed into one int64 key. 21 bits/axis covers
/// +/- 2,097,152 cells, i.e. +/- ~125 km at the 0.06 m default voxel -- no
/// LiDAR range can overflow it. `KEY_OFFSET` biases indices positive so
/// points behind/below the sensor pack correctly.
const KEY_BITS: i64 = 21;
const KEY_MASK: i64 = (1 << KEY_BITS) - 1;
const KEY_OFFSET: i64 = 1 << (KEY_BITS - 1);

/// (biased voxel index) -> int64 key. Ports `background.py:_pack`.
pub fn pack(idx: [i64; 3]) -> i64 {
    (idx[0] & KEY_MASK) | ((idx[1] & KEY_MASK) << KEY_BITS) | ((idx[2] & KEY_MASK) << (2 * KEY_BITS))
}

fn voxel_idx(p: &Point3<f64>, voxel: f64) -> [i64; 3] {
    [
        (p.x / voxel).floor() as i64 + KEY_OFFSET,
        (p.y / voxel).floor() as i64 + KEY_OFFSET,
        (p.z / voxel).floor() as i64 + KEY_OFFSET,
    ]
}

/// Per-source voxel occupancy with a `>= min_sources` consensus background.
pub struct BackgroundModel {
    pub voxel: f64,
    pub dilation_radius: i64,
    pub min_sources: usize,
    sources: HashMap<String, BTreeSet<i64>>,
    keys: Option<Vec<i64>>,
    /// `(2r+1)^3` neighbour offsets; radius 0 is the centre cell alone.
    stencil: Vec<[i64; 3]>,
}

impl BackgroundModel {
    pub fn new(voxel: f64, dilation_radius: i64, min_sources: usize) -> Self {
        let r = dilation_radius;
        let mut stencil = Vec::with_capacity((2 * r as usize + 1).pow(3));
        for dx in -r..=r {
            for dy in -r..=r {
                for dz in -r..=r {
                    stencil.push([dx, dy, dz]);
                }
            }
        }
        Self {
            voxel,
            dilation_radius,
            min_sources,
            sources: HashMap::new(),
            keys: None,
            stencil,
        }
    }

    /// Rebuild a model directly from a finalized (sorted, deduped) key set --
    /// used to replay a Python-exported background for fixture parity, where
    /// the per-source occupancy that produced it isn't available.
    pub fn from_keys(keys: Vec<i64>, voxel: f64, dilation_radius: i64, min_sources: usize) -> Self {
        let mut m = Self::new(voxel, dilation_radius, min_sources);
        let mut k = keys;
        k.sort_unstable();
        k.dedup();
        m.keys = Some(k);
        m
    }

    /// Fold `points` into `source`'s own occupancy set.
    ///
    /// Callable repeatedly for one source (frame by frame or all at once --
    /// a set union is order- and grouping-independent). Invalidates any
    /// previous `finalize()`.
    pub fn observe(&mut self, points: &[Point3<f64>], source: &str) {
        if points.is_empty() {
            return;
        }
        let e = self.sources.entry(source.to_string()).or_default();
        for p in points {
            e.insert(pack(voxel_idx(p, self.voxel)));
        }
        self.keys = None;
    }

    /// Collapse the per-source sets into the consensus background.
    pub fn finalize(&mut self) {
        let mut counts: HashMap<i64, usize> = HashMap::new();
        for s in self.sources.values() {
            for &k in s {
                *counts.entry(k).or_default() += 1;
            }
        }
        let mut keys: Vec<i64> = counts
            .into_iter()
            .filter(|&(_, c)| c >= self.min_sources)
            .map(|(k, _)| k)
            .collect();
        keys.sort_unstable();
        self.keys = Some(keys);
    }

    /// The finalized consensus background's sorted voxel keys (for fixture
    /// export parity).
    pub fn keys(&self) -> &[i64] {
        self.keys.as_deref().unwrap_or(&[])
    }

    /// Points of `dn` whose own voxel AND every dilation-stencil neighbour
    /// are unoccupied in the consensus background.
    ///
    /// Dilation is applied at QUERY time, not at storage time: it keeps the
    /// stored set small while still absorbing the voxel-boundary aliasing
    /// that would otherwise report a static surface as new whenever range
    /// noise nudges a point across a cell edge.
    ///
    /// # Panics
    /// If `finalize()` (or `from_keys`) hasn't been called since the last
    /// `observe()`.
    pub fn foreground_points(&self, dn: &[Point3<f64>]) -> Vec<Point3<f64>> {
        let keys = self.keys.as_ref().expect("finalize() before foreground_points()");
        if keys.is_empty() || dn.is_empty() {
            return dn.to_vec();
        }
        dn.iter()
            .filter(|p| {
                let base = voxel_idx(p, self.voxel);
                let is_bg = self.stencil.iter().any(|d| {
                    let k = pack([base[0] + d[0], base[1] + d[1], base[2] + d[2]]);
                    keys.binary_search(&k).is_ok()
                });
                !is_bg
            })
            .copied()
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_roundtrips_through_bit_layout() {
        let idx = [KEY_OFFSET + 1, KEY_OFFSET + 2, KEY_OFFSET + 3];
        let k = pack(idx);
        let x = k & KEY_MASK;
        let y = (k >> KEY_BITS) & KEY_MASK;
        let z = (k >> (2 * KEY_BITS)) & KEY_MASK;
        assert_eq!([x, y, z], idx);
    }

    #[test]
    fn min_sources_requires_consensus() {
        let mut bg = BackgroundModel::new(0.1, 0, 2);
        let p = [Point3::new(1.0, 1.0, 1.0)];
        bg.observe(&p, "a");
        bg.finalize();
        assert!(bg.keys().is_empty(), "single source must not satisfy min_sources=2");

        bg.observe(&p, "b");
        bg.finalize();
        assert_eq!(bg.keys().len(), 1);
    }
}
