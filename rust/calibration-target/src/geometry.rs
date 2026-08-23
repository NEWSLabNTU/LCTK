//! Canonical corner-aligned target geometry and nearest-surface projection.
//!
//! All local points use `corner_aligned_plate_center_v1`: origin at plate centre,
//! `+X` toward the left corner, `+Y` toward the top corner, and `+Z` along the
//! plate normal.  The square is therefore the closed diamond `|x| + |y| <= W/sqrt(2)`.

use std::borrow::Borrow;

use indexmap::IndexMap;
use nalgebra::{Isometry3, Point3, UnitVector3, Vector3};

use crate::{CircularCutout, Surface, ValidatedTarget};

/// One source item paired with its nearest physical point on the target.
#[derive(Debug, Clone, PartialEq)]
pub struct Correspondence<P> {
    pub input: P,
    pub closest: Point3<f64>,
}

/// Canonical target axes expressed in the sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetAxes {
    pub toward_left_corner: UnitVector3<f64>,
    pub toward_top_corner: UnitVector3<f64>,
    pub normal: UnitVector3<f64>,
}

/// A Target Definition placed in a sensor frame.  Its pose is the pose of the plate
/// centre; all named points retain the target's canonical corner-aligned meaning.
#[derive(Debug, Clone)]
pub struct PosedTarget<'a> {
    target: &'a ValidatedTarget,
    pose: Isometry3<f64>,
    surface: &'a SurfaceAdapter,
}

impl<'a> PosedTarget<'a> {
    pub(crate) fn new(target: &'a ValidatedTarget, pose: Isometry3<f64>) -> Self {
        Self {
            target,
            pose,
            surface: target.surface_adapter(),
        }
    }

    /// Pose of the plate centre in the sensor frame.
    pub fn pose(&self) -> &Isometry3<f64> {
        &self.pose
    }

    pub fn target(&self) -> &'a ValidatedTarget {
        self.target
    }

    pub fn center(&self) -> Point3<f64> {
        self.world_point(self.target.local_center())
    }

    pub fn top_corner(&self) -> Point3<f64> {
        self.world_point(self.target.local_top_corner())
    }

    pub fn bottom_corner(&self) -> Point3<f64> {
        self.world_point(self.target.local_bottom_corner())
    }

    pub fn left_corner(&self) -> Point3<f64> {
        self.world_point(self.target.local_left_corner())
    }

    pub fn right_corner(&self) -> Point3<f64> {
        self.world_point(self.target.local_right_corner())
    }

    pub fn x_axis(&self) -> UnitVector3<f64> {
        self.pose * Vector3::x_axis()
    }

    pub fn y_axis(&self) -> UnitVector3<f64> {
        self.pose * Vector3::y_axis()
    }

    pub fn z_axis(&self) -> UnitVector3<f64> {
        self.pose * Vector3::z_axis()
    }

    /// Unit vector from plate centre toward its named top corner.
    pub fn board_up(&self) -> UnitVector3<f64> {
        self.y_axis()
    }

    pub fn axes(&self) -> TargetAxes {
        TargetAxes {
            toward_left_corner: self.x_axis(),
            toward_top_corner: self.y_axis(),
            normal: self.z_axis(),
        }
    }

    pub fn paper_center(&self) -> Point3<f64> {
        self.world_point(self.target.local_paper_center())
    }

    /// Map paper-edge coordinates (metres, each in `[0, paper_side]`) into sensor
    /// coordinates. `u` moves toward the plate's left corner and `v` toward right.
    pub fn marker_paper_point(&self, u_m: f64, v_m: f64) -> Point3<f64> {
        self.world_point(self.target.local_marker_paper_point(u_m, v_m))
    }

    /// ArUco corners keyed by marker ID.  Each array is `[right, top, left, bottom]`.
    pub fn marker_corners_by_id(&self) -> IndexMap<u32, [Point3<f64>; 4]> {
        self.target
            .marker_corners_by_id()
            .into_iter()
            .map(|(&id, corners)| (id, corners.map(|point| self.world_point(point))))
            .collect()
    }

    /// Nearest physical target point for one sensor-frame query.
    pub fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        let local = self.pose.inverse_transform_point(point);
        self.world_point(self.surface.closest_local(local))
    }

    /// Nearest physical target point for every query, in exactly input order.
    pub fn closest_points<I, P>(&self, points: I) -> Vec<Correspondence<P>>
    where
        I: IntoIterator<Item = P>,
        P: Borrow<Point3<f64>>,
    {
        points
            .into_iter()
            .map(|input| {
                let closest = self.closest_point(input.borrow());
                Correspondence { input, closest }
            })
            .collect()
    }

    fn world_point(&self, local: Point3<f64>) -> Point3<f64> {
        self.pose.transform_point(&local)
    }

    #[cfg(test)]
    pub(crate) fn surface_adapter_address(&self) -> *const SurfaceAdapter {
        self.surface
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SurfaceAdapter {
    Solid(SolidSquareSurface),
    Perforated(PerforatedSquareSurface),
}

impl SurfaceAdapter {
    pub(crate) fn from_plate(side_um: i64, surface: &Surface) -> Self {
        let half_diagonal_m = side_um as f64 / 1_000_000.0 / std::f64::consts::SQRT_2;
        match surface {
            Surface::Solid => Self::Solid(SolidSquareSurface { half_diagonal_m }),
            Surface::Perforated { circular_cutouts } => Self::Perforated(PerforatedSquareSurface {
                plate: SolidSquareSurface { half_diagonal_m },
                cutouts: circular_cutouts
                    .iter()
                    .map(Cutout::from_definition)
                    .collect(),
            }),
        }
    }

    fn closest_local(&self, point: Point3<f64>) -> Point3<f64> {
        match self {
            Self::Solid(surface) => surface.closest_local(point),
            Self::Perforated(surface) => surface.closest_local(point),
        }
    }
}

/// Real adapter for the closed, solid square plate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SolidSquareSurface {
    half_diagonal_m: f64,
}

impl SolidSquareSurface {
    fn closest_local(&self, point: Point3<f64>) -> Point3<f64> {
        let (x, y) = project_onto_diamond(point.x, point.y, self.half_diagonal_m);
        Point3::new(x, y, 0.0)
    }
}

/// Real adapter for the perforated square plate.  First project to the same closed
/// plate, then move an interior-cutout point to its closest circular rim.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PerforatedSquareSurface {
    plate: SolidSquareSurface,
    cutouts: Vec<Cutout>,
}

impl PerforatedSquareSurface {
    fn closest_local(&self, point: Point3<f64>) -> Point3<f64> {
        let projected = self.plate.closest_local(point);
        for cutout in &self.cutouts {
            let dx = projected.x - cutout.center_x_m;
            let dy = projected.y - cutout.center_y_m;
            let distance = dx.hypot(dy);
            if distance < cutout.radius_m {
                if dx == 0.0 && dy == 0.0 {
                    // Centre has no unique radial direction. Pick +X deterministically,
                    // matching legacy hollow-board projection behaviour.
                    return Point3::new(
                        cutout.center_x_m + cutout.radius_m,
                        cutout.center_y_m,
                        0.0,
                    );
                }
                let scale = cutout.radius_m / distance;
                return Point3::new(
                    cutout.center_x_m + dx * scale,
                    cutout.center_y_m + dy * scale,
                    0.0,
                );
            }
        }
        projected
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Cutout {
    center_x_m: f64,
    center_y_m: f64,
    radius_m: f64,
}

impl Cutout {
    fn from_definition(cutout: &CircularCutout) -> Self {
        Self {
            center_x_m: cutout.x_um as f64 / 1_000_000.0,
            center_y_m: cutout.y_um as f64 / 1_000_000.0,
            radius_m: cutout.radius_um as f64 / 1_000_000.0,
        }
    }
}

/// Euclidean projection onto closed diamond `|x| + |y| <= radius`.
fn project_onto_diamond(x: f64, y: f64, radius: f64) -> (f64, f64) {
    let (a, b) = (x.abs(), y.abs());
    if a + b <= radius {
        return (x, y);
    }
    let t = (a + b - radius) / 2.0;
    let (mut projected_a, mut projected_b) = (a - t, b - t);
    if projected_a < 0.0 {
        projected_a = 0.0;
        projected_b = radius;
    } else if projected_b < 0.0 {
        projected_a = radius;
        projected_b = 0.0;
    }
    (projected_a.copysign(x), projected_b.copysign(y))
}

#[cfg(test)]
mod tests {
    use super::project_onto_diamond;

    #[test]
    fn diamond_projection_snaps_edges_and_vertices() {
        let radius = std::f64::consts::FRAC_1_SQRT_2;
        assert_eq!(project_onto_diamond(0.1, -0.2, radius), (0.1, -0.2));
        let (x, y) = project_onto_diamond(radius, radius, radius);
        assert!((x - radius / 2.0).abs() < 1e-12);
        assert!((y - radius / 2.0).abs() < 1e-12);
        assert_eq!(project_onto_diamond(3.0, 0.1, radius), (radius, 0.0));
    }
}
