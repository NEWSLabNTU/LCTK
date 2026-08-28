//! Independent geometry evidence for the target seam.
//!
//! Nearestness oracle samples the physical set directly. It intentionally does not
//! restate the diamond projection or cutout-push formulas used by production code.

use calibration_target::{Surface, ValidatedTarget};
use nalgebra::{
    Isometry3, Matrix3, Point3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3,
};
use serde::Deserialize;
use std::collections::HashMap;

const SOLID: &str = include_str!("../../../fixtures/targets/solid_600_aruco_1_v1.json5");
const HOLLOW: &str = include_str!("../../../fixtures/targets/hollow_1000_aruco_4_v1.json5");
const MARKER_GOLDEN: &str =
    include_str!("../../../fixtures/targets/marker_corners_world.golden.json");
const TOL: f64 = 1e-9;

#[derive(Clone, Copy)]
struct Cutout {
    x: f64,
    y: f64,
    radius: f64,
}

fn cutouts(target: &ValidatedTarget) -> Vec<Cutout> {
    match &target.plate().surface {
        Surface::Solid => Vec::new(),
        Surface::Perforated { circular_cutouts } => circular_cutouts
            .iter()
            .map(|cutout| Cutout {
                x: cutout.x_um as f64 / 1e6,
                y: cutout.y_um as f64 / 1e6,
                radius: cutout.radius_um as f64 / 1e6,
            })
            .collect(),
    }
}

fn is_on_surface(target: &ValidatedTarget, point: Point3<f64>, tolerance: f64) -> bool {
    point.z.abs() <= tolerance
        && point.x.abs() + point.y.abs() <= target.half_diagonal_m() + tolerance
        && cutouts(target).into_iter().all(|cutout| {
            (point.x - cutout.x).hypot(point.y - cutout.y) + tolerance >= cutout.radius
        })
}

/// Dense samples of plate interior, all four plate edges, and every circular rim.
fn sampled_surface(target: &ValidatedTarget) -> Vec<Point3<f64>> {
    let radius = target.half_diagonal_m();
    let cutouts = cutouts(target);
    let mut samples = Vec::new();

    let grid_step = 0.01;
    let grid_count = (2.0 * radius / grid_step).ceil() as i32;
    for ix in 0..=grid_count {
        for iy in 0..=grid_count {
            let x = -radius + 2.0 * radius * ix as f64 / grid_count as f64;
            let y = -radius + 2.0 * radius * iy as f64 / grid_count as f64;
            let point = Point3::new(x, y, 0.0);
            if is_on_surface(target, point, 0.0) {
                samples.push(point);
            }
        }
    }

    let corners = [
        Point3::new(radius, 0.0, 0.0),
        Point3::new(0.0, radius, 0.0),
        Point3::new(-radius, 0.0, 0.0),
        Point3::new(0.0, -radius, 0.0),
    ];
    for edge in 0..4 {
        for index in 0..=500 {
            samples.push(
                corners[edge] + (corners[(edge + 1) % 4] - corners[edge]) * index as f64 / 500.0,
            );
        }
    }

    for cutout in cutouts {
        let count = (std::f64::consts::TAU * cutout.radius / 0.001).ceil() as usize;
        for index in 0..count {
            let angle = std::f64::consts::TAU * index as f64 / count as f64;
            samples.push(Point3::new(
                cutout.x + cutout.radius * angle.cos(),
                cutout.y + cutout.radius * angle.sin(),
                0.0,
            ));
        }
    }
    samples
}

fn test_poses() -> [Isometry3<f64>; 3] {
    [
        Isometry3::identity(),
        Isometry3::from_parts(
            Translation3::new(1.2, -0.7, 2.1),
            UnitQuaternion::from_euler_angles(0.4, -1.0, 0.3),
        ),
        Isometry3::from_parts(
            Translation3::new(-2.0, 1.3, -0.4),
            UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(Vector3::new(1.0, 2.0, -0.5)),
                2.2,
            ),
        ),
    ]
}

#[test]
fn projections_are_nearest_against_independent_surface_samples() {
    let mut rng = Lcg::new(0x51de_5eed);
    for source in [SOLID, HOLLOW] {
        let target = ValidatedTarget::parse_json5(source.as_bytes()).unwrap();
        let samples = sampled_surface(&target);
        for pose in test_poses() {
            let posed = target.posed(pose);
            for _ in 0..100 {
                let query_local = Point3::new(
                    rng.range(-1.2, 1.2),
                    rng.range(-1.2, 1.2),
                    rng.range(-0.8, 0.8),
                );
                let query = pose.transform_point(&query_local);
                let actual = posed.closest_point(&query);
                let actual_local = pose.inverse_transform_point(&actual);
                assert!(is_on_surface(&target, actual_local, TOL));

                let actual_distance = (actual_local - query_local).norm();
                let sampled_distance = samples
                    .iter()
                    .map(|sample| (sample - query_local).norm())
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    actual_distance <= sampled_distance + TOL,
                    "projection {actual_distance} farther than sampled point {sampled_distance}"
                );
                assert!(
                    actual_distance >= sampled_distance - 0.008,
                    "projection {actual_distance} implausibly closer than sample {sampled_distance}"
                );
            }
        }
    }
}

#[test]
fn named_boundary_interior_and_rim_cases_hold_under_full_poses() {
    let solid = ValidatedTarget::parse_json5(SOLID.as_bytes()).unwrap();
    let hollow = ValidatedTarget::parse_json5(HOLLOW.as_bytes()).unwrap();
    for pose in test_poses() {
        let posed = solid.posed(pose);
        let radius = solid.half_diagonal_m();
        for (query, expected) in [
            (Point3::new(2.0, 0.1, 0.7), Point3::new(radius, 0.0, 0.0)),
            (
                Point3::new(-2.0, -0.1, -0.4),
                Point3::new(-radius, 0.0, 0.0),
            ),
            (
                Point3::new(0.3, radius + 0.1, 0.8),
                Point3::new(0.1, radius - 0.1, 0.0),
            ),
            (Point3::new(0.1, -0.2, -0.6), Point3::new(0.1, -0.2, 0.0)),
        ] {
            let actual = posed.closest_point(&pose.transform_point(&query));
            let expected = pose.transform_point(&expected);
            assert!((actual - expected).norm() < TOL);
        }

        let posed = hollow.posed(pose);
        for cutout in cutouts(&hollow) {
            for direction in [
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, -1.0, 0.0),
                Vector3::new(-3.0, 4.0, 0.0).normalize(),
            ] {
                let query_local = Point3::new(cutout.x, cutout.y, 0.5) + direction * 0.03;
                let expected_local =
                    Point3::new(cutout.x, cutout.y, 0.0) + direction * cutout.radius;
                let actual = posed.closest_point(&pose.transform_point(&query_local));
                assert!((actual - pose.transform_point(&expected_local)).norm() < TOL);
            }
        }
    }
}

#[derive(Deserialize)]
struct MarkerGolden {
    marker_corner_order: Vec<String>,
    mounting: GoldenMounting,
    targets: HashMap<String, GoldenTarget>,
}

#[derive(Deserialize)]
struct GoldenMounting {
    plate_center: [f64; 3],
    local_x_toward_left: [f64; 3],
    local_y_toward_top: [f64; 3],
    local_z_normal: [f64; 3],
}

#[derive(Deserialize)]
struct GoldenTarget {
    marker_ids: Vec<u32>,
    markers: HashMap<String, [[f64; 3]; 4]>,
}

#[test]
fn both_targets_match_shared_marker_world_golden() {
    let golden: MarkerGolden = serde_json::from_str(MARKER_GOLDEN).unwrap();
    assert_eq!(
        golden.marker_corner_order,
        ["right", "top", "left", "bottom"]
    );
    assert_eq!(golden.targets.len(), 2);

    let vector = |value: [f64; 3]| Vector3::new(value[0], value[1], value[2]);
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
        vector(golden.mounting.local_x_toward_left),
        vector(golden.mounting.local_y_toward_top),
        vector(golden.mounting.local_z_normal),
    ]));
    let center = golden.mounting.plate_center;
    let pose = Isometry3::from_parts(
        Translation3::new(center[0], center[1], center[2]),
        UnitQuaternion::from_rotation_matrix(&rotation),
    );

    for source in [SOLID, HOLLOW] {
        let target = ValidatedTarget::parse_json5(source.as_bytes()).unwrap();
        let expected = &golden.targets[target.target_id()];
        assert_eq!(expected.marker_ids, target.fiducial().marker_ids);
        assert_eq!(
            target
                .marker_corners_by_id()
                .keys()
                .copied()
                .collect::<Vec<_>>(),
            expected.marker_ids
        );

        let posed = target.posed(pose);
        let axes = posed.axes();
        assert!(
            (axes.toward_left_corner.into_inner() - vector(golden.mounting.local_x_toward_left))
                .norm()
                < 1e-12
        );
        assert!(
            (axes.toward_top_corner.into_inner() - vector(golden.mounting.local_y_toward_top))
                .norm()
                < 1e-12
        );
        assert!((axes.normal.into_inner() - vector(golden.mounting.local_z_normal)).norm() < 1e-12);

        let actual = posed.marker_corners_by_id();
        assert_eq!(
            actual.keys().copied().collect::<Vec<_>>(),
            expected.marker_ids
        );
        for marker_id in &expected.marker_ids {
            let expected_corners = expected.markers.get(&marker_id.to_string()).unwrap();
            for (actual, expected) in actual[marker_id].iter().zip(expected_corners) {
                assert!(
                    (actual - Point3::from(*expected)).norm() < 1e-12,
                    "target {}, marker {marker_id}: got {actual:?}, expected {expected:?}",
                    target.target_id()
                );
            }
        }
    }
}

/// The marker paper's position on the plate (`fiducial.paper_center` in the manifest) is
/// a **measurement**, not something derivable from the plate's own geometry. This pins
/// that it is honoured: sliding the stated placement must slide every marker corner by
/// exactly that world vector, and touch nothing else -- the plate corners least of all.
///
/// Ported from `hollow-board-config`'s `marker_corners_follow_the_stated_paper_placement`
/// (the migration this crate supersedes it in). It is the only test in this file that
/// would notice a `paper_center` plumbed through and silently ignored: every other
/// geometry test here pins marker corners against a fixed golden, which would pass just
/// as well with the field hard-coded.
///
/// Solid target excluded: `solid_600_aruco_1`'s paper is the same size as its plate
/// (`paper_side` == plate `side`, both 0.6 m), so the paper already sits flush with the
/// plate boundary at zero offset. Any nonzero shift immediately fails this crate's
/// `fiducial: paper corners extend outside plate` validation -- there is no slack to
/// shift into, so the solid target cannot exercise this property at all.
#[test]
fn marker_corners_follow_the_stated_paper_placement() {
    // The exact manifest text for the field under test. Asserted present below so a
    // fixture reformat fails loudly here rather than silently turning this into a
    // zero-shift no-op.
    const ORIGINAL_PAPER_CENTER: &str =
        r#"paper_center: { toward_left_corner: "0m", toward_top_corner: "-0.353553391m" }"#;
    assert!(
        HOLLOW.contains(ORIGINAL_PAPER_CENTER),
        "hollow_1000_aruco_4_v1.json5's paper_center formatting changed; \
         update this test's substring"
    );

    // Deliberately small: hollow_1000's paper sits close to three circular cutouts (see
    // the manifest), and a shift as large as the 0.1 m / 0.03 m the predecessor test
    // used collides with one of them under this crate's paper-vs-cutout validation -- a
    // check the old `hollow-board-config` crate's mutable `BoardModel` did not perform.
    // This magnitude clears every cutout and the plate boundary with wide margin
    // (tens of millimetres) while remaining unambiguously nonzero in both axes.
    let shift_left_m = 0.01;
    let shift_top_m = 0.02;
    let shifted_paper_center = format!(
        r#"paper_center: {{ toward_left_corner: "{shift_left_m}m", toward_top_corner: "{}m" }}"#,
        -0.353553391 + shift_top_m,
    );
    let shifted_source = HOLLOW.replacen(ORIGINAL_PAPER_CENTER, &shifted_paper_center, 1);

    let baseline = ValidatedTarget::parse_json5(HOLLOW.as_bytes()).unwrap();
    let shifted = ValidatedTarget::parse_json5(shifted_source.as_bytes())
        .expect("shifted manifest is still a valid target");

    for pose in test_poses() {
        let baseline_posed = baseline.posed(pose);
        let shifted_posed = shifted.posed(pose);

        // The plate itself has not moved.
        for (name, baseline_point, shifted_point) in [
            ("center", baseline_posed.center(), shifted_posed.center()),
            (
                "top corner",
                baseline_posed.top_corner(),
                shifted_posed.top_corner(),
            ),
            (
                "bottom corner",
                baseline_posed.bottom_corner(),
                shifted_posed.bottom_corner(),
            ),
            (
                "left corner",
                baseline_posed.left_corner(),
                shifted_posed.left_corner(),
            ),
            (
                "right corner",
                baseline_posed.right_corner(),
                shifted_posed.right_corner(),
            ),
        ] {
            assert!(
                (baseline_point - shifted_point).norm() < TOL,
                "plate {name} moved after sliding the paper: {baseline_point:?} -> {shifted_point:?}"
            );
        }

        // Every marker corner must move by exactly the stated shift, expressed in the
        // sensor frame via this pose's own axes.
        let axes = shifted_posed.axes();
        let expected_shift =
            axes.toward_left_corner.scale(shift_left_m) + axes.toward_top_corner.scale(shift_top_m);

        let before = baseline_posed.marker_corners_by_id();
        let after = shifted_posed.marker_corners_by_id();
        let marker_ids: Vec<u32> = before.keys().copied().collect();
        assert_eq!(marker_ids, after.keys().copied().collect::<Vec<_>>());

        for marker_id in marker_ids {
            let before_corners = &before[&marker_id];
            let after_corners = &after[&marker_id];
            for (index, (was, now)) in before_corners.iter().zip(after_corners).enumerate() {
                let name = ["right", "top", "left", "bottom"][index];
                let expected = was + expected_shift;
                assert!(
                    (now - expected).norm() < TOL,
                    "marker {marker_id} {name} after sliding the paper: \
                     got {now:?}, expected {expected:?}"
                );
            }
        }
    }
}

struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn range(&mut self, low: f64, high: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let unit = (self.0 >> 11) as f64 / (1_u64 << 53) as f64;
        low + (high - low) * unit
    }
}
