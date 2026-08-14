mod common;

use board_cluster_detector::{
    config::IsolationBand,
    geometry::PlaneModel,
    pose::{board_pose, isolation_density, stance_3d},
};
use nalgebra::{Point3, Vector3};

/// A vertical plane at x=2 (sensor at the origin), in-plane u=y, v=z.
fn vertical_plane_at_x2() -> PlaneModel {
    PlaneModel {
        center: Point3::new(2.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
        u: Vector3::new(0.0, 1.0, 0.0),
        v: Vector3::new(0.0, 0.0, 1.0),
    }
}

/// Diamond corners (top / left / bottom / right) in the plane's (u, v) basis.
const DIAMOND_UV: [[f64; 2]; 4] = [[0.0, 0.7], [-0.7, 0.0], [0.0, -0.7], [0.7, 0.0]];

#[test]
fn board_pose_normal_faces_sensor_and_winds_ccw() {
    let plane = vertical_plane_at_x2();
    let det = board_pose(&plane, &DIAMOND_UV, 1.0, [0.0, 0.0, 1.0]);
    // REP-103 frame: +X is the normal pointing AWAY from the sensor, so for a
    // board at x=+2 facing the origin the sensor-facing normal is -x and the
    // board's +X is +x. (Pre-REP-103 this was `rotation.column(2).x < 0.0`.)
    assert!(det.rotation.column(0).x > 0.0);
    // center ~ plane center
    assert!((det.center.x - 2.0).abs() < 1e-9);
}

/// Pins the REP-103 / Autoware convention: X forward (away from the sensor),
/// Y left, Z up. The fixture is chosen so the expected frame is the identity.
#[test]
fn board_pose_frame_is_x_forward_y_left_z_up() {
    let plane = vertical_plane_at_x2();
    let det = board_pose(&plane, &DIAMOND_UV, 1.0, [0.0, 0.0, 1.0]);
    let r = det.rotation;

    // Column 0 = X = -n = away from the sensor at the origin.
    let x: Vector3<f64> = r.column(0).into_owned();
    assert!((x - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9, "X = {x:?}");
    // Column 2 = Z = toward the up-most corner (this diamond's top is at +z).
    let z: Vector3<f64> = r.column(2).into_owned();
    assert!((z - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-9, "Z = {z:?}");
    // Column 1 = Y = Z x X = left.
    let y: Vector3<f64> = r.column(1).into_owned();
    assert!((y - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-9, "Y = {y:?}");
}

/// Convention pin that does not depend on the fixture being axis-aligned:
/// orthonormal, right-handed, X away from the sensor, Z along the up-most
/// corner, on a tilted plane with a non-world-Z `up`.
#[test]
fn board_pose_rotation_is_orthonormal_right_handed_on_a_tilted_plane() {
    // Plane whose normal is tilted in x/y, centered off-axis; up axis = +z.
    let n = Vector3::new(1.0, 0.6, -0.2).normalize();
    let u = Vector3::new(-0.6, 1.0, 0.0).normalize();
    let v = n.cross(&u).normalize();
    let plane = PlaneModel {
        center: Point3::new(2.0, 1.2, 0.4),
        normal: n,
        u,
        v,
    };
    let up = [0.0, 0.0, 1.0];
    // A diamond in the (u, v) basis so there is an unambiguous up-most corner.
    let det = board_pose(&plane, &DIAMOND_UV, 1.0, up);
    let r = det.rotation;

    // Orthonormal, right-handed.
    let should_be_identity = r.transpose() * r;
    assert!(
        (should_be_identity - nalgebra::Matrix3::<f64>::identity()).norm() < 1e-9,
        "R^T R = {should_be_identity:?}"
    );
    assert!(
        (r.determinant() - 1.0).abs() < 1e-9,
        "det = {}",
        r.determinant()
    );

    // X points AWAY from the sensor at the origin: the board center is on the
    // +X side of the board plane.
    let x: Vector3<f64> = r.column(0).into_owned();
    assert!(
        x.dot(&det.center.coords) > 0.0,
        "X must point away from the sensor, got {x:?} vs center {:?}",
        det.center
    );

    // Z is in-plane (perpendicular to the normal) and points up-ish, toward the
    // up-most corner.
    let z: Vector3<f64> = r.column(2).into_owned();
    assert!(z.dot(&x).abs() < 1e-9, "Z must be in the board plane");
    let up_v = Vector3::new(up[0], up[1], up[2]);
    assert!(z.dot(&up_v) > 0.0, "Z must point up-ish, got {z:?}");
    let top = det
        .corners_3d
        .iter()
        .copied()
        .fold(det.corners_3d[0], |best, c| {
            if c.coords.dot(&up_v) > best.coords.dot(&up_v) {
                c
            } else {
                best
            }
        });
    let toward_top = {
        let mut t = top - det.center;
        t -= x * t.dot(&x);
        t.normalize()
    };
    assert!(
        (z - toward_top).norm() < 1e-9,
        "Z = {z:?} must aim at the up-most corner direction {toward_top:?}"
    );
}

/// The corner winding is CCW about the SENSOR-FACING normal (-X), i.e. CCW as
/// seen from the sensor and clockwise about the frame's own +X. This is the
/// pre-REP-103 physical ordering, preserved deliberately.
#[test]
fn board_pose_corners_wind_ccw_about_minus_x() {
    let plane = vertical_plane_at_x2();
    let det = board_pose(&plane, &DIAMOND_UV, 1.0, [0.0, 0.0, 1.0]);
    let x: Vector3<f64> = det.rotation.column(0).into_owned();

    // Consecutive edge cross products of a convex quad all point along the
    // winding axis. CCW about -X => they oppose +X.
    for i in 0..4 {
        let a = det.corners_3d[(i + 1) % 4] - det.corners_3d[i];
        let b = det.corners_3d[(i + 2) % 4] - det.corners_3d[(i + 1) % 4];
        let cross = a.cross(&b);
        assert!(
            cross.dot(&x) < 0.0,
            "edge {i}: winding must be CCW about -X (CW about +X), got {cross:?}"
        );
    }

    // And the exact sequence, so a future reversal cannot slip through:
    // left (-y), top (+z), right (+y), bottom (-z).
    let expect = [
        Point3::new(2.0, -0.7, 0.0),
        Point3::new(2.0, 0.0, 0.7),
        Point3::new(2.0, 0.7, 0.0),
        Point3::new(2.0, 0.0, -0.7),
    ];
    for (got, want) in det.corners_3d.iter().zip(expect.iter()) {
        assert!((got - want).norm() < 1e-9, "corners = {:?}", det.corners_3d);
    }

    // Opposite corners are two apart, which is what `stance_3d` relies on.
    let d1 = det.corners_3d[2] - det.corners_3d[0];
    let d2 = det.corners_3d[3] - det.corners_3d[1];
    assert!(d1.dot(&d2).abs() < 1e-9, "diagonals must be perpendicular");
}

#[test]
fn pose_corners_parity_against_python() {
    // Compare pose corners only where Python detected AND this port also detects
    // in agreement -- the documented, controller-accepted per-frame divergences
    // (`common::KNOWN_PER_FRAME_MISMATCHES`, RNG-driven) are frames Rust rejects,
    // so there are no corners to compare. See detect_parity.rs / task-9 report.
    for f in common::load_all().into_iter().filter(|f| {
        f.golden.detected && !common::KNOWN_PER_FRAME_MISMATCHES.contains(&f.name.as_str())
    }) {
        common::assert_pose_corners_parity(&f); // corners_3d set within a few cm of Python
    }
}

#[test]
fn stance_3d_diamond_on_corner_is_near_one() {
    // Same diamond as above, unprojected into the x=2 vertical plane, CCW.
    let plane = PlaneModel {
        center: Point3::new(2.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
        u: Vector3::new(0.0, 1.0, 0.0),
        v: Vector3::new(0.0, 0.0, 1.0),
    };
    let corners = [[0.0, 0.7], [-0.7, 0.0], [0.0, -0.7], [0.7, 0.0]];
    let det = board_pose(&plane, &corners, 1.0, [0.0, 0.0, 1.0]);
    let stance = stance_3d(&det.corners_3d, [0.0, 0.0, 1.0]);
    assert!((stance - 1.0).abs() < 1e-6, "stance = {stance}");
}

#[test]
fn stance_3d_axis_aligned_square_is_about_point_seven_one() {
    // Axis-aligned square in the x=2 vertical plane (u=y, v=z): corners at
    // the square's actual corners (not a diamond), so both diagonals sit at
    // ~45deg off vertical.
    let plane = PlaneModel {
        center: Point3::new(2.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
        u: Vector3::new(0.0, 1.0, 0.0),
        v: Vector3::new(0.0, 0.0, 1.0),
    };
    let corners = [[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]];
    let det = board_pose(&plane, &corners, 1.0, [0.0, 0.0, 1.0]);
    let stance = stance_3d(&det.corners_3d, [0.0, 0.0, 1.0]);
    assert!(
        (stance - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6,
        "stance = {stance}"
    );
}

#[test]
fn isolation_density_free_standing_quad_is_zero() {
    let plane = PlaneModel {
        center: Point3::new(2.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
        u: Vector3::new(0.0, 1.0, 0.0),
        v: Vector3::new(0.0, 0.0, 1.0),
    };
    let corners_2d = [[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]];
    // A grid of points strictly inside the quad -- own board points, no
    // exterior continuation.
    let mut dn = vec![];
    let mut u = -0.4;
    while u <= 0.4 {
        let mut v = -0.4;
        while v <= 0.4 {
            dn.push(Point3::new(2.0, u, v));
            v += 0.1;
        }
        u += 0.1;
    }
    let density = isolation_density(
        &dn,
        &plane,
        &corners_2d,
        &IsolationBand {
            coplanar_tol: 0.03,
            lo: 0.05,
            hi: 0.30,
        },
    );
    assert_eq!(density, 0.0);
}

#[test]
fn isolation_density_coplanar_wall_past_edge_is_positive() {
    let plane = PlaneModel {
        center: Point3::new(2.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
        u: Vector3::new(0.0, 1.0, 0.0),
        v: Vector3::new(0.0, 0.0, 1.0),
    };
    let corners_2d = [[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]];
    // A coplanar "wall" of points extending past the u=0.5 edge, in the
    // 0.05-0.30 exterior band, still coplanar (same x=2 plane).
    let mut dn = vec![];
    let mut u = 0.55;
    while u <= 0.75 {
        let mut v = -0.4;
        while v <= 0.4 {
            dn.push(Point3::new(2.0, u, v));
            v += 0.1;
        }
        u += 0.05;
    }
    let density = isolation_density(
        &dn,
        &plane,
        &corners_2d,
        &IsolationBand {
            coplanar_tol: 0.03,
            lo: 0.05,
            hi: 0.30,
        },
    );
    assert!(density > 0.0, "density = {density}");
}
