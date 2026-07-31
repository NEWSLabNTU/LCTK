mod common;

use board_projection_detector::{
    geometry::PlaneModel,
    pose::{board_pose, isolation_density, stance_3d},
};
use nalgebra::{Point3, Vector3};

#[test]
fn board_pose_normal_faces_sensor_and_winds_ccw() {
    // vertical plane at x=2, in-plane u=y, v=z
    let plane = PlaneModel {
        center: Point3::new(2.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
        u: Vector3::new(0.0, 1.0, 0.0),
        v: Vector3::new(0.0, 0.0, 1.0),
    };
    // diamond corners (top/left/bottom/right) in (u,v)
    let corners = [[0.0, 0.7], [-0.7, 0.0], [0.0, -0.7], [0.7, 0.0]];
    let det = board_pose(&plane, &corners, 1.0, [0.0, 0.0, 1.0]);
    // normal faces origin -> -x
    assert!(det.rotation.column(2).x < 0.0);
    // center ~ plane center
    assert!((det.center.x - 2.0).abs() < 1e-9);
}

#[test]
fn pose_corners_parity_against_python() {
    for f in common::load_all().into_iter().filter(|f| f.golden.detected) {
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
    let density = isolation_density(&dn, &plane, &corners_2d);
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
    let density = isolation_density(&dn, &plane, &corners_2d);
    assert!(density > 0.0, "density = {density}");
}
