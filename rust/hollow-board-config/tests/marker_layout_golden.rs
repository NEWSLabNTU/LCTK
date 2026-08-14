//! World-coordinate golden for the ArUco marker layout on the calibration board.
//!
//! # Why this test exists
//!
//! `BoardModel::multi_marker_corners` is a **cross-language contract**: the Rust
//! detector and the two Python solvers each reimplement it, and all three must agree
//! on where every marker corner sits **in the world**. The fixture
//! `fixtures/board/marker_corners_world.golden.json` is that contract, and it is keyed by
//! ArUco marker *id* — the binding whose corruption produces a silent quarter-turn
//! of the solved extrinsic.
//!
//! # Why it is expressed in world coordinates
//!
//! The board model's canonical *local* frame is being redefined (see
//! `docs/superpowers/specs/2026-08-13-corner-aligned-board-frame.md`): its in-plane
//! axes move from the plate's edges to its diagonals, and its origin from a corner to
//! the plate centre. Local coordinates therefore change by construction and cannot
//! pin anything across that change.
//!
//! The **physical** board does not move. So this test states a physical mounting —
//! plate centre, board normal, and which way the up-most corner points — and asserts
//! world positions. The fixture must pass **byte-identical before and after the frame
//! change**. Only [`board_at_mounting`] below is allowed to change, and the plate-corner
//! assertions guard it: if the reconstructed pose is wrong by the very 45° this work is
//! about, those assertions fail before the marker comparison is even reached.
//!
//! A golden that gets re-baselined at the moment of the flip verifies nothing. Do not
//! regenerate this fixture as part of the frame change.
//!
//! # Where the fixture's numbers come from
//!
//! `fixtures/board/generate_marker_corners_world.py` — an independent Python derivation from
//! the printed pattern's dimensions and the stated mounting. It does not call the Rust
//! code. Regenerate it only when the *printed board* changes.

use aruco_config::{ArucoDictionary, MultiArucoPattern};
use hollow_board_config::{BoardModel, BoardShape, MarkerPaperPlacement};
use measurements::Length;
use nalgebra as na;
use noisy_float::prelude::*;
use serde::Deserialize;

/// The fixture lives outside this crate on purpose: it is a contract between the Rust
/// detector and the Python camera solver, and a contract should not be a guest in one
/// side's test tree. Both languages read `fixtures/board/` at the repository root.
const GOLDEN: &str =
    include_str!("../../../fixtures/board/marker_corners_world.golden.json");

/// Positional tolerance, in metres. The fixture carries exact decimal geometry, so the
/// only error in play is f64 round-off through a rotation and a translation.
const TOL_M: f64 = 1e-9;

#[derive(Debug, Deserialize)]
struct Golden {
    board_width_m: f64,
    marker_paper_size_m: f64,
    pattern: GoldenPattern,
    mounting: Mounting,
    plate_corners: PlateCorners,
    /// Marker id -> its four corners, in the order `multi_marker_corners` returns them:
    /// `[right, top, left, bottom]`.
    markers: std::collections::HashMap<String, Vec<[f64; 3]>>,
}

#[derive(Debug, Deserialize)]
struct GoldenPattern {
    marker_ids: Vec<u32>,
    board_border_size_m: f64,
    num_squares_per_side: u32,
    marker_square_size_ratio: f64,
}

/// The physical mounting, stated in world coordinates. Survives the frame change.
#[derive(Debug, Deserialize)]
struct Mounting {
    /// Centre of the square plate.
    plate_center: [f64; 3],
    /// Board normal, pointing toward the sensor.
    normal: [f64; 3],
    /// Unit vector from the plate centre toward the physically up-most corner.
    up_diagonal: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct PlateCorners {
    top: [f64; 3],
    bottom: [f64; 3],
    left: [f64; 3],
    right: [f64; 3],
    center: [f64; 3],
}

fn point(p: [f64; 3]) -> na::Point3<f64> {
    na::Point3::new(p[0], p[1], p[2])
}

fn vector(v: [f64; 3]) -> na::Vector3<f64> {
    na::Vector3::new(v[0], v[1], v[2])
}

fn assert_close(actual: &na::Point3<f64>, expected: &na::Point3<f64>, what: &str) {
    let err = (actual - expected).norm();
    assert!(
        err < TOL_M,
        "{what}: got {actual:?}, expected {expected:?} (error {err:e} m)"
    );
}

/// Builds the board model for a physical mounting, **in the board model's current
/// canonical local frame**.
///
/// This is the one function the frame change is allowed to rewrite, and it has now been
/// rewritten. Everything else in this file — and the fixture — survived untouched, which
/// is the point: the same golden passed before and after.
///
/// Canonical (corner-aligned) frame: the origin is the plate *centre*, +X runs from the
/// centre toward the left corner and +Y from the centre toward the top corner, each
/// reaching a corner at the half-diagonal `W/√2`.
fn board_at_mounting(mounting: &Mounting, width: f64, marker_paper_size: f64) -> BoardModel {
    let center = point(mounting.plate_center);
    let normal = na::Unit::new_normalize(vector(mounting.normal));
    let up = na::Unit::new_normalize(vector(mounting.up_diagonal));
    // Toward the left corner: completes a right-handed (left, up, normal) triple.
    let left_dir = na::Unit::new_normalize(up.cross(&normal));

    let rotation = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(
        na::Matrix3::from_columns(&[left_dir.into_inner(), up.into_inner(), normal.into_inner()]),
    ));

    BoardModel {
        pose: na::Isometry3::from_parts(
            na::Translation3::from(center - na::Point3::origin()),
            rotation,
        ),
        marker_paper_size: Length::from_meters(marker_paper_size),
        marker_paper_placement: MarkerPaperPlacement::flush_with_bottom_corner(
            Length::from_meters(width),
            Length::from_meters(marker_paper_size),
        ),
        board_shape: BoardShape {
            board_width: Length::from_meters(width),
            // Irrelevant to marker layout; kept plausible so the model is well formed.
            hole_radius: Length::from_meters(0.15),
            hole_center_shift: Length::from_meters(0.2),
        },
    }
}

/// The printed pattern, exactly as the fixture describes it.
fn pattern_from(golden: &Golden) -> MultiArucoPattern {
    MultiArucoPattern {
        marker_ids: golden.pattern.marker_ids.clone(),
        dictionary: ArucoDictionary::DICT_5X5_1000,
        board_size: Length::from_meters(golden.marker_paper_size_m),
        board_border_size: Length::from_meters(golden.pattern.board_border_size_m),
        marker_square_size_ratio: r64(golden.pattern.marker_square_size_ratio),
        num_squares_per_side: golden.pattern.num_squares_per_side,
        border_bits: 1,
        // The board model carries the placement for this test; the pattern's copy is the
        // config-file channel and is deliberately left unset here.
        paper_placement: None,
    }
}

#[test]
fn plate_corners_land_at_the_stated_physical_mounting() {
    let golden: Golden = serde_json::from_str(GOLDEN).expect("golden fixture parses");
    let board = board_at_mounting(
        &golden.mounting,
        golden.board_width_m,
        golden.marker_paper_size_m,
    );

    assert_close(
        &board.board_center(),
        &point(golden.plate_corners.center),
        "plate centre",
    );
    assert_close(
        &board.top_corner(),
        &point(golden.plate_corners.top),
        "top corner",
    );
    assert_close(
        &board.bottom_corner(),
        &point(golden.plate_corners.bottom),
        "bottom corner",
    );
    assert_close(
        &board.left_corner(),
        &point(golden.plate_corners.left),
        "left corner",
    );
    assert_close(
        &board.right_corner(),
        &point(golden.plate_corners.right),
        "right corner",
    );
}

#[test]
fn marker_corners_match_the_world_golden_keyed_by_marker_id() {
    let golden: Golden = serde_json::from_str(GOLDEN).expect("golden fixture parses");
    let board = board_at_mounting(
        &golden.mounting,
        golden.board_width_m,
        golden.marker_paper_size_m,
    );

    let pattern = pattern_from(&golden);

    let computed = board.multi_marker_corners(&pattern);
    assert_eq!(
        computed.len(),
        golden.pattern.marker_ids.len(),
        "one corner set per marker id"
    );

    // `multi_marker_corners` returns corner sets in the same order as `marker_ids`,
    // which is x-major by (x, y). That positional binding is what makes the fixture's
    // id keys meaningful, so name it here rather than letting it stay implicit.
    for (&marker_id, corners) in golden.pattern.marker_ids.iter().zip(&computed) {
        let expected = golden
            .markers
            .get(&marker_id.to_string())
            .unwrap_or_else(|| panic!("golden has no entry for marker {marker_id}"));
        assert_eq!(
            corners.len(),
            expected.len(),
            "marker {marker_id}: corner count"
        );
        for (index, (actual, want)) in corners.iter().zip(expected).enumerate() {
            let name = ["right", "top", "left", "bottom"][index];
            assert_close(actual, &point(*want), &format!("marker {marker_id} {name}"));
        }
    }
}

/// The marker paper's position on the plate is a **measurement**, not a derivation from
/// the plate's width. This pins that it is honoured: sliding the stated placement must
/// slide every marker corner by exactly the same world vector, and do nothing else.
///
/// Without this, a placement field could be plumbed through and silently ignored — the
/// failure mode that leaves Rust and the Python solvers disagreeing about where the
/// markers are while every other test still passes.
#[test]
fn marker_corners_follow_the_stated_paper_placement() {
    let golden: Golden = serde_json::from_str(GOLDEN).expect("golden fixture parses");
    let pattern = pattern_from(&golden);

    let baseline = board_at_mounting(
        &golden.mounting,
        golden.board_width_m,
        golden.marker_paper_size_m,
    );

    // Slide the paper 0.1 m toward the plate's top corner and 0.03 m toward its left
    // corner, leaving the plate itself untouched.
    let shift_up = 0.1;
    let shift_left = 0.03;
    let mut shifted = baseline.clone();
    shifted.marker_paper_placement = MarkerPaperPlacement {
        toward_left_corner: baseline.marker_paper_placement.toward_left_corner
            + Length::from_meters(shift_left),
        toward_top_corner: baseline.marker_paper_placement.toward_top_corner
            + Length::from_meters(shift_up),
    };

    let up = na::Unit::new_normalize(vector(golden.mounting.up_diagonal));
    let normal = na::Unit::new_normalize(vector(golden.mounting.normal));
    let left = na::Unit::new_normalize(up.cross(&normal));
    let expected_shift = up.scale(shift_up) + left.scale(shift_left);

    // The plate has not moved.
    assert_close(
        &shifted.board_center(),
        &point(golden.plate_corners.center),
        "plate centre after sliding the paper",
    );
    assert_close(
        &shifted.top_corner(),
        &point(golden.plate_corners.top),
        "top corner after sliding the paper",
    );

    let before = baseline.multi_marker_corners(&pattern);
    let after = shifted.multi_marker_corners(&pattern);
    for (marker_index, (was, now)) in before.iter().zip(&after).enumerate() {
        let marker_id = golden.pattern.marker_ids[marker_index];
        for (corner_index, (was, now)) in was.iter().zip(now).enumerate() {
            let name = ["right", "top", "left", "bottom"][corner_index];
            assert_close(
                now,
                &(was + expected_shift),
                &format!("marker {marker_id} {name} after sliding the paper"),
            );
        }
    }
}
