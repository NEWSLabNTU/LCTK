//! Validated, immutable physical definitions of calibration targets.
//!
//! This crate deliberately owns only Target Definition parsing, validation and semantic
//! identity. Surface projection and posed geometry belong to the next migration packet.

use anyhow::{bail, Context, Result};
use serde::Deserialize;
use sha2::{Digest, Sha256};

const DIAMOND_TOLERANCE_UM: i64 = 2;

/// The stable Target Definition seam.  `CalibrationTarget` remains as the compatibility
/// name while callers migrate.
pub type ValidatedTarget = CalibrationTarget;

#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationTarget {
    schema_version: u32,
    target_id: String,
    revision: u32,
    board_frame_convention: BoardFrameConvention,
    plate: Plate,
    fiducial: Fiducial,
    lidar_orientation_reference: LidarOrientationReference,
    identity: TargetIdentity,
}

impl CalibrationTarget {
    pub fn parse_json5(bytes: &[u8]) -> Result<Self> {
        Self::from_json5(bytes)
    }

    pub fn from_json5(bytes: &[u8]) -> Result<Self> {
        let source = std::str::from_utf8(bytes).context("Target Definition is not UTF-8")?;
        let raw: RawTarget = json5::from_str(source).context("invalid Target Definition JSON5")?;
        Self::validate(raw)
    }

    pub fn identity(&self) -> &TargetIdentity {
        &self.identity
    }

    pub fn target_id(&self) -> &str {
        &self.target_id
    }

    pub fn revision(&self) -> u32 {
        self.revision
    }

    pub fn board_frame_convention(&self) -> &str {
        self.board_frame_convention.as_str()
    }

    pub fn plate(&self) -> &Plate {
        &self.plate
    }

    pub fn fiducial(&self) -> &Fiducial {
        &self.fiducial
    }

    pub fn lidar_orientation_reference(&self) -> &LidarOrientationReference {
        &self.lidar_orientation_reference
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut output = String::new();
        macro_rules! field {
            ($name:expr, $value:expr) => {
                // `name:length:value\n` is intentionally a tiny fixed grammar, not a
                // serialized map. Length-prefixing makes arbitrary string values
                // unambiguous without depending on JSON escaping or map ordering.
                let name = $name.to_string();
                let value = $value.to_string();
                output.push_str(&name);
                output.push(':');
                output.push_str(&value.len().to_string());
                output.push(':');
                output.push_str(&value);
                output.push('\n');
            };
        }
        field!("schema_version", self.schema_version);
        field!("target_id", self.target_id);
        field!("revision", self.revision);
        field!(
            "board_frame_convention",
            self.board_frame_convention.as_str()
        );
        field!("plate.side_um", self.plate.side_um);
        field!("plate.surface.kind", self.plate.surface.kind_name());
        if let Surface::Perforated { circular_cutouts } = &self.plate.surface {
            for (index, cutout) in circular_cutouts.iter().enumerate() {
                field!(
                    format!("plate.surface.circular_cutouts[{index}].x_um"),
                    cutout.x_um
                );
                field!(
                    format!("plate.surface.circular_cutouts[{index}].y_um"),
                    cutout.y_um
                );
                field!(
                    format!("plate.surface.circular_cutouts[{index}].radius_um"),
                    cutout.radius_um
                );
            }
        }
        field!("fiducial.kind", self.fiducial.kind.as_str());
        field!("fiducial.dictionary", self.fiducial.dictionary.as_str());
        for (index, marker_id) in self.fiducial.marker_ids.iter().enumerate() {
            field!(format!("fiducial.marker_ids[{index}]"), marker_id);
        }
        field!("fiducial.paper_side_um", self.fiducial.paper_side_um);
        field!(
            "fiducial.paper_center.toward_left_corner_um",
            self.fiducial.paper_center_x_um
        );
        field!(
            "fiducial.paper_center.toward_top_corner_um",
            self.fiducial.paper_center_y_um
        );
        field!("fiducial.outer_border_um", self.fiducial.outer_border_um);
        field!("fiducial.cells_per_side", self.fiducial.cells_per_side);
        // IEEE-754 bits, rather than a formatter-dependent decimal spelling.  JSON5
        // spellings such as `0.8` and `0.80` deserialize to the same semantic value.
        field!(
            "fiducial.marker_fill_ratio_f64_bits",
            self.fiducial.marker_fill_ratio.to_bits()
        );
        field!("fiducial.border_bits", self.fiducial.border_bits);
        field!(
            "lidar_orientation_reference.kind",
            self.lidar_orientation_reference.kind_name()
        );
        if let LidarOrientationReference::MountingUp { local_axis } =
            self.lidar_orientation_reference
        {
            field!(
                "lidar_orientation_reference.local_axis",
                local_axis.as_str()
            );
        }
        output.into_bytes()
    }

    fn validate(raw: RawTarget) -> Result<Self> {
        if raw.schema_version != 1 {
            bail!("schema_version: expected 1, got {}", raw.schema_version);
        }
        if raw.target_id.trim().is_empty() {
            bail!("target_id: must not be empty");
        }
        if raw.revision == 0 {
            bail!("revision: must be greater than zero");
        }
        let board_frame_convention = BoardFrameConvention::parse(&raw.board_frame_convention)?;
        let side_um = parse_positive_length("plate.side", &raw.plate.side)?;
        let surface = Surface::validate(raw.plate.surface, side_um)?;
        let fiducial = Fiducial::validate(raw.fiducial, side_um, &surface)?;
        let lidar_orientation_reference =
            LidarOrientationReference::validate(raw.lidar_orientation_reference, &surface)?;
        let mut target = Self {
            schema_version: raw.schema_version,
            target_id: raw.target_id,
            revision: raw.revision,
            board_frame_convention,
            plate: Plate { side_um, surface },
            fiducial,
            lidar_orientation_reference,
            identity: TargetIdentity::placeholder(),
        };
        target.identity = TargetIdentity::from_canonical(&target);
        Ok(target)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TargetIdentity {
    pub schema_version: u32,
    pub target_id: String,
    pub revision: u32,
    pub semantic_sha256: String,
    pub board_frame_convention: String,
}

impl TargetIdentity {
    fn placeholder() -> Self {
        Self {
            schema_version: 0,
            target_id: String::new(),
            revision: 0,
            semantic_sha256: String::new(),
            board_frame_convention: String::new(),
        }
    }

    fn from_canonical(target: &CalibrationTarget) -> Self {
        let digest = Sha256::digest(target.canonical_bytes());
        Self {
            schema_version: target.schema_version,
            target_id: target.target_id.clone(),
            revision: target.revision,
            semantic_sha256: format!("{digest:x}"),
            board_frame_convention: target.board_frame_convention.as_str().to_owned(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Plate {
    pub side_um: i64,
    pub surface: Surface,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Surface {
    Solid,
    Perforated {
        circular_cutouts: Vec<CircularCutout>,
    },
}

impl Surface {
    fn kind_name(&self) -> &'static str {
        match self {
            Self::Solid => "solid",
            Self::Perforated { .. } => "perforated",
        }
    }

    fn validate(raw: RawSurface, side_um: i64) -> Result<Self> {
        match raw {
            RawSurface::Solid {} => Ok(Self::Solid),
            RawSurface::Perforated { circular_cutouts } => {
                if circular_cutouts.is_empty() {
                    bail!("plate.surface.circular_cutouts: perforated surface needs at least one cutout");
                }
                let mut cutouts = circular_cutouts
                    .into_iter()
                    .enumerate()
                    .map(|(index, raw)| CircularCutout::validate(index, raw, side_um))
                    .collect::<Result<Vec<_>>>()?;
                cutouts.sort_unstable_by_key(|cutout| (cutout.x_um, cutout.y_um, cutout.radius_um));
                for left in 0..cutouts.len() {
                    for right in left + 1..cutouts.len() {
                        let dx = i128::from(cutouts[left].x_um) - i128::from(cutouts[right].x_um);
                        let dy = i128::from(cutouts[left].y_um) - i128::from(cutouts[right].y_um);
                        let radius_sum = cutouts[left]
                            .radius_um
                            .checked_add(cutouts[right].radius_um)
                            .context("plate.surface.circular_cutouts: radii are too large")?;
                        if (dx as f64).hypot(dy as f64) <= radius_sum as f64 {
                            bail!("plate.surface.circular_cutouts: cutouts {left} and {right} overlap");
                        }
                    }
                }
                if quarter_turn_invariant(&cutouts)? {
                    bail!(
                        "plate.surface.circular_cutouts: geometry must break quarter-turn symmetry"
                    );
                }
                Ok(Self::Perforated {
                    circular_cutouts: cutouts,
                })
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CircularCutout {
    pub x_um: i64,
    pub y_um: i64,
    pub radius_um: i64,
}

impl CircularCutout {
    fn validate(index: usize, raw: RawCutout, side_um: i64) -> Result<Self> {
        let x_um = parse_length(
            &format!("plate.surface.circular_cutouts[{index}].center.x"),
            &raw.center.x,
        )?;
        let y_um = parse_length(
            &format!("plate.surface.circular_cutouts[{index}].center.y"),
            &raw.center.y,
        )?;
        let radius_um = parse_positive_length(
            &format!("plate.surface.circular_cutouts[{index}].radius"),
            &raw.radius,
        )?;
        let half_diagonal_um = side_um as f64 / 2f64.sqrt();
        if abs_i64_as_f64(x_um) + abs_i64_as_f64(y_um) + radius_um as f64 * 2f64.sqrt()
            > half_diagonal_um + DIAMOND_TOLERANCE_UM as f64
        {
            bail!("plate.surface.circular_cutouts[{index}]: cutout extends outside plate");
        }
        Ok(Self {
            x_um,
            y_um,
            radius_um,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Fiducial {
    pub kind: FiducialKind,
    pub dictionary: ArucoDictionary,
    pub marker_ids: Vec<u32>,
    pub paper_side_um: i64,
    pub paper_center_x_um: i64,
    pub paper_center_y_um: i64,
    pub outer_border_um: i64,
    pub cells_per_side: u32,
    pub marker_fill_ratio: f64,
    pub border_bits: u32,
}

impl Fiducial {
    fn validate(raw: RawFiducial, plate_side_um: i64, surface: &Surface) -> Result<Self> {
        let kind = FiducialKind::parse(&raw.kind)?;
        let dictionary = ArucoDictionary::parse(&raw.dictionary)?;
        if raw.cells_per_side == 0 {
            bail!("fiducial.cells_per_side: must be greater than zero");
        }
        let expected =
            raw.cells_per_side
                .checked_mul(raw.cells_per_side)
                .context("fiducial.cells_per_side: square overflows u32")? as usize;
        if raw.marker_ids.len() != expected {
            bail!(
                "fiducial.marker_ids: expected {expected} IDs for {}x{} grid, got {}",
                raw.cells_per_side,
                raw.cells_per_side,
                raw.marker_ids.len()
            );
        }
        let mut ids = std::collections::HashSet::new();
        for id in &raw.marker_ids {
            if *id >= dictionary.capacity() {
                bail!(
                    "fiducial.marker_ids: ID {id} is outside {}",
                    dictionary.as_str()
                );
            }
            if !ids.insert(*id) {
                bail!("fiducial.marker_ids: duplicate ID {id}");
            }
        }
        let paper_side_um = parse_positive_length("fiducial.paper_side", &raw.paper_side)?;
        let paper_center_x_um = parse_length(
            "fiducial.paper_center.toward_left_corner",
            &raw.paper_center.toward_left_corner,
        )?;
        let paper_center_y_um = parse_length(
            "fiducial.paper_center.toward_top_corner",
            &raw.paper_center.toward_top_corner,
        )?;
        let outer_border_um = parse_length("fiducial.outer_border", &raw.outer_border)?;
        if outer_border_um < 0 {
            bail!("fiducial.outer_border: must not be negative");
        }
        if outer_border_um
            .checked_mul(2)
            .context("fiducial.outer_border: value is too large")?
            >= paper_side_um
        {
            bail!("fiducial.outer_border: twice border must be less than paper_side");
        }
        if !raw.marker_fill_ratio.is_finite()
            || raw.marker_fill_ratio <= 0.0
            || raw.marker_fill_ratio > 1.0
        {
            bail!("fiducial.marker_fill_ratio: must be finite and in (0, 1]");
        }
        if raw.border_bits < 1 {
            bail!("fiducial.border_bits: must be at least 1");
        }
        let fiducial = Self {
            kind,
            dictionary,
            marker_ids: raw.marker_ids,
            paper_side_um,
            paper_center_x_um,
            paper_center_y_um,
            outer_border_um,
            cells_per_side: raw.cells_per_side,
            marker_fill_ratio: raw.marker_fill_ratio,
            border_bits: raw.border_bits,
        };
        fiducial.validate_inside_plate(plate_side_um)?;
        if let Surface::Perforated { circular_cutouts } = surface {
            for (index, cutout) in circular_cutouts.iter().enumerate() {
                let x_offset = cutout
                    .x_um
                    .checked_sub(fiducial.paper_center_x_um)
                    .context(
                    "fiducial.paper_center.toward_left_corner: offset is outside supported range",
                )?;
                let y_offset = cutout
                    .y_um
                    .checked_sub(fiducial.paper_center_y_um)
                    .context(
                    "fiducial.paper_center.toward_top_corner: offset is outside supported range",
                )?;
                let paper_radius_um = (fiducial.paper_side_um as f64 / 2f64.sqrt()).round() as i64;
                if distance_to_diamond(x_offset, y_offset, paper_radius_um)
                    < cutout.radius_um as f64 - DIAMOND_TOLERANCE_UM as f64
                {
                    bail!("fiducial: paper intersects circular cutout {index}");
                }
            }
        }
        Ok(fiducial)
    }

    fn validate_inside_plate(&self, plate_side_um: i64) -> Result<()> {
        let plate_radius = plate_side_um as f64 / 2f64.sqrt();
        let paper_radius = self.paper_side_um as f64 / 2f64.sqrt();
        let corners = [
            (0.0, paper_radius),
            (0.0, -paper_radius),
            (paper_radius, 0.0),
            (-paper_radius, 0.0),
        ];
        if corners.into_iter().any(|(x, y)| {
            (self.paper_center_x_um as f64 + x).abs() + (self.paper_center_y_um as f64 + y).abs()
                > plate_radius + DIAMOND_TOLERANCE_UM as f64
        }) {
            bail!("fiducial: paper corners extend outside plate");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarOrientationReference {
    MountingUp { local_axis: LocalAxis },
    AsymmetricCutouts,
}

impl LidarOrientationReference {
    fn kind_name(&self) -> &'static str {
        match self {
            Self::MountingUp { .. } => "mounting_up",
            Self::AsymmetricCutouts => "asymmetric_cutouts",
        }
    }
    fn validate(raw: RawLidarOrientationReference, surface: &Surface) -> Result<Self> {
        match raw {
            RawLidarOrientationReference::MountingUp { local_axis } => Ok(Self::MountingUp {
                local_axis: LocalAxis::parse(&local_axis)?,
            }),
            RawLidarOrientationReference::AsymmetricCutouts {} => {
                if !matches!(surface, Surface::Perforated { .. }) {
                    bail!("lidar_orientation_reference.kind: asymmetric_cutouts requires a perforated surface");
                }
                Ok(Self::AsymmetricCutouts)
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalAxis {
    PositiveY,
}
impl LocalAxis {
    fn parse(value: &str) -> Result<Self> {
        if value == "+y" {
            Ok(Self::PositiveY)
        } else {
            bail!("lidar_orientation_reference.local_axis: expected +y, got {value:?}")
        }
    }
    fn as_str(self) -> &'static str {
        "+y"
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardFrameConvention {
    CornerAlignedPlateCenterV1,
}
impl BoardFrameConvention {
    fn parse(value: &str) -> Result<Self> {
        if value == "corner_aligned_plate_center_v1" {
            Ok(Self::CornerAlignedPlateCenterV1)
        } else {
            bail!("board_frame_convention: unsupported {value:?}")
        }
    }
    fn as_str(self) -> &'static str {
        "corner_aligned_plate_center_v1"
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FiducialKind {
    SquareArucoGrid,
}
impl FiducialKind {
    fn parse(value: &str) -> Result<Self> {
        if value == "square_aruco_grid" {
            Ok(Self::SquareArucoGrid)
        } else {
            bail!("fiducial.kind: unsupported {value:?}")
        }
    }
    fn as_str(self) -> &'static str {
        "square_aruco_grid"
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArucoDictionary {
    Dict5x5x1000,
}
impl ArucoDictionary {
    fn parse(value: &str) -> Result<Self> {
        if value == "DICT_5X5_1000" {
            Ok(Self::Dict5x5x1000)
        } else {
            bail!("fiducial.dictionary: unsupported {value:?}")
        }
    }
    fn as_str(self) -> &'static str {
        "DICT_5X5_1000"
    }
    fn capacity(self) -> u32 {
        1000
    }
}

fn parse_length(field: &str, value: &str) -> Result<i64> {
    let (number, unit) = if let Some(number) = value.strip_suffix("mm") {
        (number, 1.0)
    } else if let Some(number) = value.strip_suffix('m') {
        (number, 1000.0)
    } else {
        bail!("{field}: expected a length ending in mm or m");
    };
    let parsed: f64 = number
        .trim()
        .parse()
        .with_context(|| format!("{field}: invalid length {value:?}"))?;
    if !parsed.is_finite() {
        bail!("{field}: length must be finite");
    }
    let micrometres = parsed * unit * 1000.0;
    let rounded = micrometres.round();
    // Target Identity defines length semantics at a micrometre.  This also captures
    // legacy derived dimensions such as 200 mm * sqrt(2) as 282843 um.
    if rounded.abs() > i64::MAX as f64 {
        bail!("{field}: length is outside supported range");
    }
    Ok(rounded as i64)
}
fn parse_positive_length(field: &str, value: &str) -> Result<i64> {
    let length = parse_length(field, value)?;
    if length <= 0 {
        bail!("{field}: must be positive");
    }
    Ok(length)
}
fn abs_i64_as_f64(value: i64) -> f64 {
    i128::from(value).abs() as f64
}
fn distance_to_diamond(x_um: i64, y_um: i64, radius_um: i64) -> f64 {
    let x = abs_i64_as_f64(x_um);
    let y = abs_i64_as_f64(y_um);
    let radius = radius_um as f64;
    let sum = x + y;
    if sum <= radius {
        0.0
    } else {
        (sum - radius) / 2f64.sqrt()
    }
}
fn quarter_turn_invariant(cutouts: &[CircularCutout]) -> Result<bool> {
    for cutout in cutouts {
        let rotated_x = cutout.y_um.checked_neg().context(
            "plate.surface.circular_cutouts: x coordinate cannot be quarter-turn rotated",
        )?;
        if cutouts
            .binary_search_by_key(&(rotated_x, cutout.x_um, cutout.radius_um), |candidate| {
                (candidate.x_um, candidate.y_um, candidate.radius_um)
            })
            .is_err()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawTarget {
    schema_version: u32,
    target_id: String,
    revision: u32,
    board_frame_convention: String,
    plate: RawPlate,
    fiducial: RawFiducial,
    lidar_orientation_reference: RawLidarOrientationReference,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPlate {
    side: String,
    surface: RawSurface,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum RawSurface {
    Solid {},
    Perforated { circular_cutouts: Vec<RawCutout> },
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawCutout {
    center: RawPoint,
    radius: String,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPoint {
    x: String,
    y: String,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawFiducial {
    kind: String,
    dictionary: String,
    marker_ids: Vec<u32>,
    paper_side: String,
    paper_center: RawPaperCenter,
    outer_border: String,
    cells_per_side: u32,
    marker_fill_ratio: f64,
    border_bits: u32,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPaperCenter {
    toward_left_corner: String,
    toward_top_corner: String,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum RawLidarOrientationReference {
    MountingUp { local_axis: String },
    AsymmetricCutouts {},
}

#[cfg(test)]
mod tests {
    use super::*;
    const SOLID: &str = include_str!("../../../fixtures/targets/solid_600_aruco_1_v1.json5");
    const HOLLOW: &str = include_str!("../../../fixtures/targets/hollow_1000_aruco_4_v1.json5");
    const LAUNCH_SOLID: &str =
        include_str!("../../../ros/lctk_launch/config/targets/solid_600_aruco_1_v1.json5");
    const LAUNCH_HOLLOW: &str =
        include_str!("../../../ros/lctk_launch/config/targets/hollow_1000_aruco_4_v1.json5");
    const IDENTITY_GOLDEN: &str =
        include_str!("../../../fixtures/targets/canonical_identity.golden");

    #[test]
    fn accepted_targets_have_stable_semantic_identity() {
        for (fixture, launch) in [(SOLID, LAUNCH_SOLID), (HOLLOW, LAUNCH_HOLLOW)] {
            let fixture = CalibrationTarget::from_json5(fixture.as_bytes()).unwrap();
            let launch = CalibrationTarget::from_json5(launch.as_bytes()).unwrap();
            assert_eq!(fixture.canonical_bytes(), launch.canonical_bytes());
            assert_eq!(fixture.identity(), launch.identity());
            assert_eq!(fixture.identity().semantic_sha256.len(), 64);
        }
    }

    #[test]
    fn canonical_bytes_and_hash_match_target_keyed_goldens() {
        for source in [SOLID, HOLLOW] {
            let target = ValidatedTarget::parse_json5(source.as_bytes()).unwrap();
            let (expected_hash, expected_bytes) = golden_for(target.target_id());
            assert_eq!(target.identity().semantic_sha256, expected_hash);
            assert_eq!(target.canonical_bytes(), expected_bytes.as_bytes());
        }
    }

    #[test]
    fn solid_physical_values_are_exact() {
        let target = CalibrationTarget::from_json5(SOLID.as_bytes()).unwrap();
        assert_eq!(target.plate().side_um, 600_000);
        assert_eq!(target.fiducial().paper_side_um, 600_000);
        assert_eq!(target.fiducial().outer_border_um, 60_000);
        assert_eq!(
            target.fiducial().paper_side_um - 2 * target.fiducial().outer_border_um,
            480_000
        );
        let marker_half_diagonal_um = 480_000.0 / 2f64.sqrt();
        assert!((marker_half_diagonal_um - 339_411.254_970).abs() < 1e-6);
        assert_eq!(target.fiducial().marker_ids, vec![1]);
    }

    #[test]
    fn derived_legacy_cutout_lengths_round_to_micrometres() {
        let target = CalibrationTarget::from_json5(HOLLOW.as_bytes()).unwrap();
        let Surface::Perforated { circular_cutouts } = &target.plate().surface else {
            panic!("accepted hollow target must be perforated");
        };
        assert!(circular_cutouts.iter().any(|cutout| cutout.x_um == 282_843));
        assert!(circular_cutouts.iter().any(|cutout| cutout.y_um == 282_843));
    }

    #[test]
    fn semantic_mutations_change_identity() {
        // Frame convention, surface/fiducial enum names, dictionary, and mounting-up's
        // local axis currently have only one accepted spelling/value. Their rejected
        // alternatives are covered by `invalid_definition_table_covers_schema_rejections`;
        // this table grows when another valid variant is introduced.
        //
        // Surface-kind change also changes the orientation-reference evidence because
        // `asymmetric_cutouts` is physically invalid on a solid surface. All other rows
        // mutate precisely the named semantic field (or fields required by that invariant).
        for (name, original, changed) in [
            ("target ID", LAUNCH_SOLID, LAUNCH_SOLID.replacen("solid_600_aruco_1", "other_target", 1)),
            ("revision", LAUNCH_SOLID, LAUNCH_SOLID.replacen("revision: 1", "revision: 2", 1)),
            ("plate side", LAUNCH_SOLID, LAUNCH_SOLID.replacen("side: \"600mm\"", "side: \"601mm\"", 1)),
            ("surface kind", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("kind: \"perforated\",\n      circular_cutouts: [\n        { center: { x: \"282.842712mm\", y: \"0mm\" }, radius: \"150mm\" },\n        { center: { x: \"0mm\", y: \"282.842712mm\" }, radius: \"150mm\" },\n        { center: { x: \"-282.842712mm\", y: \"0mm\" }, radius: \"150mm\" },\n      ],", "kind: \"solid\",", 1).replacen("kind: \"asymmetric_cutouts\",", "kind: \"mounting_up\",\n    local_axis: \"+y\",", 1)),
            ("cutout center", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("282.842712mm", "281mm", 1)),
            ("cutout radius", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("radius: \"150mm\"", "radius: \"149mm\"", 1)),
            ("marker ID", LAUNCH_SOLID, LAUNCH_SOLID.replacen("marker_ids: [1]", "marker_ids: [2]", 1)),
            ("marker order", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("[696, 64, 306, 195]", "[64, 696, 306, 195]", 1)),
            ("paper side", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("paper_side: \"500mm\"", "paper_side: \"499mm\"", 1)),
            ("paper center", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("-353.553391mm", "-350mm", 1)),
            ("outer border", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("outer_border: \"10mm\"", "outer_border: \"11mm\"", 1)),
            ("grid layout", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("marker_ids: [696, 64, 306, 195]", "marker_ids: [696]", 1).replacen("cells_per_side: 2", "cells_per_side: 1", 1)),
            ("marker fill ratio", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("marker_fill_ratio: 0.8", "marker_fill_ratio: 0.7", 1)),
            ("border bits", LAUNCH_SOLID, LAUNCH_SOLID.replacen("border_bits: 1", "border_bits: 2", 1)),
            ("orientation reference", LAUNCH_HOLLOW, LAUNCH_HOLLOW.replacen("kind: \"asymmetric_cutouts\",", "kind: \"mounting_up\",\n    local_axis: \"+y\",", 1)),
        ] {
            let baseline = ValidatedTarget::parse_json5(original.as_bytes())
                .unwrap_or_else(|error| panic!("{name} baseline is invalid: {error}"));
            let changed = ValidatedTarget::parse_json5(changed.as_bytes())
                .unwrap_or_else(|error| panic!("{name} mutation is invalid: {error}"));
            assert_ne!(
                baseline.identity().semantic_sha256,
                changed.identity().semantic_sha256,
                "{name} did not change identity"
            );
        }
    }

    #[test]
    fn invalid_definition_table_covers_schema_rejections() {
        for (name, source, field) in [
            (
                "unknown schema",
                LAUNCH_SOLID.replacen("schema_version: 1", "schema_version: 2", 1),
                "schema_version",
            ),
            (
                "unknown frame",
                LAUNCH_SOLID.replacen("corner_aligned_plate_center_v1", "other", 1),
                "board_frame_convention",
            ),
            (
                "unknown surface",
                SOLID.replacen("kind: \"solid\"", "kind: \"unknown\"", 1),
                "invalid Target Definition JSON5",
            ),
            (
                "unknown fiducial",
                LAUNCH_SOLID.replacen("kind: \"square_aruco_grid\"", "kind: \"other\"", 1),
                "fiducial.kind",
            ),
            (
                "unknown orientation",
                LAUNCH_SOLID.replacen("kind: \"mounting_up\"", "kind: \"other\"", 1),
                "invalid Target Definition JSON5",
            ),
            (
                "unknown field",
                LAUNCH_SOLID.replacen("revision: 1", "revision: 1, unexpected: true", 1),
                "invalid Target Definition JSON5",
            ),
            (
                "empty target ID",
                LAUNCH_SOLID.replacen("solid_600_aruco_1", " ", 1),
                "target_id",
            ),
            (
                "zero revision",
                LAUNCH_SOLID.replacen("revision: 1", "revision: 0", 1),
                "revision",
            ),
            (
                "nonfinite dimension",
                LAUNCH_SOLID.replacen("side: \"600mm\"", "side: \"NaNmm\"", 1),
                "plate.side",
            ),
            (
                "nonpositive dimension",
                LAUNCH_SOLID.replacen("side: \"600mm\"", "side: \"0mm\"", 1),
                "plate.side",
            ),
            (
                "zero cutout radius",
                LAUNCH_HOLLOW.replacen("radius: \"150mm\"", "radius: \"0mm\"", 1),
                "radius",
            ),
            (
                "cutout outside plate",
                LAUNCH_HOLLOW.replacen("282.842712mm", "600mm", 1),
                "extends outside",
            ),
            (
                "overlapping cutouts",
                LAUNCH_HOLLOW.replacen("282.842712mm", "0mm", 1),
                "overlap",
            ),
            (
                "paper outside plate",
                LAUNCH_SOLID.replacen(
                    "toward_top_corner: \"0mm\"",
                    "toward_top_corner: \"300mm\"",
                    1,
                ),
                "paper corners",
            ),
            (
                "paper intersects cutout",
                LAUNCH_HOLLOW.replacen("-353.553391mm", "0mm", 1),
                "paper intersects",
            ),
            (
                "zero cells",
                LAUNCH_SOLID.replacen("cells_per_side: 1", "cells_per_side: 0", 1),
                "cells_per_side",
            ),
            (
                "marker count",
                SOLID.replacen("marker_ids: [1]", "marker_ids: [1, 1]", 1),
                "fiducial.marker_ids",
            ),
            (
                "duplicate marker",
                LAUNCH_HOLLOW.replacen("64", "696", 1),
                "duplicate ID",
            ),
            (
                "out of dictionary marker",
                LAUNCH_SOLID.replacen("marker_ids: [1]", "marker_ids: [1000]", 1),
                "outside",
            ),
            (
                "outer border relation",
                LAUNCH_SOLID.replacen("outer_border: \"60mm\"", "outer_border: \"300mm\"", 1),
                "fiducial.outer_border",
            ),
            (
                "marker fill ratio",
                LAUNCH_SOLID.replacen("marker_fill_ratio: 1.0", "marker_fill_ratio: 0.0", 1),
                "marker_fill_ratio",
            ),
            (
                "border bits",
                LAUNCH_SOLID.replacen("border_bits: 1", "border_bits: 0", 1),
                "border_bits",
            ),
            (
                "quarter turn symmetry",
                symmetric_perforated_definition(),
                "quarter-turn symmetry",
            ),
            (
                "cutout reference on solid",
                LAUNCH_SOLID.replacen(
                    "kind: \"mounting_up\",\n    local_axis: \"+y\",",
                    "kind: \"asymmetric_cutouts\",",
                    1,
                ),
                "asymmetric_cutouts requires a perforated",
            ),
        ] {
            let error = CalibrationTarget::from_json5(source.as_bytes())
                .unwrap_err()
                .to_string();
            assert!(
                error.contains(field),
                "{name}: {error} does not name {field}"
            );
        }
    }

    #[test]
    fn hostile_finite_lengths_return_errors_without_panicking() {
        for source in [
            LAUNCH_HOLLOW.replacen("282.842712mm", "-9223372036854775.808mm", 1),
            LAUNCH_SOLID.replacen(
                "outer_border: \"60mm\"",
                "outer_border: \"9223372036854775.807mm\"",
                1,
            ),
        ] {
            let result =
                std::panic::catch_unwind(|| ValidatedTarget::parse_json5(source.as_bytes()));
            assert!(
                matches!(result, Ok(Err(_))),
                "hostile finite length panicked or parsed"
            );
        }
    }

    fn golden_for(target_id: &str) -> (&str, &str) {
        for block in IDENTITY_GOLDEN.split("---\n") {
            let Some(block) = block.find("target_id=").map(|index| &block[index..]) else {
                continue;
            };
            let Some((target, rest)) = block
                .strip_prefix("target_id=")
                .and_then(|block| block.split_once('\n'))
            else {
                continue;
            };
            let Some((hash, bytes)) = rest
                .strip_prefix("semantic_sha256=")
                .and_then(|rest| rest.split_once("\ncanonical_bytes:\n"))
            else {
                continue;
            };
            if target == target_id {
                return (hash, bytes);
            }
        }
        panic!("missing golden for {target_id}");
    }

    fn symmetric_perforated_definition() -> String {
        r#"{ schema_version: 1, target_id: "symmetric", revision: 1, board_frame_convention: "corner_aligned_plate_center_v1",
          plate: { side: "1000mm", surface: { kind: "perforated", circular_cutouts: [
            { center: { x: "400mm", y: "0mm" }, radius: "1mm" }, { center: { x: "0mm", y: "400mm" }, radius: "1mm" },
            { center: { x: "-400mm", y: "0mm" }, radius: "1mm" }, { center: { x: "0mm", y: "-400mm" }, radius: "1mm" }, ]}},
          fiducial: { kind: "square_aruco_grid", dictionary: "DICT_5X5_1000", marker_ids: [1], paper_side: "100mm",
            paper_center: { toward_left_corner: "0mm", toward_top_corner: "0mm" }, outer_border: "1mm", cells_per_side: 1, marker_fill_ratio: 1.0, border_bits: 1 },
          lidar_orientation_reference: { kind: "asymmetric_cutouts" } }"#.to_owned()
    }
}
