//! Rerun visualization logic for the board-fitter example.

use anyhow::Result;
use board_fitter::{
    diamond::DiamondSquare, BoardDetection, DetectedHole, DetectedPlane, PointCloud,
};
use nalgebra::Isometry3;
use rerun::{
    datatypes::{Quaternion, Vec3D},
    Color, RecordingStream,
};

/// Encapsulates all Rerun visualization logic.
pub struct Viewer {
    rec: RecordingStream,
}

impl Viewer {
    /// Create a new Viewer and initialize the Rerun recording stream.
    pub fn new(connect_addr: Option<String>, serve_port: Option<String>) -> Result<Self> {
        let rec = if let Some(_addr) = connect_addr {
            rerun::RecordingStreamBuilder::new("board-fitter").connect_grpc()? // Call without arguments
        } else if let Some(_port) = serve_port {
            rerun::RecordingStreamBuilder::new("board-fitter").serve_grpc()? // Call without arguments
        } else {
            rerun::RecordingStreamBuilder::new("board-fitter").spawn()?
        };
        Ok(Self { rec })
    }

    /// Log the main point cloud.
    pub fn log_point_cloud(&self, cloud: &PointCloud) -> Result<()> {
        let points: Vec<[f32; 3]> = cloud
            .points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();

        self.rec
            .log("input/point_cloud", &rerun::Points3D::new(points))?;
        Ok(())
    }

    /// Log detected planes.
    pub fn log_planes(&self, planes: &[DetectedPlane]) -> Result<()> {
        for (i, plane) in planes.iter().enumerate() {
            let center = Vec3D::new(
                plane.point.x as f32,
                plane.point.y as f32,
                plane.point.z as f32,
            );
            let normal = Vec3D::new(
                plane.normal.x as f32,
                plane.normal.y as f32,
                plane.normal.z as f32,
            );

            self.rec.log(
                format!("/processing/planes/{i}"),
                &rerun::Arrows3D::from_vectors([normal])
                    .with_origins([[center.x(), center.y(), center.z()]])
                    .with_colors([Color::from_rgb(255, 0, 255)]),
            )?;
        }
        Ok(())
    }

    /// Log detected diamond squares.
    #[allow(dead_code)]
    pub fn log_diamond_squares(&self, diamonds: &[DiamondSquare]) -> Result<()> {
        for (i, diamond) in diamonds.iter().enumerate() {
            let half_size = diamond.size as f32 / 2.0;

            // For now, log the diamond as a box without rotation
            // TODO: Add rotation support when rerun API supports it
            self.rec.log(
                format!("/processing/diamonds/{i}"),
                &rerun::Boxes3D::from_half_sizes([Vec3D::new(half_size, half_size, 0.01)])
                    .with_centers([[
                        diamond.center.x as f32,
                        diamond.center.y as f32,
                        diamond.center.z as f32,
                    ]])
                    .with_colors([Color::from_rgb(0, 255, 255)]),
            )?;
        }
        Ok(())
    }

    /// Log detected holes.
    pub fn log_holes(&self, holes: &[DetectedHole]) -> Result<()> {
        for (i, hole) in holes.iter().enumerate() {
            let radius = hole.radius as f32;

            self.rec.log(
                format!("/processing/holes/{i}"),
                &rerun::Points3D::new([[
                    hole.center.x as f32,
                    hole.center.y as f32,
                    hole.center.z as f32,
                ]])
                .with_radii([radius])
                .with_colors([Color::from_rgb(255, 255, 0)]),
            )?;
        }
        Ok(())
    }

    /// Log final board detections.
    pub fn log_detections(
        &self,
        detections: &[BoardDetection],
        original_cloud: &PointCloud,
    ) -> Result<()> {
        for detection in detections {
            self.log_detection(detection, original_cloud)?;
        }
        Ok(())
    }

    /// Log a single board detection.
    fn log_detection(&self, detection: &BoardDetection, original_cloud: &PointCloud) -> Result<()> {
        let id = detection.id.to_string();
        let entity_path = format!("/detections/final/{id}");

        // Log pose
        self.log_transform(&entity_path, &detection.pose)?;

        // Log bounding box
        let half_extents = Vec3D::new(
            (detection.dimensions.x / 2.0) as f32,
            (detection.dimensions.y / 2.0) as f32,
            (detection.dimensions.z / 2.0) as f32,
        );
        let color = self.color_from_confidence(detection.confidence.value());

        self.rec.log(
            format!("{entity_path}/bbox"),
            &rerun::Boxes3D::from_half_sizes([half_extents]).with_colors([color]),
        )?;

        // Log confidence score as text
        let confidence_text = format!("Conf: {:.2}", detection.confidence.value());
        self.rec.log(
            format!("{entity_path}/confidence"),
            &rerun::TextLog::new(confidence_text),
        )?;

        // Log inlier points
        let inlier_points: Vec<[f32; 3]> = detection
            .supporting_points
            .iter()
            .filter_map(|&idx| original_cloud.points.get(idx))
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        self.rec.log(
            format!("{entity_path}/points"),
            &rerun::Points3D::new(inlier_points).with_colors([color]),
        )?;

        Ok(())
    }

    /// Log a 3D transform.
    fn log_transform(&self, path: &str, transform: &Isometry3<f64>) -> Result<()> {
        let translation = transform.translation.vector;
        let rotation = transform.rotation.quaternion();

        self.rec.log(
            path,
            &rerun::Transform3D::from_translation_rotation(
                [
                    translation.x as f32,
                    translation.y as f32,
                    translation.z as f32,
                ],
                Quaternion::from_xyzw([
                    rotation.i as f32,
                    rotation.j as f32,
                    rotation.k as f32,
                    rotation.w as f32,
                ]),
            ),
        )?;

        Ok(())
    }

    /// Get a color based on a confidence score.
    fn color_from_confidence(&self, confidence: f64) -> Color {
        let g = (255.0 * confidence) as u8;
        let r = 255 - g;
        Color::from_rgb(r, g, 0)
    }
}
