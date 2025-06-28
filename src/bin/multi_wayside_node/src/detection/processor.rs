use crate::{
    detection::DetectionProcessor,
    pointcloud::{PointCloudFilter, PointCloudParser},
    roi::RoiManager,
    types::BoardDetection,
};
use eyre::Result;
use nalgebra::Point3;
use sensor_msgs::msg::PointCloud2;
use std::sync::Arc;

/// Complete point cloud processing pipeline
pub struct DetectionPipeline<P, F, R, D>
where
    P: PointCloudParser,
    F: PointCloudFilter,
    R: RoiManager,
    D: DetectionProcessor,
{
    parser: Arc<P>,
    filter: Arc<F>,
    roi_manager: Arc<R>,
    detector: Arc<D>,
}

impl<P, F, R, D> DetectionPipeline<P, F, R, D>
where
    P: PointCloudParser,
    F: PointCloudFilter,
    R: RoiManager,
    D: DetectionProcessor,
{
    pub fn new(parser: Arc<P>, filter: Arc<F>, roi_manager: Arc<R>, detector: Arc<D>) -> Self {
        Self {
            parser,
            filter,
            roi_manager,
            detector,
        }
    }

    /// Process a PointCloud2 message through the complete pipeline
    pub fn process_pointcloud(&self, msg: &PointCloud2, lidar_id: u8) -> Result<ProcessingResult> {
        // Step 1: Parse point cloud
        let parsed_points = self.parser.parse(msg)?;
        let nalgebra_points = self.parser.to_nalgebra_points(&parsed_points);

        // Step 2: Apply filtering
        let filtered_points = self.filter.filter_nalgebra(&nalgebra_points);

        // Step 3: Apply ROI cropping
        let cropped_points = self.roi_manager.apply_crop(&filtered_points, lidar_id);

        // Step 4: Detect board
        let detection = self.detector.process(&cropped_points)?;

        Ok(ProcessingResult {
            original_points: nalgebra_points,
            filtered_points,
            cropped_points,
            detection,
        })
    }
}

/// Result of processing a point cloud through the pipeline
pub struct ProcessingResult {
    pub original_points: Vec<Point3<f64>>,
    pub filtered_points: Vec<Point3<f64>>,
    pub cropped_points: Vec<Point3<f64>>,
    pub detection: Option<BoardDetection>,
}

impl ProcessingResult {
    pub fn has_detection(&self) -> bool {
        self.detection.is_some()
    }

    pub fn original_count(&self) -> usize {
        self.original_points.len()
    }

    pub fn filtered_count(&self) -> usize {
        self.filtered_points.len()
    }

    pub fn cropped_count(&self) -> usize {
        self.cropped_points.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        detection::MockDetectionProcessor,
        pointcloud::{DefaultPointCloudParser, RangeFilter},
        roi::DefaultRoiManager,
        types::RoiBounds,
    };
    use sensor_msgs::msg::{PointCloud2, PointField};
    use std::collections::HashMap;

    fn create_test_pointcloud() -> PointCloud2 {
        let mut msg = PointCloud2 {
            height: 1,
            width: 3,
            point_step: 12,
            fields: vec![
                PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
            ],
            ..Default::default()
        };

        // Points: inside range, outside range, inside range
        let points = vec![
            (1.0f32, 0.0f32, 0.0f32),
            (10.0f32, 0.0f32, 0.0f32),
            (2.0f32, 0.0f32, 0.0f32),
        ];

        msg.data = Vec::with_capacity(points.len() * 12);
        for (x, y, z) in points {
            msg.data.extend_from_slice(&x.to_le_bytes());
            msg.data.extend_from_slice(&y.to_le_bytes());
            msg.data.extend_from_slice(&z.to_le_bytes());
        }

        msg.row_step = msg.data.len() as u32;
        msg
    }

    #[test]
    fn test_detection_pipeline() {
        // Set up components
        let parser = Arc::new(DefaultPointCloudParser);
        let filter = Arc::new(RangeFilter::new(0.5, 5.0)); // Filter out point at x=10

        let mut roi_bounds = HashMap::new();
        roi_bounds.insert(
            1,
            RoiBounds {
                min_x: -5.0,
                max_x: 5.0,
                min_y: -5.0,
                max_y: 5.0,
                min_z: -5.0,
                max_z: 5.0,
            },
        );
        let roi_manager = Arc::new(DefaultRoiManager::with_initial_bounds(roi_bounds));

        let detector = Arc::new(MockDetectionProcessor::new(true));

        // Create pipeline
        let pipeline = DetectionPipeline::new(parser, filter, roi_manager, detector);

        // Process test data
        let msg = create_test_pointcloud();
        let result = pipeline.process_pointcloud(&msg, 1).unwrap();

        // Verify results
        assert_eq!(result.original_count(), 3);
        assert_eq!(result.filtered_count(), 2); // One point filtered out by range
        assert_eq!(result.cropped_count(), 2); // All filtered points within ROI
        assert!(result.has_detection());
    }

    #[test]
    fn test_detection_pipeline_no_detection() {
        let parser = Arc::new(DefaultPointCloudParser);
        let filter = Arc::new(RangeFilter::new(0.5, 5.0));
        let roi_manager = Arc::new(DefaultRoiManager::new());
        let detector = Arc::new(MockDetectionProcessor::new(false)); // No detection

        let pipeline = DetectionPipeline::new(parser, filter, roi_manager, detector);

        let msg = create_test_pointcloud();
        let result = pipeline.process_pointcloud(&msg, 1).unwrap();

        assert!(!result.has_detection());
    }
}
