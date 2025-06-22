use eyre::Result;
use geometry_msgs::msg::TransformStamped;
use rclrs::{Node, Publisher};
use sensor_msgs::msg::PointCloud2;
use std::sync::Arc;
use vision_msgs::msg::Detection3DArray;
use visualization_msgs::msg::MarkerArray;

/// Trait for managing ROS 2 publishers
pub trait PublisherManager: Send + Sync {
    fn publish_detection(&self, detection: &Detection3DArray, lidar_id: u8) -> Result<()>;
    fn publish_point_cloud(
        &self,
        cloud: &PointCloud2,
        cloud_type: &str,
        lidar_id: u8,
    ) -> Result<()>;
    fn publish_markers(&self, markers: &MarkerArray, marker_type: &str) -> Result<()>;
    fn publish_transform(&self, transform: &TransformStamped) -> Result<()>;
}

/// Default implementation of PublisherManager
pub struct DefaultPublisherManager {
    // Detection publishers
    lidar1_detection_pub: Arc<Publisher<Detection3DArray>>,
    lidar2_detection_pub: Arc<Publisher<Detection3DArray>>,

    // Point cloud publishers
    lidar1_filtered_pub: Arc<Publisher<PointCloud2>>,
    lidar2_filtered_pub: Arc<Publisher<PointCloud2>>,
    lidar1_cropped_pub: Arc<Publisher<PointCloud2>>,
    lidar2_cropped_pub: Arc<Publisher<PointCloud2>>,

    // Marker publishers
    board_marker_pub: Arc<Publisher<MarkerArray>>,
    roi_marker_pub: Arc<Publisher<MarkerArray>>,
    adjustment_marker_pub: Arc<Publisher<MarkerArray>>,

    // Transform publisher
    transform_pub: Arc<Publisher<TransformStamped>>,
}

impl DefaultPublisherManager {
    pub fn new(node: &Node) -> Result<Self> {
        // Create all publishers
        let lidar1_detection_pub = Arc::new(node.create_publisher("/lidar1/board_detection")?);
        let lidar2_detection_pub = Arc::new(node.create_publisher("/lidar2/board_detection")?);

        let lidar1_filtered_pub = Arc::new(node.create_publisher("/lidar1/points_filtered")?);
        let lidar2_filtered_pub = Arc::new(node.create_publisher("/lidar2/points_filtered")?);
        let lidar1_cropped_pub = Arc::new(node.create_publisher("/lidar1/points_cropped")?);
        let lidar2_cropped_pub = Arc::new(node.create_publisher("/lidar2/points_cropped")?);

        let board_marker_pub = Arc::new(node.create_publisher("/calibration_markers")?);
        let roi_marker_pub = Arc::new(node.create_publisher("/roi_markers")?);
        let adjustment_marker_pub = Arc::new(node.create_publisher("/adjustment_markers")?);

        let transform_pub = Arc::new(node.create_publisher("/calibration_transform")?);

        Ok(Self {
            lidar1_detection_pub,
            lidar2_detection_pub,
            lidar1_filtered_pub,
            lidar2_filtered_pub,
            lidar1_cropped_pub,
            lidar2_cropped_pub,
            board_marker_pub,
            roi_marker_pub,
            adjustment_marker_pub,
            transform_pub,
        })
    }
}

impl PublisherManager for DefaultPublisherManager {
    fn publish_detection(&self, detection: &Detection3DArray, lidar_id: u8) -> Result<()> {
        match lidar_id {
            1 => self.lidar1_detection_pub.publish(detection)?,
            2 => self.lidar2_detection_pub.publish(detection)?,
            _ => return Err(eyre::eyre!("Invalid lidar_id: {}", lidar_id)),
        }
        Ok(())
    }

    fn publish_point_cloud(
        &self,
        cloud: &PointCloud2,
        cloud_type: &str,
        lidar_id: u8,
    ) -> Result<()> {
        match (lidar_id, cloud_type) {
            (1, "filtered") => self.lidar1_filtered_pub.publish(cloud)?,
            (2, "filtered") => self.lidar2_filtered_pub.publish(cloud)?,
            (1, "cropped") => self.lidar1_cropped_pub.publish(cloud)?,
            (2, "cropped") => self.lidar2_cropped_pub.publish(cloud)?,
            _ => {
                return Err(eyre::eyre!(
                    "Invalid combination: lidar_id={}, type={}",
                    lidar_id,
                    cloud_type
                ))
            }
        }
        Ok(())
    }

    fn publish_markers(&self, markers: &MarkerArray, marker_type: &str) -> Result<()> {
        match marker_type {
            "board" => self.board_marker_pub.publish(markers)?,
            "roi" => self.roi_marker_pub.publish(markers)?,
            "adjustment" => self.adjustment_marker_pub.publish(markers)?,
            _ => return Err(eyre::eyre!("Invalid marker type: {}", marker_type)),
        }
        Ok(())
    }

    fn publish_transform(&self, transform: &TransformStamped) -> Result<()> {
        self.transform_pub.publish(transform)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rclrs::{Context, CreateBasicExecutor, InitOptions};

    #[test]
    fn test_publisher_manager_creation() {
        let context = Context::new(std::env::args(), InitOptions::default()).unwrap();
        let executor = context.create_basic_executor();
        let node = executor.create_node("test_node").unwrap();

        let result = DefaultPublisherManager::new(&node);
        assert!(result.is_ok());
    }
}
